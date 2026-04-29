/* Copyright (c) 2024 Osamu Watanabe Takushoku University, Japan. */
/* ht_encoder.hpp - HTJ2K encoder */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <functional>

#include "core/stream_info.hpp"
#include "core/video_options.hpp"

#include "core/options.hpp"

#include "subprojects/kakadujs/src/HTJ2KEncoder.hpp"

#include "rfc9828_packetizer.hpp"
#include "simple_tcp.hpp"

uint8_t hotfix_for_mainheader[32] = { 0xFF, 0x4F, 0xFF, 0x51, 0x00, 0x2F, 0x40, 0x00, 0x00, 0x00, 0x07,
									  0x80, 0x00, 0x00, 0x04, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									  0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x04, 0x38 };
class HT_Encoder
{
public:
	HT_Encoder(std::vector<uint8_t> &encoded_, const FrameInfo &info, Options const *options)
		: abortEncode_(false), abortOutput_(false), index_(0), enc(encoded_, info), buf(encoded_),
		  tcp_socket_("133.36.41.118", 4001), tcp_connected_(false),
		  rtp_packetizer_(
			  options->Get().rtp_host, options->Get().rtp_port,
			  RFC9828Packetizer::Colorspace { /*colorspace_set=*/true, static_cast<uint8_t>(options->Get().rtp_prims),
											  static_cast<uint8_t>(options->Get().rtp_trans),
											  static_cast<uint8_t>(options->Get().rtp_mat), options->Get().rtp_range })
	{
		tcp_connected_ = (tcp_socket_.create_client() >= 0);
		if (!tcp_connected_)
			LOG(1, "HT_Encoder: TCP connect to 133.36.41.118:4001 failed; will retry per frame");
		if (!rtp_packetizer_.is_open())
			LOG(1, "HT_Encoder: RTP UDP socket open failed for " << options->Get().rtp_host << ":"
																 << options->Get().rtp_port);
		else
			LOG(2, "HT_Encoder: RTP fan-out -> " << options->Get().rtp_host << ":" << options->Get().rtp_port);
		output_thread_ = std::thread(&HT_Encoder::outputThread, this);
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i] = std::thread(std::bind(&HT_Encoder::encodeThread, this, i));
		LOG(2, "Opened HT_Encoder");
	}
	~HT_Encoder()
	{
		abortEncode_ = true;
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i].join();
		abortOutput_ = true;
		output_thread_.join();
		LOG(2, "HT_Encoder closed");
	}
	// Encode the given buffer.  RTP fans out every frame; TCP archive only
	// fires when archive_this_frame is true (caller's rate-limit decision).
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us,
					  bool archive_this_frame = false)
	{
		std::lock_guard<std::mutex> lock(encode_mutex_);
		EncodeItem item = { mem, info, timestamp_us, index_++, archive_this_frame };
		encode_queue_.push(item);
		encode_cond_var_.notify_all();
	}

private:
	// How many threads to use. Whichever thread is idle will pick up the next frame.
	static const int NUM_ENC_THREADS = 1;

	// These threads do the actual encoding.
	void encodeThread(int num)
	{
		std::chrono::duration<double> encode_time(0);
		uint32_t frames = 0;

		// Once-per-second rolling-stats accumulators (avoid per-frame printf
		// at 30+ fps, which is itself measurable on the Pi).
		auto stats_window_start = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> stats_encode_total { 0 };
		uint32_t stats_frames = 0;
		uint64_t stats_bytes_total = 0;
		uint32_t stats_archive_frames = 0;

		EncodeItem encode_item;
		while (true)
		{
			{
				std::unique_lock<std::mutex> lock(encode_mutex_);
				while (true)
				{
					using namespace std::chrono_literals;
					if (abortEncode_ && encode_queue_.empty())
					{
						if (frames)
							LOG(2, "Encode " << frames << " frames, average time "
											 << encode_time.count() * 1000 / frames << "ms");
						// jpeg_destroy_compress(&cinfo);
						return;
					}
					if (!encode_queue_.empty())
					{
						encode_item = encode_queue_.front();
						encode_queue_.pop();
						break;
					}
					else
						encode_cond_var_.wait_for(lock, 200ms);
				}
			}

			// Encode the buffer.
			uint8_t *encoded_buffer = nullptr;
			size_t buffer_len = 0;
			auto start_time = std::chrono::high_resolution_clock::now();
			{
				// std::unique_lock<std::mutex> lock(encode_mutex_);
				encodeHTJ2K(encode_item, buf, buffer_len);
				// for (int i = 0; i < 32; ++i)
				// {
				// 	printf("%02X ", buf[i]);
				// }
				// printf("\n");
			}
			encode_time = (std::chrono::high_resolution_clock::now() - start_time);

			// RTP fan-out: every encoded frame, for continuous live monitoring.
			if (rtp_packetizer_.is_open())
			{
				// Camera timestamp is microseconds; RTP timestamp is 90 kHz ticks.
				// us * 9 / 100 = us * 90 / 1000 with less overflow risk on int64.
				const uint32_t ts90 = static_cast<uint32_t>(encode_item.timestamp_us * 9 / 100);
				if (!rtp_packetizer_.send_codestream(buf.data(), buffer_len, ts90))
					LOG(1, "HT_Encoder: RTP send_codestream failed for frame " << encode_item.index);
			}

			// TCP archive: only when caller flagged this frame (e.g. person-detected
			// + rate-limited).  Uses the persistent length-framed send from stage 1;
			// independent of the RTP path above.
			if (encode_item.archive)
			{
				if (!tcp_connected_)
					tcp_connected_ = (tcp_socket_.create_client() >= 0);
				if (tcp_connected_)
				{
					if (!tcp_socket_.SendFramed(buf.data(), static_cast<uint32_t>(buffer_len)))
					{
						LOG(1, "HT_Encoder: TCP send failed; reconnecting on next frame");
						tcp_socket_.destroy();
						tcp_connected_ = false;
					}
				}
				++stats_archive_frames;
			}

			// Per-frame line: only at verbose level 2.
			LOG(2,
				"HT codestream size=" << buffer_len << " bytes, encode_time=" << encode_time.count() * 1000.0 << " ms");
			frames++;
			stats_encode_total += encode_time;
			++stats_frames;
			stats_bytes_total += buffer_len;

			// Once-per-second summary at default verbosity.
			auto wall = std::chrono::high_resolution_clock::now();
			auto window = std::chrono::duration<double>(wall - stats_window_start);
			if (window.count() >= 1.0 && stats_frames > 0)
			{
				char line[160];
				snprintf(line, sizeof(line), "%u fps, avg %.2f ms/frame, %.0f kbps, archived %u", stats_frames,
						 stats_encode_total.count() * 1000.0 / stats_frames,
						 (stats_bytes_total * 8.0 / 1000.0) / window.count(), stats_archive_frames);
				LOG(1, "HT_Encoder: " << line);
				stats_window_start = wall;
				stats_encode_total = std::chrono::duration<double> { 0 };
				stats_frames = 0;
				stats_bytes_total = 0;
				stats_archive_frames = 0;
			}
			// Don't return buffers until the output thread as that's where they're
			// in order again.

			// We push this encoded buffer to another thread so that our
			// application can take its time with the data without blocking the
			// encode process.
			OutputItem output_item = { encoded_buffer, buffer_len, encode_item.timestamp_us, encode_item.index };
			std::lock_guard<std::mutex> lock(output_mutex_);
			output_queue_[num].push(output_item);
			output_cond_var_.notify_one();
		}
	}

	// Handle the output buffers in another thread so as not to block the encoders. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread()
	{
		OutputItem item;
		uint64_t index = 0;
		while (true)
		{
			{
				std::unique_lock<std::mutex> lock(output_mutex_);
				while (true)
				{
					using namespace std::chrono_literals;
					// We look for the thread that's completed the frame we want next.
					// If we don't find it, we wait.
					//
					// Must also check for an abort signal, and if set, all queues must
					// be empty. This is done first to ensure all frame callbacks have
					// had a chance to run.
					bool abort = abortOutput_ ? true : false;
					for (auto &q : output_queue_)
					{
						if (abort && !q.empty())
							abort = false;

						if (!q.empty() && q.front().index == index)
						{
							item = q.front();
							q.pop();
							goto got_item;
						}
					}
					if (abort)
						return;

					output_cond_var_.wait_for(lock, 200ms);
				}
			}
		got_item:
			// no need of callback
			// free(item.mem);
			index++;
		}
	}

	bool abortEncode_;
	bool abortOutput_;
	uint64_t index_;

	struct EncodeItem
	{
		void *mem;
		StreamInfo info;
		int64_t timestamp_us;
		uint64_t index;
		bool archive;
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];
	void encodeHTJ2K(EncodeItem &item, std::vector<uint8_t> &out, size_t &buffer_len)
	{
		enc.setSourceImage((uint8_t *)item.mem, item.info.width * item.info.height * 3);
		enc.encode();
		out = enc.getEncodedBytes();
		// encoded_buffer = out.data();
		buffer_len = out.size();
	}

	struct OutputItem
	{
		void *mem;
		size_t bytes_used;
		int64_t timestamp_us;
		uint64_t index;
	};
	std::queue<OutputItem> output_queue_[NUM_ENC_THREADS];
	std::mutex output_mutex_;
	std::condition_variable output_cond_var_;
	std::thread output_thread_;

	HTJ2KEncoder enc; // encoder instance
	std::vector<uint8_t> &buf;
	simple_tcp tcp_socket_;
	bool tcp_connected_;
	RFC9828Packetizer rtp_packetizer_;
};
