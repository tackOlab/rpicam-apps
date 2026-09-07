/* Copyright (c) 2024 Osamu Watanabe Takushoku University, Japan. */
/* ht_encoder.hpp - HTJ2K encoder */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

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

// Pipeline: EncodeBuffer() -> encode_queue_ -> encodeThread (Kakadu)
//                          -> send_queue_   -> sendThread    (paced RTP fan-out)
//                          -> archive_queue_ -> archiveThread (TCP snapshot)
//
// The three stages are separate threads on purpose. Pacing a frame's packets
// takes most of a frame interval, and the TCP archive talks to a remote host
// over the WAN; neither may run on the encode thread, or a single slow send
// stalls capture-to-encode for every following frame.
class HT_Encoder
{
public:
	HT_Encoder(std::vector<uint8_t> &encoded_, const FrameInfo &info, Options const *options)
		: abortEncode_(false), abortSend_(false), abortArchive_(false), index_(0), enc(encoded_, info), buf(encoded_),
		  tcp_socket_("133.36.41.118", 4001), tcp_connected_(false),
		  rtp_packetizer_(
			  options->Get().rtp_host, options->Get().rtp_port,
			  RFC9828Packetizer::Colorspace { /*colorspace_set=*/true, static_cast<uint8_t>(options->Get().rtp_prims),
											  static_cast<uint8_t>(options->Get().rtp_trans),
											  static_cast<uint8_t>(options->Get().rtp_mat), options->Get().rtp_range },
			  options->Get().rtp_mtu)
	{
		// Spread each frame's packets over a fraction of one frame interval so the
		// receiver's socket buffer sees a trickle rather than a whole-frame burst.
		const double fps = options->Get().framerate ? static_cast<double>(*options->Get().framerate) : 30.0;
		const int64_t interval_ns = (fps > 0.0) ? static_cast<int64_t>(1e9 / fps) : 33333333;
		const double frac = std::min(std::max(options->Get().rtp_pace, 0.0), 0.95);
		rtp_packetizer_.set_pacing_window(frac > 0.0 ? static_cast<int64_t>(interval_ns * frac) : 0);

		tcp_connected_ = (tcp_socket_.create_client() >= 0);
		if (!tcp_connected_)
			LOG(1, "HT_Encoder: TCP connect to 133.36.41.118:4001 failed; will retry per frame");
		if (!rtp_packetizer_.is_open())
			LOG(1, "HT_Encoder: RTP UDP socket open failed for " << options->Get().rtp_host << ":"
																 << options->Get().rtp_port);
		else
			LOG(2, "HT_Encoder: RTP fan-out -> " << options->Get().rtp_host << ":" << options->Get().rtp_port
												 << " mtu=" << options->Get().rtp_mtu
												 << " payload=" << rtp_packetizer_.chunk_size() << "B pace=" << frac);
		send_thread_ = std::thread(&HT_Encoder::sendThread, this);
		archive_thread_ = std::thread(&HT_Encoder::archiveThread, this);
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i] = std::thread(std::bind(&HT_Encoder::encodeThread, this, i));
		LOG(2, "Opened HT_Encoder");
	}
	~HT_Encoder()
	{
		abortEncode_ = true;
		encode_cond_var_.notify_all();
		for (int i = 0; i < NUM_ENC_THREADS; i++)
			encode_thread_[i].join();
		abortSend_ = true;
		send_cond_var_.notify_all();
		send_thread_.join();
		abortArchive_ = true;
		archive_cond_var_.notify_all();
		archive_thread_.join();
		LOG(2, "HT_Encoder closed");
	}
	// Encode the given buffer.  RTP fans out every frame; TCP archive only
	// fires when archive_this_frame is true (caller's rate-limit decision).
	//
	// `keepalive` is held until the encoder has finished reading `mem`. The
	// caller's camera buffer goes straight back to the sensor when its request
	// is released, so it has to outlive this asynchronous encode.
	void EncodeBuffer(int fd, size_t size, void *mem, StreamInfo const &info, int64_t timestamp_us,
					  bool archive_this_frame = false, int64_t capture_ns = 0,
					  std::shared_ptr<void> keepalive = nullptr)
	{
		std::lock_guard<std::mutex> lock(encode_mutex_);
		// Bounded queue: for a live monitor the freshest frame is worth more than
		// a backlog, so shed the oldest rather than growing latency without limit.
		while (encode_queue_.size() >= MAX_ENCODE_QUEUE)
		{
			encode_queue_.pop();
			++dropped_encode_;
		}
		encode_queue_.push(
			EncodeItem { mem, info, timestamp_us, index_++, archive_this_frame, capture_ns, std::move(keepalive) });
		encode_cond_var_.notify_all();
	}

private:
	// How many threads to use. Whichever thread is idle will pick up the next frame.
	static const int NUM_ENC_THREADS = 1;

	// Queue depths. Small on purpose -- these bound end-to-end latency, and
	// overflowing them is a signal that a stage cannot keep up.
	static constexpr size_t MAX_ENCODE_QUEUE = 2;
	static constexpr size_t MAX_SEND_QUEUE = 3;
	static constexpr size_t MAX_POOL = 6;

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
						return;
					}
					if (!encode_queue_.empty())
					{
						encode_item = std::move(encode_queue_.front());
						encode_queue_.pop();
						break;
					}
					else
						encode_cond_var_.wait_for(lock, 200ms);
				}
			}

			// Encode the buffer.
			size_t buffer_len = 0;
			auto start_time = std::chrono::high_resolution_clock::now();
			encodeHTJ2K(encode_item, buf, buffer_len);
			encode_time = (std::chrono::high_resolution_clock::now() - start_time);

			// Kakadu is done with the source pixels; let the camera have the buffer
			// back now rather than at the end of the iteration.
			encode_item.keepalive.reset();

			// RTP timestamp is the *sampling instant* (RFC 3550 5.1), i.e. the sensor
			// capture time -- not when this loop got round to the frame. Using loop
			// time would fold main-thread scheduling jitter into the RTP timeline and
			// show up as judder on any receiver that presents by timestamp.
			// ns -> us -> 90 kHz ticks, staged to keep the multiply inside int64.
			const int64_t ts_us = encode_item.capture_ns ? encode_item.capture_ns / 1000 : encode_item.timestamp_us;
			const uint32_t ts90 = static_cast<uint32_t>(ts_us * 9 / 100);

			if (rtp_packetizer_.is_open() && buffer_len)
				queueSend(buffer_len, ts90, static_cast<uint64_t>(encode_item.capture_ns), encode_item.index);

			if (encode_item.archive && buffer_len)
			{
				queueArchive(buffer_len);
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
				char line[220];
				snprintf(line, sizeof(line),
						 "%u fps, avg %.2f ms/frame, %.0f kbps, archived %u, dropped enc/send %u/%u", stats_frames,
						 stats_encode_total.count() * 1000.0 / stats_frames,
						 (stats_bytes_total * 8.0 / 1000.0) / window.count(), stats_archive_frames,
						 dropped_encode_.load(), dropped_send_.load());
				LOG(1, "HT_Encoder: " << line);
				stats_window_start = wall;
				stats_encode_total = std::chrono::duration<double> { 0 };
				stats_frames = 0;
				stats_bytes_total = 0;
				stats_archive_frames = 0;
			}
		}
	}

	// Hand the finished codestream to the paced sender. The encoder writes every
	// frame into the same output vector, so the sender needs its own copy; the
	// buffers are recycled through free_bufs_ and stop allocating after warm-up.
	void queueSend(size_t buffer_len, uint32_t ts90, uint64_t capture_ns, uint64_t index)
	{
		std::vector<uint8_t> payload = acquireBuf();
		payload.assign(buf.data(), buf.data() + buffer_len);

		std::lock_guard<std::mutex> lock(send_mutex_);
		while (send_queue_.size() >= MAX_SEND_QUEUE)
		{
			recycleLocked(std::move(send_queue_.front().data));
			send_queue_.pop();
			++dropped_send_;
		}
		send_queue_.push(SendItem { std::move(payload), ts90, capture_ns, index });
		send_cond_var_.notify_one();
	}

	void queueArchive(size_t buffer_len)
	{
		std::lock_guard<std::mutex> lock(archive_mutex_);
		// Depth 1: if the previous snapshot is still in flight, skip this one
		// rather than let a slow WAN write build a backlog.
		if (!archive_queue_.empty())
			return;
		archive_queue_.push(std::vector<uint8_t>(buf.data(), buf.data() + buffer_len));
		archive_cond_var_.notify_one();
	}

	// Paced RTP fan-out, off the encode thread.
	void sendThread()
	{
		while (true)
		{
			SendItem item;
			{
				std::unique_lock<std::mutex> lock(send_mutex_);
				while (true)
				{
					using namespace std::chrono_literals;
					if (abortSend_ && send_queue_.empty())
						return;
					if (!send_queue_.empty())
					{
						item = std::move(send_queue_.front());
						send_queue_.pop();
						break;
					}
					send_cond_var_.wait_for(lock, 200ms);
				}
			}

			if (!rtp_packetizer_.send_codestream(item.data.data(), item.data.size(), item.ts90, item.capture_ns))
				LOG(1, "HT_Encoder: RTP send_codestream failed for frame " << item.index);

			std::lock_guard<std::mutex> lock(send_mutex_);
			recycleLocked(std::move(item.data));
		}
	}

	// TCP archive snapshots, off the encode thread: connect() and send() both
	// block, and the destination is across the WAN.
	void archiveThread()
	{
		while (true)
		{
			std::vector<uint8_t> item;
			{
				std::unique_lock<std::mutex> lock(archive_mutex_);
				while (true)
				{
					using namespace std::chrono_literals;
					if (abortArchive_ && archive_queue_.empty())
						return;
					if (!archive_queue_.empty())
					{
						item = std::move(archive_queue_.front());
						archive_queue_.pop();
						break;
					}
					archive_cond_var_.wait_for(lock, 200ms);
				}
			}

			if (!tcp_connected_)
				tcp_connected_ = (tcp_socket_.create_client() >= 0);
			if (tcp_connected_)
			{
				if (!tcp_socket_.SendFramed(item.data(), static_cast<uint32_t>(item.size())))
				{
					LOG(1, "HT_Encoder: TCP send failed; reconnecting on next frame");
					tcp_socket_.destroy();
					tcp_connected_ = false;
				}
			}
		}
	}

	std::vector<uint8_t> acquireBuf()
	{
		std::lock_guard<std::mutex> lock(send_mutex_);
		if (free_bufs_.empty())
			return std::vector<uint8_t>();
		std::vector<uint8_t> b = std::move(free_bufs_.back());
		free_bufs_.pop_back();
		return b;
	}

	// Caller must hold send_mutex_.
	void recycleLocked(std::vector<uint8_t> &&b)
	{
		if (free_bufs_.size() < MAX_POOL)
			free_bufs_.push_back(std::move(b));
	}

	bool abortEncode_;
	bool abortSend_;
	bool abortArchive_;
	uint64_t index_;
	std::atomic<uint32_t> dropped_encode_ { 0 };
	std::atomic<uint32_t> dropped_send_ { 0 };

	struct EncodeItem
	{
		void *mem = nullptr;
		StreamInfo info;
		int64_t timestamp_us = 0;
		uint64_t index = 0;
		bool archive = false;
		int64_t capture_ns = 0; // absolute capture wall-clock time (UNIX epoch ns)
		std::shared_ptr<void> keepalive; // pins the source buffer until encode is done
	};
	std::queue<EncodeItem> encode_queue_;
	std::mutex encode_mutex_;
	std::condition_variable encode_cond_var_;
	std::thread encode_thread_[NUM_ENC_THREADS];
	void encodeHTJ2K(EncodeItem &item, std::vector<uint8_t> &out, size_t &buffer_len)
	{
		enc.setSourceImage((uint8_t *)item.mem, item.info.width * item.info.height * 3);
		enc.encode();
		buffer_len = out.size();
	}

	struct SendItem
	{
		std::vector<uint8_t> data;
		uint32_t ts90 = 0;
		uint64_t capture_ns = 0;
		uint64_t index = 0;
	};
	std::queue<SendItem> send_queue_;
	std::vector<std::vector<uint8_t>> free_bufs_;
	std::mutex send_mutex_;
	std::condition_variable send_cond_var_;
	std::thread send_thread_;

	std::queue<std::vector<uint8_t>> archive_queue_;
	std::mutex archive_mutex_;
	std::condition_variable archive_cond_var_;
	std::thread archive_thread_;

	HTJ2KEncoder enc; // encoder instance
	std::vector<uint8_t> &buf;
	simple_tcp tcp_socket_;
	bool tcp_connected_;
	RFC9828Packetizer rtp_packetizer_;
};
