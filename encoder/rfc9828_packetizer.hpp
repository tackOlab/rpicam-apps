#pragma once

// RFC 9828 RTP packetizer for HTJ2K codestreams.
//
// Wire format mirrors osamu620/OpenHTJ2K's source/apps/rtp_recv/rfc9828_parser.cpp:
//   12-byte RTP fixed header (RFC 3550 §5.1) + 8-byte RFC 9828 Main or Body
//   payload header + codestream chunk, one per UDP datagram.
//
// One frame = one Main packet (MH=3 single, or MH=2 followed by N Body
// packets), marker bit on the last packet of the frame. PTSTAMP/ESEQ/POS/PID
// are zero in this minimal packetizer (we don't emit sub-codestream timing or
// resync points).
//
// A frame is built into a reusable arena and then sent in small batches with
// sendmmsg(), spaced across a pacing window rather than dumped at line rate.
// An unpaced 1080p frame is ~250-400 kB arriving in 2-3 ms, which is about the
// size of the default 208 kB UDP receive buffer -- a receiver that is busy
// decoding for even that long loses the rest of the frame in the kernel.

#include <arpa/inet.h>
#include <sys/socket.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "simple_udp.hpp"

class RFC9828Packetizer
{
public:
	// ORDH constants from rfc9828_parser.hpp.
	static constexpr uint8_t ORDH_LRCP_RESYNC = 1;
	static constexpr uint8_t ORDH_RLCP_RESYNC = 2;
	static constexpr uint8_t ORDH_RPCL_RESYNC = 3;
	static constexpr uint8_t ORDH_PCRL_RESYNC = 4;
	static constexpr uint8_t ORDH_CPRL_RESYNC = 5;
	static constexpr uint8_t ORDH_PRCL_RESYNC = 6;

	// MH constants.
	static constexpr uint8_t MH_BODY = 0;
	static constexpr uint8_t MH_MAIN_MORE_MAIN = 1;
	static constexpr uint8_t MH_MAIN_THEN_BODY = 2;
	static constexpr uint8_t MH_MAIN_SINGLE = 3;

	// RTP-level constants. PT 96 is dynamic; SSRC is fixed per source.
	static constexpr uint8_t RTP_VERSION = 2;
	static constexpr uint8_t RTP_PT = 96;
	static constexpr uint32_t RTP_SSRC = 0xCAFEBABEu;

	// RFC 8285 one-byte header extension carrying the absolute capture wall-clock
	// time (UNIX epoch ns, big-endian).  ID=1, len=8 bytes.
	static constexpr uint8_t CAPTURE_TS_ID = 1;
	static constexpr size_t RTP_EXT_BYTES = 16; // 4-byte ext header + 8-byte ns + 3 pad + 1 element hdr -> 16 total

	static constexpr size_t RTP_HEADER_BYTES = 12;
	static constexpr size_t MAIN_HEADER_BYTES = 8;
	static constexpr size_t BODY_HEADER_BYTES = 8;

	// IPv4 + UDP headers that sit in front of every datagram on the wire.
	static constexpr size_t IP_UDP_OVERHEAD = 28;

	// Datagrams per sendmmsg() batch. 8 x ~1.2 kB is ~10 kB per burst, small
	// enough for any default-sized receive buffer to absorb between wakeups.
	static constexpr unsigned int BATCH_PKTS = 8;

	// H.273 colorspace fields written into the Main packet when colorspace_set
	// is true (S=1 in the spec).  When colorspace_set is false (S=0), receivers
	// fall back to their CLI/UI configuration to interpret the codestream.
	struct Colorspace
	{
		bool colorspace_set = true;
		uint8_t prims = 1; // H.273 ColourPrimaries: 1 = BT.709 / sRGB
		uint8_t trans = 13; // H.273 TransferCharacteristics: 13 = sRGB
		uint8_t mat = 5; // H.273 MatrixCoefficients: 5 = BT.470BG / BT.601 525
		bool full_range = true;
	};

	RFC9828Packetizer(const std::string &dst_host, int dst_port, Colorspace cs, int path_mtu = 1280,
					  uint8_t ordh = ORDH_RPCL_RESYNC)
		: sock_(dst_host, dst_port), seq_(0), ordh_(ordh), cs_(cs)
	{
		// Budget every byte the datagram actually carries.  Main and Body payload
		// headers are both 8 bytes, so a single chunk size covers both cases.
		const size_t fixed = RTP_HEADER_BYTES + RTP_EXT_BYTES + MAIN_HEADER_BYTES;
		const int64_t budget = static_cast<int64_t>(path_mtu) - static_cast<int64_t>(IP_UDP_OVERHEAD + fixed);
		chunk_max_ = static_cast<size_t>(std::max<int64_t>(budget, 64));
		datagram_max_ = fixed + chunk_max_;
	}

	bool is_open() const
	{
		return sock_.is_open();
	}

	// Bytes of codestream carried per datagram, after all headers.
	size_t chunk_size() const
	{
		return chunk_max_;
	}

	// Window (ns) across which one frame's packets are spread. 0 = no pacing.
	void set_pacing_window(int64_t ns)
	{
		pace_window_ns_ = ns;
	}

	// Packetize and send one HTJ2K codestream as 1+N RTP datagrams.
	// `timestamp90` is the per-frame 32-bit RTP timestamp (90 kHz); the same
	// value is written into every packet of this frame.
	bool send_codestream(const uint8_t *cs, size_t cs_len, uint32_t timestamp90, uint64_t capture_ns)
	{
		if (!sock_.is_open() || cs == nullptr || cs_len == 0)
			return false;

		capture_ns_ = capture_ns; // written into the header extension of every packet of this frame
		build_frame(cs, cs_len, timestamp90);
		return send_paced();
	}

private:
	// Lay the whole frame out in the arena, one datagram per fixed-size slot, and
	// point the iovec/mmsghdr arrays at it. Everything here is reused frame to
	// frame, so the steady state does no allocation.
	void build_frame(const uint8_t *cs, size_t cs_len, uint32_t ts)
	{
		const size_t n = (cs_len + chunk_max_ - 1) / chunk_max_;
		if (arena_.size() < n * datagram_max_)
			arena_.resize(n * datagram_max_);
		iov_.resize(n);
		msgs_.resize(n);

		size_t off = 0;
		for (size_t i = 0; i < n; ++i)
		{
			const size_t chunk = std::min(chunk_max_, cs_len - off);
			const bool last = (off + chunk == cs_len);
			uint8_t *p = arena_.data() + i * datagram_max_;

			size_t len = write_rtp_header(p, last, ts);
			if (i == 0)
				len += write_main_header(p + len, (n == 1) ? MH_MAIN_SINGLE : MH_MAIN_THEN_BODY);
			else
				len += write_body_header(p + len);
			memcpy(p + len, cs + off, chunk);
			len += chunk;
			off += chunk;

			iov_[i].iov_base = p;
			iov_[i].iov_len = len;
			memset(&msgs_[i], 0, sizeof(msgs_[i]));
			msgs_[i].msg_hdr.msg_name = sock_.dst();
			msgs_[i].msg_hdr.msg_namelen = sock_.dst_len();
			msgs_[i].msg_hdr.msg_iov = &iov_[i];
			msgs_[i].msg_hdr.msg_iovlen = 1;
		}
	}

	// Send the built frame in batches, holding each batch to a deadline derived
	// from the pacing window. Deadlines are absolute so the spacing doesn't drift.
	bool send_paced()
	{
		const size_t n = msgs_.size();
		if (n == 0)
			return true;

		const auto t0 = std::chrono::steady_clock::now();
		size_t sent = 0;
		while (sent < n)
		{
			const unsigned int batch = static_cast<unsigned int>(std::min<size_t>(BATCH_PKTS, n - sent));
			const int rc = sock_.send_group(&msgs_[sent], batch);
			if (rc <= 0)
				return false;
			sent += static_cast<size_t>(rc);
			if (pace_window_ns_ > 0 && sent < n)
			{
				const int64_t due = static_cast<int64_t>(pace_window_ns_ * sent / n);
				std::this_thread::sleep_until(t0 + std::chrono::nanoseconds(due));
			}
		}
		return true;
	}

	size_t write_rtp_header(uint8_t *p, bool marker, uint32_t ts)
	{
		size_t i = 0;
		// Byte 0: V=2 P=0 X=1 CC=0  (X=1: a header extension follows)
		p[i++] = 0x90;
		// Byte 1: M | PT
		p[i++] = static_cast<uint8_t>((marker ? 0x80 : 0x00) | (RTP_PT & 0x7F));
		// Bytes 2-3: sequence (BE)
		p[i++] = static_cast<uint8_t>((seq_ >> 8) & 0xFF);
		p[i++] = static_cast<uint8_t>(seq_ & 0xFF);
		// Bytes 4-7: timestamp (BE)
		p[i++] = static_cast<uint8_t>((ts >> 24) & 0xFF);
		p[i++] = static_cast<uint8_t>((ts >> 16) & 0xFF);
		p[i++] = static_cast<uint8_t>((ts >> 8) & 0xFF);
		p[i++] = static_cast<uint8_t>(ts & 0xFF);
		// Bytes 8-11: SSRC (BE)
		p[i++] = static_cast<uint8_t>((RTP_SSRC >> 24) & 0xFF);
		p[i++] = static_cast<uint8_t>((RTP_SSRC >> 16) & 0xFF);
		p[i++] = static_cast<uint8_t>((RTP_SSRC >> 8) & 0xFF);
		p[i++] = static_cast<uint8_t>(RTP_SSRC & 0xFF);
		++seq_;

		// RFC 8285 one-byte header extension: absolute capture time (ns, BE).
		// profile 0xBEDE, length 3 words (12 bytes of extension data).
		p[i++] = 0xBE;
		p[i++] = 0xDE;
		p[i++] = 0x00;
		p[i++] = 0x03;
		// element header: (ID<<4) | (len-1); ID=1, len=8 -> 0x17
		p[i++] = static_cast<uint8_t>((CAPTURE_TS_ID << 4) | 0x07);
		// 8 bytes: capture_ns big-endian
		for (int s = 56; s >= 0; s -= 8)
			p[i++] = static_cast<uint8_t>((capture_ns_ >> s) & 0xFF);
		// 3 bytes zero padding (9 -> 12 bytes of extension data)
		p[i++] = 0x00;
		p[i++] = 0x00;
		p[i++] = 0x00;
		return i;
	}

	size_t write_main_header(uint8_t *p, uint8_t mh)
	{
		size_t i = 0;
		// Byte 0: MH(2) | TP(3)=0 | ORDH(3)
		p[i++] = static_cast<uint8_t>(((mh & 0x03) << 6) | (ordh_ & 0x07));
		// Byte 1: P(1)=0 | XTRAC(3)=0 | PTSTAMP[11:8](4)=0
		p[i++] = 0x00;
		// Byte 2: PTSTAMP[7:0]=0
		p[i++] = 0x00;
		// Byte 3: ESEQ=0
		p[i++] = 0x00;
		// Byte 4: R(1)=0 | S(1) | C(1)=0 | RSVD(4)=0 | RANGE(1)
		// Bytes 5-7: PRIMS, TRANS, MAT (per H.273; meaningful only when S=1).
		// When S=0 the spec requires bytes 4..7 to be all zero, so we hand-build
		// either layout here.
		if (cs_.colorspace_set)
		{
			p[i++] = static_cast<uint8_t>(0x40 /* S=1 */ | (cs_.full_range ? 0x01 : 0x00));
			p[i++] = cs_.prims;
			p[i++] = cs_.trans;
			p[i++] = cs_.mat;
		}
		else
		{
			p[i++] = 0x00;
			p[i++] = 0x00;
			p[i++] = 0x00;
			p[i++] = 0x00;
		}
		return i;
	}

	size_t write_body_header(uint8_t *p)
	{
		// Byte 0: MH(2)=0 | TP(3)=0 | RES(3)=0
		// Byte 1: ORDB(1)=0 | QUAL(3)=0 | PTSTAMP[11:8](4)=0
		// Byte 2: PTSTAMP[7:0]=0
		// Byte 3: ESEQ=0
		// Bytes 4-7: POS=0 PID=0 (ORDB=0 requires both)
		memset(p, 0, BODY_HEADER_BYTES);
		return BODY_HEADER_BYTES;
	}

	simple_udp_client sock_;
	uint16_t seq_;
	uint8_t ordh_;
	Colorspace cs_;
	uint64_t capture_ns_ = 0;

	size_t chunk_max_ = 0;
	size_t datagram_max_ = 0;
	int64_t pace_window_ns_ = 0;

	std::vector<uint8_t> arena_;
	std::vector<iovec> iov_;
	std::vector<mmsghdr> msgs_;
};
