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

#include <arpa/inet.h>

#include <cstdint>
#include <cstring>
#include <string>
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

	// Conservative payload budget. Standard Ethernet MTU is 1500; subtracting
	// 20 (IPv4) + 8 (UDP) leaves 1472. Use 1300 to stay safe under VPN/tunnel
	// encapsulation and avoid IP fragmentation.
	static constexpr size_t TARGET_DATAGRAM_BYTES = 1300;
	static constexpr size_t RTP_HEADER_BYTES = 12;
	static constexpr size_t MAIN_HEADER_BYTES = 8;
	static constexpr size_t BODY_HEADER_BYTES = 8;

	// H.273 colorspace fields written into the Main packet when colorspace_set
	// is true (S=1 in the spec).  When colorspace_set is false (S=0), receivers
	// fall back to their CLI/UI configuration to interpret the codestream.
	struct Colorspace
	{
		bool colorspace_set = true;
		uint8_t prims = 1;   // H.273 ColourPrimaries: 1 = BT.709 / sRGB
		uint8_t trans = 13;  // H.273 TransferCharacteristics: 13 = sRGB
		uint8_t mat = 5;     // H.273 MatrixCoefficients: 5 = BT.470BG / BT.601 525
		bool full_range = true;
	};

	RFC9828Packetizer(const std::string &dst_host, int dst_port, Colorspace cs,
					  uint8_t ordh = ORDH_RPCL_RESYNC)
		: sock_(dst_host, dst_port), seq_(0), ordh_(ordh), cs_(cs)
	{
	}

	bool is_open() const { return sock_.is_open(); }

	// Packetize and send one HTJ2K codestream as 1+N RTP datagrams.
	// `timestamp90` is the per-frame 32-bit RTP timestamp (90 kHz); the same
	// value is written into every packet of this frame.
	bool send_codestream(const uint8_t *cs, size_t cs_len, uint32_t timestamp90)
	{
		if (!sock_.is_open() || cs == nullptr || cs_len == 0)
			return false;

		constexpr size_t MAIN_MAX = TARGET_DATAGRAM_BYTES - RTP_HEADER_BYTES - MAIN_HEADER_BYTES;
		constexpr size_t BODY_MAX = TARGET_DATAGRAM_BYTES - RTP_HEADER_BYTES - BODY_HEADER_BYTES;

		const bool single_main = (cs_len <= MAIN_MAX);
		const uint8_t mh = single_main ? MH_MAIN_SINGLE : MH_MAIN_THEN_BODY;
		const size_t main_chunk = single_main ? cs_len : MAIN_MAX;

		if (!send_main_packet(cs, main_chunk, mh, timestamp90, /*marker=*/single_main))
			return false;

		size_t sent = main_chunk;
		while (sent < cs_len)
		{
			const size_t remaining = cs_len - sent;
			const size_t body_chunk = (remaining > BODY_MAX) ? BODY_MAX : remaining;
			const bool last = (sent + body_chunk == cs_len);
			if (!send_body_packet(cs + sent, body_chunk, timestamp90, /*marker=*/last))
				return false;
			sent += body_chunk;
		}
		return true;
	}

private:
	bool send_main_packet(const uint8_t *cs_chunk, size_t chunk_len, uint8_t mh,
						  uint32_t ts, bool marker)
	{
		std::vector<uint8_t> dg;
		dg.reserve(RTP_HEADER_BYTES + MAIN_HEADER_BYTES + chunk_len);
		write_rtp_header(dg, marker, ts);

		// Byte 0: MH(2) | TP(3)=0 | ORDH(3)
		dg.push_back(static_cast<uint8_t>(((mh & 0x03) << 6) | (ordh_ & 0x07)));
		// Byte 1: P(1)=0 | XTRAC(3)=0 | PTSTAMP[11:8](4)=0
		dg.push_back(0x00);
		// Byte 2: PTSTAMP[7:0]=0
		dg.push_back(0x00);
		// Byte 3: ESEQ=0
		dg.push_back(0x00);
		// Byte 4: R(1)=0 | S(1) | C(1)=0 | RSVD(4)=0 | RANGE(1)
		// Bytes 5-7: PRIMS, TRANS, MAT (per H.273; meaningful only when S=1).
		// When S=0 the spec requires bytes 4..7 to be all zero, so we hand-build
		// either layout here.
		if (cs_.colorspace_set)
		{
			uint8_t b4 = static_cast<uint8_t>(0x40 /* S=1 */ | (cs_.full_range ? 0x01 : 0x00));
			dg.push_back(b4);
			dg.push_back(cs_.prims);
			dg.push_back(cs_.trans);
			dg.push_back(cs_.mat);
		}
		else
		{
			dg.push_back(0x00);
			dg.push_back(0x00);
			dg.push_back(0x00);
			dg.push_back(0x00);
		}

		dg.insert(dg.end(), cs_chunk, cs_chunk + chunk_len);
		return sock_.sendto_one(dg.data(), dg.size());
	}

	bool send_body_packet(const uint8_t *cs_chunk, size_t chunk_len, uint32_t ts,
						  bool marker)
	{
		std::vector<uint8_t> dg;
		dg.reserve(RTP_HEADER_BYTES + BODY_HEADER_BYTES + chunk_len);
		write_rtp_header(dg, marker, ts);

		// Byte 0: MH(2)=0 | TP(3)=0 | RES(3)=0
		dg.push_back(0x00);
		// Byte 1: ORDB(1)=0 | QUAL(3)=0 | PTSTAMP[11:8](4)=0
		dg.push_back(0x00);
		// Byte 2: PTSTAMP[7:0]=0
		dg.push_back(0x00);
		// Byte 3: ESEQ=0
		dg.push_back(0x00);
		// Bytes 4-7: POS=0 PID=0 (ORDB=0 requires both)
		dg.push_back(0x00);
		dg.push_back(0x00);
		dg.push_back(0x00);
		dg.push_back(0x00);

		dg.insert(dg.end(), cs_chunk, cs_chunk + chunk_len);
		return sock_.sendto_one(dg.data(), dg.size());
	}

	void write_rtp_header(std::vector<uint8_t> &dg, bool marker, uint32_t ts)
	{
		// Byte 0: V=2 P=0 X=0 CC=0
		dg.push_back(0x80);
		// Byte 1: M | PT
		dg.push_back(static_cast<uint8_t>((marker ? 0x80 : 0x00) | (RTP_PT & 0x7F)));
		// Bytes 2-3: sequence (BE)
		dg.push_back(static_cast<uint8_t>((seq_ >> 8) & 0xFF));
		dg.push_back(static_cast<uint8_t>(seq_ & 0xFF));
		// Bytes 4-7: timestamp (BE)
		dg.push_back(static_cast<uint8_t>((ts >> 24) & 0xFF));
		dg.push_back(static_cast<uint8_t>((ts >> 16) & 0xFF));
		dg.push_back(static_cast<uint8_t>((ts >> 8) & 0xFF));
		dg.push_back(static_cast<uint8_t>(ts & 0xFF));
		// Bytes 8-11: SSRC (BE)
		dg.push_back(static_cast<uint8_t>((RTP_SSRC >> 24) & 0xFF));
		dg.push_back(static_cast<uint8_t>((RTP_SSRC >> 16) & 0xFF));
		dg.push_back(static_cast<uint8_t>((RTP_SSRC >> 8) & 0xFF));
		dg.push_back(static_cast<uint8_t>(RTP_SSRC & 0xFF));
		++seq_;
	}

	simple_udp_client sock_;
	uint16_t seq_;
	uint8_t ordh_;
	Colorspace cs_;
};
