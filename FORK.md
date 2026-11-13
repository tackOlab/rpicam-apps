# tackOlab fork — status and overview

This document describes how [`tackOlab/rpicam-apps`](https://github.com/tackOlab/rpicam-apps) differs from upstream [`raspberrypi/rpicam-apps`](https://github.com/raspberrypi/rpicam-apps). The upstream `README.md` is kept verbatim so that upstream syncs do not collide with fork docs.

## What the fork does

`rpicam-hello` is rewired into a HAILO-driven, HT-J2K-encoding streamer. All other binaries (`rpicam-vid`, `rpicam-still`, `rpicam-jpeg`, `rpicam-raw`, `rpicam-detect`) are upstream-clean.

Per frame, `rpicam-hello` does:

1. Runs the 1920x1080 viewfinder and feeds a 640x640 lores stream to YOLOv8 on a HAILO accelerator via the `hailo_yolo_inference` post-processing stage.
2. HT-J2K-encodes the YUV420 viewfinder buffer with Kakadu (`HTJ2KEncoder`, multi-threaded, RPCL, `Cmodes=HT`, 4:2:0).
3. **Always-on:** packetizes the codestream per RFC 9828 (HTJ2K-over-RTP) and fans it out over UDP to `--rtp-host` / `--rtp-port`. Every RTP packet carries an absolute capture timestamp in a header extension (see *RTP capture-time extension* below).
4. **Gated by detection:** when a `person` is detected with confidence > 0.75, the same codestream is also archived over a persistent length-prefixed TCP connection to a separate receiver.

## Current status

As of 2026-05-12:

- **Upstream sync:** post-`v1.12.0`. The v1.11.1 sync landed in PR #7; the trailing imx296 test-bypass commit (`4c79698`) is pending in PR #17.
- **Working:**
  - HAILO YOLOv8 inference loaded at runtime as `hailo-postproc.so`.
  - Per-frame HT-J2K encode + RFC 9828 RTP fan-out (`encoder/ht_encoder.hpp`, `encoder/rfc9828_packetizer.hpp`, `encoder/simple_udp.hpp`).
  - Person-gated TCP archive over a persistent connection with length-prefix framing (`encoder/simple_tcp.hpp`).
  - `libhailort` preload before factory dlopens, to keep `.so` load order correct (PR #16).
- **Hardcoded in source** (treat as config when changing behavior):
  - Person-class filter and 0.75 confidence threshold — `apps/rpicam_hello.cpp`.
  - Archive minimum interval default — `apps/rpicam_hello.cpp` / `encoder/ht_encoder.hpp`.
  - TCP destination — `encoder/simple_tcp.hpp`.
  - RTP host/port are CLI options (`--rtp-host`, `--rtp-port`).

## RTP capture-time extension (glass-to-glass latency)

Every RFC 9828 RTP datagram emitted by `rpicam-hello` carries the frame's
**absolute capture wall-clock time** so a receiver can compute capture-to-display
latency. The 90 kHz RTP timestamp (`ts90`) is unchanged — it still drives the
receiver's pacing — and the capture time rides alongside it in an RFC 8285
one-byte header extension:

- RTP fixed header byte 0 is `0x90` (X=1: a header extension follows).
- Immediately after the 12-byte fixed header (no CSRCs):
  - profile `0xBE 0xDE`, length `0x00 0x03` (3 words = 12 bytes of ext data)
  - element header `0x17` = `(ID<<4)|(len-1)` with **ID = 1**, len = 8
  - 8 bytes: **capture time in nanoseconds since the UNIX epoch, big-endian**
  - 3 bytes zero padding (pads 9 → 12)
- The same extension is in **every** packet of a frame (Main and Body); per-datagram
  growth is +16 B (max datagram 1316 B, still < 1472 MTU).

Source of the timestamp: `apps/rpicam_hello.cpp` reads libcamera
`SensorTimestamp` (CLOCK_BOOTTIME) and shifts it to CLOCK_REALTIME with a
once-sampled offset (`kClkOffNs`); it falls back to CLOCK_REALTIME at loop time
(with a warning) if the sensor reports no timestamp. The value threads through
`HT_Encoder::EncodeBuffer` → `RFC9828Packetizer::send_codestream` →
`write_rtp_header`. Implemented in `apps/rpicam_hello.cpp`,
`encoder/ht_encoder.hpp`, `encoder/rfc9828_packetizer.hpp` (constant
`CAPTURE_TS_ID = 1`). Receivers must skip the X=1 extension before reading the
RFC 9828 payload header.

**PTP lock is mandatory for latency measurement.** The capture-to-display
figure is only the *difference* between the sender's CLOCK_REALTIME and the
receiver's wall clock, so any offset between the two hosts shows up directly as
latency error. The sender (Pi) and receiver **must** be PTP-locked to a common
master (sub-ms) before the numbers mean anything. NTP/chrony (~1 ms or worse,
and prone to step corrections) is **not** sufficient for glass-to-glass
measurement — without PTP lock the reported latency is unusable.

## Build

The fork builds only with the exact feature flags below — `enable_hailo` and `enable_opencv` are load-bearing because `rpicam-hello` is wired to use both:

```sh
meson setup --wipe build --buildtype release \
    -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled \
    -Denable_qt=disabled -Denable_opencv=enabled -Denable_tflite=disabled \
    -Denable_hailo=enabled
meson compile -C build
./afterbuild.sh
```

`afterbuild.sh` symlinks the post-processing `.so` modules, preview/encoder modules, and asset JSONs into `build/apps/`. It must be re-run after every clean configure — without it `rpicam-hello` cannot find `hailo-postproc.so` / `core-postproc.so` or their assets.

Run from the build dir:

```sh
cd build/apps && ./rpicam-hello -c ../../run-config.txt
```

There is no test suite; verification is done by running the binary against a live camera.

### Compatibility

- Builds clean against **libcamera 0.8.0** (`v0.8.0+rpt20261104`), verified 2026-11-13 — full reconfigure + compile of all targets, `rpicam-hello` links `libcamera.so.0.8`. This needed source changes: upstream's libcamera 0.8 fixes (`2d650a9`, `8b7d823`, `d37ffe5`, from v1.13.0–v1.15.0) are cherry-picked on top of the post-`v1.12.0` sync, plus `libcamera::Span` → `std::span` in `apps/rpicam_hello.cpp`. The rest of upstream v1.13.0–v1.15.0 is not synced.
- libcamera 0.7.x is no longer expected to build: the code now uses the libcamera 0.8 control and property names.

## Prerequisites

System packages must be present at meson configure time:

- HailoRT >= 4.18.0
- hailo-tappas-core >= 3.31.0
- opencv4

If any of these is missing, meson silently skips `hailo-postproc.so` and the runtime pipeline will fail to find `hailo_yolo_inference`.

The Kakadu SDK is integrated as a meson `cmake.subproject('kakadujs')`. The licensed Kakadu source tree under `subprojects/kakadujs/extern/` is user-supplied and **not** redistributed with the fork. The Kakadu license is separate from the fork's BSD-2-Clause license and from upstream `rpicam-apps`.

## Pointers

- Architecture and integration details (where Kakadu/HAILO touch upstream code, what must not be modified): [`CLAUDE.md`](CLAUDE.md).
- Runtime options consumed by `rpicam-hello -c ...`: [`run-config.txt`](run-config.txt).
- Post-processing pipeline JSON used in practice: [`assets/hailo_yolov8_inference.json`](assets/hailo_yolov8_inference.json).
- Fork-vs-upstream diff: `git diff upstream/main..HEAD`.
