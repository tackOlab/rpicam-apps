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

- **Upstream sync:** post-`v1.15.0` (`7b69b7c`), synced 2026-11-13. Earlier syncs: v1.11.1 in PR #7, post-`v1.12.0` in PRs #17 and #19.
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

## Building

The fork targets one platform: a **Raspberry Pi 5** running 64-bit **Raspberry Pi OS (Debian 13 "trixie")**, with a Raspberry Pi camera module and a **Hailo-8 / Hailo-8L** accelerator (AI Kit or AI HAT+). The steps below go from a fresh OS install to a running `rpicam-hello`.

### 1. Install system packages

Build tools and the rpicam-apps / OpenCV dependencies:

```sh
sudo apt update
sudo apt install -y git build-essential pkg-config meson ninja-build cmake \
    libcamera-dev libboost-program-options-dev libexif-dev libjpeg-dev \
    libtiff-dev libpng-dev libdrm-dev libepoxy-dev libx11-dev libopencv-dev
```

The Hailo stack, followed by a reboot so the PCIe driver loads:

```sh
sudo apt install -y hailo-all
sudo reboot
```

`hailo-all` installs HailoRT, hailo-tappas-core, the Hailo PCIe driver and `rpicam-apps-hailo-postprocess`. The latter also brings in the YOLOv8 models (`hailo-models`, in `/usr/share/hailo-models/`) and the pipeline JSONs in `/usr/share/rpi-camera-assets/` that `run-config.txt` uses.

- The build needs HailoRT >= 4.18.0, hailo-tappas-core >= 3.31.0 and OpenCV 4. Tested with HailoRT 4.23.0, hailo-tappas-core 5.2.0, OpenCV 4.10.0 and libcamera 0.8.0.
- The Hailo-10H stack (`hailo-h10-all`, which conflicts with `hailo-all`) has not been tested.
- Optional: `wayland-protocols` (with `libwayland-dev`) enables upstream's native Wayland preview window. Without it meson skips that backend; the DRM/EGL previews are unaffected.

### 2. Clone the repository

```sh
git clone https://github.com/tackOlab/rpicam-apps.git
cd rpicam-apps
```

### 3. Add the Kakadu SDK

Kakadu is commercial software and is **not** included in this repository: you need your own licensed copy. The Kakadu license is separate from the fork's BSD-2-Clause license and from upstream `rpicam-apps`. The fork is developed against Kakadu **v8.4.1**.

Unpack the Kakadu distribution, as shipped, into `subprojects/kakadujs/extern/`, in a directory whose name starts with `v`:

```
subprojects/kakadujs/extern/v8_4_1-XXXXXX/   <- your Kakadu distribution (contains Enabling_HT.txt)
subprojects/kakadujs/extern/kakadu/          <- tracked kakadujs CMake glue; leave as is
```

- **The `v` prefix keeps the licensed source out of git.** `subprojects/kakadujs/.gitignore` ignores `extern/v*`; a directory with any other name shows up as untracked in `git status` and could be committed to this public repository.
- The kakadujs CMake locates the SDK by looking for `Enabling_HT.txt` in the directories under `extern/`, so keep exactly one Kakadu version there.
- Do not replace `srclib_ht` with `altlib_ht_opt` or set `FC_BLOCK_ENABLED` as Kakadu's `Enabling_HT.txt` describes. The kakadujs CMake already builds with the processor's SIMD optimisations (see `subprojects/kakadujs/README.md`).

There is no separate Kakadu build step: meson drives kakadujs as a CMake subproject (`cmake.subproject('kakadujs')` in `meson.build`) and compiles Kakadu along with everything else.

### 4. Configure and build

The fork builds only with the exact feature flags below — `enable_hailo` and `enable_opencv` are load-bearing because `rpicam-hello` is wired to use both:

```sh
meson setup build --buildtype release \
    -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled \
    -Denable_qt=disabled -Denable_opencv=enabled -Denable_tflite=disabled \
    -Denable_hailo=enabled
meson compile -C build
./afterbuild.sh
```

The first `meson compile` takes about 3 minutes on a Pi 5. Check that `build/post_processing_stages/hailo/hailo-postproc.so` exists. If HailoRT, hailo-tappas-core or OpenCV was missing at configure time, meson **silently** skips it and `rpicam-hello` later fails to find `hailo_yolo_inference`. Install the missing packages and configure again (see *Rebuilding*).

`afterbuild.sh` symlinks the post-processing `.so` modules, preview/encoder modules, and the asset JSONs from `/usr/share/rpi-camera-assets/` into `build/apps/`. It must be re-run after every clean configure — without it `rpicam-hello` cannot find `hailo-postproc.so` / `core-postproc.so` or their assets.

### 5. Run

Set `rtp-host` / `rtp-port` in `run-config.txt` to your receiver. (The TCP archive destination is hardcoded in `encoder/simple_tcp.hpp`; see *Current status*.) Then run from the build dir:

```sh
cd build/apps && ./rpicam-hello -c ../../run-config.txt
```

`run-config.txt` loads `./hailo_yolov8_inference.json`, which `afterbuild.sh` linked into `build/apps/`; it names the HEF models under `/usr/share/hailo-models/`. `lores-width` / `lores-height` in `run-config.txt` must match the model's input size (640x640 for these YOLOv8 models), or `hailo_yolo_inference` aborts.

There is no test suite; verification is done by running the binary against a live camera.

### Rebuilding

- After pulling changes: `meson compile -C build && ./afterbuild.sh`. meson reconfigures itself when build files change.
- To configure from scratch, run the `meson setup` command above with `--wipe` added, then `meson compile -C build` and `./afterbuild.sh`. **`--wipe` deletes everything in `build/`**, including files you placed in `build/apps/` yourself (for example a custom pipeline JSON), so move those elsewhere first.

### Compatibility

- Builds clean against **libcamera 0.8.0** (`v0.8.0+rpt20261104`), verified 2026-11-13 on the post-`v1.15.0` sync — full reconfigure + compile of all targets, `rpicam-hello` links `libcamera.so.0.8`. This needed source changes: upstream's libcamera 0.8 fixes (`2d650a9`, `8b7d823`, `d37ffe5`), which arrived in the fork as cherry-picks in PR #22 ahead of the full sync, plus `libcamera::Span` → `std::span` in the fork's `apps/rpicam_hello.cpp`.
- libcamera 0.7.x is no longer expected to build: the code now uses the libcamera 0.8 control and property names.

## Pointers

- Architecture and integration details (where Kakadu/HAILO touch upstream code, what must not be modified): [`CLAUDE.md`](CLAUDE.md).
- Runtime options consumed by `rpicam-hello -c ...`: [`run-config.txt`](run-config.txt).
- Post-processing pipeline JSON used in practice: [`assets/hailo_yolov8_inference.json`](assets/hailo_yolov8_inference.json).
- Fork-vs-upstream diff: `git diff upstream/main..HEAD`.
