# tackOlab fork — status and overview

This document describes how [`tackOlab/rpicam-apps`](https://github.com/tackOlab/rpicam-apps) differs from upstream [`raspberrypi/rpicam-apps`](https://github.com/raspberrypi/rpicam-apps). The upstream `README.md` is kept verbatim so that upstream syncs do not collide with fork docs.

## What the fork does

`rpicam-hello` is rewired into a HAILO-driven, HT-J2K-encoding streamer. All other binaries (`rpicam-vid`, `rpicam-still`, `rpicam-jpeg`, `rpicam-raw`, `rpicam-detect`) are upstream-clean.

Per frame, `rpicam-hello` does:

1. Runs the 1920x1080 viewfinder and feeds a 640x640 lores stream to YOLOv8 on a HAILO accelerator via the `hailo_yolo_inference` post-processing stage.
2. HT-J2K-encodes the YUV420 viewfinder buffer with Kakadu (`HTJ2KEncoder`, multi-threaded, RPCL, `Cmodes=HT`, 4:2:0).
3. **Always-on:** packetizes the codestream per RFC 9828 (HTJ2K-over-RTP) and fans it out over UDP to `--rtp-host` / `--rtp-port`.
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
