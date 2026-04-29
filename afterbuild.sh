#!/bin/bash
set -euo pipefail
shopt -s nullglob
cd build/apps
ln -sf ../post_processing_stages/hailo/hailo-postproc.so ./
ln -sf ../post_processing_stages/core-postproc.so ./
ln -sf ../post_processing_stages/opencv-postproc.so ./
# Preview / encoder backends are dynamically loaded since upstream v1.8.0;
# nullglob silently drops backends that weren't built.
for so in ../preview/*-preview.so ../encoder/*-encoder.so; do
    ln -sf "$so" ./
done
ln -sf /usr/share/rpi-camera-assets/*.json .
cd ../../
