/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>
#include <time.h>

#include <libcamera/control_ids.h>

#include "core/options.hpp"
#include "core/rpicam_app.hpp"
#include "post_processing_stages/object_detect.hpp"
#include "subprojects/kakadujs/src/HTJ2KEncoder.hpp"

#include "opencv2/core.hpp"

#include "encoder/ht_encoder.hpp"

using namespace std::placeholders;

enum progression
{
	LRCP,
	RLCP,
	RPCL,
	PCRL,
	CPRL
};

// The main event loop for the application.

// Fixed offset (ns) to convert the libcamera SensorTimestamp (CLOCK_BOOTTIME)
// into CLOCK_REALTIME (UNIX epoch).  Sampled once at startup; both clocks tick
// at the same rate, so a single offset is sufficient for g2g timestamping.
static const int64_t kClkOffNs = [] {
	struct timespec rt, bt;
	clock_gettime(CLOCK_REALTIME, &rt);
	clock_gettime(CLOCK_BOOTTIME, &bt);
	return (int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec - ((int64_t)bt.tv_sec * 1000000000LL + bt.tv_nsec);
}();

static void event_loop(RPiCamApp &app)
{
	Options const *options = app.GetOptions();

	app.OpenCamera();
	app.ConfigureViewfinder();

	// Setup HTJ2K encoder
	std::vector<uint8_t> encbuf; // codestream buffer
	encbuf.reserve(options->Get().viewfinder_width * options->Get().viewfinder_height * 3);
	const FrameInfo finfo = { static_cast<uint16_t>(options->Get().viewfinder_width),
							  static_cast<uint16_t>(options->Get().viewfinder_height), 8, 3, false };
	// HTJ2KEncoder encoder(encbuf, finfo); // encoder instance
	HT_Encoder htenc(encbuf, finfo, options);

	app.StartCamera();

	const int64_t archive_interval_us = static_cast<int64_t>(options->Get().archive_min_interval_ms) * 1000;
	// Sentinel: a value far enough below any plausible now_us that the first frame
	// passes the gate without signed-overflow on (now_us - sentinel).
	int64_t last_archive_us = 0;

	auto start_time = std::chrono::high_resolution_clock::now();
	for (unsigned int count = 0;; count++)
	{
		RPiCamApp::Msg msg = app.Wait();
		if (msg.type == RPiCamApp::MsgType::Timeout)
		{
			LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type == RPiCamApp::MsgType::Quit)
			return;
		else if (msg.type != RPiCamApp::MsgType::RequestComplete)
			throw std::runtime_error("unrecognised message!");

		LOG(2, "Viewfinder frame " << count);
		auto now = std::chrono::high_resolution_clock::now();
		if (options->Get().timeout && (now - start_time) > options->Get().timeout.value)
			return;

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		app.ShowPreview(completed_request, app.ViewfinderStream());

		std::vector<Detection> objects;
		completed_request->post_process_metadata.Get("object_detect.results", objects);
		bool person_detected = false;
		for (auto const &v : objects)
		{
			if (v.name == "person" && v.confidence > 0.75)
			{
				person_detected = true;
				break;
			}
		}

		const int64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
		const bool archive_this_frame = person_detected && (now_us - last_archive_us >= archive_interval_us);
		if (archive_this_frame)
			last_archive_us = now_us;

		// Absolute capture wall-clock time (UNIX epoch ns) for g2g latency.
		// SensorTimestamp is the capture instant on CLOCK_BOOTTIME; shift it to
		// CLOCK_REALTIME.  Fall back to "now" if the sensor doesn't report it.
		int64_t sensor_ns = 0;
		if (auto t = completed_request->metadata.get(libcamera::controls::SensorTimestamp))
			sensor_ns = *t; // CLOCK_BOOTTIME ns
		int64_t capture_realtime_ns;
		if (sensor_ns != 0)
			capture_realtime_ns = sensor_ns + kClkOffNs; // UNIX epoch ns
		else
		{
			struct timespec rt;
			clock_gettime(CLOCK_REALTIME, &rt);
			capture_realtime_ns = (int64_t)rt.tv_sec * 1000000000LL + rt.tv_nsec;
			LOG(1, "rpicam-hello: SensorTimestamp unavailable; using CLOCK_REALTIME at loop time for capture_ns");
		}

		libcamera::Stream *stream = app.ViewfinderStream();
		BufferReadSync r(&app, completed_request->buffers[stream]);
		libcamera::Span<uint8_t> buffer = r.Get()[0];
		const int w = options->Get().viewfinder_width;
		const int h = options->Get().viewfinder_height;
		cv::Mat frame(h * 3 / 2, w, CV_8UC1, buffer.data());
		htenc.EncodeBuffer(1, w * h * 3 / 2, frame.data, app.GetStreamInfo(stream), now_us, archive_this_frame,
						   capture_realtime_ns);
	}
}

int main(int argc, char *argv[])
{
	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->Get().verbose >= 2)
				options->Get().Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}
