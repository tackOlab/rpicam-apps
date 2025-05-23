/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect_draw_cv_stage.cpp - draw object detection results
 */

#include "opencv2/imgproc.hpp"

#include "core/rpicam_app.hpp"

#include "post_processing_stages/post_processing_stage.hpp"

#include "object_detect.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace cv;

using Rectange = libcamera::Rectangle;
using Stream = libcamera::Stream;

class ObjectDetectDrawCvStage : public PostProcessingStage
{
public:
	ObjectDetectDrawCvStage(RPiCamApp *app) : PostProcessingStage(app) {}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	Stream *stream_;
	int line_thickness_;
	double font_size_;
};

#define NAME "object_detect_draw_cv"

char const *ObjectDetectDrawCvStage::Name() const
{
	return NAME;
}

void ObjectDetectDrawCvStage::Configure()
{
	stream_ = app_->GetMainStream();
}

void ObjectDetectDrawCvStage::Read(boost::property_tree::ptree const &params)
{
	line_thickness_ = params.get<int>("line_thickness", 1);
	font_size_ = params.get<double>("font_size", 1.0);
}

bool ObjectDetectDrawCvStage::Process(CompletedRequestPtr &completed_request)
{
	if (!stream_)
		return false;

	BufferWriteSync w(app_, completed_request->buffers[stream_]);
	libcamera::Span<uint8_t> buffer = w.Get()[0];
	uint32_t *ptr = (uint32_t *)buffer.data();
	StreamInfo info = app_->GetStreamInfo(stream_);

	std::vector<Detection> detections;

	completed_request->post_process_metadata.Get("object_detect.results", detections);

	Mat image(info.height * 1.5, info.width, CV_8UC1, ptr, info.stride);
	Mat tmp;
	cvtColor(image, tmp, COLOR_YUV2BGR_I420);
	// printf("orig: %x, before = %x", ptr, image.data); // for DEBUGGING address of ptr
	
	// Scalar colour = Scalar(255, 255, 255);
	Scalar YELLOW = Scalar(0, 255, 255);
	Scalar BLUE = Scalar(255, 178, 50);
	int font = FONT_HERSHEY_SIMPLEX;

	for (auto &detection : detections)
	{
		Rect r(detection.box.x, detection.box.y, detection.box.width, detection.box.height);
		rectangle(tmp, r, BLUE, line_thickness_);
		std::stringstream text_stream;
		text_stream << detection.name << " " << (int)(detection.confidence * 100) << "%";
		std::string text = text_stream.str();
		int baseline = 0;
		Size size = getTextSize(text, font, font_size_, 2, &baseline);
		Point text_origin(detection.box.x + 5, detection.box.y + size.height + 5);
		putText(tmp, text, text_origin, font, font_size_, YELLOW, 2);
	}
	cvtColor(tmp, image, COLOR_BGR2YUV_I420);
	// printf("after = %x\n", image.data); // for DEBUGGING address of ptr
	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectDrawCvStage(app);
}

static RegisterStage reg(NAME, &Create);
