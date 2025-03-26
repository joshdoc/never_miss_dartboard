#include <iostream>
#include <memory>
#include <string>

#include "core/libcamera_app.hpp"
#include "output/file_output.hpp"
#include "encoder/h264_encoder.hpp"

int main(int argc, char *argv[]) {
    // Configuration parameters (customize as needed)
    const std::string output_filename = "output.h264";
    const unsigned int width = 1280;
    const unsigned int height = 720;
    const unsigned int framerate = 30;
    const unsigned int bitrate = 1000000; // 1 Mbps

    // Initialize the camera application
    LibcameraApp app;
    try {
        app.OpenCamera();
        app.ConfigureVideo(width, height, framerate);
    } catch (const std::exception &e) {
        std::cerr << "Camera configuration error: " << e.what() << std::endl;
        return 1;
    }

    Stream *video_stream = app.VideoStream();
    if (!video_stream) {
        std::cerr << "Failed to configure video stream" << std::endl;
        return 1;
    }

    // Setup H.264 encoder
    H264Encoder encoder;
    encoder.Configure(video_stream->configuration(), bitrate, H264Encoder::Profile::Main);

    // Setup file output
    FileOutput output(output_filename);
    encoder.SetOutputReadyCallback([&](void *data, size_t size, int64_t timestamp_us, bool keyframe) {
        output.OutputReady(data, size, timestamp_us, keyframe);
    });

    // Start capturing
    app.StartCamera();
    encoder.Start();

    // Event loop to process video frames
    while (true) {
        LibcameraApp::Msg msg = app.Wait();
        if (msg.type == LibcameraApp::MsgType::Quit)
            break;
        else if (msg.type == LibcameraApp::MsgType::RequestComplete) {
            auto &completed_request = std::get<CompletedRequestPtr>(msg.payload);
            encoder.EncodeBuffer(completed_request, video_stream);
        }
    }

    // Cleanup
    encoder.Stop();
    app.StopCamera();
    return 0;
}
