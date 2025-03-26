#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>

// Libcamera headers (the include path is set so that they can be found under libcamera/)
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/camera_configuration.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/formats.h>

// OpenCV headers
#include <opencv2/opencv.hpp>

using namespace libcamera;
using namespace std;
using namespace cv;

int main()
{
    // Create and start the CameraManager.
    CameraManager cm;
    cm.start();

    if (cm.cameras().empty()) {
        cerr << "No cameras available" << endl;
        return -1;
    }

    // Open the first available camera.
    std::shared_ptr<Camera> camera = cm.cameras()[0];
    if (!camera) {
        cerr << "Unable to access camera" << endl;
        return -1;
    }
    if (camera->acquire()) {
        cerr << "Failed to acquire camera" << endl;
        return -1;
    }

    // Generate a configuration for preview (viewfinder) mode.
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({ StreamRole::Viewfinder });
    if (!config) {
        cerr << "Failed to generate camera configuration" << endl;
        return -1;
    }

    // Configure the first stream:
    // Set a desired resolution and pixel format.
    // Note: libcamera typically produces YUV formats.
    config->at(0).pixelFormat = formats::YUV420;
    config->at(0).size = { 640, 480 };
    config->at(0).bufferCount = 4;

    // Validate the configuration.
    CameraConfiguration::Status validation = config->validate();
    if (validation == CameraConfiguration::Invalid) {
        cerr << "Invalid camera configuration" << endl;
        return -1;
    }

    // Apply the configuration.
    if (camera->configure(config.get()) < 0) {
        cerr << "Failed to configure camera" << endl;
        return -1;
    }

    // Create a framebuffer allocator for the camera.
    FrameBufferAllocator allocator(camera);
    for (const StreamConfiguration &cfg : *config) {
        if (allocator.allocate(cfg.stream()) < 0) {
            cerr << "Failed to allocate buffers" << endl;
            return -1;
        }
    }

    // Create capture requests and attach allocated buffers.
    std::vector<std::unique_ptr<Request>> requests;
    for (const StreamConfiguration &cfg : *config) {
        const std::vector<std::unique_ptr<FrameBuffer>> &buffers =
            allocator.buffers(cfg.stream());
        for (std::unique_ptr<FrameBuffer> &buffer : buffers) {
            std::unique_ptr<Request> request = camera->createRequest();
            if (!request) {
                cerr << "Failed to create request" << endl;
                return -1;
            }
            if (request->addBuffer(cfg.stream(), buffer.get()) < 0) {
                cerr << "Failed to add buffer to request" << endl;
                return -1;
            }
            requests.push_back(std::move(request));
        }
    }

    // Start the camera streaming.
    if (camera->start() < 0) {
        cerr << "Failed to start camera streaming" << endl;
        return -1;
    }

    // Queue all requests.
    for (auto &req : requests) {
        if (camera->queueRequest(req.get()) < 0) {
            cerr << "Failed to queue request" << endl;
            return -1;
        }
    }

    // Main loop: Capture frames for a fixed duration.
    auto startTime = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - startTime < std::chrono::seconds(10))
    {
        // Blocking wait for a completed request.
        std::unique_ptr<Request> completedRequest = camera->waitForRequest();
        if (!completedRequest) {
            cerr << "Error waiting for request" << endl;
            break;
        }

        // Assume a single stream; retrieve the first buffer.
        const auto &buffers = completedRequest->buffers();
        if (buffers.empty()) {
            cerr << "No buffers in completed request" << endl;
            continue;
        }
        // Get the buffer for the stream.
        const FrameBuffer *buffer = buffers.begin()->second;

        // For demonstration, assume the first plane holds the Y (luma) data.
        if (buffer->planes().empty()) {
            cerr << "Buffer has no planes" << endl;
            continue;
        }
        const FrameBuffer::Plane &plane = buffer->planes()[0];

        // Create an OpenCV Mat from the Y plane data.
        uint8_t *data = static_cast<uint8_t *>(plane.mem);
        int width = config->at(0).size.width;
        int height = config->at(0).size.height;
        Mat yPlane(height, width, CV_8UC1, data);

        // Convert the grayscale image to BGR (not a full YUV conversion).
        Mat bgr;
        cvtColor(yPlane, bgr, COLOR_GRAY2BGR);

        // Display the frame.
        imshow("Camera", bgr);
        if (waitKey(1) == 27)  // Exit if ESC is pressed.
            break;

        // Re-queue the request for the next frame.
        if (camera->queueRequest(completedRequest.get()) < 0) {
            cerr << "Failed to re-queue request" << endl;
            break;
        }
    }

    // Clean up: stop streaming, release camera.
    camera->stop();
    camera->release();
    cm.stop();

    return 0;
}

