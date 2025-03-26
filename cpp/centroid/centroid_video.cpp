#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>

// Libcamera headers (adjust include path if needed)
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
    // Initialize the CameraManager and start it.
    CameraManager camManager;
    camManager.start();
    if (camManager.cameras().empty()) {
        cerr << "No cameras available" << endl;
        return -1;
    }

    // Open the first available camera.
    shared_ptr<Camera> camera = camManager.cameras()[0];
    if (!camera) {
        cerr << "Unable to access camera" << endl;
        return -1;
    }
    if (camera->acquire()) {
        cerr << "Failed to acquire camera" << endl;
        return -1;
    }

    // Generate a configuration for preview (viewfinder) mode.
    unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({ StreamRole::Viewfinder });
    if (!config) {
        cerr << "Failed to generate camera configuration" << endl;
        return -1;
    }

    // Set stream parameters:
    // Use RGB888 so that the image data can be directly used with OpenCV.
    config->at(0).pixelFormat = formats::RGB888;
    config->at(0).size = {640, 480};
    config->at(0).bufferCount = 4; // Using 4 buffers

    // Validate the configuration.
    if (config->validate() == CameraConfiguration::Invalid) {
        cerr << "Invalid camera configuration" << endl;
        return -1;
    }

    // Apply the configuration.
    if (camera->configure(config.get()) < 0) {
        cerr << "Failed to configure camera" << endl;
        return -1;
    }

    // Create a framebuffer allocator.
    FrameBufferAllocator allocator(camera);
    for (const StreamConfiguration &cfg : *config) {
        if (allocator.allocate(cfg.stream()) < 0) {
            cerr << "Failed to allocate buffers" << endl;
            return -1;
        }
    }

    // Create capture requests and attach allocated buffers.
    vector<unique_ptr<Request>> requests;
    for (const StreamConfiguration &cfg : *config) {
        const vector<unique_ptr<FrameBuffer>> &buffers = allocator.buffers(cfg.stream());
        for (auto &buffer : buffers) {
            unique_ptr<Request> request = camera->createRequest();
            if (!request) {
                cerr << "Failed to create request" << endl;
                return -1;
            }
            if (request->addBuffer(cfg.stream(), buffer.get()) < 0) {
                cerr << "Failed to add buffer to request" << endl;
                return -1;
            }
            requests.push_back(move(request));
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

    cout << "Press ESC in the OpenCV window to exit." << endl;

    // Main loop: retrieve and display frames.
    while (true) {
        // Wait for a completed request.
        unique_ptr<Request> completedRequest = camera->waitForRequest();
        if (!completedRequest) {
            cerr << "Error waiting for request" << endl;
            break;
        }

        // Get the first (and only) stream buffer.
        const auto &buffers = completedRequest->buffers();
        if (buffers.empty()) {
            cerr << "No buffers in completed request" << endl;
            continue;
        }
        const FrameBuffer *buffer = buffers.begin()->second;
        if (buffer->planes().empty()) {
            cerr << "Buffer has no planes" << endl;
            continue;
        }

        // For an RGB888 stream, we expect a single plane containing 3 bytes per pixel.
        const FrameBuffer::Plane &plane = buffer->planes()[0];

        // Wrap the data into an OpenCV Mat.
        // Note: Make sure that the buffer memory is mapped and accessible.
        uint8_t *data = static_cast<uint8_t *>(plane.mem);
        int width = config->at(0).size.width;
        int height = config->at(0).size.height;
        Mat frame(height, width, CV_8UC3, data);

        // Display the frame.
        imshow("Camera", frame);
        int key = waitKey(1);
        if (key == 27) // ESC key to exit.
            break;

        // Re-queue the request for the next frame.
        if (camera->queueRequest(completedRequest.get()) < 0) {
            cerr << "Failed to re-

