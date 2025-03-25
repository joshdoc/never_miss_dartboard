#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

// Include libcamera headers (paths may vary based on your installation)
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer.h>
#include <libcamera/controls.h>

using namespace libcamera;
using namespace std::chrono_literals;

//
// A placeholder function to convert the raw YUV buffer to an OpenCV BGR image.
// In a real application, you need to map the buffer memory, extract Y, U, and V planes,
// and then convert using cv::cvtColor or libyuv. This example creates a blank image.
//
cv::Mat convertBufferToMat(const FrameBuffer *buffer, const Size &size)
{
    // For demonstration purposes, we return a dummy black image.
    // Replace this with proper buffer mapping and conversion.
    return cv::Mat::zeros(size.height, size.width, CV_8UC3);
}

int main()
{
    // Create and start the CameraManager
    CameraManager camManager;
    if (camManager.start() < 0) {
        std::cerr << "Failed to start CameraManager" << std::endl;
        return -1;
    }

    // Select the first available camera
    if (camManager.cameras().empty()) {
        std::cerr << "No cameras available" << std::endl;
        return -1;
    }
    std::shared_ptr<Camera> camera = camManager.cameras()[0];
    camera->acquire();

    // Create a configuration for video recording
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({StreamRole::VideoRecording});
    if (!config) {
        std::cerr << "Failed to generate camera configuration" << std::endl;
        return -1;
    }

    // We expect one stream. Adjust the format if needed.
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = formats::YUV420;  // Assumed supported pixel format
    streamConfig.size = {1280, 720};

    // Apply the configuration
    if (camera->configure(config.get()) < 0) {
        std::cerr << "Failed to configure camera" << std::endl;
        return -1;
    }

    // Set frame rate via controls, if supported.
    ControlList controls = camera->controls();
    controls.set(controls::FrameRate, 70);
    camera->setControls(&controls);

    // Set up the FrameBufferAllocator and allocate buffers for the stream
    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(streamConfig.stream()) < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        return -1;
    }

    // Create requests for each allocated buffer
    std::vector<std::unique_ptr<Request>> requests;
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator.buffers(streamConfig.stream());
    for (size_t i = 0; i < buffers.size(); i++) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return -1;
        }
        if (request->addBuffer(streamConfig.stream(), buffers[i].get()) < 0) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return -1;
        }
        requests.push_back(std::move(request));
    }

    // Start the camera and queue all requests.
    if (camera->start() < 0) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    for (auto &request : requests)
        camera->queueRequest(request.get());

    std::cout << "Press 'q' to exit" << std::endl;
    cv::namedWindow("Libcamera Feed", cv::WINDOW_AUTOSIZE);

    // Main loop: In a real application, you would use an event-driven callback.
    // Here, we poll for completed requests.
    bool running = true;
    while (running) {
        // Wait for a request to complete (this is a simplified blocking wait).
        Request *completedRequest = camera->wait();
        if (!completedRequest) {
            continue;
        }

        // Retrieve the FrameBuffer from the completed request.
        // We assume a single stream; get the first buffer.
        auto it = completedRequest->buffers().begin();
        const FrameBuffer *buffer = it->second;

        // Convert the buffer to a cv::Mat.
        cv::Mat frame = convertBufferToMat(buffer, streamConfig.size);

        // Display the frame.
        cv::imshow("Libcamera Feed", frame);
        if (cv::waitKey(1) == 'q')
            running = false;

        // Requeue the request.
        camera->queueRequest(completedRequest);
    }

    // Cleanup.
    camera->stop();
    camera->release();
    camManager.stop();
    cv::destroyAllWindows();

    return 0;
}
