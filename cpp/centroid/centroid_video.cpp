#include <iostream>
#include <opencv2/opencv.hpp>

// Adjusted includes for nested directory structure
#include <libcamera/libcamera/libcamera.h>
#include <libcamera/libcamera/controls.h>
#include <libcamera/libcamera/camera_manager.h>

using namespace libcamera;

int main() {
    // Initialize camera system
    CameraManager manager;
    manager.start();

    if (manager.cameras().empty()) {
        std::cerr << "No cameras found!" << std::endl;
        return 1;
    }

    std::shared_ptr<Camera> camera = manager.get(manager.cameras()[0]->id());
    camera->acquire();

    // Configure stream (RGB format for OpenCV compatibility)
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});
    config->at(0).pixelFormat = formats::RGB888;
    config->at(0).size = {640, 480};
    config->validate();

    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    allocator.allocate(config->at(0).stream());

    if (camera->configure(config.get())) {
        std::cerr << "Camera configuration failed!" << std::endl;
        return 1;
    }

    camera->start();
    cv::namedWindow("Grayscale Feed", cv::WINDOW_NORMAL);

    // Frame processing callback
    camera->requestCompleted.connect([&](Request *request) {
        if (request->status() == Request::RequestCancelled) return;

        // Access frame data
        FrameBuffer *buffer = request->buffers().begin()->second;
        Span<uint8_t> data = buffer->planes()[0].data;

        // Convert to OpenCV Mat and process
        cv::Mat frame(config->at(0).size.height, config->at(0).size.width, 
                     CV_8UC3, data.data());
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        
        // Display grayscale frame
        cv::imshow("Grayscale Feed", gray);
        cv::waitKey(1);

        request->reuse();
    });

    // Queue requests
    for (auto &buffer : allocator.buffers()) {
        std::unique_ptr<Request> request = camera->createRequest();
        request->addBuffer(config->at(0).stream(), buffer.get());
        camera->queueRequest(request.release());
    }

    // Run until 'q' is pressed
    std::cout << "Press q to quit..." << std::endl;
    while (cv::waitKey(1) != 'q');

    // Cleanup
    camera->stop();
    camera->release();
    manager.stop();

    return 0;
}
