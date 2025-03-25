#include <iostream>
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

using namespace libcamera;

int main() {
    // Initialize Camera Manager
    std::unique_ptr<CameraManager> cameraManager = std::make_unique<CameraManager>();
    cameraManager->start();
    
    // Acquire first available camera
    if (cameraManager->cameras().empty()) {
        std::cerr << "No cameras available" << std::endl;
        return EXIT_FAILURE;
    }
    std::shared_ptr<Camera> camera = cameraManager->cameras()[0];
    if (camera->acquire()) {
        std::cerr << "Failed to acquire camera" << std::endl;
        return EXIT_FAILURE;
    }

    // Configure camera
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size = {1280, 720};
    streamConfig.pixelFormat = formats::YUV420;
    streamConfig.bufferCount = 4;

    // Set frame rate
    constexpr double fps = 70.0;
    constexpr int64_t frameDuration = 1000000 / fps;
    ControlList controls;
    controls.set(controls::FrameDurationLimits, {frameDuration, frameDuration});

    // Validate configuration
    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "Failed to valid stream configuration" << std::endl;
        return EXIT_FAILURE;
    }
    if (camera->configure(config.get())) {
        std::cerr << "Failed to configure camera" << std::endl;
        return EXIT_FAILURE;
    }

    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    allocator.allocate(streamConfig.stream());
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator.buffers(streamConfig.stream());

    // Create and queue requests
    std::vector<std::unique_ptr<Request>> requests;
    for (const auto &buffer : buffers) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return EXIT_FAILURE;
        }
        
        if (request->addBuffer(streamConfig.stream(), buffer.get())) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return EXIT_FAILURE;
        }
        
        requests.push_back(std::move(request));
    }

    // Start camera
    if (camera->start(&controls)) {
        std::cerr << "Failed to start camera" << std::endl;
        return EXIT_FAILURE;
    }

    // OpenCV window setup
    cv::namedWindow("Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera Preview", 1280, 720);

    // Signal handler for completed requests
    for (auto &request : requests) {
        request->completed.connect([&](Request *req) {
            // Get completed request
            FrameBuffer *buffer = req->buffers().at(streamConfig.stream());
            
            // Map the buffer
            MappedFrameBuffer mapped(buffer, MappedFrameBuffer::MapFlag::Read);
            if (!mapped.isValid()) {
                std::cerr << "Failed to map buffer" << std::endl;
                return;
            }

            // Create YUV matrix (NV12 format)
            const FrameMetadata &metadata = buffer->metadata();
            cv::Mat yuv(metadata.planes()[0].height + metadata.planes()[1].height,
                      metadata.planes()[0].stride, CV_8UC1, mapped.planes()[0].data());

            // Convert to BGR
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

            // Display frame
            cv::imshow("Camera Preview", bgr);

            // Requeue the request
            req->reuse(Request::ReuseBuffers);
            camera->queueRequest(req);
        });
    }

    // Queue initial requests
    for (auto &request : requests) {
        camera->queueRequest(request.get());
    }

    // Main event loop
    while (true) {
        // Process camera events with 100ms timeout
        int ret = cameraManager->eventDispatcher()->processEvents(100);
        if (ret < 0) break;

        // Check for exit key
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Cleanup
    camera->stop();
    allocator.free(streamConfig.stream());
    camera->release();
    cameraManager->stop();

    return EXIT_SUCCESS;
}
