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
    streamConfig.bufferCount = 4; // Use 4 buffers for smooth streaming

    // Set frame rate (convert FPS to duration in microseconds)
    constexpr double fps = 70.0;
    constexpr int64_t frameDuration = 1000000 / fps; // microseconds per frame
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

    // Queue all requests
    for (auto &request : requests) {
        if (camera->queueRequest(request.get())) {
            std::cerr << "Failed to queue request" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // OpenCV window setup
    cv::namedWindow("Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera Preview", 1280, 720);

    // Main capture loop
    while (true) {
        std::unique_ptr<Request> request = camera->waitForCompletedRequest();
        if (!request) {
            std::cerr << "Request error" << std::endl;
            break;
        }

        // Get frame buffer
        FrameBuffer *buffer = request->buffers().at(streamConfig.stream());
        const FrameBuffer::Plane &yPlane = buffer->planes().front();

        // Map the buffer for CPU access
        const Span<uint8_t> mem = buffer->maps().front().get();
        if (mem.empty()) {
            std::cerr << "Failed to map buffer" << std::endl;
            continue;
        }

        // Create YUV matrix (NV12 format)
        // Y plane: 720 rows, 1280 columns
        // UV plane: 360 rows, 1280 columns (interleaved)
        cv::Mat yuv(1080, 1280, CV_8UC1, mem.data()); // 720 + 360 = 1080 rows

        // Convert YUV to BGR using OpenCV
        cv::Mat bgr;
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);

        // Display frame
        cv::imshow("Camera Preview", bgr);

        // Requeue the request
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request.get());

        // Check for exit key
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Cleanup
    camera->stop();
    camera->release();
    cameraManager->stop();

    return EXIT_SUCCESS;
}
