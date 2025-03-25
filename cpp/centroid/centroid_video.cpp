#include <iostream>
#include <queue>
#include <mutex>
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

using namespace libcamera;

std::queue<Request *> requestQueue;
std::mutex queueMutex;

int main() {
    // Initialize Camera Manager
    CameraManager cameraManager;
    cameraManager.start();

    // Acquire first available camera
    if (cameraManager.cameras().empty()) {
        std::cerr << "No cameras available" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::shared_ptr<Camera> camera = cameraManager.cameras()[0];
    if (camera->acquire()) {
        std::cerr << "Failed to acquire camera" << std::endl;
        return EXIT_FAILURE;
    }

    // Configure camera
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::VideoRecording });
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size = { 1280, 720 };
    streamConfig.pixelFormat = formats::YUV420;
    streamConfig.bufferCount = 6;  // More buffers for better pipelining

    // Set frame rate
    constexpr double fps = 70.0;
    constexpr int64_t frameDuration = 1000000 / fps;
    ControlList controls;
    controls.set(controls::FrameDurationLimits, { frameDuration, frameDuration });

    // Validate configuration
    config->validate();
    if (camera->configure(config.get()) < 0) {
        std::cerr << "Failed to configure camera" << std::endl;
        return EXIT_FAILURE;
    }

    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    allocator.allocate(streamConfig.stream());
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator.buffers(streamConfig.stream());

    // Create requests
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

    // Request completion callback
    auto requestComplete = [&](Request *request) {
        if (request->status() == Request::RequestCancelled)
            return;

        std::lock_guard<std::mutex> lock(queueMutex);
        requestQueue.push(request);
    };

    // Start camera
    if (camera->start(&controls)) {
        std::cerr << "Failed to start camera" << std::endl;
        return EXIT_FAILURE;
    }

    // Queue requests
    for (auto &request : requests) {
        request->setCompletionCallback(requestComplete);
        camera->queueRequest(request.get());
    }

    // Create OpenCV window
    cv::namedWindow("Camera Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera Preview", 1280, 720);

    // Main processing loop
    while (true) {
        // Check for exit key
        if (cv::waitKey(1) == 'q') {
            break;
        }

        // Process completed requests
        std::unique_lock<std::mutex> lock(queueMutex);
        if (requestQueue.empty()) {
            lock.unlock();
            continue;
        }

        Request *request = requestQueue.front();
        requestQueue.pop();
        lock.unlock();

        // Get buffer and metadata
        FrameBuffer *buffer = request->buffers().begin()->second;
        const FrameMetadata &metadata = buffer->metadata();
        size_t size = metadata.planes()[0].length;

        // Map the buffer
        MappedBuffer mapped(buffer, MappedBuffer::MapFlag::Read);
        if (!mapped.isValid()) {
            std::cerr << "Failed to map buffer" << std::endl;
            continue;
        }

        // Get YUV planes
        Span<uint8_t> yPlane = mapped.planes()[0];
        Span<uint8_t> uvPlane = mapped.planes()[1];

        // Create OpenCV matrices for YUV planes
        cv::Mat yMat(streamConfig.size.height, streamConfig.size.width, CV_8UC1, yPlane.data());
        cv::Mat uvMat(streamConfig.size.height / 2, streamConfig.size.width / 2, CV_8UC2, uvPlane.data());

        // Convert YUV to BGR using optimized OpenCV conversion
        cv::Mat bgr;
        cv::cvtColorTwoPlane(yMat, uvMat, bgr, cv::COLOR_YUV2BGR_NV12);

        // Display frame
        cv::imshow("Camera Preview", bgr);

        // Requeue request
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
    }

    // Cleanup
    camera->stop();
    camera->release();
    cameraManager.stop();

    return EXIT_SUCCESS;
}
