#include <libcamera/libcamera.h>
#include <chrono>
#include <iostream>
#include <signal.h>

using namespace libcamera;
using namespace std::chrono;

std::atomic<bool> exit_flag{false};

void signal_handler(int)
{
    exit_flag = true;
}

int main()
{
    signal(SIGINT, signal_handler);

    // Create camera manager
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    if (cm->cameras().empty()) {
        std::cerr << "No cameras found!" << std::endl;
        return 1;
    }

    // Acquire first available camera
    std::shared_ptr<Camera> camera = cm->get(cm->cameras()[0]->id());
    camera->acquire();

    // Configure for 720p70
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::VideoRecording});
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size = {1280, 720};
    streamConfig.pixelFormat = formats::RGB888;
    streamConfig.bufferCount = 6;

    // Set frame duration for 70 FPS (14285µs per frame)
    const int64_t frameDuration = 14285;
    ControlList controls;
    controls.set(controls::FrameDurationLimits, {frameDuration, frameDuration});

    // Validate configuration
    config->validate();
    if (camera->configure(config.get()) {
        std::cerr << "Configuration failed" << std::endl;
        return 1;
    }

    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    allocator.allocate(streamConfig.stream());

    // Create and queue requests
    std::vector<std::unique_ptr<Request>> requests;
    for (auto &buffer : allocator.buffers(streamConfig.stream())) {
        std::unique_ptr<Request> request = camera->createRequest();
        request->addBuffer(streamConfig.stream(), buffer.get());
        requests.push_back(std::move(request));
    }

    camera->start(&controls);

    // Queue initial requests
    for (auto &request : requests)
        camera->queueRequest(request.get());

    // Timing variables
    auto t_start = high_resolution_clock::now();
    int frame_count = 0;
    constexpr int fps_window = 70; // Update FPS every second

    std::cout << "Capturing 720p70 - Press CTRL-C to exit..." << std::endl;

    while (!exit_flag) {
        Request *request = camera->waitForCompletedRequest();
        if (!request) continue;

        // FPS calculation
        frame_count++;
        if (frame_count % fps_window == 0) {
            auto t_now = high_resolution_clock::now();
            double fps = fps_window / duration<double>(t_now - t_start).count();
            t_start = t_now;
            std::cout << "\rActual FPS: " << fps << std::flush;
        }

        // Requeue the request
        request->reuse();
        camera->queueRequest(request);
    }

    // Cleanup
    camera->stop();
    allocator.free(streamConfig.stream());
    camera->release();
    cm->stop();

    std::cout << "\nCapture stopped" << std::endl;
    return 0;
}
