#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <unistd.h>           // for usleep
#include <sys/mman.h>         // for mmap, munmap
#include <fcntl.h>            // for O_RDONLY

// Libcamera headers (using headers available in /usr/include/libcamera/libcamera)
#include <libcamera.h>
#include <camera_manager.h>
#include <camera.h>
#include <framebuffer_allocator.h>
#include <request.h>
#include <formats.h>

// OpenCV headers
#include <opencv2/opencv.hpp>

using namespace libcamera;
using namespace std;
using namespace cv;

int main()
{
    // Start the CameraManager.
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
    auto config = camera->generateConfiguration({ StreamRole::Viewfinder });
    if (!config) {
        cerr << "Failed to generate camera configuration" << endl;
        return -1;
    }

    // Configure the stream: use RGB888 for easy conversion to OpenCV Mat.
    config->at(0).pixelFormat = formats::RGB888;
    config->at(0).size = {640, 480};
    config->at(0).bufferCount = 4;

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

    // Allocate frame buffers.
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

    // Start camera streaming.
    if (camera->start() < 0) {
        cerr << "Failed to start camera streaming" << endl;
        return -1;
    }

    // Queue all requests initially.
    for (auto &req : requests) {
        if (camera->queueRequest(req.get()) < 0) {
            cerr << "Failed to queue request" << endl;
            return -1;
        }
    }

    cout << "Press ESC in the OpenCV window to exit." << endl;

    // Main loop: poll for completed requests and display frames.
    while (true) {
        // Poll for completed requests.
        // (Assuming getCompletedRequests() returns a vector<Request*> of completed requests.)
        auto completedRequests = camera->getCompletedRequests();
        if (completedRequests.empty()) {
            // No completed request yet: sleep briefly and try again.
            usleep(1000); // sleep 1 ms
            continue;
        }

        // Process each completed request.
        for (Request *completedRequest : completedRequests) {
            // Retrieve the buffer from the request.
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
            // For RGB888, expect one plane.
            const FrameBuffer::Plane &plane = buffer->planes()[0];

            // Map the buffer memory.
            // Note: mapping on every frame is not optimal.
            uint8_t *data = static_cast<uint8_t*>(
                mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd, plane.offset));
            if (data == MAP_FAILED) {
                perror("mmap");
                continue;
            }

            int width = config->at(0).size.width;
            int height = config->at(0).size.height;
            // Wrap the data into an OpenCV Mat.
            Mat frame(height, width, CV_8UC3, data);

            // Display the frame.
            imshow("Camera", frame);
            int key = waitKey(1);
            if (key == 27) { // ESC key
                munmap(data, plane.length);
                goto exit_loop;
            }

            munmap(data, plane.length);

            // Re-queue the request for the next frame.
            if (camera->queueRequest(completedRequest) < 0) {
                cerr << "Failed to re-queue request" << endl;
                goto exit_loop;
            }
        }
    }

exit_loop:
    // Cleanup.
    camera->stop();
    camera->release();
    camManager.stop();
    destroyAllWindows();

    return 0;
}
