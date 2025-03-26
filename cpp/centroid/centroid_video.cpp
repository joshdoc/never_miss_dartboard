#include <iostream>
#include <opencv2/opencv.hpp>
#include "libcamera_app.hpp"  // Raspberry Pi libcamera wrapper

int main() {
    try {
        // Initialize libcamera
        LibcameraApp app;
        app.OpenCamera();

        // Configure the camera for 640x480 @ 30FPS
        StreamInfo streamConfig = app.ConfigureVideo({{"width", 640}, {"height", 480}, {"framerate", 30}});
        app.StartCamera();

        std::cout << "Starting video feed... Press 'q' to exit.\n";

        while (true) {
            // Capture frame
            LibcameraApp::Msg msg = app.Wait();
            if (msg.type == LibcameraApp::MsgType::RequestComplete) {
                CompletedRequestPtr &request = std::get<CompletedRequestPtr>(msg.payload);
                libcamera::FrameBuffer *buffer = app.ViewfinderStream()->GetFrameBuffer(request);

                if (buffer) {
                    // Convert frame to OpenCV format
                    cv::Mat frame(streamConfig.height, streamConfig.width, CV_8UC3, buffer->planes()[0].data());
                    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);  // Convert from RGB to BGR for OpenCV
                    
                    // Display frame
                    cv::imshow("Camera Feed", frame);
                    
                    // Exit if 'q' is pressed
                    if (cv::waitKey(1) == 'q') break;
                }
            }
        }

        // Cleanup
        app.StopCamera();
        cv::destroyAllWindows();
        std::cout << "Camera stopped.\n";

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
