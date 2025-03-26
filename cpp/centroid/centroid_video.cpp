#include <iostream>
#include <opencv2/opencv.hpp>
#include "core/rpicam_app.hpp"
#include "core/options.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    try {
        // Initialize the RPi Camera App
        RPiCamApp app;
        Options *options = app.GetOptions();

        // Set custom resolution and framerate
        options->width = 640;
        options->height = 480;
        options->framerate = 30;

        // Open and configure the camera
        app.OpenCamera();
        app.ConfigureVideo();
        app.StartCamera();

        cout << "Starting video feed... Press 'q' to exit.\n";

        while (true) {
            // Wait for a frame
            RPiCamApp::Msg msg = app.Wait();
            if (msg.type == RPiCamApp::MsgType::RequestComplete) {
                CompletedRequestPtr &request = std::get<CompletedRequestPtr>(msg.payload);

                // Get frame data from the buffer
                libcamera::FrameBuffer *buffer = app.VideoStream()->GetFrameBuffer(request);
                if (buffer) {
                    // Convert frame to OpenCV format
                    cv::Mat frame(options->height, options->width, CV_8UC3, buffer->planes()[0].data());
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
        cout << "Camera stopped.\n";

    } catch (const std::exception &e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}
