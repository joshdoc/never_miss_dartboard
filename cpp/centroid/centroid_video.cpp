#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Use V4L2 device created by libcamera
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Camera not accessible" << std::endl;
        return -1;
    }

    // libcamera often uses these resolutions
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    cv::Mat frame;
    std::cout << "Press 'q' to quit" << std::endl;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
