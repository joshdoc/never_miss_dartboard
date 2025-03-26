#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // GStreamer pipeline configuration
    const std::string pipeline = 
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,format=NV12 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1";
        
    // Open video capture
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera!" << std::endl;
        return -1;
    }

    // Verify backend
    std::cout << "Using backend: " << cap.getBackendName() << std::endl;

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame)) {
            std::cerr << "Capture read error!" << std::endl;
            break;
        }

        // Your processing here
        cv::imshow("Camera Feed", frame);
        
        if (cv::waitKey(1) == 27) {  // ESC key
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
