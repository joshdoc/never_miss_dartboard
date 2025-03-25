#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

int main() {
    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Camera not accessible" << std::endl;
        return -1;
    }

    // Critical settings for Pi camera
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 15);  // Start with lower FPS
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3);  // Reduce buffer size
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Allow camera to warm up
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    cv::Mat frame;
    int frame_count = 0;
    const int max_attempts = 30;

    while (frame_count < max_attempts) {
        if (!cap.grab()) {
            std::cerr << "WARNING: Frame not grabbed" << std::endl;
            continue;
        }
        
        if (!cap.retrieve(frame)) {
            std::cerr << "WARNING: Frame not retrieved" << std::endl;
            continue;
        }
        
        if (frame.empty()) {
            std::cerr << "WARNING: Empty frame received" << std::endl;
        } else {
            std::cout << "Success! Frame size: " 
                      << frame.cols << "x" << frame.rows << std::endl;
            cv::imshow("Camera Feed", frame);
            frame_count++;
        }
        
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
