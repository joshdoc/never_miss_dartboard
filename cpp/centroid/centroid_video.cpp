#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

// Non-blocking keyboard check
bool keyPressed(int &key) {
    struct termios original, modified;
    int flags = fcntl(0, F_GETFL, 0);
    
    tcgetattr(0, &original);
    modified = original;
    modified.c_lflag &= ~(ICANON | ECHO);
    modified.c_cc[VMIN] = 0;
    modified.c_cc[VTIME] = 0;
    
    tcsetattr(0, TCSANOW, &modified);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);
    
    int ch = getchar();
    if(ch != EOF) key = ch;
    
    tcsetattr(0, TCSANOW, &original);
    fcntl(0, F_SETFL, flags);
    
    return (ch != EOF);
}

int main() {
    cv::VideoCapture cap(0, cv::CAP_V4L2); // Use V4L2 backend
    
    if(!cap.isOpened()) {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }

    // Set camera parameters
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 70);
    
    // Verify settings
    std::cout << "Actual FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
    std::cout << "Resolution: " 
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    cv::Mat frame, gray, thresh;
    int key = 0;
    
    std::cout << "Headless capture running. Press 'q' to quit..." << std::endl;

    while(true) {
        double start = cv::getTickCount();
        
        // Capture frame
        if(!cap.read(frame)) {
            std::cerr << "Capture error" << std::endl;
            break;
        }

        // Centroid calculation
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        
        cv::Moments m = cv::moments(thresh, true);
        cv::Point centroid(-1, -1);
        if(m.m00 != 0) {
            centroid.x = m.m10 / m.m00;
            centroid.y = m.m01 / m.m00;
        }

        // FPS calculation
        double elapsed = (cv::getTickCount() - start) / cv::getTickFrequency();
        std::cout << "Frame time: " << elapsed << "s | FPS: " 
                  << 1/elapsed << " | Centroid: (" 
                  << centroid.x << "," << centroid.y << ")" << "\r" << std::flush;

        // Check for quit
        if(keyPressed(key) && (key == 'q' || key == 'Q')) {
            break;
        }
    }

    cap.release();
    std::cout << "\nCamera released. Exiting." << std::endl;
    return 0;
}
