#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Simple test pipeline
    string pipeline = "libcamerasrc ! video/x-raw,width=640,height=480 ! videoconvert ! appsink";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        cerr << "FAILED: Camera not accessible" << endl;
        return -1;
    }

    cout << "Camera opened successfully!" << endl;
    
    Mat frame;
    cap >> frame;
    
    if (frame.empty()) {
        cerr << "FAILED: Empty frame" << endl;
    } else {
        cout << "Frame size: " << frame.size() << endl;
        imwrite("test_frame.jpg", frame);
        cout << "Saved test_frame.jpg - check this file" << endl;
    }
    
    cap.release();
    return 0;
}
