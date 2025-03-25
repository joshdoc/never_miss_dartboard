#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // GStreamer pipeline for libcamera
    string pipeline = "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30 ! "
                      "videoconvert ! video/x-raw,format=BGR ! appsink";
    
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        cerr << "Failed to open camera!" << endl;
        return -1;
    }

    cout << "Displaying feed - Press ESC to quit" << endl;
    
    Mat frame;
    while(true) {
        cap >> frame;
        if(frame.empty()) {
            cerr << "Empty frame!" << endl;
            continue;
        }
        
        imshow("Pi Camera", frame);
        if(waitKey(1) == 27) break;
    }
    
    cap.release();
    return 0;
}
