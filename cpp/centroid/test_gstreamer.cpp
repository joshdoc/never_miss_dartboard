#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // Use V4L2 backend for Raspberry Pi camera
    VideoCapture cap(0, CAP_V4L2); 
    
    if (!cap.isOpened()) {
        cerr << "Error: Could not open camera!" << endl;
        return -1;
    }

    // Set camera parameters
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 30); // Start with 30 FPS for testing

    cout << "Displaying camera feed. Press 'q' to quit..." << endl;

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cerr << "Warning: Empty frame!" << endl;
            continue;
        }

        // Display frame
        imshow("Camera Feed", frame);

        // Exit on 'q' key
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
