#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// GStreamer pipeline configuration for libcamera
const string pipeline = 
    "libcamerasrc camera-name=/base/axi/pcie@120000/rp1/i2c@88000/imx219@10 "
    "! video/x-raw, format=RGBx, width=640, height=480, framerate=200/1 "
    "! videoconvert ! video/x-raw, format=BGR "
    "! appsink drop=1";

int main()
{
    // Initialize video capture with GStreamer pipeline
    VideoCapture vid_cam(pipeline, CAP_GSTREAMER);
    
    if(!vid_cam.isOpened()) {
        cerr << "Failed to open camera!" << endl;
        return -1;
    }

    // Create windows for display
    namedWindow("Camera Feed", WINDOW_NORMAL);
    namedWindow("ROI View", WINDOW_NORMAL);

    Mat frame, roi;
    
    // Set up video writer (if needed)
    // VideoWriter vid_recorder("output.mp4", 
    //     VideoWriter::fourcc('M','J','P','G'), 
    //     30, Size(640, 480));

    while(true) {
        // Capture frame
        if(!vid_cam.read(frame)) {
            cerr << "Failed to capture frame!" << endl;
            break;
        }

        // Process frame
        flip(frame, frame, -1);  // Flip both axes
        // copy2ROI(frame);  // Your custom ROI function
        // vid_recorder.write(roi);  // If recording

        // Display frames
        imshow("Camera Feed", frame);
        imshow("ROI View", roi);

        // Handle ESC key
        if(waitKey(1000/30) == 27) {  // 30 FPS
            cout << "Stopped by user" << endl;
            break;
        }
    }

    // Cleanup
    vid_cam.release();
    destroyAllWindows();
    return 0;
}
