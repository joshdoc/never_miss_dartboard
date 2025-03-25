#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <chrono>
#include <sys/syscall.h>
#include <linux/sched.h>

using namespace cv;
using namespace std;
using namespace chrono;

// Real-time priority setup
void set_realtime_priority() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        cerr << "Warning: Failed to set real-time priority. Run with sudo!" << endl;
    }
}

// Keep original pipeline exactly as you designed it
bool process_frame(const Mat& frame, Mat& output, Point& centroid,
                  Size kernel_size = Size(9, 9), int threshold_value = 30,
                  double scale_factor = 0.4) {
    static Mat gray, eroded, dilated, top_hat, binary;  // Reuse memory
    static vector<vector<Point>> contours;  // Reuse contour storage
    
    // Your original processing steps (unchanged)
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
    GaussianBlur(gray, gray, Size(3, 3), 0);
    
    // Your manual top-hat implementation (preserved)
    static Mat kernel = getStructuringElement(MORPH_RECT, kernel_size);
    erode(gray, eroded, kernel);
    dilate(eroded, dilated, kernel);
    subtract(gray, dilated, top_hat);
    
    threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int max_index = -1;
    if (!contours.empty()) {
        auto max_it = max_element(contours.begin(), contours.end(),
                                [](const vector<Point>& a, const vector<Point>& b) {
                                    return contourArea(a) < contourArea(b);
                                });
        max_index = static_cast<int>(distance(contours.begin(), max_it));
    }

    if (max_index >= 0) {
        Moments M = moments(contours[max_index]);
        if (M.m00 > 10) {  // Minimum area threshold
            centroid.x = static_cast<int>((M.m10 / M.m00) / scale_factor);
            centroid.y = static_cast<int>((M.m01 / M.m00) / scale_factor);
            
            // Avoid clone() using pre-allocated memory
            frame.copyTo(output);
            circle(output, centroid, 5, Scalar(0, 0, 255), -1);
            return true;
        }
    }
    return false;
}

int main() {
    set_realtime_priority();  // Set highest priority
    
    // GStreamer pipeline for 720p70 capture (low-latency)
    // Use the appropriate GStreamer pipeline for the Raspberry Pi Camera Module V2
    string pipeline = "v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=70/1 "
                      "! videoconvert ! video/x-raw,format=BGR ! appsink sync=false";
    VideoCapture cap(pipeline, CAP_GSTREAMER);

    if (!cap.isOpened()) {
        cerr << "Failed to initialize camera!" << endl;
        return -1;
    }

    Mat frame, output;
    Point centroid;
    int frame_count = 0;
    auto last_report = steady_clock::now();

    // Warm-up camera
    for(int i = 0; i < 5; ++i) cap.grab();

    cout << "Starting 720p70 processing..." << endl;
    
    while(true) {
        auto start = steady_clock::now();
        
        if(!cap.read(frame)) {
            cerr << "Capture error" << endl;
            break;
        }

        bool detected = process_frame(frame, output, centroid);
        
        // Only save when detected (JPEG instead of PNG for speed)
        if(detected) {
            imwrite(format("detected/frame_%05d.jpg", frame_count), output);
            cout << "Detected: " << centroid << endl;
        }

        // Performance reporting
        if(++frame_count % 100 == 0) {
            auto now = steady_clock::now();
            double fps = 100000.0 / duration_cast<microseconds>(now - last_report).count();
            cout << "FPS: " << fps << " | Latency: " 
                 << duration_cast<microseconds>(now - start).count() << "μs" << endl;
            last_report = now;
        }

        // Exit on ESC (without GUI dependency)
        if(cap.get(CAP_PROP_POS_MSEC) > 0 && waitKey(1) == 27) break;
    }

    cap.release();
    return 0;
}
