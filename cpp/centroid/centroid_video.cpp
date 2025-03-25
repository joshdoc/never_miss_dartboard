#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <pthread.h>
#include <unistd.h>

using namespace cv;
using namespace std;

// Real-time priority setup
void set_realtime_priority() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        cerr << "Warning: Failed to set real-time priority. Run with sudo!" << endl;
    }
}

void process_frame(Mat& frame, double scale_factor) {
    static Mat gray, processed;
    static vector<vector<Point>> contours;
    const Size kernel_size(9, 9);
    const int threshold_value = 10;
    
    // Processing pipeline
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    resize(gray, processed, Size(), scale_factor, scale_factor, INTER_NEAREST);
    GaussianBlur(processed, processed, Size(3, 3), 0);
    
    // Top-hat filtering
    static Mat kernel = getStructuringElement(MORPH_RECT, kernel_size);
    morphologyEx(processed, processed, MORPH_TOPHAT, kernel);
    
    threshold(processed, processed, threshold_value, 255, THRESH_BINARY);
    findContours(processed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        auto max_contour = *max_element(contours.begin(), contours.end(),
            [](const vector<Point>& a, const vector<Point>& b) {
                return contourArea(a) < contourArea(b);
            });

        Moments M = moments(max_contour);
        if (M.m00 > 10) {  // Minimum contour area threshold
            Point centroid(
                static_cast<int>((M.m10 / M.m00) / scale_factor),
                static_cast<int>((M.m01 / M.m00) / scale_factor)
            );

            circle(frame, centroid, 5, Scalar(0, 0, 255), -1);
            cout << "CENTROID: " << centroid.x << "," << centroid.y << endl;
            imwrite("detection.jpg", frame);
        }
    }
}

int main() {
    set_realtime_priority();
    
    // GStreamer pipeline for 720p70 capture
    string pipeline = "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=70/1 ! "
                      "videoconvert ! appsink sync=false";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        cerr << "Failed to open camera!" << endl;
        return -1;
    }

    Mat frame;
    const double scale_factor = 0.4;
    unsigned long frame_count = 0;
    time_t start_time = time(nullptr);

    while (true) {
        if (!cap.read(frame)) {
            cerr << "Capture error" << endl;
            break;
        }

        process_frame(frame, scale_factor);
        frame_count++;

        // Calculate FPS every 2 seconds
        time_t current_time = time(nullptr);
        if (current_time - start_time >= 2) {
            cout << "FPS: " << frame_count/2 << endl;
            frame_count = 0;
            start_time = current_time;
        }
    }

    return 0;
}
