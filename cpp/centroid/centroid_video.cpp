#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <pthread.h>

using namespace cv;
using namespace std;

// Real-time priority setup
void set_realtime_priority() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        cerr << "Warning: Failed to set real-time priority. Run with sudo?" << endl;
    }
}

void process_frame(Mat& frame, double scale_factor) {
    // Pipeline constants (adjust based on your application)
    const Size kernel_size(9, 9);
    const int threshold_value = 10;
    
    Mat gray, processed;
    vector<vector<Point>> contours;
    
    // Processing pipeline
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    resize(gray, processed, Size(), scale_factor, scale_factor, INTER_NEAREST);
    GaussianBlur(processed, processed, Size(3, 3), 0);
    
    // Top-hat filtering
    Mat kernel = getStructuringElement(MORPH_RECT, kernel_size);
    morphologyEx(processed, processed, MORPH_TOPHAT, kernel);
    
    // Threshold and find contours
    threshold(processed, processed, threshold_value, 255, THRESH_BINARY);
    findContours(processed, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Find largest contour
    if (!contours.empty()) {
        auto max_contour = *max_element(contours.begin(), contours.end(),
            [](const vector<Point>& a, const vector<Point>& b) {
                return contourArea(a) < contourArea(b);
            });

        Moments M = moments(max_contour);
        if (M.m00 > 0) {
            // Calculate and scale centroid coordinates
            Point centroid(
                static_cast<int>((M.m10 / M.m00) / scale_factor),
                static_cast<int>((M.m01 / M.m00) / scale_factor)
            );

            // Draw and output
            circle(frame, centroid, 5, Scalar(0, 0, 255), -1);
            cout << "Centroid detected at: " << centroid << endl;
            imwrite("detected_frame.jpg", frame);
        }
    }
}

int main() {
    set_realtime_priority();  // Set highest possible priority
    
    VideoCapture cap(0);  // Use appropriate camera index
    if (!cap.isOpened()) {
        cerr << "Error: Could not open video source" << endl;
        return -1;
    }

    // Set camera parameters for predictable performance
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);

    Mat frame;
    const double scale_factor = 0.4;  // Downsample factor

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        process_frame(frame, scale_factor);

        // Exit on ESC key
        if (waitKey(1) == 27) break;
    }

    return 0;
}
