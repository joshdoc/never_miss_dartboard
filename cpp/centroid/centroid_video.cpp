#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

// Function to process each frame and detect the largest centroid
bool process_frame(const Mat& frame, Mat& output, Point& centroid,
                   Size kernel_size = Size(9, 9), int threshold_value = 30,
                   double scale_factor = 0.4) {
    Mat gray, binary;
    vector<vector<Point>> contours;
    
    // Convert to grayscale
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Downsample for speed
    resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);

    // Apply Gaussian blur
    GaussianBlur(gray, gray, Size(3, 3), 0);

    // Create rectangular kernel
    Mat kernel = getStructuringElement(MORPH_RECT, kernel_size);

    // Top-Hat filtering (manual implementation for speed)
    Mat eroded, dilated, top_hat;
    erode(gray, eroded, kernel);
    dilate(eroded, dilated, kernel);
    top_hat = gray - dilated;

    // Apply threshold
    threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

    // Find contours
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Find the largest contour
    int max_index = -1;
    if (!contours.empty()) {
        auto max_it = max_element(contours.begin(), contours.end(),
                                  [](const vector<Point>& a, const vector<Point>& b) {
                                      return contourArea(a) < contourArea(b);
                                  });
        max_index = static_cast<int>(distance(contours.begin(), max_it));
    }

    // Compute centroid if a valid contour exists
    if (max_index >= 0) {
        Moments M = moments(contours[max_index]);
        if (M.m00 != 0) {
            int cx_downsampled = static_cast<int>(M.m10 / M.m00);
            int cy_downsampled = static_cast<int>(M.m01 / M.m00);

            // Scale back to original coordinates
            centroid.x = static_cast<int>(cx_downsampled / scale_factor);
            centroid.y = static_cast<int>(cy_downsampled / scale_factor);

            // Draw red dot at centroid location
            output = frame.clone();
            circle(output, centroid, 5, Scalar(0, 0, 255), -1);
            
            return true;  // Centroid found
        }
    }

    return false;  // No centroid detected
}

int main() {
    VideoCapture cap(0, CAP_V4L2); // Open Raspberry Pi Camera (V4L2 mode for low latency)
    
    if (!cap.isOpened()) {
        cerr << "Error: Could not open Raspberry Pi camera!" << endl;
        return -1;
    }

    // Set 720p resolution and 70 FPS
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 70);

    string output_folder = "detected_frames";
    if (!filesystem::exists(output_folder)) {
        filesystem::create_directories(output_folder);
    }

    Mat frame, output;
    Point centroid;
    int frame_count = 0;

    cout << "Processing video stream at 1280x720 @ 70 FPS. Press 'q' to exit..." << endl;

    while (true) {
        auto start_total = high_resolution_clock::now();

        cap >> frame;
        if (frame.empty()) {
            cerr << "Warning: Empty frame captured!" << endl;
            continue;
        }

        // Process frame and check if a centroid is detected
        if (process_frame(frame, output, centroid)) {
            string filename = output_folder + "/frame_" + to_string(frame_count) + ".png";
            imwrite(filename, output);
            cout << "Saved: " << filename << " (Centroid: " << centroid << ")" << endl;
        }

        auto end_total = high_resolution_clock::now();
        auto total_time = duration_cast<milliseconds>(end_total - start_total).count();
        cout << "Frame " << frame_count << " processed in " << total_time << " ms" << endl;

        frame_count++;

        // Press 'q' to quit
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    return 0;
}
