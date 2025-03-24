#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;
using namespace std::chrono;

// Function to set real-time priority
void set_real_time_priority() {
    struct sched_param param;
    param.sched_priority = 99; // Highest priority for real-time
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        cerr << "Warning: Failed to set real-time priority!" << endl;
    }
    // Lock memory to prevent swapping
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        cerr << "Warning: Failed to lock memory!" << endl;
    }
}

// Real-time frame processing function
vector<Point> process_frames_timed(const string& input_folder, const string& output_folder,
                                   const Size& kernel_size = Size(15, 15), int threshold_value = 10) {
    vector<Point> centroids;
    centroids.reserve(1000);  // Preallocate vector to reduce dynamic memory allocation

    // Ensure output folder exists
    if (!fs::exists(output_folder)) {
        fs::create_directories(output_folder);
    }

    // Precompute structuring element (ellipse)
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);

    // Collect all filenames before processing to minimize system calls inside loop
    vector<string> file_paths;
    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".png") {
            file_paths.push_back(entry.path().string());
        }
    }

    // Reusable Mats to avoid frequent allocations
    Mat frame, gray, top_hat, binary;

    for (const auto& file_path : file_paths) {
        frame = imread(file_path, IMREAD_COLOR);
        if (frame.empty()) {
            cerr << "Warning: Could not open " << file_path << endl;
            continue;
        }

        // **Start Timing for This Frame**
        auto start = high_resolution_clock::now();

        // Convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Apply Gaussian blur (preallocated buffer)
        GaussianBlur(gray, gray, Size(5, 5), 0);

        // Apply morphological top-hat (preallocated buffer)
        morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);

        // Thresholding (preallocated buffer)
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Morphological closing (reduces flickering effects)
        morphologyEx(binary, binary, MORPH_CLOSE, kernel, Point(-1, -1), 1);

        // Find contours (reuse container to avoid allocations)
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find the largest contour by area
            auto max_iter = max_element(contours.begin(), contours.end(),
                                        [](const vector<Point>& a, const vector<Point>& b) {
                                            return contourArea(a) < contourArea(b);
                                        });

            if (max_iter != contours.end()) {
                Moments M = moments(*max_iter);
                if (M.m00 != 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);
                    centroids.push_back(Point(cx, cy));

                    // Draw the centroid in red
                    circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
                }
            }
        }

        // **End Timing for This Frame**
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();
        cout << "Frame: " << fs::path(file_path).filename().string()
             << " | Processing time: " << duration << " ms" << endl;

        // Save processed frames (avoid frequent string operations)
        string binary_path = output_folder + "/" + fs::path(file_path).filename().string();
        string marked_path = output_folder + "/marked_" + fs::path(file_path).filename().string();
        imwrite(binary_path, binary);
        imwrite(marked_path, frame);
    }

    cout << "Centroids computed:" << endl;
    for (const auto& pt : centroids) {
        cout << "(" << pt.x << ", " << pt.y << ")" << endl;
    }

    return centroids;
}

int main() {
    set_real_time_priority(); // Set RT scheduling

    // Input and output folders
    string input_folder = "frames_output";
    string output_folder = "binary_frames";

    // Run frame processing
    vector<Point> centroids = process_frames_timed(input_folder, output_folder, Size(15, 15), 10);

    return 0;
}
