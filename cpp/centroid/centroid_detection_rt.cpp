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

// Set real-time priority
void set_real_time_priority() {
    struct sched_param param;
    param.sched_priority = 99; // Highest priority for real-time execution
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        cerr << "Warning: Failed to set real-time priority!" << endl;
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        cerr << "Warning: Failed to lock memory!" << endl;
    }
}

// Process frames with step-by-step benchmarking
vector<Point> process_frames_timed(const string& input_folder, const string& output_folder,
                                   const Size& kernel_size = Size(15, 15), int threshold_value = 10) {
    vector<Point> centroids;
    centroids.reserve(1000);

    if (!fs::exists(output_folder)) {
        fs::create_directories(output_folder);
    }

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);
    vector<string> file_paths;

    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".png") {
            file_paths.push_back(entry.path().string());
        }
    }

    Mat frame, gray, top_hat, binary;
    
    for (const auto& file_path : file_paths) {
        frame = imread(file_path, IMREAD_COLOR);
        if (frame.empty()) {
            cerr << "Warning: Could not open " << file_path << endl;
            continue;
        }

        // Start timing
        auto start_total = high_resolution_clock::now();

        auto start = high_resolution_clock::now();
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        auto time_gray = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        GaussianBlur(gray, gray, Size(5, 5), 0);
        auto time_blur = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);
        auto time_tophat = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        auto time_threshold = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        morphologyEx(binary, binary, MORPH_CLOSE, kernel);
        auto time_morph_close = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        auto time_contours = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        auto time_moments = 0, time_draw = 0;
        if (!contours.empty()) {
            start = high_resolution_clock::now();
            auto max_iter = max_element(contours.begin(), contours.end(),
                                        [](const vector<Point>& a, const vector<Point>& b) {
                                            return contourArea(a) < contourArea(b);
                                        });

            if (max_iter != contours.end()) {
                Moments M = moments(*max_iter);
                time_moments = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

                if (M.m00 != 0) {
                    int cx = static_cast<int>(M.m10 / M.m00);
                    int cy = static_cast<int>(M.m01 / M.m00);
                    centroids.push_back(Point(cx, cy));

                    start = high_resolution_clock::now();
                    circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
                    time_draw = duration_cast<microseconds>(high_resolution_clock::now() - start).count();
                }
            }
        }

        auto total_time = duration_cast<microseconds>(high_resolution_clock::now() - start_total).count();

        // Print timing results for this frame
        cout << "Frame: " << fs::path(file_path).filename().string() << " (Total: " << total_time << " us)" << endl;
        cout << "  Grayscale:     " << time_gray << " us" << endl;
        cout << "  Blur:          " << time_blur << " us" << endl;
        cout << "  Top-Hat:       " << time_tophat << " us" << endl;
        cout << "  Threshold:     " << time_threshold << " us" << endl;
        cout << "  Morph Close:   " << time_morph_close << " us" << endl;
        cout << "  Contour Detect:" << time_contours << " us" << endl;
        cout << "  Moments Calc:  " << time_moments << " us" << endl;
        cout << "  Draw Centroid: " << time_draw << " us" << endl;

        // Save processed frames
        imwrite(output_folder + "/" + fs::path(file_path).filename().string(), binary);
        imwrite(output_folder + "/marked_" + fs::path(file_path).filename().string(), frame);
    }

    return centroids;
}

int main() {
    set_real_time_priority(); // Set real-time scheduling

    string input_folder = "frames_output";
    string output_folder = "binary_frames";

    vector<Point> centroids = process_frames_timed(input_folder, output_folder, Size(15, 15), 10);

    return 0;
}
