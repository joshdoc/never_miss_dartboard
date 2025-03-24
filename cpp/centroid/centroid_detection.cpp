#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;
using namespace std::chrono;

vector<Point> process_frames_timed(const string& input_folder, const string& output_folder,
                                   Size kernel_size = Size(15, 15), int threshold_value = 10) {
    vector<Point> centroids;

    // Create output folder if it doesn't exist
    if (!fs::exists(output_folder)) {
        fs::create_directories(output_folder);
    }

    // Create structuring element (ellipse)
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);

    // Iterate over all .png files in the input folder
    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".png") {
            string file_path = entry.path().string();
            Mat frame = imread(file_path);
            if (frame.empty()) {
                cerr << "Warning: Could not open " << file_path << endl;
                continue;
            }

            // Start timing for this frame
            auto start = high_resolution_clock::now();

            // Convert to grayscale
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            // Apply a slight Gaussian blur to reduce noise
            GaussianBlur(gray, gray, Size(5, 5), 0);

            // Apply morphological top-hat (top_hat = original - opening)
            Mat top_hat;
            morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);

            // Threshold the top-hat result to isolate bright regions
            Mat binary;
            threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

            // Optional morphological closing to fill small holes
            morphologyEx(binary, binary, MORPH_CLOSE, kernel, Point(-1, -1), 1);

            // Find contours in the binary image
            vector<vector<Point>> contours;
            findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                // Find the largest contour (by area)
                double max_area = 0.0;
                int max_index = -1;
                for (size_t i = 0; i < contours.size(); i++) {
                    double area = contourArea(contours[i]);
                    if (area > max_area) {
                        max_area = area;
                        max_index = static_cast<int>(i);
                    }
                }
                if (max_index >= 0) {
                    Moments M = moments(contours[max_index]);
                    if (M.m00 != 0) {
                        int cx = static_cast<int>(M.m10 / M.m00);
                        int cy = static_cast<int>(M.m01 / M.m00);
                        centroids.push_back(Point(cx, cy));
                        // Draw a red circle on the original frame at the centroid
                        circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
                    }
                }
            }

            // End timing for this frame
            auto end = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(end - start).count();
            cout << "Frame: " << entry.path().filename().string()
                 << " | Processing time: " << duration << " ms" << endl;

            // Save the binary image
            string binary_path = output_folder + "/" + entry.path().filename().string();
            imwrite(binary_path, binary);

            // Save the original frame with the marked centroid
            string marked_path = output_folder + "/marked_" + entry.path().filename().string();
            imwrite(marked_path, frame);
        }
    }

    cout << "Centroids computed:" << endl;
    for (const auto& pt : centroids) {
        cout << "(" << pt.x << ", " << pt.y << ")" << endl;
    }
    return centroids;
}

int main() {
    // Input and output folder paths
    string input_folder = "frames_output";      // Folder with your .png frames
    string output_folder = "binary_frames";       // Folder to save results

    // Process frames using top-hat filtering and time each frame's processing
    vector<Point> centroids = process_frames_timed(input_folder, output_folder,
                                                   Size(15, 15), 10);
    return 0;
}
