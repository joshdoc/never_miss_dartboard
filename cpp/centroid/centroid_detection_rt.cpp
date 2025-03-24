#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>
#include <iterator>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;
using namespace std::chrono;

vector<Point> process_frames_tophat_downsampled(const string& input_folder, const string& output_folder,
                                                Size kernel_size = Size(15, 15), int threshold_value = 30,
                                                double scale_factor = 0.5) {
    vector<Point> centroids;

    // Create output folder if it doesn't exist
    if (!fs::exists(output_folder)) {
        fs::create_directories(output_folder);
    }

    // Create structuring element (ellipse)
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);

    double total_time = 0.0;
    int frame_count = 0;

    // Iterate over all .png files in the input folder
    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".png") {
            string file_path = entry.path().string();
            Mat frame = imread(file_path);
            if (frame.empty()) {
                cerr << "Warning: Could not open " << file_path << endl;
                continue;
            }

            auto start = high_resolution_clock::now();

            // Downsample the image
            resize(frame, frame, Size(), scale_factor, scale_factor, INTER_AREA);

            // Convert to grayscale
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);

            // Apply Gaussian blur to reduce noise
            GaussianBlur(gray, gray, Size(5, 5), 0);

            // Apply morphological top-hat
            Mat top_hat;
            morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);

            // Threshold the top-hat result
            Mat binary;
            threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

            // Optional morphological closing
            morphologyEx(binary, binary, MORPH_CLOSE, kernel, Point(-1, -1), 1);

            // Find contours
            vector<vector<Point>> contours;
            findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                // Find the largest contour (by area)
                auto max_it = max_element(contours.begin(), contours.end(),
                                          [](const vector<Point>& a, const vector<Point>& b) {
                                              return contourArea(a) < contourArea(b);
                                          });
                int max_index = static_cast<int>(distance(contours.begin(), max_it));

                if (max_index >= 0) {
                    Moments M = moments(contours[max_index]);
                    if (M.m00 != 0) {
                        int cx = static_cast<int>(M.m10 / M.m00);
                        int cy = static_cast<int>(M.m01 / M.m00);
                        centroids.push_back(Point(cx, cy));
                        // Draw a red circle on the frame at the centroid
                        circle(frame, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
                    }
                }
            }

            auto end = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(end - start).count();
            total_time += duration;
            frame_count++;

            cout << "Processed " << entry.path().filename().string() << " in " << duration << " ms" << endl;

            // Save the binary image
            string binary_path = output_folder + "/" + entry.path().filename().string();
            imwrite(binary_path, binary);

            // Save the marked frame
            string marked_path = output_folder + "/marked_" + entry.path().filename().string();
            imwrite(marked_path, frame);
        }
    }

    double avg_time = (frame_count > 0) ? (total_time / frame_count) : 0.0;
    cout << "Average processing time per frame: " << avg_time << " ms" << endl;

    cout << "Centroids computed:" << endl;
    for (const auto& pt : centroids) {
        cout << "(" << pt.x << ", " << pt.y << ")" << endl;
    }

    return centroids;
}

int main() {
    string input_folder = "frames_output";      // Folder with .png frames
    string output_folder = "binary_frames";     // Folder to save results

    // Process frames using downsampled top-hat filtering
    vector<Point> centroids = process_frames_tophat_downsampled(input_folder, output_folder,
                                                                Size(12, 12), 8, 0.4);
    return 0;
}
