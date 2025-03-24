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

vector<Point> process_frames_tophat_downsampled(const string& input_folder, const string& output_folder,
                                                Size kernel_size = Size(15, 15), int threshold_value = 30,
                                                double scale_factor = 0.5) {
    vector<Point> centroids;
    if (!fs::exists(output_folder)) {
        fs::create_directories(output_folder);
    }

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);
    double total_time = 0;

    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() != ".png") continue;

        string file_path = entry.path().string();
        Mat frame = imread(file_path);
        if (frame.empty()) continue;

        auto start_time = high_resolution_clock::now();

        // Downsample the image
        Mat resized;
        resize(frame, resized, Size(), scale_factor, scale_factor, INTER_AREA);

        // Convert to grayscale
        Mat gray;
        cvtColor(resized, gray, COLOR_BGR2GRAY);

        // Apply Gaussian blur
        GaussianBlur(gray, gray, Size(5, 5), 0);

        // Morphological Top-Hat
        Mat top_hat;
        morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);

        // Thresholding
        Mat binary;
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Morphological closing
        morphologyEx(binary, binary, MORPH_CLOSE, kernel);

        // Find contours
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            auto largest_contour = max_element(contours.begin(), contours.end(),
                                               [](const vector<Point>& a, const vector<Point>& b) {
                                                   return contourArea(a) < contourArea(b);
                                               });
            Moments M = moments(*largest_contour);
            if (M.m00 != 0) {
                int cx = static_cast<int>(M.m10 / M.m00);
                int cy = static_cast<int>(M.m01 / M.m00);
                centroids.emplace_back(cx, cy);
                circle(resized, Point(cx, cy), 5, Scalar(0, 0, 255), -1);
            }
        }

        auto end_time = high_resolution_clock::now();
        double frame_time = duration_cast<milliseconds>(end_time - start_time).count();
        total_time += frame_time;

        cout << "Processed " << entry.path().filename().string() << " in " << frame_time << " ms" << endl;

        // Save images
        imwrite(output_folder + "/" + entry.path().filename().string(), binary);
        imwrite(output_folder + "/marked_" + entry.path().filename().string(), resized);
    }

    cout << "Average processing time per frame: " << (total_time / fs::directory_iterator(input_folder).distance()) << " ms" << endl;
    return centroids;
}

int main() {
    string input_folder = "frames_output";
    string output_folder = "binary_frames";
    process_frames_tophat_downsampled(input_folder, output_folder, Size(12, 12), 8, 0.4);
    return 0;
}


