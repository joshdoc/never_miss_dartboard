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

            auto start_total = high_resolution_clock::now();

            // Step 1: Downsample the image
            auto start_downsample = high_resolution_clock::now();
            resize(frame, frame, Size(), scale_factor, scale_factor, INTER_AREA);
            auto end_downsample = high_resolution_clock::now();

            // Step 2: Convert to grayscale
            auto start_grayscale = high_resolution_clock::now();
            Mat gray;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            auto end_grayscale = high_resolution_clock::now();

            // Step 3: Apply Gaussian blur
            auto start_blur = high_resolution_clock::now();
            GaussianBlur(gray, gray, Size(5, 5), 0);
            auto end_blur = high_resolution_clock::now();

            // Step 4: Apply morphological top-hat
            auto start_tophat = high_resolution_clock::now();
            Mat top_hat;
            morphologyEx(gray, top_hat, MORPH_TOPHAT, kernel);
            auto end_tophat = high_resolution_clock::now();

            // Step 5: Thresholding
            auto start_threshold = high_resolution_clock::now();
            Mat binary;
            threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
            auto end_threshold = high_resolution_clock::now();

            // Step 6: Morphological closing
            auto start_morph_close = high_resolution_clock::now();
            morphologyEx(binary, binary, MORPH_CLOSE, kernel, Point(-1, -1), 1);
            auto end_morph_close = high_resolution_clock::now();

            // Step 7: Find contours
            auto start_contours = high_resolution_clock::now();
            vector<vector<Point>> contours;
            findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            auto end_contours = high_resolution_clock::now();

            if (!contours.empty()) {
                // Step 8: Find the largest contour
                auto start_largest_contour = high_resolution_clock::now();
                auto max_it = max_element(contours.begin(), contours.end(),
                                          [](const vector<Point>& a, const vector<Point>& b) {
                                              return contourArea(a) < contourArea(b);
                                          });
                int max_index = static_cast<int>(distance(contours.begin(), max_it));
                auto end_largest_contour = high_resolution_clock::now();

                // Step 9: Compute centroid
                auto start_centroid = high_resolution_clock::now();
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
                auto end_centroid = high_resolution_clock::now();
            }

            auto end_total = high_resolution_clock::now();

            // Compute timing durations
            auto time_downsample = duration_cast<milliseconds>(end_downsample - start_downsample).count();
            auto time_grayscale = duration_cast<milliseconds>(end_grayscale - start_grayscale).count();
            auto time_blur = duration_cast<milliseconds>(end_blur - start_blur).count();
            auto time_tophat = duration_cast<milliseconds>(end_tophat - start_tophat).count();
            auto time_threshold = duration_cast<milliseconds>(end_threshold - start_threshold).count();
            auto time_morph_close = duration_cast<milliseconds>(end_morph_close - start_morph_close).count();
            auto time_contours = duration_cast<milliseconds>(end_contours - start_contours).count();
            auto time_largest_contour = duration_cast<milliseconds>(end_largest_contour - start_largest_contour).count();
            auto time_centroid = duration_cast<milliseconds>(end_centroid - start_centroid).count();
            auto total_time_frame = duration_cast<milliseconds>(end_total - start_total).count();

            total_time += total_time_frame;
            frame_count++;

            cout << "Processed " << entry.path().filename().string() << " in " << total_time_frame << " ms" << endl;
            cout << "  Downsampling: " << time_downsample << " ms" << endl;
            cout << "  Grayscale: " << time_grayscale << " ms" << endl;
            cout << "  Gaussian Blur: " << time_blur << " ms" << endl;
            cout << "  Top-hat: " << time_tophat << " ms" << endl;
            cout << "  Thresholding: " << time_threshold << " ms" << endl;
            cout << "  Morphological Closing: " << time_morph_close << " ms" << endl;
            cout << "  Contour Detection: " << time_contours << " ms" << endl;
            cout << "  Largest Contour Selection: " << time_largest_contour << " ms" << endl;
            cout << "  Centroid Calculation: " << time_centroid << " ms" << endl;

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
                                                                Size(15, 15), 10, 0.5);
    return 0;
}
