#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

Point processFrame(Mat &frame, Mat &kernel, double scale_factor, int threshold_value) {
    Mat gray;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Downsample the grayscale image
    Mat gray_downsampled;
    resize(gray, gray_downsampled, Size(), scale_factor, scale_factor, INTER_NEAREST);

    // Noise reduction
    GaussianBlur(gray_downsampled, gray_downsampled, Size(3, 3), 0);

    // Top-hat morphology
    Mat eroded, dilated, top_hat;
    erode(gray_downsampled, eroded, kernel);
    dilate(eroded, dilated, kernel);
    subtract(gray_downsampled, dilated, top_hat);

    // Thresholding
    Mat binary;
    threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

    // Find contours
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Find largest contour and compute centroid
    Point centroid(-1, -1);
    double max_area = 0;
    int max_index = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_index = i;
        }
    }

    if (max_index != -1) {
        Moments M = moments(contours[max_index]);
        if (M.m00 > 0) {
            int cx_down = static_cast<int>(M.m10 / M.m00);
            int cy_down = static_cast<int>(M.m01 / M.m00);
            centroid.x = static_cast<int>(cx_down / scale_factor);
            centroid.y = static_cast<int>(cy_down / scale_factor);
        }
    }

    return centroid;
}

int main() {
    // Initialize camera
    VideoCapture cap(0); // Use 0 for default camera
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        return -1;
    }

    // Set camera resolution (lower for better performance)
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // Create morphology kernel once
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
    double scale_factor = 0.4;
    int threshold_value = 25; // Adjust based on your lighting

    namedWindow("Centroid Tracking", WINDOW_NORMAL);

    while (true) {
        Mat frame;
        cap >> frame; // Capture frame
        if (frame.empty()) break;

        // Process frame and get centroid
        auto start = high_resolution_clock::now();
        Point centroid = processFrame(frame, kernel, scale_factor, threshold_value);
        auto stop = high_resolution_clock::now();

        // Draw centroid
        if (centroid.x != -1 && centroid.y != -1) {
            circle(frame, centroid, 10, Scalar(0, 0, 255), -1);
        }

        // Display processing time
        auto duration = duration_cast<milliseconds>(stop - start);
        putText(frame, "Process Time: " + to_string(duration.count()) + "ms", 
                Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        imshow("Centroid Tracking", frame);

        // Exit on 'q' key
        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
