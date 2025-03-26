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
    // GStreamer pipeline for libcamera
    string pipeline = "libcamerasrc ! video/x-raw,width=640,height=480,format=NV12 "
                      "! videoconvert ! video/x-raw,format=BGR "
                      "! appsink drop=1";
    
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        cerr << "Failed to open camera! Try these fixes:" << endl;
        cerr << "1. Update system: sudo apt update && sudo apt upgrade" << endl;
        cerr << "2. Install GStreamer plugins: sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev" << endl;
        cerr << "3. Verify camera works: libcamera-hello" << endl;
        return -1;
    }

    // Create morphology kernel
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
    double scale_factor = 0.4;
    int threshold_value = 25;

    namedWindow("Centroid Tracking", WINDOW_NORMAL);

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) {
            cerr << "Empty frame received!" << endl;
            break;
        }

        // Process frame
        Point centroid = processFrame(frame, kernel, scale_factor, threshold_value);

        // Visualization
        if (centroid.x != -1) {
            circle(frame, centroid, 10, Scalar(0, 0, 255), -1);
        }
        imshow("Centroid Tracking", frame);

        if (waitKey(1) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
