#include <opencv2/opencv.hpp>
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

int main() {
      // GStreamer pipeline configuration
    const std::string pipeline = 
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,format=NV12 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1";
    
    // Initialize camera (replace with your camera setup)
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        return -1;
    }

    // Parameters
    float scale_factor = 0.5;  // Downsample factor
    int threshold_value = 50;  // Adjust based on your needs
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5,5));

    namedWindow("Binary Feed", WINDOW_NORMAL);
    
    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Processing pipeline
        Mat gray, binary;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(gray, gray, Size(3, 3), 0);
        
        // Top-hat transform
        Mat eroded, dilated, top_hat;
        erode(gray, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = gray - dilated;
        
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Find contours
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Find largest contour
        Point centroid(-1, -1);
        if (!contours.empty()) {
            auto max_it = max_element(contours.begin(), contours.end(),
                                    [](const vector<Point>& a, const vector<Point>& b) {
                                        return contourArea(a) < contourArea(b);
                                    });
            
            Moments M = moments(contours[distance(contours.begin(), max_it)]);
            if (M.m00 != 0) {
                centroid.x = static_cast<int>(M.m10/M.m00 / scale_factor);
                centroid.y = static_cast<int>(M.m01/M.m00 / scale_factor);
                
                // Print centroid coordinates
                cout << "Centroid detected at: (" << centroid.x 
                     << ", " << centroid.y << ")" << endl;
            }
        }

        // Display binary image
        imshow("Binary Feed", binary);

        if (waitKey(1) == 27) break;  // ESC to exit
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
