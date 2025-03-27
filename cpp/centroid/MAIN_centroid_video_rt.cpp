#include <opencv2/opencv.hpp>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>

using namespace cv;
using namespace std;
using namespace std::chrono;

// Add real-time configuration function
void configure_realtime() {
    // Lock memory to prevent swapping
    mlockall(MCL_CURRENT | MCL_FUTURE);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &param);
}

int main() {
    configure_realtime();  // Added real-time config

    const std::string pipeline = 
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,format=NV12,framerate=120/1 ! "  // Increased framerate
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 sync=false max-buffers=1";  // Low-latency options

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        return -1;
    }

    // Parameters
    float scale_factor = 0.4;
    int threshold_value = 50;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    // Pre-allocate Mats
    Mat gray, binary;
    vector<vector<Point>> contours;
    namedWindow("Binary Feed", WINDOW_NORMAL);

    // Warm-up frame
    Mat dummy;
    cap.read(dummy);

    // Timing variables
    auto last_frame = high_resolution_clock::now();
    
    while (true) {
        // Original frame capture
        Mat frame;
        if (!cap.read(frame)) break;

        // Grayscale, resize, and blur
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(gray, gray, Size(3, 3), 0);

        // Top-hat
        Mat eroded, dilated, top_hat;
        erode(gray, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = gray - dilated;

        // Thresholding
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Contouring
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Point centroid(-1, -1);
        if (!contours.empty()) {
            double max_area = 0;
            int max_index = 0;
            for(size_t i = 0; i < contours.size(); ++i) {
                double area = moments(contours[i]).m00;
                if(area > max_area) {
                    max_area = area;
                    max_index = i;
                }
            }
            
            Moments M = moments(contours[max_index]);
            if (M.m00 != 0) {
                centroid.x = static_cast<int>(M.m10/M.m00 / scale_factor);
                centroid.y = static_cast<int>(M.m01/M.m00 / scale_factor);
                // Minimal-output version
                printf("%d,%d\n", centroid.x, centroid.y);
            }
        }

        // Binary Display
        imshow("Binary Feed", binary);

        // Frame time monitoring
        auto now = high_resolution_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - last_frame);
        last_frame = now;
        if(waitKey(1) == 27) break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
