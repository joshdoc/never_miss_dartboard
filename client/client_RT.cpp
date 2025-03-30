#include <opencv2/opencv.hpp>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <pthread.h>

using namespace cv;
using namespace std;
using namespace std::chrono;

// Shared frame data structure.
struct FrameData {
    steady_clock::time_point timestamp; // When the frame was processed.
    int centroidX;
    int centroidY;
    int frameWidth;
    int frameHeight;
};

// Shared state variables.
std::mutex data_mutex;
std::condition_variable data_cv;
bool newFrameData = false;
FrameData g_frameData;  // Latest frame data with a valid centroid.

// Real-time configuration for the main thread.
void configure_realtime() {
    // Lock memory to avoid swapping.
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    // Set CPU affinity for the main thread.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // Bind to core 3.
    sched_setaffinity(0, sizeof(cpuset), &cpuset);
    
    // Set SCHED_FIFO with maximum priority.
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &param);
}

// Utility to set a thread's CPU affinity and real-time priority.
void set_thread_realtime(std::thread &thr, int core, int priority) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    pthread_t handle = thr.native_handle();
    pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset);
    
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(handle, SCHED_FIFO, &sch_params)) {
        cerr << "Failed to set thread scheduling: " << strerror(errno) << "\n";
    }
}

// Camera processing thread.
// When a valid centroid is detected, it stores the frame's original dimensions,
// timestamp, and centroid coordinates, then signals the UART thread.
void cameraThread() {
    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,format=NV12,framerate=120/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 sync=false max-buffers=1";
    
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        return;
    }
    
    // Image processing parameters.
    const float scale_factor = 0.4f;
    const int threshold_value = 50;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));
    
    Mat gray, binary;
    vector<vector<Point>> contours;
    namedWindow("Binary Feed", WINDOW_NORMAL);
    
    // Warm-up frame.
    Mat dummy;
    cap.read(dummy);
    
    while (true) {
        Mat frame;
        if (!cap.read(frame)) break;
        
        // Save original frame dimensions.
        int frameWidth = frame.cols;
        int frameHeight = frame.rows;
        
        // Convert to grayscale, resize, and blur.
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(gray, gray, Size(3, 3), 0);
        
        // Top-hat filtering.
        Mat eroded, dilated, top_hat;
        erode(gray, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = gray - dilated;
        
        // Thresholding.
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        
        // Find contours.
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            // Find the contour with the maximum area.
            double max_area = 0;
            int max_index = -1;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = moments(contours[i]).m00;
                if (area > max_area) {
                    max_area = area;
                    max_index = i;
                }
            }
            if (max_index >= 0) {
                Moments M = moments(contours[max_index]);
                if (M.m00 != 0) {
                    // Compute centroid in original pixel coordinates.
                    int centroidX = static_cast<int>(M.m10 / M.m00 / scale_factor);
                    int centroidY = static_cast<int>(M.m01 / M.m00 / scale_factor);
                    
                    // Store valid frame data and notify the UART thread.
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        g_frameData.timestamp = steady_clock::now();
                        g_frameData.centroidX = centroidX;
                        g_frameData.centroidY = centroidY;
                        g_frameData.frameWidth = frameWidth;
                        g_frameData.frameHeight = frameHeight;
                        newFrameData = true;
                    }
                    data_cv.notify_one();
                    
                    // Optionally, print for debugging.
                    printf("Detected centroid: %d, %d (Frame: %dx%d)\n",
                           centroidX, centroidY, frameWidth, frameHeight);
                }
            }
        }
        
        imshow("Binary Feed", binary);
        if (waitKey(1) == 27) break;  // Exit on ESC key.
    }
    
    cap.release();
    destroyAllWindows();
}

// UART port configuration.
int configurePort(int fd, speed_t baud) {
    termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "tcgetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;  // 8-bit characters.
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;  // Timeout (in deciseconds).
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

void waitForStartCommand(int fd) {
    std::string command;
    char ch;
    cout << "UART Thread: Waiting for START command...\n";
    
    while (true) {
        ssize_t n = read(fd, &ch, 1);
        if (n > 0) {
            command.push_back(ch);
            
            // Check if "START\n" has been received
            if (command.find("START\n") != std::string::npos) {
                cout << "UART Thread: Received START command.\n";
                break;
            }
        }
        this_thread::sleep_for(10ms);  // Avoid CPU overuse
    }
}


// UART thread function.
// After receiving the start command, it waits for a notification each time
// the camera thread detects a valid centroid. Then it transmits the frame's
// pixel dimensions and timestamp (plus the centroid coordinates).
void uartThread() {
    const char* port = "/dev/ttyAMA0";  // Adjust if needed.
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "UART Thread: Error opening " << port << ": " << strerror(errno) << "\n";
        return;
    }
    if (configurePort(fd, B115200) != 0) {
        close(fd);
        return;
    }
    
    // Wait for a start command from the server.
    waitForStartCommand(fd);
    
    // Record the UART thread start time.
    auto uartStart = steady_clock::now();
    
    while (true) {
        // Wait until new frame data is available.
        unique_lock<mutex> lock(data_mutex);
        data_cv.wait(lock, [] { return newFrameData; });
        
        // Copy the data for transmission.
        FrameData data = g_frameData;
        newFrameData = false;
        lock.unlock();
        
        // Calculate elapsed time in milliseconds.
        uint16_t elapsed = static_cast<uint16_t>(
            duration_cast<milliseconds>(data.timestamp - uartStart).count()
        );
        
        // Prepare a 5-field message:
        // [elapsed time (ms), frame width, frame height, centroidX, centroidY]
        uint16_t msg[5] = { elapsed,
                            static_cast<uint16_t>(data.frameWidth),
                            static_cast<uint16_t>(data.frameHeight),
                            static_cast<uint16_t>(data.centroidX),
                            static_cast<uint16_t>(data.centroidY) };
        
        ssize_t written = write(fd, msg, sizeof(msg));
        if (written != sizeof(msg)) {
            cerr << "UART Thread: Write error: " << strerror(errno) << "\n";
        } else {
            cout << "UART Sent: time=" << elapsed
                 << " ms, dims=" << data.frameWidth << "x" << data.frameHeight
                 << ", centroid=(" << data.centroidX << "," << data.centroidY << ")\n";
        }
    }
    
    close(fd);
}

int main() {
    // Configure real-time settings for the main process.
    configure_realtime();
    
    // Launch the camera processing thread.
    thread camThread(cameraThread);
    set_thread_realtime(camThread, 3, sched_get_priority_max(SCHED_FIFO));
    
    // Launch the UART thread.
    thread serialThread(uartThread);
    set_thread_realtime(serialThread, 2, sched_get_priority_max(SCHED_FIFO) - 1);
    
    // Wait for both threads.
    camThread.join();
    serialThread.join();
    
    return 0;
}
