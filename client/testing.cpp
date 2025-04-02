#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>

#define DEBUG  // Uncomment this line to enable debug output

using namespace cv;
using namespace std;
using namespace std::chrono;

// Real-time configuration functions
void configureRealtimeSettings() {
    // Set highest priority (1-99 for SCHED_FIFO)
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // Lock all current and future memory to prevent paging
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }

    // Set CPU affinity to a single core (adjust mask as needed)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset); // Use core 0
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity failed");
    }

    // Disable memory overcommit
    system("echo 2 > /proc/sys/vm/overcommit_memory");
}

// Opens and configures the provided UART serial port for transmission only
int configurePort(const char* port, speed_t baudRate = B115200) {
    int fd = open(port, O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        #ifdef DEBUG
        cerr << "Error opening " << port << endl;
        #endif
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        #ifdef DEBUG
        cerr << "Error from tcgetattr" << endl;
        #endif
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudRate);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cflag |= CLOCAL;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        #ifdef DEBUG
        cerr << "Error from tcsetattr" << endl;
        #endif
        close(fd);
        return -1;
    }

    return fd;
}

// Function to send a 4-byte message over UART
void sendMessage(int fd, uint16_t cx, uint16_t cy) {
    unsigned char msg[4] = {
        static_cast<unsigned char>(cx & 0xFF),
        static_cast<unsigned char>((cx >> 8) & 0xFF),
        static_cast<unsigned char>(cy & 0xFF),
        static_cast<unsigned char>((cy >> 8) & 0xFF)
    };

    auto start = high_resolution_clock::now();
    ssize_t bytesWritten = write(fd, msg, sizeof(msg));
    auto end = high_resolution_clock::now();
    auto uartTime = duration_cast<microseconds>(end - start).count();

    #ifdef DEBUG
    if (bytesWritten != sizeof(msg)) {
        cerr << "Error writing to UART" << endl;
    } else {
        cout << "UART Sent: (" << cx << ", " << cy << ") | Time: " << uartTime << " us" << endl;
    }
    #endif
}

int main() {
    // Configure real-time settings
    configureRealtimeSettings();

    const char* uartPort = "/dev/ttyAMA0";
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) return -1;

    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! "
        "videoconvert ! "
        "video/x-raw,format=GRAY8 ! "
        "appsink drop=1 sync=false";

    // Pre-allocate all OpenCV mats to avoid dynamic allocation during runtime
    Mat frame, resizedFrame, blurredFrame, eroded, dilated, top_hat, binary;
    vector<vector<Point>> contours;
    Moments bestMoments;
    Point centroid(-1, -1);
    
    // Initialize with expected sizes
    const Size targetSize(1280 * 0.4, 720 * 0.4);
    resizedFrame.create(targetSize, CV_8UC1);
    blurredFrame.create(targetSize, CV_8UC1);
    eroded.create(targetSize, CV_8UC1);
    dilated.create(targetSize, CV_8UC1);
    top_hat.create(targetSize, CV_8UC1);
    binary.create(targetSize, CV_8UC1);

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        #ifdef DEBUG
        cerr << "Error opening camera!" << endl;
        #endif
        close(uart_fd);
        return -1;
    }

    const float scale_factor = 0.4f;
    const int threshold_value = 32;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    // Timing control variables
    microseconds targetFrameTime(14285); // ~70 FPS (1,000,000 us / 70)
    auto prevFrameStart = high_resolution_clock::now();
    
    while (true) {
        auto frameStart = high_resolution_clock::now();
        
        // Capture frame
        if (!cap.read(frame)) break;
        
        // Processing pipeline with enforced timing
        auto start = high_resolution_clock::now();
        resize(frame, resizedFrame, Size(), scale_factor, scale_factor, INTER_NEAREST);
        auto resizeTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        GaussianBlur(resizedFrame, blurredFrame, Size(3, 3), 0);
        auto blurTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        erode(blurredFrame, eroded, kernel);
        dilate(eroded, dilated, kernel);
        subtract(blurredFrame, dilated, top_hat);
        auto tophatTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        auto thresholdTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        auto contoursTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        double maxArea = 0;
        bestMoments = Moments();
        centroid = Point(-1, -1);

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestMoments = moments(contour);
            }
        }
        auto centroidsTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        if (bestMoments.m00 != 0) {
            centroid.x = static_cast<int>(bestMoments.m10 / bestMoments.m00 / scale_factor);
            centroid.y = static_cast<int>(bestMoments.m01 / bestMoments.m00 / scale_factor);
            
            start = high_resolution_clock::now();
            sendMessage(uart_fd, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
            auto uartTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();
        }

        auto frameEnd = high_resolution_clock::now();
        auto frameTime = duration_cast<microseconds>(frameEnd - frameStart);
        auto remainingTime = targetFrameTime - frameTime;

        // Sleep if we finished early to maintain consistent frame rate
        if (remainingTime.count() > 0) {
            this_thread::sleep_for(remainingTime);
        }

        #ifdef DEBUG
        auto actualFrameTime = duration_cast<microseconds>(high_resolution_clock::now() - frameStart).count();
        cout << "FPS: " << 1e6/actualFrameTime << endl;
        cout << "Timings (us): Resize=" << resizeTime
             << ", Blur=" << blurTime
             << ", Top-hat=" << tophatTime
             << ", Threshold=" << thresholdTime
             << ", Contours=" << contoursTime
             << ", Centroids=" << centroidsTime
             << ", Total Frame=" << actualFrameTime << endl;
        #endif
    }

    cap.release();
    close(uart_fd);
    return 0;
}
