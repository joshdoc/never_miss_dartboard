#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#define DEBUG  // Enable debug output

using namespace cv;
using namespace std;
using namespace std::chrono;

// ------------------- Real-Time Setup -------------------
void setRealtimePriority() {
    struct sched_param param;
    param.sched_priority = 99; // Max priority for SCHED_FIFO
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        cerr << "Failed to set real-time priority!" << endl;
    }
    // Lock memory to avoid swapping
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        cerr << "Failed to lock memory!" << endl;
    }
    // Pin process to core 3 for minimal OS interference
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        cerr << "Failed to set CPU affinity!" << endl;
    }
}

// ------------------- UART Configuration -------------------
int configurePort(const char* port, speed_t baudRate = B115200) {
    int fd = open(port, O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;
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
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return fd;
}

void sendMessage(int fd, uint16_t cx, uint16_t cy) {
    unsigned char msg[4] = {
        static_cast<unsigned char>(cx & 0xFF),
        static_cast<unsigned char>((cx >> 8) & 0xFF),
        static_cast<unsigned char>(cy & 0xFF),
        static_cast<unsigned char>((cy >> 8) & 0xFF)
    };
    auto start = high_resolution_clock::now();
    write(fd, msg, sizeof(msg));
    auto end = high_resolution_clock::now();
    auto uartTime = duration_cast<microseconds>(end - start).count();
    #ifdef DEBUG
    cout << "UART Sent: (" << cx << ", " << cy << ") | Time: " << uartTime << " us" << endl;
    #endif
}

int main() {
    setRealtimePriority(); // Apply real-time settings

    const char* uartPort = "/dev/ttyAMA0";
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) return -1;

    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! "
        "videoconvert ! "
        "video/x-raw,format=GRAY8 ! "
        "appsink drop=1 sync=false";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        close(uart_fd);
        return -1;
    }

    float scale_factor = 0.4f;
    int threshold_value = 32;
    Mat frame, binary;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    auto frameInterval = microseconds(1000000 / 70); // 70 FPS target
    auto nextFrameTime = high_resolution_clock::now();

    while (true) {
        auto frameStart = high_resolution_clock::now();
        
        cap >> frame;
        if (frame.empty()) break;

        // Processing steps with timing
        auto start = high_resolution_clock::now();
        resize(frame, frame, Size(), scale_factor, scale_factor, INTER_NEAREST);
        auto resizeTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        GaussianBlur(frame, frame, Size(3, 3), 0);
        auto blurTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        Mat eroded, dilated, top_hat;
        erode(frame, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = frame - dilated;
        auto tophatTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        auto thresholdTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        auto contoursTime = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        double maxArea = 0;
        Point centroid(-1, -1);
        Moments bestMoments;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestMoments = moments(contour);
            }
        }
        auto centroids = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        start = high_resolution_clock::now();
        if (bestMoments.m00 != 0) {
            centroid.x = static_cast<int>(bestMoments.m10 / bestMoments.m00 / scale_factor);
            centroid.y = static_cast<int>(bestMoments.m01 / bestMoments.m00 / scale_factor);
            sendMessage(uart_fd, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
        }
        auto uart = duration_cast<microseconds>(high_resolution_clock::now() - start).count();

        auto frameEnd = high_resolution_clock::now();
        auto frameTime = duration_cast<microseconds>(frameEnd - frameStart).count();
        auto fps = 1e6 / frameTime;

        #ifdef DEBUG
        cout << "FPS: " << fps << endl;
        cout << "Timings (us): Resize=" << resizeTime
             << ", Blur=" << blurTime
             << ", Top-hat=" << tophatTime
             << ", Threshold=" << thresholdTime
             << ", Contours=" << contoursTime
             << ", Centroids=" << centroids
             << ", UART=" << uart
             << ", Total Frame=" << frameTime << endl;
        #endif

        // Sleep to maintain consistent frame timing
        nextFrameTime += frameInterval;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(timespec){nextFrameTime.time_since_epoch().count() / 1000000000, nextFrameTime.time_since_epoch().count() % 1000000000}, NULL);
    }

    cap.release();
    close(uart_fd);
    return 0;
}
