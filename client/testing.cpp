#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#define LOOP_PERIOD_NS 16666667  // ~60 FPS loop period (16.67ms)
#define DEBUG

using namespace cv;
using namespace std;

void setupRealtime() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // Set highest priority
    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
        cerr << "Failed to set real-time scheduler" << endl;
    }
    
    // Lock memory to prevent paging
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        cerr << "Failed to lock memory" << endl;
    }
}

int configurePort(const char* port, speed_t baudRate = B115200) {
    int fd = open(port, O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
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
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

void sendMessage(int fd, uint16_t cx, uint16_t cy) {
    unsigned char msg[4] = { cx & 0xFF, (cx >> 8) & 0xFF, cy & 0xFF, (cy >> 8) & 0xFF };
    write(fd, msg, sizeof(msg));
}

int main() {
    setupRealtime();

    const char* uartPort = "/dev/ttyAMA0";
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) return -1;

    const std::string pipeline = "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        close(uart_fd);
        return -1;
    }

    float scale_factor = 0.4f;
    int threshold_value = 32;
    Mat frame, binary, kernel = getStructuringElement(MORPH_RECT, Size(9,9));
    
    struct timespec next_time, start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (true) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        next_time.tv_nsec += LOOP_PERIOD_NS;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_nsec -= 1000000000;
            next_time.tv_sec++;
        }

        cap >> frame;
        if (frame.empty()) break;

        struct timespec process_start, process_end;
        clock_gettime(CLOCK_MONOTONIC, &process_start);

        resize(frame, frame, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(frame, frame, Size(3, 3), 0);

        Mat eroded, dilated, top_hat;
        erode(frame, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = frame - dilated;

        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

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

        if (bestMoments.m00 != 0) {
            centroid.x = static_cast<int>(bestMoments.m10 / bestMoments.m00 / scale_factor);
            centroid.y = static_cast<int>(bestMoments.m01 / bestMoments.m00 / scale_factor);
            sendMessage(uart_fd, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
        }

        clock_gettime(CLOCK_MONOTONIC, &process_end);
        #ifdef DEBUG
        double processing_time = (process_end.tv_sec - process_start.tv_sec) * 1000.0 + (process_end.tv_nsec - process_start.tv_nsec) / 1000000.0;
        double total_time = (process_end.tv_sec - start_time.tv_sec) * 1000.0 + (process_end.tv_nsec - start_time.tv_nsec) / 1000000.0;
        cout << "Processing Time: " << processing_time << " ms" << endl;
        cout << "Total Loop Execution Time: " << total_time << " ms" << endl;
        #endif
    }
    cap.release();
    close(uart_fd);
    return 0;
}
