#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <chrono>

#define DEBUG  // Uncomment this line to enable debug output

using namespace cv;
using namespace std;
using namespace std::chrono;

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
        #ifdef DEBUG
        cerr << "Error opening camera!" << endl;
        #endif
        close(uart_fd);
        return -1;
    }

    float scale_factor = 0.4f;
    int threshold_value = 32;
    Mat frame, binary;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    auto prevTime = high_resolution_clock::now();
    
    while (true) {
        auto frameStart = high_resolution_clock::now();
        cap >> frame;
        if (frame.empty()) break;
        
        // Timing for each step
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
        auto fps = 1e6 / frameTime; // Convert microseconds to FPS

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
    }

    cap.release();
    close(uart_fd);
    return 0;
}
