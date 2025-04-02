#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>

// Uncomment the following line to enable debug output
//#define DEBUG

using namespace cv;
using namespace std;

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
    unsigned char msg[4];
    msg[0] = cx & 0xFF;
    msg[1] = (cx >> 8) & 0xFF;
    msg[2] = cy & 0xFF;
    msg[3] = (cy >> 8) & 0xFF;

    ssize_t bytesWritten = write(fd, msg, sizeof(msg));
    #ifdef DEBUG
    if (bytesWritten != sizeof(msg)) {
        cerr << "Error writing to UART" << endl;
    } else {
        cout << "Transmitted: cx=" << cx << ", cy=" << cy << endl;
    }
    #endif
}

int main() {
    const char* uartPort = "/dev/ttyAMA0";
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) return -1;

    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12 ! "
        "videoconvert ! "
        "video/x-raw,format=GRAY8 ! "
        "appsink drop=1";

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

    #ifdef DEBUG
    double prevTick = (double)getTickCount();
    #endif

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        #ifdef DEBUG
        double t1 = (double)getTickCount();
        #endif

        resize(frame, frame, Size(), scale_factor, scale_factor, INTER_NEAREST);

        #ifdef DEBUG
        double t2 = (double)getTickCount();
        cout << "Resize time: " << (t2 - t1) / getTickFrequency() * 1000 << " ms" << endl;
        #endif

        GaussianBlur(frame, frame, Size(3, 3), 0);

        #ifdef DEBUG
        double t3 = (double)getTickCount();
        cout << "GaussianBlur time: " << (t3 - t2) / getTickFrequency() * 1000 << " ms" << endl;
        #endif

        Mat eroded, dilated, top_hat;
        erode(frame, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = frame - dilated;

        #ifdef DEBUG
        double t4 = (double)getTickCount();
        cout << "Top-hat transform time: " << (t4 - t3) / getTickFrequency() * 1000 << " ms" << endl;
        #endif

        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        #ifdef DEBUG
        double t5 = (double)getTickCount();
        cout << "Thresholding time: " << (t5 - t4) / getTickFrequency() * 1000 << " ms" << endl;
        #endif

        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        #ifdef DEBUG
        double t6 = (double)getTickCount();
        cout << "Contour detection time: " << (t6 - t5) / getTickFrequency() * 1000 << " ms" << endl;
        #endif

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

            #ifdef DEBUG
            double currentTick = (double)getTickCount();
            double fps = getTickFrequency() / (currentTick - prevTick);
            prevTick = currentTick;
            cout << "FPS: " << fps << endl;
            cout << "Centroid detected at: (" << centroid.x << ", " << centroid.y << ")" << endl;
            #endif

            sendMessage(uart_fd, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
        }
    }

    cap.release();
    close(uart_fd);
    return 0;
}
