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
    // Open for write-only, since we are only transmitting
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

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit characters
    tty.c_iflag = 0;                             // disable input processing (not needed for TX)
    tty.c_oflag = 0;                             // no remapping, no delays
    tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tty.c_cflag |= CLOCAL;                       // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);           // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;                     // no hardware flow control

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
// Message layout: [centroid.x (2 bytes), centroid.y (2 bytes)]
void sendMessage(int fd, uint16_t cx, uint16_t cy) {
    unsigned char msg[4];
    // Pack the centroid values in little-endian order
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
    // ---------------------- UART Transmitter Initialization ----------------------
    const char* uartPort = "/dev/ttyAMA0"; // Transmitter port
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) {
        return -1;
    }

    // ---------------------- Camera Capture Initialization ----------------------
    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        #ifdef DEBUG
        cerr << "Error opening camera!" << endl;
        #endif
        close(uart_fd);
        return -1;
    }

    // Parameters for image processing
    float scale_factor = 0.4f;  // Downsample factor
    int threshold_value = 32;   // Adjust based on your needs
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    while (true) {
        Mat frame;
        cap >> frame;
        if (frame.empty())
            break;

        // Processing pipeline: convert to grayscale, downsample, and blur
        Mat gray, binary;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(gray, gray, Size(3, 3), 0);

        // Top-hat transform to emphasize features
        Mat eroded, dilated, top_hat;
        erode(gray, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = gray - dilated;

        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Find contours
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Find largest contour and calculate its centroid
        Point centroid(-1, -1);
        if (!contours.empty()) {
            auto max_it = max_element(contours.begin(), contours.end(),
                [](const vector<Point>& a, const vector<Point>& b) {
                    return contourArea(a) < contourArea(b);
                });
            Moments M = moments(*max_it);
            if (M.m00 != 0) {
                // Scale back the centroid to original resolution
                centroid.x = static_cast<int>(M.m10 / M.m00 / scale_factor);
                centroid.y = static_cast<int>(M.m01 / M.m00 / scale_factor);
                #ifdef DEBUG
                cout << "Centroid detected at: (" << centroid.x
                     << ", " << centroid.y << ")" << endl;
                #endif
                // Transmit the centroid values over UART
                sendMessage(uart_fd, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
            }
        }
    }

    // Cleanup
    cap.release();
    close(uart_fd);
    return 0;
}
