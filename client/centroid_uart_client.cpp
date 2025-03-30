#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <vector>

using namespace cv;
using namespace std;

// =====================================================================
// UART Configuration
// =====================================================================
int configurePort(const char* port, speed_t baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "Error opening " << port << ": " << strerror(errno) << "\n";
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "tcgetattr error: " << strerror(errno) << "\n";
        return -1;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;  // Timeout for reads

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }

    return fd;
}

// =====================================================================
// Wait for START Command
// =====================================================================
void waitForStartCommand(int fd) {
    string command;
    char ch;
    cout << "Waiting for START command...\n";
    
    while (true) {
        ssize_t n = read(fd, &ch, 1);
        if (n > 0) {
            command.push_back(ch);
            if (command.find("START\n") != string::npos) {
                cout << "Received START command. Beginning processing.\n";
                break;
            }
        }
        usleep(10000);  // Sleep 10ms to avoid busy-waiting
    }
}

// =====================================================================
// Camera Capture and Centroid Detection
// =====================================================================
void processFrames(int fd) {
    const string pipeline = 
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,format=NV12,framerate=60/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 sync=false max-buffers=1";  

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!\n";
        return;
    }

    float scale_factor = 0.4;
    int threshold_value = 50;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9, 9));

    // Pre-allocate Mats
    Mat gray, binary;
    vector<vector<Point>> contours;

    while (true) {
        Mat frame;
        if (!cap.read(frame)) break;

        // Convert to grayscale, resize, and blur
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
        GaussianBlur(gray, gray, Size(3, 3), 0);

        // Apply top-hat filter
        Mat eroded, dilated, top_hat;
        erode(gray, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = gray - dilated;

        // Thresholding
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);

        // Find contours
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        Point centroid(-1, -1);

        if (!contours.empty()) {
            double max_area = 0;
            int max_index = 0;
            for (size_t i = 0; i < contours.size(); ++i) {
                double area = moments(contours[i]).m00;
                if (area > max_area) {
                    max_area = area;
                    max_index = i;
                }
            }

            Moments M = moments(contours[max_index]);
            if (M.m00 != 0) {
                centroid.x = static_cast<int>(M.m10 / M.m00 / scale_factor);
                centroid.y = static_cast<int>(M.m01 / M.m00 / scale_factor);

                // Transmit over UART
                uint16_t timestamp = static_cast<uint16_t>(
                    chrono::duration_cast<chrono::milliseconds>(
                        chrono::steady_clock::now().time_since_epoch()
                    ).count() % 65536
                );

                uint16_t data[3] = { timestamp, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y) };
                write(fd, data, sizeof(data));
            }
        }

        usleep(30000);  // Small delay to balance CPU usage
    }

    cap.release();
}

// =====================================================================
// Main Function
// =====================================================================
int main() {
    const char* port = "/dev/ttyAMA0";  
    int fd = configurePort(port, B115200);
    if (fd < 0) return 1;

    waitForStartCommand(fd);
    processFrames(fd);

    close(fd);
    return 0;
}
