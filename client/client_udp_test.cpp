#include <opencv2/opencv.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <arpa/inet.h>
#include <cstring>

#define LOOP_PERIOD_NS 22000000  // ~45 FPS loop period (22 ms)
#define DEST_IP "172.20.10.9"    // Receiver (laptop) IP
#define DEST_PORT 4600           // Chosen free UDP port
// #define DEBUG

using namespace cv;
using namespace std;

// Real-time scheduling setup
void setupRealtime() {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) < 0)
        cerr << "Failed to set real-time scheduler" << endl;

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        cerr << "Failed to lock memory" << endl;
}

// Setup UDP socket
int setupUDPSocket(sockaddr_in &dest_addr) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);
    inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr);

    return sockfd;
}

// Send centroid over UDP with a unique Pi ID
void sendMessage(int sockfd, sockaddr_in &dest_addr, uint8_t pi_id, uint16_t cx, uint16_t cy) {
    uint8_t data[5];
    data[0] = pi_id;
    uint16_t net_cx = htons(cx);
    uint16_t net_cy = htons(cy);
    memcpy(data + 1, &net_cx, sizeof(net_cx));
    memcpy(data + 3, &net_cy, sizeof(net_cy));

    sendto(sockfd, data, sizeof(data), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

int main(int argc, char** argv) {
    setupRealtime();

    // Default values
    int threshold_value = 32;
    uint8_t PI_ID = 1;

    // Parse command-line arguments
    if (argc > 1) {
        int parsed_threshold = atoi(argv[1]);
        if (parsed_threshold >= 0 && parsed_threshold <= 255) {
            threshold_value = parsed_threshold;
        } else {
            cerr << "Invalid threshold value. Must be between 0 and 255." << endl;
            return -1;
        }
    }

    if (argc > 2) {
        int parsed_id = atoi(argv[2]);
        if (parsed_id >= 1 && parsed_id <= 2) {
            PI_ID = static_cast<uint8_t>(parsed_id);
        } else {
            cerr << "Invalid PI_ID. Must be 1 or 2." << endl;
            return -1;
        }
    }

    if (argc > 3) {
        cout << "Usage: " << argv[0] << " [threshold (0-255)] [PI_ID (1-2)]" << endl;
        return 0;
    }

    cout << "Using threshold: " << threshold_value << ", PI_ID: " << (int)PI_ID << endl;

    sockaddr_in dest_addr;
    int udp_sock = setupUDPSocket(dest_addr);

    const std::string pipeline = "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12,framerate=70/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Failed to open camera stream." << endl;
        close(udp_sock);
        return -1;
    }

    float scale_factor = 0.4f;
    Mat frame, binary, kernel = getStructuringElement(MORPH_RECT, Size(9, 9));

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);

    while (true) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
        next_time.tv_nsec += LOOP_PERIOD_NS;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_nsec -= 1000000000;
            next_time.tv_sec++;
        }

        cap >> frame;
        if (frame.empty()) break;

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
            sendMessage(udp_sock, dest_addr, PI_ID, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
        }

        #ifdef DEBUG
        cout << "Centroid: " << centroid << " from Pi " << (int)PI_ID << endl;
        #endif
    }

    cap.release();
    close(udp_sock);
    return 0;
}

