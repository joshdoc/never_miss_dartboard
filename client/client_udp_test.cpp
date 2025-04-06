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
#define DEST_IP "172.20.10.9"  // Receiver (laptop) IP
#define DEST_PORT 4600           // Chosen free UDP port
#define DEBUG

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

// Send centroid over UDP
void sendMessage(int sockfd, sockaddr_in &dest_addr, uint16_t cx, uint16_t cy) {
    uint8_t data[4];
    uint16_t net_cx = htons(cx);
    uint16_t net_cy = htons(cy);
    memcpy(data, &net_cx, sizeof(net_cx));
    memcpy(data + sizeof(net_cx), &net_cy, sizeof(net_cy));

    sendto(sockfd, data, sizeof(data), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

int main() {
    setupRealtime();

    sockaddr_in dest_addr;
    int udp_sock = setupUDPSocket(dest_addr);

    const std::string pipeline = "libcamerasrc ! video/x-raw,width=1280,height=720,format=NV12 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink drop=1";
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Failed to open camera stream." << endl;
        close(udp_sock);
        return -1;
    }

    float scale_factor = 0.4f;
    int threshold_value = 32;
    Mat frame, binary, kernel = getStructuringElement(MORPH_RECT, Size(9, 9));

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

        struct timespec t1, t2;

        // Frame capture
        clock_gettime(CLOCK_MONOTONIC, &t1);
        cap >> frame;
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double cap_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        if (frame.empty()) break;

        // Resize
        clock_gettime(CLOCK_MONOTONIC, &t1);
        resize(frame, frame, Size(), scale_factor, scale_factor, INTER_NEAREST);
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double resize_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        // Gaussian Blur
        clock_gettime(CLOCK_MONOTONIC, &t1);
        GaussianBlur(frame, frame, Size(3, 3), 0);
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double blur_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        // Top-hat (Erode -> Dilate -> Subtract)
        clock_gettime(CLOCK_MONOTONIC, &t1);
        Mat eroded, dilated, top_hat;
        erode(frame, eroded, kernel);
        dilate(eroded, dilated, kernel);
        top_hat = frame - dilated;
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double tophat_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        // Thresholding + Contour detection
        clock_gettime(CLOCK_MONOTONIC, &t1);
        threshold(top_hat, binary, threshold_value, 255, THRESH_BINARY);
        vector<vector<Point>> contours;
        findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double thresh_contour_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        // Centroid calc + UDP send
        clock_gettime(CLOCK_MONOTONIC, &t1);
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
            sendMessage(udp_sock, dest_addr, static_cast<uint16_t>(centroid.x), static_cast<uint16_t>(centroid.y));
        }
        clock_gettime(CLOCK_MONOTONIC, &t2);
        double centroid_udp_time = (t2.tv_nsec - t1.tv_nsec) / 1e6 + (t2.tv_sec - t1.tv_sec) * 1000;

        #ifdef DEBUG
        cout << fixed;
        cout << "Capture Time: " << cap_time << " ms" << endl;
        cout << "Resize Time: " << resize_time << " ms" << endl;
        cout << "Gaussian Blur Time: " << blur_time << " ms" << endl;
        cout << "Top-Hat Time: " << tophat_time << " ms" << endl;
        cout << "Threshold + Contour Time: " << thresh_contour_time << " ms" << endl;
        cout << "Centroid + UDP Send Time: " << centroid_udp_time << " ms" << endl;
        cout << "----------------------------------------" << endl;
        #endif
    }

    cap.release();
    close(udp_sock);
    return 0;
}
