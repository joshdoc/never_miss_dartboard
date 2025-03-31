#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <vector>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;
using namespace std::chrono;

// Function to configure and open the serial port on AMA0
int configurePort(const char* port, speed_t baudRate = B115200) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        cerr << "Error opening " << port << endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        cerr << "Error from tcgetattr" << endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;    // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Error from tcsetattr" << endl;
        close(fd);
        return -1;
    }
    return fd;
}

// Function to send a 6-byte message over UART
// Message layout: [timestamp (2 bytes), centroid.x (2 bytes), centroid.y (2 bytes)]
void sendMessage(int fd, uint16_t timestamp, uint16_t cx, uint16_t cy) {
    unsigned char msg[6];
    // Pack values in little-endian order
    msg[0] = timestamp & 0xFF;
    msg[1] = (timestamp >> 8) & 0xFF;
    msg[2] = cx & 0xFF;
    msg[3] = (cx >> 8) & 0xFF;
    msg[4] = cy & 0xFF;
    msg[5] = (cy >> 8) & 0xFF;

    ssize_t bytesWritten = write(fd, msg, sizeof(msg));
    if (bytesWritten != sizeof(msg)) {
        cerr << "Error writing to UART" << endl;
    } else {
        cout << "Transmitted: ts=" << timestamp
             << ", cx=" << cx << ", cy=" << cy << endl;
    }
}

int main() {
    // ---------------------- UART Transmitter Initialization ----------------------
    const char* uartPort = "/dev/ttyAMA0"; // Transmitter port
    int uart_fd = configurePort(uartPort);
    if (uart_fd < 0) {
        return -1;
    }

    // ---------------------- Camera Capture Initialization ----------------------
    // GStreamer pipeline configuration
    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,format=NV12 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "Error opening camera!" << endl;
        close(uart_fd);
        return -1;
    }

    // Parameters for image processing
    float scale_factor = 0.4f;  // Downsample factor
    int threshold_value = 32;   // Adjust based on your needs
    Mat kernel = getStructuringElement(MORPH_RECT, Size(9,9));

    // Global start time for timestamp calculations
    auto globalStartTime = steady_clock::now();

    int cycleCount = 0;

    cout << "System ready." << endl;
    cout << "Press Enter to start a detection cycle (or Ctrl+C to exit)..." << endl;

    // Main loop for repeatable detection cycles
    while (true) {
        // Wait for Enter key press from the terminal.
        cin.get();
        cycleCount++;
        cout << "\n--- Detection Cycle " << cycleCount << " started ---" << endl;

        // Vector to store detection data: each element is a tuple (timestamp, centroid.x, centroid.y)
        vector<tuple<uint16_t, int, int>> detections;

        // Record the cycle start time and initialize lastDetectionTime
        auto cycleStartTime = steady_clock::now();
        auto lastDetectionTime = cycleStartTime;

        // Continue processing frames until a 2-second timeout occurs without any detection.
        while (true) {
            Mat frame;
            cap >> frame;
            if (frame.empty())
                break;

            // Processing pipeline: convert to grayscale and downsample
            Mat gray, binary;
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            resize(gray, gray, Size(), scale_factor, scale_factor, INTER_NEAREST);
            GaussianBlur(gray, gray, Size(3, 3), 0);

            // Top-hat transform
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
            bool detected = false;
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
                    detected = true;
                }
            }

            if (detected) {
                // Update last detection time and record the detection
                lastDetectionTime = steady_clock::now();
                auto now = steady_clock::now();
                uint16_t timestamp = static_cast<uint16_t>(duration_cast<milliseconds>(now - globalStartTime).count() & 0xFFFF);
                detections.push_back(make_tuple(timestamp, centroid.x, centroid.y));

                cout << "Centroid detected at: (" << centroid.x << ", " << centroid.y << ")"
                     << " at timestamp " << timestamp << " ms" << endl;
                
                // Transmit the message over UART
                sendMessage(uart_fd, timestamp, static_cast<uint16_t>(centroid.x),
                            static_cast<uint16_t>(centroid.y));
            }

            // Check for 2 second timeout (no centroid detected for 2 seconds)
            auto currentTime = steady_clock::now();
            if (duration_cast<seconds>(currentTime - lastDetectionTime).count() >= 2) {
                cout << "2-second timeout reached (no new centroid detected)." << endl;
                break;
            }

            // Optional: Display the binary image if needed (requires uncommenting imshow and waitKey)
            // imshow("Binary Feed", binary);
            // if (waitKey(1) == 27) // ESC to exit cycle early
            //     break;
        }

        // Save detections to a text file if any were captured
        if (!detections.empty()) {
            stringstream filename;
            filename << "centroids_cycle_" << cycleCount << ".txt";
            ofstream outfile(filename.str());
            if (outfile.is_open()) {
                outfile << "Timestamp (ms), Centroid X, Centroid Y\n";
                for (const auto& det : detections) {
                    outfile << std::get<0>(det) << ", " 
                            << std::get<1>(det) << ", " 
                            << std::get<2>(det) << "\n";
                }
                outfile.close();
                cout << "Detection cycle " << cycleCount << " complete. Data saved to " << filename.str() << endl;
            } else {
                cerr << "Error saving to file " << filename.str() << endl;
            }
        } else {
            cout << "No detections were captured during cycle " << cycleCount << "." << endl;
        }

        cout << "\nPress Enter to start a new detection cycle, or Ctrl+C to exit." << endl;
    }

    // Cleanup
    cap.release();
    // destroyAllWindows();
    close(uart_fd);
    return 0;
}
