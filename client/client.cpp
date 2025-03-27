// ===================================================================
// Section: Imports
// ===================================================================

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>
#include <string>

// ===================================================================
// Section: UART Functions
// ===================================================================

int configurePort(int fd, speed_t baud) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;  // 8-bit characters
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;  // 1 second timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

// Wait for "Enter" to be pressed on server RPi
void waitForStartCommand(int fd) {
    std::string command;
    char ch;
    std::cout << "Client 1: Waiting for start command from server...\n";
    while (true) {
        ssize_t n = read(fd, &ch, 1);
        if (n > 0) {
            command.push_back(ch);
            // Look for "START" in the received string.
            if (command.find("START") != std::string::npos) {
                std::cout << "Client 1: Received start command: " << command;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Transmits a single 6-byte message containing timestamp and centroids.
void transmitSensorData(int fd, const std::chrono::steady_clock::time_point& start, uint16_t centroid1, uint16_t centroid2) {
    // Calculate elapsed time since start.
    auto now = std::chrono::steady_clock::now();
    uint16_t elapsed = static_cast<uint16_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
    );
    
    // Prepare the 6-byte message: [time, centroid1, centroid2]
    uint16_t data[3] = { elapsed, centroid1, centroid2 };
    ssize_t written = write(fd, data, sizeof(data));
    if (written != sizeof(data)) {
        std::cerr << "Write error: " << strerror(errno) << "\n";
    } else {
        std::cout << "Sent: time=" << elapsed 
                  << " ms, centroid1=" << centroid1 
                  << ", centroid2=" << centroid2 << "\n";
    }
}

// ===================================================================
// Section: Main
// ===================================================================

int main() {

    // Opens UART Port, Error Handling
    const char* port = "/dev/ttyAMA0";
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);  // Open in read-write mode.
    if (fd < 0) {
        std::cerr << "Error opening " << port << ": " << strerror(errno) << "\n";
        return 1;
    }
    if (configurePort(fd, B115200) != 0) {
        close(fd);
        return 1;
    }

    // Wait for "Enter" to be pressed on server RPi
    waitForStartCommand(fd);

    // Record the start time.
    auto start = std::chrono::steady_clock::now();
    
    // Begin transmitting sensor data with dummy centroid values.
    transmitSensorData(fd, start, 100, 200);  // For Client 1, dummy centroids.
    
    close(fd);
    return 0;
}
