#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>

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
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

int main() {
    const char* port = "/dev/ttyAMA0"; // Client 1 uses AMA0
    int fd = open(port, O_WRONLY | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << port << ": " << strerror(errno) << "\n";
        return 1;
    }

    if (configurePort(fd, B115200) != 0) {
        close(fd);
        return 1;
    }

    uint16_t sensorData = 1234; // Example value
    while (true) {
        ssize_t written = write(fd, &sensorData, sizeof(sensorData));
        if (written != sizeof(sensorData)) {
            std::cerr << "Write error: " << strerror(errno) << "\n";
        } else {
            std::cout << "Client 1 Sent: " << sensorData << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    close(fd);
    return 0;
}
