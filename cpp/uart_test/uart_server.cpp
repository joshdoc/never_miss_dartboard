#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <sys/select.h>

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
    
    // Set VMIN to 0 and VTIME to 10 (1 sec timeout)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

int main() {
    const char* port0 = "/dev/ttyAMA0"; // Primary UART (PL011)
    const char* port2 = "/dev/ttyAMA2"; // Additional UART channel (via overlay)
    
    int fd0 = open(port0, O_RDONLY | O_NOCTTY | O_SYNC);
    int fd2 = open(port2, O_RDONLY | O_NOCTTY | O_SYNC);
    if (fd0 < 0 || fd2 < 0) {
        std::cerr << "Error opening ports: " << strerror(errno) << "\n";
        return 1;
    }
    
    if (configurePort(fd0, B115200) != 0 || configurePort(fd2, B115200) != 0) {
        close(fd0);
        close(fd2);
        return 1;
    }
    
    std::cout << "Listening on " << port0 << " and " << port2 << "...\n";
    
    uint16_t sensorData;
    fd_set readfds;
    while (true) {
        FD_ZERO(&readfds);
        FD_SET(fd0, &readfds);
        FD_SET(fd2, &readfds);
        int maxfd = (fd0 > fd2) ? fd0 : fd2;
        
        // Wait indefinitely until data is available on either channel
        int ret = select(maxfd + 1, &readfds, NULL, NULL, NULL);
        if (ret < 0) {
            std::cerr << "select error: " << strerror(errno) << "\n";
            break;
        }
        
        if (FD_ISSET(fd0, &readfds)) {
            ssize_t n = read(fd0, &sensorData, sizeof(sensorData));
            if (n == sizeof(sensorData)) {
                std::cout << "Received on AMA0: " << sensorData << "\n";
            }
        }
        
        if (FD_ISSET(fd2, &readfds)) {
            ssize_t n = read(fd2, &sensorData, sizeof(sensorData));
            if (n == sizeof(sensorData)) {
                std::cout << "Received on AMA2: " << sensorData << "\n";
            }
        }
    }
    
    close(fd0);
    close(fd2);
    return 0;
}