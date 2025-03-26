#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <sys/select.h>

// --- Data Structures ---

// Sync command: a single byte value (0xAA)
const uint8_t SYNC_CMD = 0xAA;

// Client data packet: 3 x uint16_t = 6 bytes
#pragma pack(push, 1)
struct ClientData {
    uint16_t centroid1;
    uint16_t centroid2;
    uint16_t elapsedTime;  // e.g. fixed-point milliseconds since sync
};
#pragma pack(pop)

// --- UART Configuration Function ---
int configurePort(int fd, speed_t baud) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, local mode
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8-bit characters
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);  // no parity, 1 stop bit, no flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // disable software flow control
    tty.c_lflag = 0;                    // raw input
    tty.c_oflag = 0;                    // raw output

    tty.c_cc[VMIN]  = 0;                // Non-blocking read
    tty.c_cc[VTIME] = 10;               // 1 second timeout

    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

// --- Server Functions ---

// Send the sync command over both UART channels.
void sendSyncCommand(int fd0, int fd2) {
    ssize_t w0 = write(fd0, &SYNC_CMD, sizeof(SYNC_CMD));
    ssize_t w2 = write(fd2, &SYNC_CMD, sizeof(SYNC_CMD));
    if(w0 != sizeof(SYNC_CMD) || w2 != sizeof(SYNC_CMD)) {
        std::cerr << "Error sending sync command: " << strerror(errno) << "\n";
    } else {
        std::cout << "Sync command sent on both channels.\n";
    }
}

// Listen on both UART channels for client data packets.
void receiveClientData(int fd0, int fd2) {
    fd_set readfds;
    uint8_t buffer[sizeof(ClientData)];
    while (true) {
        FD_ZERO(&readfds);
        FD_SET(fd0, &readfds);
        FD_SET(fd2, &readfds);
        int maxfd = (fd0 > fd2) ? fd0 : fd2;
        
        int ret = select(maxfd + 1, &readfds, NULL, NULL, NULL);
        if(ret < 0) {
            std::cerr << "select error: " << strerror(errno) << "\n";
            break;
        }
        
        if (FD_ISSET(fd0, &readfds)) {
            ssize_t n = read(fd0, buffer, sizeof(buffer));
            if(n == sizeof(ClientData)) {
                ClientData data;
                memcpy(&data, buffer, sizeof(data));
                std::cout << "[AMA0] Received: Centroid1=" << data.centroid1 
                          << " Centroid2=" << data.centroid2 
                          << " ElapsedTime=" << data.elapsedTime << "\n";
            }
        }
        
        if (FD_ISSET(fd2, &readfds)) {
            ssize_t n = read(fd2, buffer, sizeof(buffer));
            if(n == sizeof(ClientData)) {
                ClientData data;
                memcpy(&data, buffer, sizeof(data));
                std::cout << "[AMA2] Received: Centroid1=" << data.centroid1 
                          << " Centroid2=" << data.centroid2 
                          << " ElapsedTime=" << data.elapsedTime << "\n";
            }
        }
    }
}

// --- Main Function (Server) ---
int main() {
    // Define device paths for AMA0 and AMA2
    const char* portAMA0 = "/dev/ttyAMA0"; // Primary UART (PL011)
    const char* portAMA2 = "/dev/ttyAMA2"; // Extra UART (from overlay, e.g., uart2)

    int fd0 = open(portAMA0, O_RDWR | O_NOCTTY | O_SYNC);
    int fd2 = open(portAMA2, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd0 < 0 || fd2 < 0) {
        std::cerr << "Error opening UART ports: " << strerror(errno) << "\n";
        return 1;
    }
    
    if(configurePort(fd0, B115200) != 0 || configurePort(fd2, B115200) != 0) {
        close(fd0);
        close(fd2);
        return 1;
    }
    
    // Send one-time sync command on both channels
    sendSyncCommand(fd0, fd2);
    
    // Now continuously receive client data on both channels
    receiveClientData(fd0, fd2);
    
    close(fd0);
    close(fd2);
    return 0;
}
