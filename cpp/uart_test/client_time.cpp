#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <thread>

// --- Data Structures ---

// Sync command
const uint8_t SYNC_CMD = 0xAA;

// Client data packet structure (6 bytes total)
#pragma pack(push, 1)
struct ClientData {
    uint16_t centroid1;
    uint16_t centroid2;
    uint16_t elapsedTime; // e.g., fixed-point value (milliseconds)
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
    
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // 1-second timeout
    
    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr error: " << strerror(errno) << "\n";
        return -1;
    }
    return 0;
}

// --- Client Functions ---

// Wait until the sync command is received on fd.
bool waitForSync(int fd) {
    uint8_t byte;
    std::cout << "Waiting for sync command...\n";
    while (true) {
        ssize_t n = read(fd, &byte, sizeof(byte));
        if (n > 0) {
            if (byte == SYNC_CMD) {
                std::cout << "Sync command received.\n";
                return true;
            }
        }
        // Sleep briefly to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

// Send a client data packet with given values.
void sendClientData(int fd, uint16_t centroid1, uint16_t centroid2, uint16_t elapsedTime) {
    ClientData packet;
    packet.centroid1 = centroid1;
    packet.centroid2 = centroid2;
    packet.elapsedTime = elapsedTime;
    
    ssize_t n = write(fd, &packet, sizeof(packet));
    if (n != sizeof(packet)) {
        std::cerr << "Write error: " << strerror(errno) << "\n";
    } else {
        std::cout << "Sent packet: C1=" << packet.centroid1 
                  << " C2=" << packet.centroid2 
                  << " T=" << packet.elapsedTime << "\n";
    }
}

// --- Main Function (Client) ---
int main() {
    // Client uses AMA0 for both receiving sync and sending data
    const char* port = "/dev/ttyAMA0";
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0) {
        std::cerr << "Error opening " << port << ": " << strerror(errno) << "\n";
        return 1;
    }
    
    if(configurePort(fd, B115200) != 0) {
        close(fd);
        return 1;
    }
    
    // Wait for sync command from server
    waitForSync(fd);
    
    // Capture sync time (using steady_clock for elapsed time)
    auto syncTime = std::chrono::steady_clock::now();
    
    // Now, routinely send data packets.
    while(true) {
        // For demonstration, these values are static or can be updated from your centroid detection
        uint16_t centroid1 = 100;  // e.g., x position
        uint16_t centroid2 = 200;  // e.g., y position
        
        // Calculate elapsed time in milliseconds since sync
        auto now = std::chrono::steady_clock::now();
        uint16_t elapsedTime = static_cast<uint16_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - syncTime).count() & 0xFFFF);
        
        sendClientData(fd, centroid1, centroid2, elapsedTime);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    close(fd);
    return 0;
}
