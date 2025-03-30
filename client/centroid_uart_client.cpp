#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <cstdint>
#include <cstdlib>

using namespace std;
using namespace std::chrono;

// Configure the UART port (AMA0) at 115200 baud.
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
        close(fd);
        return -1;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;               // 8-bit characters
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;             // 1-second read timeout
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "tcsetattr error: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }
    
    return fd;
}

int main() {
    const char* port = "/dev/ttyAMA0";
    int fd = configurePort(port, B115200);
    if (fd < 0) {
        return EXIT_FAILURE;
    }
    
    // Wait for the user to type "start" (or just press ENTER)
    cout << "Press ENTER (or type 'start' then ENTER) to begin transmission..." << endl;
    string input;
    getline(cin, input);
    
    cout << "Start command received. Beginning transmission..." << endl;
    
    // Dummy values for centroid; you can replace these later.
    const uint16_t dummyCentroid1 = 100;
    const uint16_t dummyCentroid2 = 200;
    
    // Get the starting time.
    auto start_time = steady_clock::now();
    
    while (true) {
        // Calculate elapsed time in milliseconds, wrapped to 16 bits.
        auto now = steady_clock::now();
        uint16_t elapsed_ms = static_cast<uint16_t>(
            duration_cast<milliseconds>(now - start_time).count() % 65536
        );
        
        // Prepare the packet: [elapsed_ms, dummyCentroid1, dummyCentroid2]
        uint16_t packet[3] = { elapsed_ms, dummyCentroid1, dummyCentroid2 };
        
        ssize_t written = write(fd, packet, sizeof(packet));
        if (written != sizeof(packet)) {
            cerr << "Error writing packet: " << strerror(errno) << "\n";
        } else {
            cout << "Transmitted: Time=" << elapsed_ms 
                 << " ms, Centroid1=" << dummyCentroid1 
                 << ", Centroid2=" << dummyCentroid2 << "\n";
        }
        
        // Transmit once per second.
        this_thread::sleep_for(seconds(1));
    }
    
    close(fd);
    return 0;
}
