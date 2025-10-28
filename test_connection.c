#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

int main() {
    const char* portname = "/dev/ttyACM0";
    int fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return 1; }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B420000);
    cfsetospeed(&options, B420000);
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    tcsetattr(fd, TCSANOW, &options);

    uint8_t test_frame[8] = {0xEE, 0x06, 0x2D, 0xEE, 0xEF, 0x06, 0x00, 0xF1};

    while (1) {
        write(fd, test_frame, sizeof(test_frame));
        printf("Sent test frame\n");
        usleep(500000); // 0.5 sec
    }

    close(fd);
    return 0;
}
