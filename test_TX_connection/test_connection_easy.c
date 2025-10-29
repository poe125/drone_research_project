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
    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    tcsetattr(fd, TCSANOW, &options);

    uint8_t ch_data[26] = {
        0xEE,0x18,0x16,0xD2,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00
    };

    // AUX1のチャンネル値 (LOW/HIGH)
    uint16_t SW_L = 200;
    uint16_t SW_H = 1700;

    printf("*** Start AUX1 LED Blink Test ***\n");

    while (1) {
        // AUX1 LOW
        ch_data[3] = SW_L & 0xFF;
        ch_data[4] = (SW_L >> 8) & 0xFF;
        write(fd, ch_data, sizeof(ch_data));
        printf("AUX1 = LOW\n");
        usleep(500000); // 0.5秒

        // AUX1 HIGH
        ch_data[3] = SW_H & 0xFF;
        ch_data[4] = (SW_H >> 8) & 0xFF;
        write(fd, ch_data, sizeof(ch_data));
        printf("AUX1 = HIGH\n");
        usleep(500000); // 0.5秒
    }

    close(fd);
    return 0;
}
