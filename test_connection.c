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

    // CRSFフレームテンプレート（25バイト）
    uint8_t frame_template[25] = {
        0xEE, 0x18, 0x16, 0xD2,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };

    uint16_t SW_L = 200;   // AUX LED OFF
    uint16_t SW_H = 1700;  // AUX LED ON

    printf("*** Start Nano TX AUX LED Test ***\n");

    while (1) {
        // AUX1をLOWにセットして送信
        frame_template[3] = SW_L & 0xFF;
        frame_template[4] = (SW_L >> 8) & 0xFF;
        write(fd, frame_template, sizeof(frame_template));
        printf("AUX1 = LOW\n");
        usleep(500000); // 0.5秒

        // AUX1をHIGHにセットして送信
        frame_template[3] = SW_H & 0xFF;
        frame_template[4] = (SW_H >> 8) & 0xFF;
        write(fd, frame_template, sizeof(frame_template));
        printf("AUX1 = HIGH\n");
        usleep(500000); // 0.5秒
    }

    close(fd);
    return 0;
}
