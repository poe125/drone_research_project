#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define MSP_BEEPER_COMMAND 184  // Betaflight MSP code for beeper
#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE B460800

// Helper to send MSP command
void send_msp_command(int fd, uint8_t cmd, uint8_t *payload, uint8_t length) {
    uint8_t header[] = {'$', 'M', '<'};
    uint8_t checksum = 0;
    uint8_t packet[256];
    int i = 0;

    // Build packet
    memcpy(packet + i, header, 3); i += 3;
    packet[i++] = length;
    packet[i++] = cmd;

    checksum = length ^ cmd;
    for (int j = 0; j < length; j++) {
        packet[i++] = payload[j];
        checksum ^= payload[j];
    }
    packet[i++] = checksum;

    // Send it
    write(fd, packet, i);
    tcdrain(fd);  // Wait for transmission
}

int main(void) {
    int fd;
    struct termios options;

    // Open serial
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return 1;
    }

    fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &options);

    // --- Send beeper command ---
    uint8_t beep_payload[1] = {1};  // 1 = turn on, 0 = off
    send_msp_command(fd, MSP_BEEPER_COMMAND, beep_payload, 1);

    printf("Beep command sent!\n");

    close(fd);
    return 0;
}
