#include "pico/stdlib.h"
#include "hardware/uart.h"

int main() {
    stdio_init_all();

    // UART0 → TX = GPIO0, RX = GPIO1
    uart_init(uart0, 420000);  // 420 kbps
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Simple test frame
    uint8_t test_frame[8] = {0xEE, 0x06, 0x2D, 0xEE, 0xEF, 0x06, 0x00, 0xF1};

    while (1) {
        uart_write_blocking(uart0, test_frame, sizeof(test_frame));
        sleep_ms(500);  // send every 0.5 second
    }
}
