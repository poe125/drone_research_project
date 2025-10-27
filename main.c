#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>      // close
#include <arpa/inet.h>   // sockaddr_in

int main() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { perror("socket"); return 1; }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // 0.0.0.0
    server_addr.sin_port = htons(5005);

    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(server_fd, 1) < 0) { perror("listen"); return 1; }

    printf("Waiting for Python client...\n");
    int client_fd = accept(server_fd, NULL, NULL);
    if (client_fd < 0) { perror("accept"); return 1; }

    char buffer[1024];
    while (1) {
        int len = recv(client_fd, buffer, sizeof(buffer)-1, 0);
        if (len <= 0) break;
        buffer[len] = '\0';
        float delta;
        sscanf(buffer, "%f", &delta);
        printf("Received delta_start = %.1f\n", delta);
    }

    close(client_fd);
    close(server_fd);
    return 0;
}
