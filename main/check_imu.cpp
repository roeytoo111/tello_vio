// Simple standalone tool to check that the Tello SDK and IMU/state stream are working.
// Builds as a separate executable so you can run it without affecting the main app.

#include <iostream>
#include <string>
#include <cstring>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

class TelloCalibrator {
private:
    int command_sock{};
    int state_sock{};
    struct sockaddr_in tello_addr{};
    const std::string TELLO_IP = "192.168.10.1";
    const int COMMAND_PORT = 8889;
    const int STATE_PORT = 8890;

public:
    TelloCalibrator() {
        // Setup Command Socket
        command_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (command_sock < 0) {
            throw std::runtime_error("Failed to create command socket");
        }

        std::memset(&tello_addr, 0, sizeof(tello_addr));
        tello_addr.sin_family = AF_INET;
        tello_addr.sin_port = htons(COMMAND_PORT);
        inet_pton(AF_INET, TELLO_IP.c_str(), &tello_addr.sin_addr);

        // Setup State Socket (Listen for IMU/Battery/etc.)
        state_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (state_sock < 0) {
            throw std::runtime_error("Failed to create state socket");
        }

        struct sockaddr_in local_addr;
        std::memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(STATE_PORT);
        local_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(state_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            throw std::runtime_error("Failed to bind state socket to port 8890");
        }
    }

    void sendCommand(const std::string &cmd) {
        sendto(command_sock,
               cmd.c_str(),
               cmd.size(),
               0,
               (struct sockaddr *)&tello_addr,
               sizeof(tello_addr));
    }

    void checkIMU() {
        std::cout << "[Info] Initializing SDK Mode..." << std::endl;
        sendCommand("command");  // Required to start

        char buffer[1024];
        std::cout << "[Info] Listening for IMU and state data on port 8890..." << std::endl;

        // Loop to capture a few state packets
        for (int i = 0; i < 5; ++i) {
            int len = recv(state_sock, buffer, sizeof(buffer) - 1, 0);
            if (len > 0) {
                buffer[len] = '\0';
                std::cout << "\n--- Tello State Packet ---\n" << buffer << std::endl;
            } else {
                std::cout << "[Warn] No data received on this attempt." << std::endl;
            }
            usleep(500000);  // 0.5s delay
        }

        std::cout << "[Info] Done listening. If you saw state packets, the drone is sending IMU/state data correctly." << std::endl;
    }

    ~TelloCalibrator() {
        if (command_sock >= 0) {
            close(command_sock);
        }
        if (state_sock >= 0) {
            close(state_sock);
        }
    }
};

int main() {
    try {
        TelloCalibrator drone;
        drone.checkIMU();
    } catch (const std::exception &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }
    return 0;
}


