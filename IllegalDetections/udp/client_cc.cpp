#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>

class TCPClient {
public:
    TCPClient(const std::string& server_ip, int server_port)
        : server_ip_(server_ip), server_port_(server_port), sockfd_(-1), message_number_(0) {
        init();
    }

    ~TCPClient() {
        if (sockfd_ != -1) {
            close(sockfd_);
        }
    }

    void sendMessages() {
        char message[50];
        while (true) {
            std::sprintf(message, "Message number %d\n", message_number_++);
            send(sockfd_, message, std::strlen(message), 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟快速发送
        }
    }

private:
    void init() {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ < 0) {
            throw std::runtime_error("Error creating socket");
        }

        struct sockaddr_in servaddr;
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(server_port_);
        servaddr.sin_addr.s_addr = inet_addr(server_ip_.c_str());

        if (connect(sockfd_, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
            throw std::runtime_error("Error connecting to server");
        }
    }

    std::string server_ip_;
    int server_port_;
    int sockfd_;
    int message_number_;
};

int main() {
    TCPClient client("127.0.0.1", 10000);
    try {
        
        client.sendMessages();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
