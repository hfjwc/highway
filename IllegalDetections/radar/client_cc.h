#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include "decode.h"

class TCPClient {
public:
    TCPClient(const std::string& server_ip, int server_port);
    ~TCPClient();
    void sendMessages(TrajactoryOutput message, int client_sock);
    void clearSocketBuffer(int sockfd);
private:
    void connectToServer(const std::string& ip, int port);
    void init();
    struct sockaddr_in serverAddress;
    std::string server_ip_;
    int server_port_;
    int sockfd_;
    int message_number_;
};

