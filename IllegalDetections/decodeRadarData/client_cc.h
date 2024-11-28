#include "decodeRadarData.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

class TCPClient
{
public:
    TCPClient(const std::string& server_ip, int server_port);
    ~TCPClient();
    void sendMessages(TrajactoryOutput message, int client_sock);
    void clearSocketBuffer(int sockfd);

private:
    void               connectToServer(const std::string& ip, int port);
    void               init();
    struct sockaddr_in serverAddress;
    std::string        server_ip_;
    int                server_port_;
    int                sockfd_;
    int                message_number_;
};
