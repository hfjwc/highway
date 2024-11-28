#include "client_cc.h"

std::string server_ip_;
int         server_port_;
int         sockfd_;

TCPClient::TCPClient(const std::string& ip, int port)
{
    server_ip_   = ip;
    server_port_ = port;
    connectToServer(ip, port);
}

TCPClient::~TCPClient()
{
    if (sockfd_ != -1)
    {
        close(sockfd_);
    }
}

// std::vector<std::string> getIPv4Addresses() {
//     struct ifaddrs *ifaddr, *ifa;
//     char host[NI_MAXHOST];
//     std::vector<std::string> ipv4Addresses;

//     if (getifaddrs(&ifaddr) == -1) {
//         perror("getifaddrs");
//         return ipv4Addresses;
//     }

//     for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
//         if (ifa->ifa_addr == nullptr)
//             continue;

//         if (ifa->ifa_addr->sa_family == AF_INET) {
//             int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST);
//             if (s != 0) {
//                 std::cerr << "getnameinfo() failed: " << gai_strerror(s) << std::endl;
//                 continue;
//             }
//             if (std::string(host) != "127.0.0.1") {
//                 ipv4Addresses.push_back(host);
//             }
//         }
//     }

//     freeifaddrs(ifaddr);
//     return ipv4Addresses;
// }

void TCPClient::clearSocketBuffer(int sockfd)
{
    // 清空socket缓冲区
    std::vector<char> buffer(4096);
    while (true)
    {
        int n = recv(sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT);
        if (n <= 0)
        {
            break;
        }
    }
}

void TCPClient::connectToServer(const std::string& ip, int port)
{
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0)
    {
        throw std::runtime_error("Socket creation error");
    }

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port   = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr) <= 0)
    {
        close(sockfd_);
        throw std::runtime_error("Invalid address/ Address not supported");
    }

    // std::vector<std::string> ipv4 = getIPv4Addresses();
    // std::cout << ipv4[0] << std::endl;

    while (connect(sockfd_, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "Failed to connect to server. Retrying in 1 second...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Connected to server\n";
}

void TCPClient::sendMessages(TrajactoryOutput message, int client_sock)
{
    while (true)
    {
        ssize_t sent = send(sockfd_, &message, sizeof(message), 0);
        if (sent < 0)
        {
            std::cerr << "Failed to send message. Reconnecting...\n";
            close(sockfd_);
            connectToServer(server_ip_, server_port_);
            // 清空接收缓冲区以确保接收最新消息
            clearSocketBuffer(client_sock);
        }
        else
        {
            break;
        }
    }
}

void TCPClient::init()
{
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0)
    {
        throw std::runtime_error("Error creating socket");
    }

    struct sockaddr_in servaddr;
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(server_port_);
    servaddr.sin_addr.s_addr = inet_addr(server_ip_.c_str());

    if (connect(sockfd_, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        throw std::runtime_error("Error connecting to server");
    }
}
