#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <vector>
// #include <unistd.h>
// #include "decode.h"

typedef unsigned   char    		u8_t;
typedef signed     char    		s8_t;
typedef unsigned   short   		u16_t;
typedef signed     short   		s16_t;
typedef unsigned   int    		u32_t;
typedef signed     int    		s32_t;
typedef char                	char_t;
typedef char                	bool_t;
typedef float					float_th;
typedef double					double_t;

// typedef unsigned long long      uint64_t;


#pragma pack(1)
typedef struct {
    u8_t id;                  // 目标ID，大端存储，+3字节
    float posX;               // X位置，大端存储
    float posY;               // Y位置，大端存储
    float velX;               // X方向速度，大端存储
    float velY;               // Y方向速度，大端存储
    float accX;               // X方向加速度，大端存储
    float accY;               // Y方向加速度，大端存储
    u8_t laneNum;              // 车道数，大端存储
    u8_t carType;              // 车型，大端存储
    char event;
    u8_t snowId[8];				// +1字节
    /*float axis_longitude1;
    float axis_latitude1;*/
    char reservedSpace3[4];
    u8_t Confidence;
    char reservedSpace4[3];
    float x1;
    u16_t tarNum;
    char carLen;
    //char reservedSpace[29];   // 保留字
    char reservedSpace1;   // 保留字
    u16_t idHw;
    char reservedSpace2[4];
    float ath;
    double axis_longitude;
    double axis_latitude;
    char reservedSpace[1];   // 保留字
    u8_t RCS;
}TrjInfo;
#pragma pack()

#pragma pack(1)
typedef struct {
    unsigned char header[4];  //协议标头：TRAJ
    unsigned int length;      //数据长度，大端存储,值为4+sizeof(TrjInfo)*numObject+2
    u8_t time1[6];
    uint64_t time2;
    u16_t id;
    u8_t posBegin;
    u8_t queueLength[12];
    //u8_t reserved[42];

    u16_t res1;
    u8_t res2;
    u16_t changeLanes;
    u16_t retrograde;
    u16_t stopCar;
    u16_t NOLINE;
    u16_t LINES;
    u16_t CLOSECAR;
    u16_t OUTTIME;
    u16_t HIGHPEED;
    u16_t LOWPEED;
    u16_t NUMBERS;
    u8_t reserved[19];

    u8_t numObject;            //目标个数,大端存储
    TrjInfo trj[256];   //这个地方是变长的，与目标数目有关。TrjInfo是一个结构体,定义见下
    //unsigned short CRC;       //CRC校验值，大端存储
}TrajactoryOutput;
#pragma pack()

uint16_t qFromBigEndian_16(uint16_t value){
    return (value >> 8) | (value << 8);
}

uint64_t qFromBigEndian_64(uint64_t value) {
    return ((value >> 56) & 0x00000000000000FFULL) |
            ((value >> 40) & 0x000000000000FF00ULL) |
            ((value >> 24) & 0x0000000000FF0000ULL) |
            ((value >> 8)  & 0x00000000FF000000ULL) |
            ((value << 8)  & 0x000000FF00000000ULL) |
            ((value << 24) & 0x0000FF0000000000ULL) |
            ((value << 40) & 0x00FF000000000000ULL) |
            ((value << 56) & 0xFF00000000000000ULL);
}

std::string getTimestamp(uint64_t t) {
    time_t rawtime = t / 1000;
    struct tm * timeinfo = localtime(&rawtime);
    char buffer[80];
    strftime(buffer, 80, "%H:%M:%S", timeinfo);
    return std::string(buffer);
}

void clearSocketBuffer(int sockfd) {
    // 清空socket缓冲区
    std::vector<char> buffer(4096);
    while (true) {
        int n = recv(sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT);
        if (n <= 0) {
            break;
        }
    }
}

void processMessage(const TrajactoryOutput& info) {
    // std::cout << "Processing: numObject=" << static_cast<int>(info.numObject) << ", length=" << info.length <<  std::endl;
    std::cout << "Timestamp: " << getTimestamp(qFromBigEndian_64(info.time2)) << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(2));  // 模拟消息处理时间
    sleep(2);
}

int main() {
    int listenfd, connfd;
    struct sockaddr_in servaddr, cliaddr;

    // 创建TCP套接字
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return -1;
    }

    // 设置服务器地址信息
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(6731);
    servaddr.sin_addr.s_addr = INADDR_ANY;
    // inet_pton(AF_INET, addr, &address.sin_addr);

    // 绑定套接字到地址和端口
    if (bind(listenfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "Error binding socket" << std::endl;
        close(listenfd);
        return -1;
    }

    // 监听连接
    if (listen(listenfd, 5) < 0) {
        std::cerr << "Error listening" << std::endl;
        close(listenfd);
        return -1;
    }

    // 接受连接
    socklen_t len = sizeof(cliaddr);
    connfd = accept(listenfd, (struct sockaddr *)&cliaddr, &len);
    if (connfd < 0) {
        std::cerr << "Error accepting connection" << std::endl;
        close(listenfd);
        return -1;
    }

    TrajactoryOutput buffer;
    int total_bytes = 0;

    while (true) {
        int n;
        bool received_message = false;

        // 尝试接收消息
        while (total_bytes < sizeof(buffer)) {
            n = recv(connfd, reinterpret_cast<char*>(&buffer) + total_bytes, sizeof(buffer) - total_bytes, 0);
            if (n < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    break;  // 没有新消息，超时
                } else {
                    std::cerr << "Error receiving message: " << strerror(errno) << std::endl;
                    close(connfd);
                    close(listenfd);
                    return -1;
                }
            } else if (n == 0) {
                std::cout << "Connection closed by peer" << std::endl;
                close(connfd);
                close(listenfd);
                return 0;
            } else {
                total_bytes += n;
            }
        }

        if (total_bytes == sizeof(buffer)) {
            total_bytes = 0;  // 重置计数器
            processMessage(buffer);

            // 清空接收缓冲区以确保接收最新消息
            clearSocketBuffer(connfd);
        }
    }

    close(connfd);
    close(listenfd);
    return 0;
}
