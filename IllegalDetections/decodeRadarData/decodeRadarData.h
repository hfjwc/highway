#ifndef CLIENT_CC_H
#define CLIENT_CC_H

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <mutex>
#include <queue>
#include <sys/socket.h>
#include <thread>
#include <vector>

typedef unsigned char  u8_t;
typedef signed char    s8_t;
typedef unsigned short u16_t;
typedef signed short   s16_t;
typedef unsigned int   u32_t;
typedef signed int     s32_t;
typedef char           char_t;
typedef char           bool_t;
typedef float          float_th;
typedef double         double_t;

// typedef unsigned long long      uint64_t;

#pragma pack(1)
typedef struct
{
    u8_t  id;        // 目标ID，大端存储，+3字节
    float posX;      // X位置，大端存储
    float posY;      // Y位置，大端存储
    float velX;      // X方向速度，大端存储
    float velY;      // Y方向速度，大端存储
    float accX;      // X方向加速度，大端存储
    float accY;      // Y方向加速度，大端存储
    u8_t  laneNum;   // 车道数，大端存储
    u8_t  carType;   // 车型，大端存储
    char  event;
    u8_t  snowId[8];   // +1字节
    /*float axis_longitude1;
    float axis_latitude1;*/
    char  reservedSpace3[4];
    u8_t  Confidence;
    char  reservedSpace4[3];
    float x1;
    u16_t tarNum;
    char  carLen;
    // char reservedSpace[29];   // 保留字
    char   reservedSpace1;   // 保留字
    u16_t  idHw;
    char   reservedSpace2[4];
    float  ath;
    double axis_longitude;
    double axis_latitude;
    char   reservedSpace[1];   // 保留字
    u8_t   RCS;
} TrjInfo;
#pragma pack()

#pragma pack(1)
typedef struct
{
    unsigned char header[4];   //协议标头：TRAJ
    unsigned int  length;      //数据长度，大端存储,值为4+sizeof(TrjInfo)*numObject+2
    u8_t          time1[6];
    uint64_t      time2;
    u16_t         id;
    u8_t          posBegin;
    u8_t          queueLength[12];
    // u8_t reserved[42];

    u16_t res1;   //帧计数器
    u8_t  res2;   // 0表示不拥堵，1表示拥堵，默认是0
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
    u8_t  reserved[19];

    u8_t    numObject;   //目标个数,大端存储
    TrjInfo trj[256];    //这个地方是变长的，与目标数目有关。TrjInfo是一个结构体,定义见下
    // unsigned short CRC;       //CRC校验值，大端存储
} TrajactoryOutput;
#pragma pack()

extern std::queue<std::vector<char>>  radarmsglist;
extern std::vector<std::vector<char>> listTar;
extern std::mutex                     mtx;
extern bool                           m_end[1];
extern std::string                    eventArray[256];
extern int                            client_sock;

class TcpThread
{
public:
    void run(const char* address, int port);

private:
    void Slot_RevData(int client_sock, int bytes_read, char* buffer);
};

// class TcpThread {
// public:
//     TcpThread() : m_sockfd(-1) {}
//     ~TcpThread() {
//         if (m_sockfd != -1) {
//             close(m_sockfd);
//         }
//     }

//     void run(const std::string &ip, int port);
//     void Slot_RevData();

// private:
//     int m_sockfd;
//     std::thread m_thread;
//     // std::vector<std::vector<char>> radarmsglist;
//     // std::mutex mtx;
// };

class MyThread
{
public:
    void run();

private:
    void analyserData();
    // void Init_event(std::string &eventArray);
    std::vector<char> radarmsgary;
    // bool m_end[1] = { false };
};

class RadarSoft
{
public:
    RadarSoft();
    void parseData(const std::vector<char>& data, int length);

private:
    float       BLEndianFloat(float value);
    double      BLEndianDouble(double value);
    uint16_t    qFromBigEndian_16(uint16_t value);
    uint32_t    qFromBigEndian(uint32_t value);
    uint64_t    qFromBigEndian_64(uint64_t value);
    std::string getTimestamp(uint64_t t);
    std::string eventArray[256];
};

#endif   // TRJINFO_H
