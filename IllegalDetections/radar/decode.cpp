#include "decode.h"
#include "client_cc.h"

std::queue<std::vector<char>> radarmsglist;
std::vector<std::vector<char>> listTar;
std::mutex mtx;
bool m_end[1] = { false };
std::string eventArray[256];
TCPClient client("127.0.0.1", 6731);
int message_number_ = 0;
int client_sock;


// void clearSocketBuffer(int sockfd) {
//     // 清空socket缓冲区
//     std::vector<char> buffer(4096);
//     while (true) {
//         int n = recv(sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT);
//         if (n <= 0) {
//             break;
//         }
//     }
// }


//数据接收线程
void TcpThread::run(const char* address, int port) {
    // struct sockaddr_in server_addr, cliaddr;
    // int server_sock = socket(AF_INET, SOCK_STREAM, 0);
    // if (server_sock == -1) {
    //     std::cerr << "Failed to create socket." << std::endl;
    //     return;
    // }
    // // std::cout << "B+++++++++++++++++++++++." << std::endl;
    // // sockaddr_in server_addr;
    // server_addr.sin_family = AF_INET;
    // server_addr.sin_port = htons(port);
    // inet_pton(AF_INET, address, &server_addr.sin_addr);

    // if (bind(server_sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
    //     std::cerr << "Bind failed." << std::endl;
    //     close(server_sock);
    //     return;
    // }

    // if (listen(server_sock, 5) == -1) {
    //     // printf("client_sock:%d\n", client_sock);
    //     close(server_sock);
    //     return;
    // }
    // socklen_t len = sizeof(cliaddr);
    // std::cout << "Listening on port: " << port << std::endl;
    while(true){
        struct sockaddr_in server_addr, cliaddr;
        int server_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sock == -1) {
            std::cerr << "Failed to create socket." << std::endl;
            continue;
        }
        // std::cout << "B+++++++++++++++++++++++." << std::endl;
        // sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, address, &server_addr.sin_addr);

        if (bind(server_sock, (sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
            std::cerr << "Bind failed." << std::endl;
            close(server_sock);
            continue;
        }

        if (listen(server_sock, 5) == -1) {
            // printf("client_sock:%d\n", client_sock);
            close(server_sock);
            continue;
        }
        socklen_t len = sizeof(cliaddr);
        std::cout << "Listening on port: " << port << std::endl;
        // client_sock = 5;
        std::cout << "test1: "<< std::endl;
        client_sock = accept(server_sock, (struct sockaddr *)&cliaddr, &len);
        std::cout << "test2: "<< std::endl;
        // std::cout << "client_sock: " << client_sock << std::endl;
        if (client_sock < 0) {
            // std::cerr << "Error accepting connection" << std::endl;
            continue;
        }
        else{
            std::cerr << "New connect..." << std::endl;
        }
        
        while (true) {
            // std::cout << "Bind fasdsdsdsd." << std::endl;
            // client_sock = accept(server_sock, nullptr, nullptr);
            // printf("client_sock:%d\n", client_sock);
            if (client_sock == -1) {
                std::cerr << "Accept failed." << std::endl;
                continue;
            }
            char buffer[8100];
            int bytes_read = read(client_sock, buffer, sizeof(buffer));
            // printf("bytes_read %d\n", bytes_read);
            if (bytes_read < 0) {
                if (errno == EWOULDBLOCK || errno == EAGAIN) {
                    break;  // 没有新消息，超时
                } else {
                    std::cerr << "Error receiving message: " << strerror(errno) << std::endl;
                    close(client_sock);
                    client.clearSocketBuffer(client_sock);
                    break; // 跳出当前循环，等待新的连接
                }
            } else if (bytes_read == 0) {
                    std::cout << "Connection closed by peer" << std::endl;
                    close(client_sock);
                    client.clearSocketBuffer(client_sock);
                    break; // 跳出当前循环，等待新的连接
            } else {
                Slot_RevData(client_sock, bytes_read, buffer);
            }
        }
        close(server_sock);
        }
}

void TcpThread::Slot_RevData(int client_sock, int bytes_read, char* buffer) {

    while (bytes_read > 0) {
        printf("接收到数据 %d\n", bytes_read);
        std::vector<char> data(buffer, buffer + bytes_read);
        std::lock_guard<std::mutex> lock(mtx);
        radarmsglist.push(data);
        bytes_read = 0;
        // printf("sizeof(buffer)：%d \n", sizeof(buffer));
        // printf("sizeof(data):%d \n", static_cast<int>(data.size()));
        // 打印接收到的数据
        // for (int i = 0; i < data.size(); ++i) 
        // {
        //     printf("%x ", static_cast<unsigned char>(data[i]));
        // }
        printf("\n");
        // printf("数据处理完成\n");
    }   
    // printf("Closing client_sock: %d\n", client_sock);
    // close(client_sock); 
}



//数据校验线程（校验完整性）
void MyThread::run() {
	for (int i = 0; i < 1;i++){
        while (!m_end[0]) {
            std::lock_guard<std::mutex> lock(mtx);
            //判断接收缓存是否存在数据
            // printf("radarmsglist.size:%d\n", radarmsglist.size());
            while (!radarmsglist.empty()) {
                radarmsgary.insert(radarmsgary.end(), radarmsglist.front().begin(), radarmsglist.front().end());
                radarmsglist.pop();
                std::cout << "thread radarmsgary.size():" << radarmsgary.size() << std::endl;
            }
            // for(int i=0;i<radarmsgary.size();i++)
            // {
            //     std::cout << "radarmsgary:" << radarmsgary[i] << std::endl;
            // }

            // printf("222222222222222222222\n");
            analyserData();
            if (i == 0)
            {
                //强制当前子线程一直循环不退出
                i = -1;
                //10毫秒
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                // printf("222222222222222222222\n");
            }
            
        }
    }
}

void MyThread::analyserData() {
    std::string datehead = "TRAJ";
    RadarSoft radarSoft;
    auto it = std::search(radarmsgary.begin(), radarmsgary.end(), datehead.begin(), datehead.end());
    if (it != radarmsgary.end()) {
        size_t index = std::distance(radarmsgary.begin(), it);
        radarmsgary.erase(radarmsgary.begin(), it);
        int length = (static_cast<unsigned char>(radarmsgary[4]) << 24) |
                     (static_cast<unsigned char>(radarmsgary[5]) << 16) |
                     (static_cast<unsigned char>(radarmsgary[6]) << 8) |
                     static_cast<unsigned char>(radarmsgary[7]) + 8;
        // std::cout << "length:" << length << std::endl;
        // std::cout << "radarmsgary.size():" << radarmsgary.size() << std::endl;
        // if (radarmsgary.size()>2048) std::exit(0);
        // std::exit(0);
        if (radarmsgary.size() >= length) {
            std::vector<char> frame(radarmsgary.begin(), radarmsgary.begin() + length);
            radarmsgary.erase(radarmsgary.begin(), radarmsgary.begin() + length);
            // radarmsgary.clear();
            std::cout << "after radarmsgary.size():" << radarmsgary.size() << std::endl;
            // 发送数据到主线程进行解析
            // 这里直接调用解析函数
            // for (int i = 0; i < frame.size(); ++i) 
            // {
            // for (int i = 0; i < frame.size(); ++i) 
            //     printf("%x ", static_cast<unsigned char>(frame[i]));
            // }
            //  printf("\n");
            radarSoft.parseData(frame, length); // 调用解析函数
        }
    } else {
        radarmsgary.clear();
    }
}


RadarSoft::RadarSoft()
{
    for (int i = 0; i < 256; ++i) {
        eventArray[i] = "无";// 错误事件
    }
	eventArray[0] = "无";			// 无
	eventArray[1] = "违法停车";			// 停止
	eventArray[2] = "反向行驶";	// 反向行驶
	eventArray[3] = "追尾";	// 追尾
	eventArray[4] = "超速";	// 超速行驶
	eventArray[5] = "实线变道";// 实线变道
	eventArray[6] = "超高速";
	eventArray[7] = "超低速";
	eventArray[8] = "未保持安全车距";
	eventArray[9] = "压实线";
	eventArray[10] = "蛇形行驶";
	eventArray[11] = "异常停车";
	eventArray[12] = "占用应急车道";
    
	eventArray[101] = "违法停车消失";// 停止消失
	eventArray[102] = "反向行驶消失";// 反向行驶消失
	eventArray[103] = "追尾消失";// 追尾消失
	eventArray[104] = "超速行驶消失";// 超速行驶消失
	eventArray[105] = "实线变道消失";// 实线变道消失
	eventArray[106] = "超高速消失";
	eventArray[107] = "超低速消失";
	eventArray[108] = "未保持安全车距消失";
	eventArray[109] = "压实线消失";
	eventArray[110] = "蛇形行驶消失";//蛇形行驶消失
	eventArray[111] = "异常停车消失";//异常停车消失
}

float RadarSoft::BLEndianFloat(float value) {
    unsigned char s[4], t[4];
    float result;
    memcpy(s, &value, sizeof(float));
    t[0] = s[3];
    t[1] = s[2];
    t[2] = s[1];
    t[3] = s[0];
    memcpy(&result, t, sizeof(float));
    return result;
}

double RadarSoft::BLEndianDouble(double value) {
    unsigned char s[8], t[8];
    double result;
    memcpy(s, &value, sizeof(double));
    t[0] = s[7];
    t[1] = s[6];
    t[2] = s[5];
    t[3] = s[4];
    t[4] = s[3];
    t[5] = s[2];
    t[6] = s[1];
    t[7] = s[0];
    memcpy(&result, t, sizeof(double));
    return result;
}

uint16_t RadarSoft::qFromBigEndian_16(uint16_t value){
    return (value >> 8) | (value << 8);
}

uint32_t RadarSoft::qFromBigEndian(uint32_t value) {
    return ((value >> 24) & 0x000000FF) |
            ((value >> 8)  & 0x0000FF00) |
            ((value << 8)  & 0x00FF0000) |
            ((value << 24) & 0xFF000000);
}

uint64_t RadarSoft::qFromBigEndian_64(uint64_t value) {
    return ((value >> 56) & 0x00000000000000FFULL) |
            ((value >> 40) & 0x000000000000FF00ULL) |
            ((value >> 24) & 0x0000000000FF0000ULL) |
            ((value >> 8)  & 0x00000000FF000000ULL) |
            ((value << 8)  & 0x000000FF00000000ULL) |
            ((value << 24) & 0x0000FF0000000000ULL) |
            ((value << 40) & 0x00FF000000000000ULL) |
            ((value << 56) & 0xFF00000000000000ULL);
}

std::string RadarSoft::getTimestamp(uint64_t t) {
    time_t rawtime = t / 1000;
    struct tm * timeinfo = localtime(&rawtime);
    char buffer[80];
    strftime(buffer, 80, "%H:%M:%S", timeinfo);
    return std::string(buffer);
}

void RadarSoft::parseData(const std::vector<char>& data, int length) {
    TrajactoryOutput pTrjOutputs;
    memcpy(&pTrjOutputs.header[0], data.data(), 80); //保存到numObject为止
    // std::cout << "pTrjOutputs.header[0]: " << pTrjOutputs.header[0] << std::endl;
    // std::cout << "pTrjOutputs.header[0]: " << pTrjOutputs.header[1] << std::endl;
    // std::cout << "pTrjOutputs.header[0]: " << pTrjOutputs.header[2] << std::endl;
    std::cout << "pTrjOutputs.length: " << qFromBigEndian(pTrjOutputs.length) << std::endl;
    std::cout << "pTrjOutputs.id: " << qFromBigEndian_16(pTrjOutputs.id) << std::endl;
    std::cout << "pTrjOutputs.res1: " << qFromBigEndian_16(pTrjOutputs.res1) << std::endl;
    // printf("pTrjOutputs.numObject: %d\n", pTrjOutputs.numObject);
    // std::cout << "pTrjOutputs.numObject: " << static_cast<int>(pTrjOutputs.numObject) << std::endl;
    // uint8_t* ptr8 = reinterpret_cast<uint8_t*>(&pTrjOutputs) + 79; // 第 80 个字节
    // std::cout << "pTrjOutputs.numObject ptr: " << static_cast<int>(*ptr8) << std::endl;
    // pTrjOutputs.numObject = qFromBigEndian(pTrjOutputs.numObject);
    int nums = static_cast<int>(pTrjOutputs.numObject);
    std::cout << "Number of objects: " << nums << std::endl;
    for (int i = 0; i < nums; ++i) {
        // std::cout << "test1" << std::endl;
        memcpy(&pTrjOutputs.trj[i], data.data() + 80 + i*(80), (80));
        // std::cout << "test2" << std::endl;
    }
    
    for (int i = 0; i < nums; ++i) {
        // memcpy(&pTrjOutputs.trj[i], data.data() + 80 + i * 80, 80);
        pTrjOutputs.trj[i].id = pTrjOutputs.trj[i].id;
        pTrjOutputs.trj[i].posX = BLEndianFloat(pTrjOutputs.trj[i].posX);
        pTrjOutputs.trj[i].posY = BLEndianFloat(pTrjOutputs.trj[i].posY);
        pTrjOutputs.trj[i].velX = BLEndianFloat(pTrjOutputs.trj[i].velX);
        pTrjOutputs.trj[i].velY = BLEndianFloat(pTrjOutputs.trj[i].velY);
        pTrjOutputs.trj[i].accX = BLEndianFloat(pTrjOutputs.trj[i].accX);
        pTrjOutputs.trj[i].accY = BLEndianFloat(pTrjOutputs.trj[i].accY);
        pTrjOutputs.trj[i].laneNum = static_cast<int>(pTrjOutputs.trj[i].laneNum);
        pTrjOutputs.trj[i].carType = qFromBigEndian(pTrjOutputs.trj[i].carType);
        pTrjOutputs.trj[i].ath = BLEndianFloat(pTrjOutputs.trj[i].ath);
        pTrjOutputs.trj[i].axis_longitude = BLEndianDouble(pTrjOutputs.trj[i].axis_longitude);
        pTrjOutputs.trj[i].axis_latitude = BLEndianDouble(pTrjOutputs.trj[i].axis_latitude);
        // printf("pTrjOutputs.trj[i].id: %d\n", pTrjOutputs.trj[i].id);
        // printf("pTrjOutputs.trj[i].speed_x: %f\n", pTrjOutputs.trj[i].velX);
        // printf("pTrjOutputs.trj[i].speed_y: %f\n", pTrjOutputs.trj[i].velY);
        if (pTrjOutputs.trj[i].event >= 0) {
            std::string strEvent = eventArray[pTrjOutputs.trj[i].event];
            std::cout << "Event: " << strEvent << std::endl;
            // printf("enevt: %s\n", strEvent);
        }
    }
    std::cout << "Timestamp: " << getTimestamp(qFromBigEndian_64(pTrjOutputs.time2)) << std::endl;
    // char message[50];
    // std::sprintf(message, "Message number %d\n", message_number_++);
    // // printf("-----------------message:%s\n", message);
    // std::cout << "pTrjOutputs.res1: " << qFromBigEndian_16(pTrjOutputs.res1) << std::endl;
    // printf("-------------------------------pTrjOutputs.id:%d\n", qFromBigEndian_16(pTrjOutputs.res1));
    // std::cout << "client_sock: " << client_sock << std::endl;
    client.sendMessages(pTrjOutputs, client_sock);

}
