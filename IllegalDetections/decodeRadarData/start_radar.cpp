#include "decodeRadarData.h"
void start()
{
    TcpThread   tcpThread;
    MyThread    myThread;
    RadarSoft   radarSoft;
    std::thread tcpThreadHandle(&TcpThread::run, &tcpThread, "192.168.0.54", 12345);
    std::thread myThreadHandle(&MyThread::run, &myThread);

    tcpThreadHandle.join();
    myThreadHandle.join();

    return 0;
}
