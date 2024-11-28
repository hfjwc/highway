#include "decodeRadarDataJni.h"

JNIEXPORT void JNICALL Java_org_jeecg_modules_camera_jni_decodeRadarDataJni_inference(JNIEnv* env, jobject job, jstring jstr)
{

    const char* str = env->GetStringUTFChars(jstr, nullptr);
    const char* ip(str);

    //
    TcpThread tcpThread;
    MyThread  myThread;
    RadarSoft radarSoft;
    std::cout << "ip: " << ip << std::endl;
    std::thread tcpThreadHandle(&TcpThread::run, &tcpThread, ip, 6371);
    std::cout << "ip11111111111111111111111111111111111111 " << std::endl;
    std::thread myThreadHandle(&MyThread::run, &myThread);
    std::cout << "ip22222222222222222222222222222222222 " << std::endl;

    tcpThreadHandle.join();
    myThreadHandle.join();
    env->ReleaseStringUTFChars(jstr, str);
}
