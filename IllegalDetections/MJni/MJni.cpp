////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////不按规定车道行驶////////////////////////////////////////////////////
#include <MJni.h>
#include <iostream>
JNIEXPORT void JNICALL Java_org_jeecg_modules_camera_jni_MJni_creator(JNIEnv*, jobject)
{
    if (truckdetector != nullptr)
    {
        delete truckdetector;
        truckdetector = nullptr;
    }
    TruckCVLibrary::Functions myfunctions;
    truckdetector = myfunctions.CreateDetector();
}
JNIEXPORT jstring JNICALL Java_org_jeecg_modules_camera_jni_MJni_inference(JNIEnv* env, jobject job, jstring jstr)
{
    const char* str = env->GetStringUTFChars(jstr, nullptr);
    std::string para(str);
    env->ReleaseStringUTFChars(jstr, str);
    vector<json> results_json;
    // detect
    std::cout << "start detect" << std::endl;
    //    PV_ERROR_CODE rc = myfunctions.inference_test(truckDetector, para, results_json);
    PV_ERROR_CODE rc = truckdetector->inference_test(para, results_json);

    std::string newJsonString = "";
    for (int i = 0; i < results_json.size(); i++)
    {
        // 将JSON数据转换为字符串
        if (newJsonString != "")
        {
            newJsonString = newJsonString + "|" + results_json[i].dump();
        }
        else
        {
            newJsonString = results_json[i].dump();
        }
    }
    jstring result = env->NewStringUTF(newJsonString.c_str());
    std::cout << "end detect" << std::endl;
    if (results_json.size() == 0)
    {
        string  result1 = "{}";
        jstring result2 = env->NewStringUTF(result1.c_str());
        return result2;
    }
    return result;
}
JNIEXPORT void JNICALL Java_org_jeecg_modules_camera_jni_MJni_delete(JNIEnv*, jobject)
{
    if (truckdetector != nullptr)
    {
        delete truckdetector;
        truckdetector = nullptr;
    }
}
