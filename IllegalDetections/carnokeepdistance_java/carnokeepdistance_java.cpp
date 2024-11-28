////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////不按规定保持车距行驶////////////////////////////////////////////////////
#include <carnokeepdistance_java.h>
#include <iostream>
JNIEXPORT void JNICALL Java_org_jeecg_modules_camera_jni_carnokeepdistance_creator(JNIEnv*, jobject)
{
    if (carDetector != nullptr)
    {
        delete carDetector;
        carDetector = nullptr;
    }
    Functions myfunctions;
    carDetector = myfunctions.CreateDetector();
}
JNIEXPORT jstring JNICALL Java_org_jeecg_modules_camera_jni_carnokeepdistance_inference(JNIEnv* env, jobject job, jstring jstr)
{
    const char* str = env->GetStringUTFChars(jstr, nullptr);
    std::string para(str);
    env->ReleaseStringUTFChars(jstr, str);
    cout << "JString: " << para << endl;
    vector<json> results_json;
    // detect
    std::cout << "start detect" << std::endl;
    //    PV_ERROR_CODE rc = myfunctions.inference_test(truckDetector, para, results_json);
    PV_ERROR_CODE rc = carDetector->inference_test(para, results_json);

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
JNIEXPORT void JNICALL Java_org_jeecg_modules_camera_jni_carnokeepdistance_delete(JNIEnv*, jobject)
{
    if (carDetector != nullptr)
    {
        delete carDetector;
        carDetector = nullptr;
    }
}
