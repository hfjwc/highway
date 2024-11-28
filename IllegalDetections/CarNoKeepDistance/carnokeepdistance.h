#ifndef CARNOKEEPDISTANCE_H
#define CARNOKEEPDISTANCE_H
#include "common.h"
//#include "decodeRadarData.h"
#include "json.hpp"
#include "opencv2/opencv.hpp"
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
using json = nlohmann::json;
using namespace std;
using namespace cv;
struct pre_car_detect_data
{
    int         pre_track_id;
    int         pre_car_y = 0;
    vector<int> pre_car_speed;
    int         pre_car_lane_id;
    int         pre_car_is_illegal     = 0;   //前车位置一直在改变，需要保证4张违法图像中每次前车都违法
    int         pre_car_is_illegal_num = 1;
};

//检测数据
struct car_detect_data
{
    vector<cv::Mat>      vehicle_images;
    vector<plate_data>   vehicle_plate_images;
    vector<cv::Rect>     vehicle_location_box;
    vector<YoloV8BoxVec> phone_safetybelt_detected;   //用于记录接打电话或不系安全带情况
    int                  class_id;                    //车辆车牌类别
    int                  track_id;
    float                vehicle_confidence;
    cv::Rect             palte_location;
    double               start_time;
    int                  start_nframe;
    int                  end_nframe;
    int                  is_overtime    = 0;   //判断是否到达第二次抓拍时间间隔
    int                  is_illegal_num = 0;
    vector<string>       illegal_time;
    /////////////////////以下部分为新增///////////////////////
    int                         is_cross_zeroline = 0;   //是否压到0m 线
    int                         is_pass_zeroline  = 0;   //是否越过0m 线
    int                         lane_id           = 0;   //车道号
    float                       speed;
    int                         is_illegal           = 0;      //是否存在违法前车，和后车形成完整违法
    int                         illegal_car_is_exist = 0;      //违法图像中违法车辆是否存在
    int                         illegal_car_y        = 0;      //车辆的y值，用来区分前车和后车，此项目拍摄车尾，后车y值大于前车
    vector<pre_car_detect_data> illegal_pre_car_detect_data;   //记录每辆违法车辆的前车信息，可能同时存在多个前车，记录所有满足条件的前车id，如果有任一辆前车，可以和后车形成完整违法，则该车辆为违法；
    int                         plate_recognition_flag = 0;    //车牌识别标志位，只进行一次识别
    int                         plate_roi_flag         = 0;    //确定车牌图像，只进行一
    int                         kakou_id;
    int                         flag_save = 0;
};

class CarNoKeepDistance
{
public:
    virtual PV_ERROR_CODE inference_test(std::string para_data, vector<json>& results_json){};
};

class Functions
{
public:
    /////////////////////////////////////////////////////////////////////////////////
    // Factory function for creating the detector object
    // static std::shared_ptr<CarNoKeepDistance> CreateDetector();
    CarNoKeepDistance*   CreateDetector();
    static PV_ERROR_CODE inference_test(CarNoKeepDistance* obj, std::string para_data, vector<json>& results_json);
};

// namespace CarCVLibrary
#endif   // CARNOKEEPDISTANCE_H
