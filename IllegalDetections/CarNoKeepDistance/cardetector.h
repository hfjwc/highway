#ifndef CARDETECTOR_H
#define CARDETECTOR_H
#include "bytetrack.h"
#include "carnokeepdistance.h"
#include "opencv2/freetype.hpp"
#include "platenet.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <QStringList>
#include <random>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/spdlog.h>

#include "decodeRadarData.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <queue>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
struct algo_parameters2
{
    /*
     *  "2,3,34,85,67,143,89,278,4,718,0,914,2,617,830,3,813,/home/data/1.jpg,2,1000,1,0.8,1,0.7,3,0.85,0.8,5xdad324,4312,4608,1094,1101,1223,南京高速"
     *
     * 如上是完整的参数示例，按顺序解析如下：
     * 2表示2个区域，如果有两个区域，第一个区域是最左侧车道，第二个区域为应急车道
     * 第一个区域有3个点，34,85,67,143,89,278是这3个点的x,y坐标，
     * 第二个区域有4个点，718,0,914,2,617,830,3,813是这4个点的x,y坐标，
     * 图片的路径是/home/data/1.jpg，
     * 2表示关注最左侧和应急车道
     * 取证的时间间隔是1000毫秒，
     * 1表示车辆只检测大货车、大客车，
     * 0.8是车辆检测的置信度，
     * 1表示需要检测车牌，
     * 0.7是车牌检测的置信度，
     * 3表示需要检测安全带和打电话都需要检测，
     * 0.85是安全带检测的置信度
     * 0.8打电话检测的置信度
     * 5xdad324表示设备编号
     * 不按规定车道行驶违法代码：4312
     * 占用应急车道违法代码：4608
     * 未按规定保持车距违法代码：1094
     * 不系安全带违法代码：1101
     * 开车打电话违法代码：1223
     * 南京高速 表示违法地点
     */

    vector<vector<cv::Point>> regions;                   //所有区域集合
    std::string               img_path;                  //图片路径
    int                       detect_lane;               //违法车道，0表示都不需要，1表示只关注最左侧车道，2表示只关注应急车道，3表示关注最左侧和应急车道
    int                       timeInterval;              //取证的时间间隔，单位秒
    int                       vehicle_type;              //需要检测的车辆类型，0表示检测所有车辆，1表示检测大货车、大客车
    double                    vehicle_conf;              //车辆检测的置信度
    int                       is_detect_plate;           //是否需要检测车牌，0表示不需要，1表示需要
    double                    plate_conf;                //车牌检测的置信度
    int                       is_detect_BeltAndPhone;    //是否需要检测安全带和打电话，0表示都不需要检测，1表示打电话，2表示不系安全带，3表示两种都需要
    double                    belt_conf;                 //安全带检测的置信度
    double                    phone_conf;                //打电话检测的置信度
    std::string               device_number;             //设备编号，盒子备案后会确定，每台盒子不一样
    std::string               illegal_code_occupyLane;   //不按规定车道行驶违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_occupyEmergencyLane;   //占用应急车道违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_safeDistance;          //未按规定保持车距违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_nobelt;                //不系安全带违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_phone;                 //开车打电话违法代码,根据算法功能确定，可能会改
    std::string               camera_place;                       //违法地点，根据相机确定
    std::string               camera_id;                          //相机id
};

struct illegal_data
{
    int                         nframe = 0;
    vector<car_detect_data>     detection_data;   //用于记录检测数据，如形成完整违法则存入输出数据，并在检测数据中清除
    vector<vehicle_output_data> output_data;      //用于记录完整违法证据，并转成相应格式输出,
    vector<int>                 illegal_trackid;

    vector<vehicle_detect_data> detection_data_emergence;   //用于记录检测数据，如形成完整违法则存入输出数据，并在检测数据中清除
    vector<vehicle_output_data> output_data_emergence;
    vector<int>                 illegal_trackid_emergence;

    algo_parameters  algo_para;   //车距参数和不按规定车道参数不一致
    string           camera_id;
    BYTETracker      bytetrack;
    bytetrack_params params;
};

typedef struct
{
    float x1;
    float y1;
    float x2;
    float y2;
} FloatRet_;

struct radar_data_time
{
    TrajactoryOutput radar_output;
    size_t           second_time;
    size_t           milli_second_time;
};

class cardetector : public CarNoKeepDistance
{
public:
    cardetector();
    ~cardetector();
    PV_ERROR_CODE cardetect(std::vector<bm_image> batch_imgs, std::vector<YoloV8BoxVec>& boxes);
    PV_ERROR_CODE cartrack(std::vector<std::vector<BoundingBox>> yolov8_boxes_BoundingBox, vector<STracks>& results_stracks, illegal_data& one_illegal_data);
    PV_ERROR_CODE platerecognition(cv::Mat plate_img, vector<string>& results);
    PV_ERROR_CODE img_merge(vector<vehicle_output_data>& output_data, int detect_lane, vector<cv::Point> contour, illegal_data& one_illegal_data);
    PV_ERROR_CODE postprocess_nokeepdistance(cv::Mat frame_to_draw, cv::Mat img_origin, int time_interval, int detect_lane, vector<vector<cv::Point>> lines, vector<vector<cv::Point>> contour,
                                             std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json, illegal_data& one_illegal_data);
    PV_ERROR_CODE inference_test(std::string para_data, vector<json>& results_json);
    PV_ERROR_CODE worldtoPixel(std::vector<cv::Point2f> srcPt, std::vector<cv::Point2f>& dstPt);
    PV_ERROR_CODE get_speed(car_detect_data& one_detect, std::shared_ptr<STrack> bbox, cv::Mat frame_to_draw, radar_data_time radar_data_one);

private:
    YoloV8*                         m_truckyolov8;
    PLATENET*                       m_platenet;
    vector<illegal_data>            m_illegal_data;
    std::shared_ptr<spdlog::logger> logger_car;
    std::string                     filename_radar_matrix;
};

#endif   // CARDETECTOR_H
