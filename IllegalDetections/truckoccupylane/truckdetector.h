#ifndef TRUCKDETECTOR_H
#define TRUCKDETECTOR_H
#include "bytetrack.h"
#include "opencv2/freetype.hpp"
#include "phonesafetybelt.h"
#include "platenet.hpp"
#include "truckoccupylane.h"
//#include "savelog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "yolov8.hpp"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <random>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/spdlog.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <QCoreApplication>
#include <QtConcurrent/QtConcurrent>
struct detection_data_lane
{
    vector<vehicle_detect_data> detection_data;
    vector<vehicle_output_data> output_data;   //用于记录完整违法证据，并转成相应格式输出,
    vector<int>                 illegal_trackid;
    int                         lane_id;
};

struct illegal_data
{
    int                         nframe = 0;
    vector<detection_data_lane> detection_data_prohibited;   //禁行车道

    vector<vehicle_detect_data> detection_data_emergence;   //应急车道
    vector<vehicle_output_data> output_data_emergence;
    vector<int>                 illegal_trackid_emergence;

    vector<vehicle_detect_data> detection_data_secondlane;   //超车道
    vector<vehicle_output_data> output_data_secondlane;
    vector<int>                 illegal_trackid_secondlane;

    // C类区间抓拍
    vector<vehicle_detect_data> detection_data_secondlane_interval_detection;
    vector<vehicle_output_data> output_data_secondlane_interval_detection;
    vector<int>                 illegal_trackid_secondlane_interval_detection;

    //区间测速
    vector<vehicle_detect_data> detection_data_speedmeasurement;

    algo_parameters  algo_para;
    string           camera_id;
    BYTETracker      bytetrack;
    bytetrack_params params;
};

class truckDetector : public Truckoccupylane
{
public:
    truckDetector();
    ~truckDetector();
    PV_ERROR_CODE cardetect(std::vector<bm_image> batch_imgs, std::vector<YoloV8BoxVec>& boxes);
    PV_ERROR_CODE cartrack(std::vector<std::vector<BoundingBox>> yolov8_boxes_BoundingBox, vector<STracks>& results_stracks, illegal_data& one_illegal_data);
    PV_ERROR_CODE platerecognition(cv::Mat plate_img, vector<string>& results);
    PV_ERROR_CODE img_merge(vector<vehicle_output_data>& output_data, int detect_lane, vector<cv::Point> contour, illegal_data& one_illegal_data);
    PV_ERROR_CODE inference_test(std::string para_data, vector<json>& results_json);
    PV_ERROR_CODE postprocess_occupyleftlane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json,
                                             illegal_data& one_illegal_data, int flag_is_C);
    PV_ERROR_CODE postprocess_occupysecondlane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, vector<laneElements> contour_rightlane, std::vector<YoloV8Box> boundingbox,
                                               STracks output_stracks, vector<json>& results_json, illegal_data& one_illegal_data);
    PV_ERROR_CODE postprocess_occupyemergencelane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json,
                                                  illegal_data& one_illegal_data);
    PV_ERROR_CODE postprocess_speedmeasurement(vector<illegal_data> m_illegal_data, double distance, double speed_limit);
    PV_ERROR_CODE postprocess_interval_detection(vector<illegal_data> m_illegal_data);
    PV_ERROR_CODE postprocess_occupyemergencelane_driving(cv::Mat frame_to_draw, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json,
                                                          illegal_data& one_illegal_data);

    PV_ERROR_CODE postprocess_occupyemergencelane_parking(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks,
                                                          vector<json>& results_json, illegal_data& one_illegal_data);
    //    PV_ERROR_CODE postprocess_occupysecondlane_interval_detection(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks,
    //                                                                  vector<json>& results_json, illegal_data& one_illegal_data, vector<vehicle_detect_data> detection_data_speedmeasurement_temp);

private:
    YoloV8*              m_truckyolov8;
    PLATENET*            m_platenet;
    PhoneSafetyBelt      m_phone_safetybelt;
    vector<illegal_data> m_illegal_data;
    // std::shared_ptr<spdlog::logger> logger_truck;
};

#endif   // TRUCKDETECTOR_H
