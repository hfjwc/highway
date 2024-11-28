#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>
#include "bytetrack.h"
#include "platenet.hpp"
#include "yolov8.hpp"
#include "json.hpp"
#include "opencv2/opencv.hpp"
#include "truckoccupylane.h"
using json = nlohmann::json;
using namespace std;
using namespace cv;

// define error codes
enum ERROR_CODE {
    ERROR_OK, // no execution error
    ERROR_EXCEPTION
};
/*
//检测数据
struct vehicle_detect_data {
    vector<cv::Mat> vehicle_image;
    vector<cv::Mat> vehicle_plate_images;
    vector<cv::Rect> vehicle_location_box;
    int class_id;
    int track_id;
    float vehicle_confidence;
    cv::Rect palte_location;
    double start_time;
    int start_nframe;
    int end_nframe;
    int is_overtime = 0; //判断是否到达第二次抓拍时间间隔
    int is_illegal_num = 0;
};

//输出数据
struct vehicle_output_data {
    vector<cv::Mat> vehicle_image; //包含4张车辆图像
    cv::Mat vehicle_plate_image; //车牌图像
    vector<cv::Rect> vehicle_location_box; //车辆位置
    string vehicle_plate; //车牌号
    string vehicle_type; //车型
    float vehicle_confidence; //车辆位置置信度
    float vehicle_plate__confidence; //车牌位置置信度
    float vehicle_plate_recognition_confidence; //车牌号置信度
    int track_id; //跟踪ID
};
*/
class P_S_DETECT
{
public:
    P_S_DETECT();
    ~P_S_DETECT();
    ERROR_CODE cardetect(std::vector<bm_image> batch_imgs, std::vector<YoloV8BoxVec>& boxes);
    ERROR_CODE cartrack(std::vector<std::vector<BoundingBox>> yolov8_boxes_BoundingBox, vector<STracks>& results_stracks);
    ERROR_CODE platerecognition(cv::Mat plate_img, vector<string>& results);
    ERROR_CODE p_s_detect(cv::Mat img, std::vector<YoloV8BoxVec>& boxes);  // 接打电话和不系安全带检测
    ERROR_CODE inference(int nframe, bm_image image, float confidence, int time_interval, vector<json>& results_json);
    ERROR_CODE postprocess(int nframe, cv::Mat frame_to_draw, int time_interval, std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json);

private:
    YoloV8* m_truckyolov8 = NULL;
    YoloV8* p_s_yolov8 = NULL;
    bytetrack_params params;
    BYTETracker bytetrack;
    PLATENET* m_platenet = NULL;
};

