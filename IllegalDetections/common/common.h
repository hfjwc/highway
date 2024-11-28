#include "opencv2/opencv.hpp"
#include "yolov8.hpp"
#include <string>
#include <vector>
using namespace std;
using namespace cv;

enum PV_ERROR_CODE
{
    PV_ERROR_OK,   // no execution error
    PV_ERROR_EXCEPTION
};

//车牌数据
struct plate_data
{
    cv::Mat vehicle_plate_image;
    int     plate_id;
    float   vehicle_plate_recognition_confidence;   //车牌号置信度
};
//卡口数据
struct bayonet_data
{
    cv::Mat      bayonet_vehicle_image;
    string       bayonet_plate_number;
    Rect_<float> bayonet_plate_box;
    Rect_<float> bayonet_vehicle_box;
};

//检测数据
struct vehicle_detect_data
{
    vector<cv::Mat>      vehicle_images;
    vector<bayonet_data> bayonet_vehicle_images;
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
};

//输出数据
struct vehicle_output_data
{
    vector<cv::Mat>      vehicle_images;              //包含4张车辆图像
    vector<plate_data>   vehicle_plate_images;        //车牌图像
    vector<cv::Rect>     vehicle_location_box;        //车辆位置
    vector<YoloV8BoxVec> phone_safetybelt_detected;   //用于记录接打电话或不系安全带情况
    string               illegal_code;                //违法行为（驾驶员接打电话 ， 驾驶员未系安全带）
    string               illegal_behavior;
    cv::Mat              plate_img;
    string               plate_color;
    string               plate_number;               //车牌号
    string               vehicle_type;               //车型
    float                vehicle_confidence;         //车辆位置置信度
    float                vehicle_plate_confidence;   //车牌位置置信度
    float                speed;                      //车辆速度
    double               start_time;

    int            track_id;   //跟踪ID
    vector<string> illegal_time;
    string         illegal_image_name;
};

struct laneElements
{
    int                    plateYmax;            //车牌检测位置
    int                    id;                   //车道号
    std::vector<int>       prohibitedVehicles;   //禁行的车辆种类
    std::vector<cv::Point> laneRegion;           //车道区域坐标
};

struct algo_parameters
{
    /*
     *  "1|1,2,4|34,85,67,143,89,278;2|4|718,0,914,2,617,830,3,813;/home/data/1.jpg,1,1000,1,0.8,1,0.7,3,0.85,0.8,5xdad324,4312,4608,1094,1101,1223,南京高速，abc223233,1"
     *
     * 如上是完整的参数示例，按顺序解析如下：
     * 车道之间使用;分隔，每个车道第一个部分表示车道号，第二部分表示禁行的车辆，第三部分表示车道区域坐标
     * 第一车道的车道号为1，第二车道的车道号为2，......，应急车道的车道号为0。注：车辆行驶方向的最左侧车道为第一车道
     * 车辆型号：小汽车为0，中型货车为1，大型货车为2，中型客车为3，大型客车为4，车型使用|与其他参数进行分隔
     * 如果一条车道没有禁行的车辆，则使用-1表示,如果是超车道则使用-2表示
     * 1|1,2,4|34,85,67,143,89,278  表示在第一车道内，禁行车辆为中型货车、大型货车、大型客车，34,85,67,143,89,278是车道区域坐标
     * 图片的路径是/home/data/1.jpg，
     * 1表示当前相机为第二个相机，
     * 取证的时间间隔是1000毫秒，
     * 1表示需要使用占用应急车道算法，
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
     * abc223233表示相机id
     * 1表示当前为禁止超车路段
     */

    std::vector<laneElements> lanes;                     //所有区域集合
    std::string               img_path;                  //图片路径
    std::string               img_time;                  //图像时间戳
    int                       camera_index;              //当前是第几个相机（用于C类多相机长距离判断车辆是否一直占道），0表示第一个相机，1表示第二个相机
    int                       timeInterval;              //取证的时间间隔，单位毫秒
    int                       isEmergencyLane;           //是否需要应急车道算法，0表示不需要，1表示需要
    float                     vehicle_conf;              //车辆检测的置信度
    int                       is_detect_plate;           //是否需要检测车牌，0表示不需要，1表示需要
    float                     plate_conf;                //车牌检测的置信度
    int                       is_detect_BeltAndPhone;    //是否需要检测安全带和打电话，0表示都不需要检测，1表示打电话，2表示不系安全带，3表示两种都需要
    float                     belt_conf;                 //安全带检测的置信度
    float                     phone_conf;                //打电话检测的置信度
    std::string               device_number;             //设备编号，盒子备案后会确定，每台盒子不一样
    std::string               illegal_code_occupyLane;   //不按规定车道行驶违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_occupyEmergencyLane;   //占用应急车道违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_safeDistance;          //未按规定保持车距违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_nobelt;                //不系安全带违法代码,根据算法功能确定，可能会改
    std::string               illegal_code_phone;                 //开车打电话违法代码,根据算法功能确定，可能会改
    std::string               camera_place;                       //违法地点，根据相机确定
    std::string               camera_id;                          //相机id
    int                       isNoOvertaking;                     //当前路段是否禁止超车，0表示不禁止，1表示禁止
    //卡口图像数据
    std::string  plate_no;              //卡口图像车牌号
    std::string  vehicle_coordinate;    //卡口图像车辆位置
    std::string  plate_no_coordinate;   //卡口图像车牌位置
    Rect_<float> kakou_car_box;
    Rect_<float> kakou_plate_box;
    std::string  bayonet_img_path;   //卡口图像路径
    cv::Mat      bayonet_img;        //卡口图像
    int          bayonet_lane_no;
};
