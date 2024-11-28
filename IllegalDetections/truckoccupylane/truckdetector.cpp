#include "truckdetector.h"
std::shared_ptr<spdlog::logger> logger_truck;
int64_t                         getCurrentTimeInt64()
{
    auto        currentTime  = std::chrono::system_clock::now();
    std::time_t time         = std::chrono::system_clock::to_time_t(currentTime);
    std::tm*    timeinfo     = std::localtime(&time);
    int         year         = timeinfo->tm_year + 1900;
    int         month        = timeinfo->tm_mon + 1;
    int         day          = timeinfo->tm_mday;
    int         hour         = timeinfo->tm_hour;
    int         minute       = timeinfo->tm_min;
    int         second       = timeinfo->tm_sec;
    int         milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count() % 1000;
    int64_t     result       = year * 10000000000000LL + month * 100000000000LL + day * 1000000000LL + hour * 10000000LL + minute * 100000LL + second * 1000LL + milliseconds;
    return result;
}

truckDetector::truckDetector()
    : m_truckyolov8(NULL)
    , m_platenet(NULL)
{
    //创建保存路径
    string      save_path_log     = "/data2/camera-handler/results/log";
    string      save_path_log_txt = "/data2/camera-handler/results/log/不按规定车道行驶.txt";
    struct stat info;
    if (stat(save_path_log.c_str(), &info) != 0)
    {
        logger_truck->info("保存路径不存在，创建目录...");
        mkdir(save_path_log.c_str(), 0777);
    }

    int64_t currentTimeInt64_save_name = getCurrentTimeInt64();
    logger_truck                       = spdlog::rotating_logger_mt(to_string(currentTimeInt64_save_name) + "logger", save_path_log_txt, 100 * 1024 * 1024, 2, true);
    logger_truck->set_level(spdlog::level::trace);   //跟踪所有日志
    logger_truck->flush_on(spdlog::level::trace);    //刷新所有跟踪的日志
    logger_truck->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    // 添加终端输出目标
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    logger_truck->sinks().push_back(console_sink);

    // 1. yolov8车辆车牌检测模型加载和初始化
    string bmodel_file = "yolov8_car.bmodel";
    // creat handle
    int           dev_id = 0;
    BMNNHandlePtr handle = make_shared<BMNNHandle>(dev_id);
    // load bmodel
    shared_ptr<BMNNContext> bm_ctx = make_shared<BMNNContext>(handle, bmodel_file.c_str());

    // initialize net
    if (m_truckyolov8 == NULL)
        m_truckyolov8 = new YoloV8(bm_ctx);
    float       conf_thresh = 0.25;
    float       nms_thresh  = 0.7;
    std::string truck_names = "truck.names";
    CV_Assert(0 == m_truckyolov8->Init(conf_thresh, nms_thresh, truck_names));
    // 2. bytetrack跟踪参数初始化
    // bytetrack.init(params);

    // 3. platenet车牌识别模型加载和初始化
    string bmodel_file_platenet = "PlateNet.bmodel";
    // load bmodel
    shared_ptr<BMNNContext> bm_ctx_platenet = make_shared<BMNNContext>(handle, bmodel_file_platenet.c_str());

    // initialize net
    if (m_platenet == NULL)
        m_platenet = new PLATENET(bm_ctx_platenet);
    CV_Assert(0 == m_platenet->Init());
}

truckDetector::~truckDetector()
{
    delete m_truckyolov8;
    delete m_platenet;
    for (int i = 0; i < m_illegal_data.size(); i++)
    {
        m_illegal_data[i].bytetrack.clear();
    }
    // delete m_phone_safetybelt;
}

double GetIOU_rect(Rect_<float> bb_leftbox, Rect_<float> bb_truck)
{
    float in = (bb_truck & bb_leftbox).area();
    float un = bb_truck.area();
    if (un != 0)
    {
        return (double)(in / un);
    }
    else
    {
        return 0;
    }
}
cv::Mat get_split_merge(cv::Mat& img)   //双层车牌 分割 拼接
{
    cv::Rect upper_rect_area = cv::Rect(0, 0, img.cols, int(5.0 / 12 * img.rows));
    cv::Rect lower_rect_area = cv::Rect(0, int(1.0 / 3 * img.rows), img.cols, img.rows - int(1.0 / 3 * img.rows));
    cv::Mat  img_upper       = img(upper_rect_area);
    cv::Mat  img_lower       = img(lower_rect_area);
    cv::resize(img_upper, img_upper, img_lower.size());
    cv::Mat out(img_lower.rows, img_lower.cols + img_upper.cols, CV_8UC3, cv::Scalar(114, 114, 114));
    img_upper.copyTo(out(cv::Rect(0, 0, img_upper.cols, img_upper.rows)));
    img_lower.copyTo(out(cv::Rect(img_upper.cols, 0, img_lower.cols, img_lower.rows)));
    return out;
}
string getCurrentTimeInt64_save()
{
    auto        currentTime  = std::chrono::system_clock::now();
    std::time_t time         = std::chrono::system_clock::to_time_t(currentTime);
    std::tm*    timeinfo     = std::localtime(&time);
    int         year         = timeinfo->tm_year + 1900;
    int         month        = timeinfo->tm_mon + 1;
    int         day          = timeinfo->tm_mday;
    int         hour         = timeinfo->tm_hour;
    int         minute       = timeinfo->tm_min;
    int         second       = timeinfo->tm_sec;
    int         milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()).count() % 1000;
    // int64_t     result       = year * 10000000000000LL + month * 100000000000LL + day * 1000000000LL + hour * 10000000LL + minute * 100000LL + second * 1000LL + milliseconds;
    string detection_time;
    string hour_str, minute_str, second_str;
    if (hour < 10)
    {
        hour_str = "0" + to_string(hour);
    }
    else
    {
        hour_str = to_string(hour);
    }

    if (minute < 10)
    {
        minute_str = "0" + to_string(minute);
    }
    else
    {
        minute_str = to_string(minute);
    }

    if (second < 10)
    {
        second_str = "0" + to_string(second);
    }
    else
    {
        second_str = to_string(second);
    }
    detection_time = to_string(year) + "年" + to_string(month) + "月" + to_string(day) + "日 " + hour_str + ":" + minute_str + ":" + second_str + "." + to_string(milliseconds);

    return detection_time;
}

string getCurrentTimeInt64_save2(string dateTimeStr)
{
    int year         = std::stoi(dateTimeStr.substr(0, 4));   // 年份
    int month        = std::stoi(dateTimeStr.substr(4, 2));   // 月份
    int day          = std::stoi(dateTimeStr.substr(6, 2));   // 日期
    int hour         = std::stoi(dateTimeStr.substr(8, 2));
    int minute       = std::stoi(dateTimeStr.substr(10, 2));
    int second       = std::stoi(dateTimeStr.substr(12, 2));
    int milliseconds = std::stoi(dateTimeStr.substr(14, 3));

    // int64_t     result       = year * 10000000000000LL + month * 100000000000LL + day * 1000000000LL + hour * 10000000LL + minute * 100000LL + second * 1000LL + milliseconds;
    string detection_time;
    string hour_str, minute_str, second_str;
    if (hour < 10)
    {
        hour_str = "0" + to_string(hour);
    }
    else
    {
        hour_str = to_string(hour);
    }

    if (minute < 10)
    {
        minute_str = "0" + to_string(minute);
    }
    else
    {
        minute_str = to_string(minute);
    }

    if (second < 10)
    {
        second_str = "0" + to_string(second);
    }
    else
    {
        second_str = to_string(second);
    }
    detection_time = to_string(year) + "年" + to_string(month) + "月" + to_string(day) + "日 " + hour_str + ":" + minute_str + ":" + second_str + "." + to_string(milliseconds);

    return detection_time;
}

std::string generate_random_string(int length)
{
    std::string                     charset = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    std::string                     result;
    std::random_device              rd;
    std::mt19937                    gen(rd());
    std::uniform_int_distribution<> dis(0, charset.size() - 1);

    for (int i = 0; i < length; ++i)
    {
        result += charset[dis(gen)];
    }

    return result;
}

//车牌车辆检测
PV_ERROR_CODE truckDetector::cardetect(std::vector<bm_image> batch_imgs, std::vector<YoloV8BoxVec>& boxes)
{
    //(1) yolov8车牌车辆检测
    // get batch_size
    int batch_size = m_truckyolov8->batch_size();
    if ((int)batch_imgs.size() == batch_size)
    {
        // predict
        CV_Assert(0 == m_truckyolov8->Detect(batch_imgs, boxes));
    }
    return PV_ERROR_OK;
}

//车牌车辆跟踪
PV_ERROR_CODE truckDetector::cartrack(std::vector<std::vector<BoundingBox>> yolov8_BoundingBoxes, vector<STracks>& results_stracks, illegal_data& one_illegal_data)
{
    //(2) bytetrack跟踪
    for (size_t i = 0; i < yolov8_BoundingBoxes.size(); i++)
    {
        std::vector<BoundingBox> yolov8_boxes_BoundingBox = yolov8_BoundingBoxes[i];
        STracks                  output_stracks;
        one_illegal_data.bytetrack.update(output_stracks, yolov8_boxes_BoundingBox);
        results_stracks.push_back(output_stracks);
    }
    return PV_ERROR_OK;
}
//车牌识别
PV_ERROR_CODE truckDetector::platerecognition(cv::Mat plate_img, vector<string>& results)
{
    //(3) 车牌识别
    vector<bm_image> batch_imgs_plate;
    bm_image         bmimage_plate;
    bm_status_t      bmret = cv::bmcv::toBMI(plate_img, &bmimage_plate, true);

    if (bmret != BM_SUCCESS)
    {
        return PV_ERROR_EXCEPTION;
    }
    batch_imgs_plate.push_back(bmimage_plate);
    // get batch_size
    int batch_size_plate = m_platenet->batch_size();
    if ((int)batch_imgs_plate.size() == batch_size_plate)
    {
        CV_Assert(0 == m_platenet->Detect(batch_imgs_plate, results));
    }
    return PV_ERROR_OK;
}

//后端参数解析
algo_parameters ParseParameters(QJsonObject& jsonObj)
{
    algo_parameters algo_para;
    algo_para.lanes.clear();
    algo_para.bayonet_img_path    = "";
    algo_para.vehicle_coordinate  = "";
    algo_para.plate_no_coordinate = "";
    algo_para.plate_no            = "";
    algo_para.img_path            = "";
    QJsonArray regionsArray       = jsonObj["regions"].toArray();
    for (auto regionValue : regionsArray)
    {
        QJsonObject  regionObj = regionValue.toObject();
        laneElements laneEle;

        // 解析 carType
        QJsonArray carTypeArray = regionObj["carType"].toArray();
        for (const QJsonValue& carTypeValue : carTypeArray)
        {
            laneEle.prohibitedVehicles.push_back(carTypeValue.toString().toInt());
        }

        // 解析 laneNo
        laneEle.id = regionObj["laneNo"].toString().toInt();
        if (laneEle.id == 0)
        {
            laneEle.id = 100;
        }

        // 解析 coordinates
        QJsonArray coordinatesArray = regionObj["coordinates"].toArray();
        for (auto coordinateValue : coordinatesArray)
        {
            QJsonArray coordinateArray = coordinateValue.toArray();
            if (coordinateArray.size() == 2)
            {
                cv::Point lanepoint = {coordinateArray[0].toInt(), coordinateArray[1].toInt()};
                laneEle.laneRegion.push_back(lanepoint);
            }
        }

        algo_para.lanes.push_back(laneEle);
    }
    if (jsonObj.contains("img_path"))
    {
        algo_para.img_path = jsonObj["img_path"].toString().toStdString();   //图片路径
    }
    else
    {
        algo_para.img_path = "";
    }
    algo_para.camera_index = jsonObj["camera_index"].toInt();   //当前是第几个相机（用于C类多相机长距离判断车辆是否一直占道），0表示第一个相机，1表示第二个相机
    algo_para.timeInterval    = jsonObj["timeInterval"].toString().toInt();   //取证时间间隔，单位毫秒
    algo_para.isEmergencyLane = jsonObj["isEmergencyLane"].toInt();           //是否需要应急车道算法，0表示不需要，1表示需要
    algo_para.vehicle_conf    = jsonObj["vehicle_conf"].toDouble();           //车辆检测的置信度
    algo_para.is_detect_plate = jsonObj["is_detect_plate"].toInt();           //是否需要检测车牌，0表示不需要，1表示需要
    algo_para.plate_conf      = jsonObj["plate_conf"].toDouble();             //车牌检测的置信度
    algo_para.is_detect_BeltAndPhone = jsonObj["is_detect_BeltAndPhone"].toInt();   //是否需要检测安全带和打电话，0表示都不需要检测，1表示打电话，2表示不系安全带，3表示两种都需要
    algo_para.belt_conf                        = jsonObj["belt_conf"].toDouble();                                        //安全带检测的置信度
    algo_para.phone_conf                       = jsonObj["phone_conf"].toDouble();                                       //打电话检测的置信度
    algo_para.device_number                    = jsonObj["device_number"].toString().toStdString();                      //设备编号，盒子备案后会确定，每台盒子不一样
    algo_para.illegal_code_occupyLane          = jsonObj["illegal_code_occupyLane"].toString().toStdString();            //不按规定车道行驶违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_occupyEmergencyLane = jsonObj["illegal_code_occupyEmergencyLane"].toString().toStdString();   //占用应急车道违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_safeDistance        = jsonObj["illegal_code_safeDistance"].toString().toStdString();          //未按规定保持车距违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_nobelt              = jsonObj["illegal_code_nobelt"].toString().toStdString();                //不系安全带违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_phone               = jsonObj["illegal_code_phone"].toString().toStdString();                 //开车打电话违法代码,根据算法功能确定，可能会改
    algo_para.camera_place                     = jsonObj["camera_place"].toString().toStdString();                       //违法地点，根据相机确定
    algo_para.camera_id                        = jsonObj["camera_id"].toString().toStdString();
    algo_para.isNoOvertaking                   = jsonObj["isNoOvertaking"].toInt();   //当前路段是否禁止超车，0表示不禁止，1表示禁止//相机id
    //卡口数据解析
    if (jsonObj.contains("plate_no"))
    {
        algo_para.plate_no = jsonObj["plate_no"].toString().toStdString();   //卡口图像车牌号
    }
    else
    {
        algo_para.plate_no = "";   //不是卡口图像
    }
    if (jsonObj.contains("vehicle_coordinate"))
    {
        algo_para.vehicle_coordinate = jsonObj["vehicle_coordinate"].toString().toStdString();   //卡口图像车辆位置
    }
    else
    {
        algo_para.vehicle_coordinate = "";   //不是卡口图像
    }

    if (jsonObj.contains("plate_no_coordinate"))
    {
        algo_para.plate_no_coordinate = jsonObj["plate_no_coordinate"].toString().toStdString();   //卡口图像车牌位置
    }
    else
    {
        algo_para.plate_no_coordinate = "";   //不是卡口图像
    }

    if (jsonObj.contains("unv_img_path"))
    {
        algo_para.bayonet_img_path = jsonObj["unv_img_path"].toString().toStdString();   //卡口图像路径
    }
    else
    {
        algo_para.bayonet_img_path = "";   //不是卡口图像
    }

    if (jsonObj.contains("lane_no"))
    {
        algo_para.bayonet_lane_no = jsonObj["lane_no"].toString().toInt();   //卡口车辆车道号
    }
    else
    {
        algo_para.bayonet_lane_no = -1;   //不是卡口图像
    }

    return algo_para;
}

//高速匝道口-应急车道行车
PV_ERROR_CODE truckDetector::postprocess_occupyemergencelane_driving(cv::Mat frame_to_draw, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json,
                                                                     illegal_data& one_illegal_data)
{
    //判断图像是否为空
    if (frame_to_draw.empty())
    {
        return PV_ERROR_EXCEPTION;
    }
    //根据检测和跟踪结果获取track_id和预测框，判定是否违法
    for (auto& bbox : output_stracks)
    {
        Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};
        //判断车辆下边界中点是否在多边形区域内
        cv::Point center_point     = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1] + bbox->det_tlwh[3]);
        double    is_inside_center = -1;
        try
        {
            is_inside_center = pointPolygonTest(lane.laneRegion, center_point, true);
        }
        catch (...)
        {
        }

        int is_illegal = 0;         //判定当前车辆是否违法标志，违法为1，不违法为0
        if (is_inside_center > 0)   //车辆下边界中点在区域内
        {
            is_illegal = 1;
        }
        logger_truck->info("is_inside_center: {}", is_inside_center);
        logger_truck->info("is_illegal: {}", is_illegal);
        logger_truck->info("detection_data_emergence.size(): {}", one_illegal_data.detection_data_emergence.size());

        ////////////////////////////////////////////////////Part1:车辆第一次违法抓拍//////////////////////////////////////////////////////////
        //(a)对第一次进入违法区域的车辆进行记录，并记录第一次抓拍时间
        if (is_illegal == 1)
        {
            //(a1.1)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_detect = 0;
            for (int j = 0; j < one_illegal_data.detection_data_emergence.size(); j++)
            {
                if (one_illegal_data.detection_data_emergence[j].track_id == bbox->track_id)
                {
                    //对于已经出现违法，但是未形成完整违法数据的车辆，记录其从开始违法到最终违法的违法总次数，总次数大于给定阈值，则判定最终违法
                    one_illegal_data.detection_data_emergence[j].is_illegal_num = one_illegal_data.detection_data_emergence[j].is_illegal_num + 1;
                    one_illegal_data.detection_data_emergence[j].end_nframe     = one_illegal_data.nframe;
                    if ((one_illegal_data.detection_data_emergence[j].end_nframe - one_illegal_data.detection_data_emergence[j].start_nframe + 1) <= 5)
                    {
                        if (one_illegal_data.detection_data_emergence[j].is_illegal_num <
                            (one_illegal_data.detection_data_emergence[j].end_nframe - one_illegal_data.detection_data_emergence[j].start_nframe + 1))
                        {
                            //对第一次违法做统计分析，如果连续5帧违法则继续跟踪记录，不需要更新第一次抓拍记录，如果连续5帧出现不违法，则更新第一次抓拍记录
                            if (one_illegal_data.detection_data_emergence[j].vehicle_images.size() > 0)
                            {
                                one_illegal_data.detection_data_emergence[j].vehicle_images.clear();
                                one_illegal_data.detection_data_emergence[j].vehicle_location_box.clear();
                                one_illegal_data.detection_data_emergence[j].illegal_time.clear();
                                // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(frame_to_draw);
                                one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                                double begin                                                = (double)cv::getTickCount();
                                double start_time                                           = begin * 1000 / cv::getTickFrequency();
                                one_illegal_data.detection_data_emergence[j].start_time     = start_time;
                                one_illegal_data.detection_data_emergence[j].is_illegal_num = 1;
                                one_illegal_data.detection_data_emergence[j].start_nframe   = one_illegal_data.nframe;
                            }
                        }
                    }
                    flag_temp_detect = 1;
                }
            }

            //(a1.2)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_illegal = 0;
            for (int j = 0; j < one_illegal_data.illegal_trackid_emergence.size(); j++)
            {
                if (one_illegal_data.illegal_trackid_emergence[j] == bbox->track_id)
                {
                    flag_temp_illegal = 1;
                }
            }
            //(a2)超过指定时间间隔，如果检测数据违法，会被存入输出数据中并在检测数据中清除，如果检测数据未违法，则会被清除；
            //检测数据已经被清除，如果输出数据中已经存在该track_id, 则不再重复记录
            int flag_temp_output = 0;
            for (int j = 0; j < one_illegal_data.output_data_emergence.size(); j++)
            {
                if (one_illegal_data.output_data_emergence[j].track_id == bbox->track_id)
                {
                    flag_temp_output = 1;
                }
            }
            //(a3)车辆第一次进入违法区域，记录进入时间点,并获取对应的车牌号图像
            if (flag_temp_detect == 0 && flag_temp_output == 0 && flag_temp_illegal == 0)
            {
                vehicle_detect_data one_detect;

                //第一次抓拍后车辆数据记录
                double begin          = (double)cv::getTickCount();
                double start_time     = begin * 1000 / cv::getTickFrequency();
                one_detect.start_time = start_time;   //车辆违法第一次抓拍时间
                // check time first
                // one_detect.illegal_time.push_back(getCurrentTimeInt64_save());
                one_detect.illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                one_detect.vehicle_images.push_back(frame_to_draw);                                                                                                //第一张违法图像
                one_detect.vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                one_detect.track_id       = bbox->track_id;                                                                                                        //车辆跟踪ID
                one_detect.class_id       = bbox->class_id;                                                                                                        //车辆类型
                one_detect.is_illegal_num = one_detect.is_illegal_num + 1;
                one_detect.start_nframe   = one_illegal_data.nframe;
                logger_truck->info("first illegal image");
                one_illegal_data.detection_data_emergence.push_back(one_detect);
            }
        }

        ///////////////////////////////////////////////////Part2:车辆第一次违法后，继续跟踪抓拍///////////////////////////////////////////////////////
        //对于第一次违法车辆，继续跟踪抓拍违法图像，直至到达指定位置，停止抓拍
        for (int j = 0; j < one_illegal_data.detection_data_emergence.size(); j++)
        {
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //(b)抓拍到第一张违法图像后继续跟踪抓拍违法图像
            if (one_illegal_data.detection_data_emergence[j].is_overtime == 0)
            {
                if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id)
                {
                    if (is_illegal == 1)
                    {
                        logger_truck->info("second illegal image");
                        // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                        one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                        one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(frame_to_draw);
                        one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                            Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                    }
                }
            }
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //(c)确定车辆到达指定位置，抓拍最后一张违法图像
            if (center_point.y > lane.plateYmax)
            {
                //当车辆到达90%plateYmax位置，停止违法抓拍
                //最后一张违法图像
                if (one_illegal_data.detection_data_emergence[j].is_overtime == 0)
                {
                    if (is_illegal == 1)
                    {
                        if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id)
                        {
                            one_illegal_data.detection_data_emergence[j].is_overtime = 1;   //停止抓拍的标志
                            logger_truck->info("fourth illegal image");
                            //获取违法图像对应的时间， 如果视频流自带时间则使用视频流时间，如果没有时间则使用系统时间
                            // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                            one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                            one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(frame_to_draw);
                            one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                                Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                        }
                    }
                }
            }
        }
    }

    /////////////////////////////////////////////////////////Part3:车辆消失后数据清除，避免数据存储持续增大////////////////////////////////////////////////
    auto removed_stracks = one_illegal_data.bytetrack.get_removed_stracks();
    //记录完整违法车辆id，若图像中该id消失，则清除该id
    for (auto& bbox : removed_stracks)
    {
        for (vector<int>::iterator it = one_illegal_data.illegal_trackid_emergence.begin(); it != one_illegal_data.illegal_trackid_emergence.end(); it++)
        {
            if (*it == bbox->track_id)
            {
                one_illegal_data.illegal_trackid_emergence.erase(it);
                it--;
            }
        }
        for (vector<vehicle_detect_data>::iterator it_it = one_illegal_data.detection_data_emergence.begin(); it_it != one_illegal_data.detection_data_emergence.end(); it_it++)
        {
            if (it_it->track_id == bbox->track_id)
            {
                one_illegal_data.detection_data_emergence.erase(it_it);
                it_it--;
            }
        }
    }
    /////////////////////////////////////////////////////////Part4:确定形成完整违法，输出四合一图像////////////////////////////////////////////////
    //输出结果画框，四合一
    logger_truck->info("start img_merge");
    if (one_illegal_data.output_data_emergence.size() > 0)
    {
        img_merge(one_illegal_data.output_data_emergence, 1, lane.laneRegion, one_illegal_data);
    }

    logger_truck->info("end img_merge");

    for (auto& output : one_illegal_data.output_data_emergence)
    {
        // save result
        json output_json;
        output_json["illegal_image_name"] = output.illegal_image_name;
        output_json["illegal_time"]       = output.illegal_time[3];
        output_json["illegal_place"]      = one_illegal_data.algo_para.camera_place;
        ////////////////////////////////////////////////////
        output_json["plate_number"]     = output.plate_number;
        output_json["plate_color"]      = output.plate_color;
        output_json["vehicle_type"]     = output.vehicle_type;
        output_json["illegal_code"]     = one_illegal_data.algo_para.illegal_code_occupyEmergencyLane;
        output_json["illegal_behavior"] = "高速匝道口应急车道行车";
        ////////////////////////////////////////////////////
        output_json["device_number"] = one_illegal_data.algo_para.device_number;
        output_json["camera_id"]     = one_illegal_data.algo_para.camera_id;

        results_json.push_back(output_json);
    }

    one_illegal_data.output_data_emergence.clear();
    return PV_ERROR_OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//不按规定车道行驶-占用应急车道
PV_ERROR_CODE truckDetector::postprocess_occupyemergencelane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks,
                                                             vector<json>& results_json, illegal_data& one_illegal_data)
{
    int time_interval = one_illegal_data.algo_para.timeInterval;
    if (frame_to_draw.empty() || img_origin.empty())
    {
        return PV_ERROR_EXCEPTION;
    }

    logger_truck->info("output_stracks.size(): {}", output_stracks.size());
    //根据检测和跟踪结果获取track_id和预测框，判定是否违法
    for (auto& bbox : output_stracks)
    {
        Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};
        //(a)判断车辆下边界中点是否在多边形区域内
        cv::Point center_point     = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1] + bbox->det_tlwh[3]);
        double    is_inside_center = -1;
        try
        {
            is_inside_center = pointPolygonTest(lane.laneRegion, center_point, true);
        }
        catch (...)
        {
        }

        logger_truck->info("is_inside_center: {}", is_inside_center);

        int is_illegal = 0;   //判定当前车辆是否违法标志，违法为1，不违法为0
        //(b)判定货车、载客汽车是否进入违法区域，对于违法车辆进行第一次抓拍
        if (bbox->class_id == 1 || bbox->class_id == 2 || bbox->class_id == 0)
        {
            if (is_inside_center > 0)   //车辆下边界中点在区域内
            {
                is_illegal = 1;
            }
            logger_truck->info("is_illegal: {}", is_illegal);

            logger_truck->info("detection_data_emergence.size(): {}", one_illegal_data.detection_data_emergence.size());

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //(e)对于第一次违法车辆，继续跟踪，等待到达指定的时间间隔，开始第二次抓拍
            //到达时间间隔5秒后，判定车辆是否形成完整违法，如果形成，则进行第二次抓拍并存入输出数据，每条输出数据包含一辆车的完整违法证据，
            //主要包括：2张过程图像，1张车牌图像，车牌号码，车辆在图像中的位置,车辆类型，置信度
            //同一车辆第二次违法判断
            //到达时间间隔5秒，判定是否可以形成完整违法证据，若可以则存入输出数据，若不可以，则删除该检测数据
            for (int j = 0; j < one_illegal_data.detection_data_emergence.size(); j++)
            {
                //确定抓取车牌对应的图像
                // if (center_point.y > lane.plateYmax)
                if (one_illegal_data.algo_para.plate_no != "" && one_illegal_data.algo_para.vehicle_coordinate != "")
                {
                    //当出现卡口图像，计算触发卡口图像的车辆位置和违法车辆位置关系
                    //计算违法车辆和卡口车辆的iou
                    double ratio_kakou = GetIOU_rect(one_illegal_data.algo_para.kakou_car_box, bb_truck);
                    logger_truck->info("ratio_kakou: {}", ratio_kakou);
                    logger_truck->info("bbox->track_id: {}", bbox->track_id);
                    logger_truck->info("one_illegal_data.detection_data_emergence[j].track_id: {}", one_illegal_data.detection_data_emergence[j].track_id);

                    //第四张违法图像
                    if (one_illegal_data.detection_data_emergence[j].is_overtime == 0 && ratio_kakou > 0.8)
                    {
                        one_illegal_data.detection_data_emergence[j].end_nframe = one_illegal_data.nframe;

                        //到达时间间隔5秒，同一个track_id的车辆依然在违法区域内，则抓拍第四张图像
                        // if (is_illegal == 1)//第四张图像不需要一定违法
                        {
                            if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id)   //同一车辆间隔5秒后违法
                            {
                                one_illegal_data.detection_data_emergence[j].is_overtime = 1;   //记录时间超过5秒，如果未抓拍到第四张违法图像，则删除该检测数据
                                logger_truck->info("fourth illegal image");

                                // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));

                                //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                                bayonet_data one_bayonet;
                                if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                                {
                                    // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                                    one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                                    one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                                    one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                                    one_illegal_data.detection_data_emergence[j].bayonet_vehicle_images.push_back(one_bayonet);
                                }
                            }
                        }
                    }

                    // if (is_illegal == 1)
                    {
                        if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id && one_illegal_data.detection_data_emergence[j].vehicle_plate_images.size() <= 0)
                        {
                            logger_truck->info("illegal plate image");
                            int yy = 0;
                            //获取违法车辆对应的车牌
                            for (auto& b : boundingbox)
                            {
                                cv::Rect box_plate = {(int)b.x1, (int)b.y1, (int)(b.x2 - b.x1), (int)(b.y2 - b.y1)};

                                if (b.class_id == 3 || b.class_id == 4 || b.class_id == 5 || b.class_id == 6 || b.class_id == 8 || b.class_id == 9)
                                {
                                    Rect_<float> bb_plate;
                                    bb_plate.x      = box_plate.x;
                                    bb_plate.y      = box_plate.y;
                                    bb_plate.width  = box_plate.width;
                                    bb_plate.height = box_plate.height;

                                    //计算车辆和车牌的iou
                                    double ratio_plate = GetIOU_rect(bb_truck, bb_plate);

                                    if (ratio_plate > 0.99 && (bb_plate.y + bb_plate.height) > yy)
                                    {
                                        if (bb_plate.width > 16 && bb_plate.height > 16)
                                        {
                                            try
                                            {
                                                yy                   = bb_plate.y + bb_plate.height;
                                                cv::Mat    plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                plate_data one_plate;
                                                one_plate.vehicle_plate_image = plate_img;
                                                one_plate.plate_id            = b.class_id;
                                                one_illegal_data.detection_data_emergence[j].vehicle_plate_images.push_back(one_plate);
                                            }
                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                        else
                                        {
                                            try
                                            {
                                                yy                = bb_plate.y + bb_plate.height;
                                                cv::Mat plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                Mat     dst_plate;
                                                resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                plate_data one_plate;
                                                one_plate.vehicle_plate_image = dst_plate;
                                                one_plate.plate_id            = b.class_id;
                                                one_illegal_data.detection_data_emergence[j].vehicle_plate_images.push_back(one_plate);
                                            }

                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                double end      = (double)cv::getTickCount();
                double end_time = end * 1000 / cv::getTickFrequency();
                //第二张违法图像
                if (one_illegal_data.detection_data_emergence[j].is_overtime == 0)
                {

                    if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id)
                    {
                        //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                        bayonet_data one_bayonet;
                        if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                        {
                            // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                            one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                            one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                            one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                            one_illegal_data.detection_data_emergence[j].bayonet_vehicle_images.push_back(one_bayonet);
                        }
                        /////////////////////////////////////////////////////////////////////////////////////////////////
                        if (is_illegal == 1)   //同一车辆间隔5秒后违法
                        {
                            logger_truck->info("second illegal image");
                            // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                            one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                            one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(img_origin);
                            one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                                Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                        }
                    }
                }
                if (one_illegal_data.detection_data_emergence[j].vehicle_images.size() >= 4 && one_illegal_data.detection_data_emergence[j].is_overtime == 1)
                {
                    if (is_illegal == 1)
                    {
                        int flag_illegal_trackid = 0;
                        for (int jj = 0; jj < one_illegal_data.illegal_trackid_emergence.size(); jj++)
                        {
                            if (one_illegal_data.detection_data_emergence[j].track_id == one_illegal_data.illegal_trackid_emergence[jj])
                            {
                                flag_illegal_trackid = 1;
                            }
                        }
                        if (bbox->track_id == one_illegal_data.detection_data_emergence[j].track_id && flag_illegal_trackid == 0)   //同一车辆间隔5秒后违法
                        {
                            if (one_illegal_data.detection_data_emergence[j].vehicle_images.size() >= 4 && one_illegal_data.detection_data_emergence[j].vehicle_plate_images.size() > 0)
                            {
                                vehicle_output_data one_output;   //形成完整违法，获取完整输出数据
                                for (int idx = 0; idx < one_illegal_data.detection_data_emergence[j].vehicle_images.size(); idx++)
                                {
                                    one_output.vehicle_images.push_back(one_illegal_data.detection_data_emergence[j].vehicle_images[idx]);
                                    one_output.vehicle_location_box.push_back(one_illegal_data.detection_data_emergence[j].vehicle_location_box[idx]);
                                }
                                one_output.illegal_time = one_illegal_data.detection_data_emergence[j].illegal_time;
                                one_output.track_id     = bbox->track_id;

                                //确定车型
                                if (one_illegal_data.detection_data_emergence[j].class_id == 1)
                                {
                                    one_output.vehicle_type = "载客汽车";
                                }
                                else
                                {
                                    if (one_illegal_data.detection_data_emergence[j].class_id == 2)
                                    {
                                        one_output.vehicle_type = "载货汽车";
                                    }
                                    else
                                    {
                                        one_output.vehicle_type = "小汽车";
                                    }
                                }
                                //根据检测数据中的车牌图像获取输出数据中的车牌图像
                                one_output.vehicle_plate_images = one_illegal_data.detection_data_emergence[j].vehicle_plate_images;
                                //车牌识别
                                vector<string> results;
                                if (one_output.vehicle_plate_images.size() > 0)
                                {
                                    int plate_size       = one_output.vehicle_plate_images.size();
                                    one_output.plate_img = one_output.vehicle_plate_images[plate_size - 1].vehicle_plate_image;

                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                    {
                                        cv::Mat plate_img_split = get_split_merge(one_output.plate_img);
                                        platerecognition(plate_img_split, results);
                                    }
                                    else
                                    {
                                        platerecognition(one_output.plate_img, results);
                                    }
                                    if (results.size() > 0)
                                    {
                                        logger_truck->info("车牌号: {}", results[0]);
                                        logger_truck->info("车牌号-卡口: {}", one_illegal_data.algo_para.plate_no);

                                        if (one_illegal_data.algo_para.plate_no != "")
                                        {
                                            one_output.plate_number = one_illegal_data.algo_para.plate_no;   //使用自动识别的车牌号
                                        }
                                        else
                                        {
                                            // one_output.plate_number = results[0];   //使用自动识别的车牌号
                                            one_output.plate_number = "";
                                        }

                                        /////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        //根据卡口图获取车牌图像

                                        Rect_<float> bb_plate = one_illegal_data.algo_para.kakou_plate_box;

                                        logger_truck->info("bb_plate.x : {}", bb_plate.x);
                                        logger_truck->info("bb_plate.y : {}", bb_plate.y);
                                        logger_truck->info("bb_plate.width : {}", bb_plate.width);
                                        logger_truck->info("bb_plate.height : {}", bb_plate.height);
                                        if (bb_plate.width > 16 && bb_plate.height > 16)
                                        {
                                            try
                                            {
                                                int     size_2    = one_illegal_data.detection_data_emergence[j].vehicle_images.size() - 1;
                                                cv::Mat plate_img = one_illegal_data.detection_data_emergence[j].vehicle_images[size_2](
                                                    Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                int size_1 = one_illegal_data.detection_data_emergence[j].vehicle_plate_images.size() - 1;
                                                one_illegal_data.detection_data_emergence[j].vehicle_plate_images[size_1].vehicle_plate_image = plate_img;
                                                //替换车牌图像为卡口图像中的车牌图像
                                                one_output.plate_img = plate_img;
                                            }
                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                        else
                                        {
                                            try
                                            {
                                                int     size_2    = one_illegal_data.detection_data_emergence[j].vehicle_images.size() - 1;
                                                cv::Mat plate_img = one_illegal_data.detection_data_emergence[j].vehicle_images[size_2](
                                                    Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                Mat dst_plate;
                                                resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                int size_1 = one_illegal_data.detection_data_emergence[j].vehicle_plate_images.size() - 1;
                                                one_illegal_data.detection_data_emergence[j].vehicle_plate_images[size_1].vehicle_plate_image = dst_plate;
                                                //替换车牌图像为卡口图像中的车牌图像
                                                one_output.plate_img = dst_plate;
                                            }

                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                    }

                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 3)
                                    {
                                        one_output.plate_color = "蓝牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                    {
                                        one_output.plate_color = "双层黄牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 5)
                                    {
                                        one_output.plate_color = "单层黄牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 6)
                                    {
                                        one_output.plate_color = "绿牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 8)
                                    {
                                        one_output.plate_color = "白牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 9)
                                    {
                                        one_output.plate_color = "双层绿牌";
                                    }
                                }
                                if (one_output.vehicle_images.size() >= 4 && !one_output.plate_img.empty() && (one_output.plate_number.size() == 9 || one_output.plate_number.size() == 10))
                                {
                                    one_illegal_data.output_data_emergence.push_back(one_output);
                                    one_illegal_data.illegal_trackid_emergence.push_back(one_illegal_data.detection_data_emergence[j].track_id);
                                }
                            }
                        }
                    }
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(c)对第一次进入违法区域的车辆进行记录，并记录第一次抓拍时间
        if (is_illegal == 1)
        {

            //(c1.1)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_detect = 0;
            for (int j = 0; j < one_illegal_data.detection_data_emergence.size(); j++)
            {
                if (one_illegal_data.detection_data_emergence[j].track_id == bbox->track_id)
                {
                    //对于已经出现违法，但是未形成完整违法数据的车辆，记录其从开始违法到最终违法的违法总次数，总次数大于给定阈值，则判定最终违法
                    one_illegal_data.detection_data_emergence[j].is_illegal_num = one_illegal_data.detection_data_emergence[j].is_illegal_num + 1;
                    one_illegal_data.detection_data_emergence[j].end_nframe     = one_illegal_data.nframe;
                    if ((one_illegal_data.detection_data_emergence[j].end_nframe - one_illegal_data.detection_data_emergence[j].start_nframe + 1) <= 5)
                    {
                        if (one_illegal_data.detection_data_emergence[j].is_illegal_num <
                            (one_illegal_data.detection_data_emergence[j].end_nframe - one_illegal_data.detection_data_emergence[j].start_nframe + 1))
                        {
                            //对第一次违法做统计分析，如果连续5帧违法则继续跟踪记录，不需要更新第一次抓拍记录，如果连续5帧出现不违法，则更新第一次抓拍记录
                            if (one_illegal_data.detection_data_emergence[j].vehicle_images.size() > 0)
                            {
                                one_illegal_data.detection_data_emergence[j].vehicle_images.clear();
                                one_illegal_data.detection_data_emergence[j].vehicle_location_box.clear();
                                one_illegal_data.detection_data_emergence[j].illegal_time.clear();
                                // one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_emergence[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_emergence[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data_emergence[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                                double begin                                                = (double)cv::getTickCount();
                                double start_time                                           = begin * 1000 / cv::getTickFrequency();
                                one_illegal_data.detection_data_emergence[j].start_time     = start_time;
                                one_illegal_data.detection_data_emergence[j].is_illegal_num = 1;
                                one_illegal_data.detection_data_emergence[j].start_nframe   = one_illegal_data.nframe;
                            }
                        }
                    }
                    flag_temp_detect = 1;
                }
            }

            //(c1.2)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_illegal = 0;
            for (int j = 0; j < one_illegal_data.illegal_trackid_emergence.size(); j++)
            {
                if (one_illegal_data.illegal_trackid_emergence[j] == bbox->track_id)
                {
                    flag_temp_illegal = 1;
                }
            }
            //(c2)超过指定时间间隔，如果检测数据违法，会被存入输出数据中并在检测数据中清除，如果检测数据未违法，则会被清除；
            //检测数据已经被清除，如果输出数据中已经存在该track_id, 则不再重复记录
            int flag_temp_output = 0;
            for (int j = 0; j < one_illegal_data.output_data_emergence.size(); j++)
            {
                if (one_illegal_data.output_data_emergence[j].track_id == bbox->track_id)
                {
                    flag_temp_output = 1;
                }
            }
            //(c3)车辆第一次进入违法区域，记录进入时间点,并获取对应的车牌号图像
            if (flag_temp_detect == 0 && flag_temp_output == 0 && flag_temp_illegal == 0)
            {
                vehicle_detect_data one_detect;

                //第一次抓拍后车辆数据记录
                double begin          = (double)cv::getTickCount();
                double start_time     = begin * 1000 / cv::getTickFrequency();
                one_detect.start_time = start_time;   //车辆违法第一次抓拍时间
                // check time first
                // one_detect.illegal_time.push_back(getCurrentTimeInt64_save());
                one_detect.illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                one_detect.vehicle_images.push_back(img_origin);   //第一张违法图像
                //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                bayonet_data one_bayonet;
                if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                {
                    // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                    one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                    one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                    one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                    one_detect.bayonet_vehicle_images.push_back(one_bayonet);
                }

                one_detect.vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                one_detect.track_id       = bbox->track_id;                                                                                                        //车辆跟踪ID
                one_detect.class_id       = bbox->class_id;                                                                                                        //车辆类型
                one_detect.is_illegal_num = one_detect.is_illegal_num + 1;
                one_detect.start_nframe   = one_illegal_data.nframe;
                logger_truck->info("first illegal image");
                one_illegal_data.detection_data_emergence.push_back(one_detect);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto removed_stracks = one_illegal_data.bytetrack.get_removed_stracks();
    //记录完整违法车辆id，若图像中该id消失，则清除该id
    for (auto& bbox : removed_stracks)
    {
        for (vector<int>::iterator it = one_illegal_data.illegal_trackid_emergence.begin(); it != one_illegal_data.illegal_trackid_emergence.end(); it++)
        {
            if (*it == bbox->track_id)
            {
                one_illegal_data.illegal_trackid_emergence.erase(it);
                it--;
            }
        }
        for (vector<vehicle_detect_data>::iterator it_it = one_illegal_data.detection_data_emergence.begin(); it_it != one_illegal_data.detection_data_emergence.end(); it_it++)
        {
            if (it_it->track_id == bbox->track_id)
            {
                one_illegal_data.detection_data_emergence.erase(it_it);
                it_it--;
            }
        }
    }

    //输出结果画框，四合一
    logger_truck->info("start img_merge");
    if (one_illegal_data.output_data_emergence.size() > 0)
    {
        img_merge(one_illegal_data.output_data_emergence, 1, lane.laneRegion, one_illegal_data);
    }

    logger_truck->info("end img_merge");

    for (auto& output : one_illegal_data.output_data_emergence)
    {
        // save result
        json output_json;
        output_json["illegal_image_name"] = output.illegal_image_name;
        output_json["illegal_time"]       = output.illegal_time[3];
        output_json["illegal_place"]      = one_illegal_data.algo_para.camera_place;
        ////////////////////////////////////////////////////
        output_json["plate_number"]     = output.plate_number;
        output_json["plate_color"]      = output.plate_color;
        output_json["vehicle_type"]     = output.vehicle_type;
        output_json["illegal_code"]     = one_illegal_data.algo_para.illegal_code_occupyEmergencyLane;
        output_json["illegal_behavior"] = "占用应急车道行驶";
        ////////////////////////////////////////////////////
        output_json["device_number"] = one_illegal_data.algo_para.device_number;
        output_json["camera_id"]     = one_illegal_data.algo_para.camera_id;

        results_json.push_back(output_json);
    }

    one_illegal_data.output_data_emergence.clear();
    return PV_ERROR_OK;
}
//判断右侧车道是否为空
int is_empty_right_lane(vector<laneElements> contour_rightlane, std::vector<YoloV8Box> boundingbox, int h_img)
{
    //判断右侧车道是否空旷
    int is_empty_rightlane_final = -1;   //存在右侧车道为空旷
    if (contour_rightlane.size() > 0)
    {
        vector<int> flag_empty;
        for (auto& right_lane : contour_rightlane)
        {
            int is_empty_rightlane = 0;   //存在右侧车道为空旷
            for (auto& b : boundingbox)   //所有检测到的车辆,车牌在右侧车道均不可以出现
            {
                // if (b.class_id == 0 || b.class_id == 1 || b.class_id == 2)
                {
                    double is_inside_center_all = -1;
                    Point  center_point_all;   //车辆下边界中点
                    if ((h_img - b.y2) < 100)
                    {
                        center_point_all = Point((b.x2 + b.x1) / 2, b.y2 - 20);
                    }
                    else
                    {
                        center_point_all = Point((b.x2 + b.x1) / 2, b.y2);
                    }

                    is_inside_center_all = pointPolygonTest(right_lane.laneRegion, center_point_all, true);
                    if (is_inside_center_all > 0)   //该右侧车道内存在车辆
                    {
                        is_empty_rightlane = 1;
                        break;
                    }
                }
            }
            flag_empty.push_back(is_empty_rightlane);
        }

        for (auto& d : flag_empty)
        {
            if (d == 0)   //存在右侧车道为空
            {
                is_empty_rightlane_final = 0;
            }
        }
    }

    return is_empty_rightlane_final;
}

void Stringsplit(string str, const char split, vector<string>& res)
{
    istringstream iss(str);              // 输入流
    string        token;                 // 接收缓冲区
    while (getline(iss, token, split))   // 以split为分隔符
    {
        res.push_back(token);
    }
}

//根据违法过程图获取违法取证视频
PV_ERROR_CODE getvideo(string videofilename, vector<cv::Mat> vehicle_images)
{
    //获取违法四合一图像对应的视频，用于行政复议
    // 视频编码器设置
    if (vehicle_images.size() >= 4)
    {
        int      fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');   // 使用MJPG编码
        double   fps    = 10.0;                                          // 每秒10帧
        cv::Size frameSize(4096, 2160);                                  // 设置视频帧大小

        // 创建VideoWriter对象
        cv::VideoWriter video(videofilename, fourcc, fps, frameSize);
        // 检查是否成功打开视频
        if (!video.isOpened())
        {
            std::cerr << "Error opening video for write" << std::endl;
            return PV_ERROR_EXCEPTION;
        }

        for (int mj = 0; mj < vehicle_images.size(); mj++)
        {
            // imwrite("/data2/camera-handler/upFiles/output_" + output_data[i].plate_number + "_" + to_string(mj) + "_.jpg", output_data[i].vehicle_images[mj]);
            // 循环写入图像到视频

            if (vehicle_images[mj].empty())
            {
                std::cerr << "Error loading image " << std::endl;
                continue;
            }
            //将4096*2160原始图像resize为1920*1080
            // resize(vehicle_images[mj], vehicle_images[mj], Size(1920, 1080), 0, 0, INTER_LINEAR);
            video.write(vehicle_images[mj]);   // 将图像写入视频
        }
    }
}

//不按规定车道行驶-超车道
PV_ERROR_CODE truckDetector::postprocess_occupysecondlane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, vector<laneElements> contour_rightlane, std::vector<YoloV8Box> boundingbox,
                                                          STracks output_stracks, vector<json>& results_json, illegal_data& one_illegal_data)
{
    if (frame_to_draw.empty() || img_origin.empty())
    {
        return PV_ERROR_EXCEPTION;
    }
    logger_truck->info("output_stracks.size(): {}", output_stracks.size());
    //根据检测和跟踪结果获取track_id和预测框，判定是否违法
    for (auto& bbox : output_stracks)
    {
        Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};
        //(a)判断车辆下边界中点是否在多边形区域内
        cv::Point center_point     = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1] + bbox->det_tlwh[3]);
        double    is_inside_center = -1;
        try
        {
            is_inside_center = pointPolygonTest(lane.laneRegion, center_point, true);
        }
        catch (...)
        {
            return PV_ERROR_EXCEPTION;
        }

        logger_truck->info("is_inside_center: {}", is_inside_center);

        int is_illegal = 0;   //判定当前车辆是否违法标志，违法为1，不违法为0
        //(b)判定货车、载客汽车是否进入违法区域，对于违法车辆进行第一次抓拍

        if (bbox->class_id == 1 || bbox->class_id == 2 || bbox->class_id == 0)
        {
            if (is_inside_center > 0)   //车辆下边界中点在区域内
            {
                is_illegal = 1;
            }

            logger_truck->info("is_illegal: {}", is_illegal);
            logger_truck->info("one_illegal_data.detection_data_secondlane.size(): {}", one_illegal_data.detection_data_secondlane.size());

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //(e)对于第一次违法车辆，继续跟踪，等待到达指定的时间间隔，开始第二次抓拍
            //到达时间间隔5秒后，判定车辆是否形成完整违法，如果形成，则进行第二次抓拍并存入输出数据，每条输出数据包含一辆车的完整违法证据，
            //主要包括：2张过程图像，1张车牌图像，车牌号码，车辆在图像中的位置,车辆类型，置信度
            //同一车辆第二次违法判断
            //到达时间间隔5秒，判定是否可以形成完整违法证据，若可以则存入输出数据，若不可以，则删除该检测数据
            for (int j = 0; j < one_illegal_data.detection_data_secondlane.size(); j++)
            {
                logger_truck->info("one_illegal_data.detection_data_secondlane[j].is_overtime: {}", one_illegal_data.detection_data_secondlane[j].is_overtime);
                logger_truck->info("one_illegal_data.detection_data_secondlane[j].vehicle_images.size(): {}", one_illegal_data.detection_data_secondlane[j].vehicle_images.size());
                //确定抓取车牌对应的图像即为第四张违法图像
                // if (center_point.y > lane.plateYmax)
                if (one_illegal_data.algo_para.plate_no != "" && one_illegal_data.algo_para.vehicle_coordinate != "")
                {
                    //当出现卡口图像，计算触发卡口图像的车辆位置和违法车辆位置关系
                    //计算违法车辆和卡口车辆的iou
                    double ratio_kakou = GetIOU_rect(one_illegal_data.algo_para.kakou_car_box, bb_truck);
                    logger_truck->info("ratio_kakou: {}", ratio_kakou);
                    logger_truck->info("bbox->track_id: {}", bbox->track_id);
                    logger_truck->info("one_illegal_data.detection_data_secondlane[j].track_id: {}", one_illegal_data.detection_data_secondlane[j].track_id);
                    //第四张违法图像
                    if (one_illegal_data.detection_data_secondlane[j].is_overtime == 0 && ratio_kakou > 0.8)
                    {
                        one_illegal_data.detection_data_secondlane[j].end_nframe = one_illegal_data.nframe;

                        //到达时间间隔5秒，同一个track_id的车辆依然在违法区域内，则抓拍第四张图像
                        // if (is_illegal == 1)//第四张图像不需要一定违法
                        {
                            if (bbox->track_id == one_illegal_data.detection_data_secondlane[j].track_id)   //同一车辆间隔5秒后违法
                            {
                                one_illegal_data.detection_data_secondlane[j].is_overtime = 1;   //记录时间超过5秒，如果未抓拍到第四张违法图像，则删除该检测数据
                                logger_truck->info("fourth illegal image");

                                // one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_secondlane[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data_secondlane[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));

                                //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                                bayonet_data one_bayonet;
                                logger_truck->info("one_illegal_data.algo_para.bayonet_img_path3333 : {}", one_illegal_data.algo_para.bayonet_img_path);
                                logger_truck->info("one_illegal_data.algo_para.bayonet_lane_no3333: {}", one_illegal_data.algo_para.bayonet_lane_no);
                                logger_truck->info("lane.id33333: {}", lane.id);
                                if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                                {
                                    // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                                    one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                                    one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                                    one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                                    one_illegal_data.detection_data_secondlane[j].bayonet_vehicle_images.push_back(one_bayonet);
                                }
                            }
                        }
                    }

                    // if (is_illegal == 1)
                    {
                        if (bbox->track_id == one_illegal_data.detection_data_secondlane[j].track_id && one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.size() <= 0)
                        {
                            logger_truck->info("illegal plate image");
                            int yy = 0;
                            //获取违法车辆对应的车牌
                            for (auto& b : boundingbox)
                            {
                                cv::Rect box_plate = {(int)b.x1, (int)b.y1, (int)(b.x2 - b.x1), (int)(b.y2 - b.y1)};

                                if (b.class_id == 3 || b.class_id == 4 || b.class_id == 5 || b.class_id == 6 || b.class_id == 8 || b.class_id == 9)
                                {
                                    Rect_<float> bb_plate;
                                    bb_plate.x      = box_plate.x;
                                    bb_plate.y      = box_plate.y;
                                    bb_plate.width  = box_plate.width;
                                    bb_plate.height = box_plate.height;

                                    //计算车辆和车牌的iou
                                    double ratio_plate = GetIOU_rect(bb_truck, bb_plate);

                                    if (ratio_plate > 0.99 && (bb_plate.y + bb_plate.height) > yy)
                                    {
                                        if (bb_plate.width > 16 && bb_plate.height > 16)
                                        {
                                            try
                                            {
                                                yy                   = bb_plate.y + bb_plate.height;
                                                cv::Mat    plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                plate_data one_plate;
                                                one_plate.vehicle_plate_image = plate_img;
                                                one_plate.plate_id            = b.class_id;
                                                one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.push_back(one_plate);
                                            }
                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                        else
                                        {
                                            try
                                            {
                                                yy                = bb_plate.y + bb_plate.height;
                                                cv::Mat plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                Mat     dst_plate;
                                                resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                plate_data one_plate;
                                                one_plate.vehicle_plate_image = dst_plate;
                                                one_plate.plate_id            = b.class_id;
                                                one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.push_back(one_plate);
                                            }

                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                double end      = (double)cv::getTickCount();
                double end_time = end * 1000 / cv::getTickFrequency();
                //第二张违法图像
                if (one_illegal_data.detection_data_secondlane[j].is_overtime == 0)
                {

                    if (bbox->track_id == one_illegal_data.detection_data_secondlane[j].track_id)
                    {
                        //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                        bayonet_data one_bayonet;
                        logger_truck->info("one_illegal_data.algo_para.bayonet_img_path2222 : {}", one_illegal_data.algo_para.bayonet_img_path);
                        logger_truck->info("one_illegal_data.algo_para.bayonet_lane_no2222: {}", one_illegal_data.algo_para.bayonet_lane_no);
                        logger_truck->info("lane.id2222: {}", lane.id);
                        if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                        {
                            // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                            one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                            one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                            one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                            one_illegal_data.detection_data_secondlane[j].bayonet_vehicle_images.push_back(one_bayonet);
                        }
                        ////////////////////////////////////////////////////////////////////////////////////////////////
                        //判断右侧车道是否空旷
                        int is_empty_lane = is_empty_right_lane(contour_rightlane, boundingbox, frame_to_draw.rows);

                        logger_truck->info("clear clear clear clear1111111111111111");
                        logger_truck->info("is_empty_lane: {}", is_empty_lane);
                        logger_truck->info("is_illegal: {}", is_illegal);
                        if (is_illegal == 1 && is_empty_lane == 0)   //右侧空旷且违法
                        {
                            logger_truck->info("second illegal image");
                            // one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save());
                            one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                            one_illegal_data.detection_data_secondlane[j].vehicle_images.push_back(img_origin);
                            one_illegal_data.detection_data_secondlane[j].vehicle_location_box.push_back(
                                Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                        }
                        else
                        {
                            logger_truck->info("clear clear clear clear22222222222222");
                            //判断右侧空旷时必须全程右侧都没有车辆， 满足违法，否则舍弃该违法数据
                            one_illegal_data.detection_data_secondlane[j].is_overtime = 1;
                            one_illegal_data.detection_data_secondlane[j].vehicle_images.clear();
                        }
                    }
                }

                if (one_illegal_data.detection_data_secondlane[j].vehicle_images.size() >= 4 && one_illegal_data.detection_data_secondlane[j].is_overtime == 1)
                {
                    if (is_illegal == 1)
                    {
                        int flag_illegal_trackid = 0;
                        for (int jj = 0; jj < one_illegal_data.illegal_trackid_secondlane.size(); jj++)
                        {
                            if (one_illegal_data.detection_data_secondlane[j].track_id == one_illegal_data.illegal_trackid_secondlane[jj])
                            {
                                flag_illegal_trackid = 1;
                            }
                        }

                        if (bbox->track_id == one_illegal_data.detection_data_secondlane[j].track_id && flag_illegal_trackid == 0)   //同一车辆间隔5秒后违法
                        {
                            if (one_illegal_data.detection_data_secondlane[j].vehicle_images.size() >= 4 && one_illegal_data.detection_data_secondlane[j].is_overtime == 1 &&
                                one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.size() > 0)
                            {
                                vehicle_output_data one_output;   //形成完整违法，获取完整输出数据
                                for (int idx = 0; idx < one_illegal_data.detection_data_secondlane[j].vehicle_images.size(); idx++)
                                {
                                    one_output.vehicle_images.push_back(one_illegal_data.detection_data_secondlane[j].vehicle_images[idx]);
                                    one_output.vehicle_location_box.push_back(one_illegal_data.detection_data_secondlane[j].vehicle_location_box[idx]);
                                }
                                one_output.illegal_time = one_illegal_data.detection_data_secondlane[j].illegal_time;
                                one_output.track_id     = bbox->track_id;

                                //确定车型
                                if (one_illegal_data.detection_data_secondlane[j].class_id == 1)
                                {
                                    one_output.vehicle_type = "载客汽车";
                                }
                                else
                                {
                                    if (one_illegal_data.detection_data_secondlane[j].class_id == 2)
                                    {
                                        one_output.vehicle_type = "载货汽车";
                                    }
                                    else
                                    {
                                        one_output.vehicle_type = "小汽车";
                                    }
                                }
                                //根据检测数据中的车牌图像获取输出数据中的车牌图像
                                one_output.vehicle_plate_images = one_illegal_data.detection_data_secondlane[j].vehicle_plate_images;
                                //车牌识别
                                vector<string> results;
                                if (one_output.vehicle_plate_images.size() > 0)
                                {
                                    int plate_size       = one_output.vehicle_plate_images.size();
                                    one_output.plate_img = one_output.vehicle_plate_images[plate_size - 1].vehicle_plate_image;

                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                    {
                                        cv::Mat plate_img_split = get_split_merge(one_output.plate_img);
                                        platerecognition(plate_img_split, results);
                                    }
                                    else
                                    {
                                        platerecognition(one_output.plate_img, results);
                                    }
                                    if (results.size() > 0)
                                    {
                                        logger_truck->info("车牌号: {}", results[0]);
                                        logger_truck->info("车牌号-卡口: {}", one_illegal_data.algo_para.plate_no);

                                        if (one_illegal_data.algo_para.plate_no != "")
                                        {
                                            one_output.plate_number = one_illegal_data.algo_para.plate_no;   //使用自动识别的车牌号
                                        }
                                        else
                                        {
                                            // one_output.plate_number = results[0];   //使用自动识别的车牌号
                                            one_output.plate_number = "";
                                        }

                                        /////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        //根据卡口图获取车牌图像

                                        Rect_<float> bb_plate = one_illegal_data.algo_para.kakou_plate_box;

                                        logger_truck->info("bb_plate.x : {}", bb_plate.x);
                                        logger_truck->info("bb_plate.y : {}", bb_plate.y);
                                        logger_truck->info("bb_plate.width : {}", bb_plate.width);
                                        logger_truck->info("bb_plate.height : {}", bb_plate.height);
                                        if (bb_plate.width > 16 && bb_plate.height > 16)
                                        {
                                            try
                                            {
                                                int     size_2    = one_illegal_data.detection_data_secondlane[j].vehicle_images.size() - 1;
                                                cv::Mat plate_img = one_illegal_data.detection_data_secondlane[j].vehicle_images[size_2](
                                                    Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                int size_1 = one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.size() - 1;
                                                one_illegal_data.detection_data_secondlane[j].vehicle_plate_images[size_1].vehicle_plate_image = plate_img;
                                                //替换车牌图像为卡口图像中的车牌图像
                                                one_output.plate_img = plate_img;
                                            }
                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                        else
                                        {
                                            try
                                            {
                                                int     size_2    = one_illegal_data.detection_data_secondlane[j].vehicle_images.size() - 1;
                                                cv::Mat plate_img = one_illegal_data.detection_data_secondlane[j].vehicle_images[size_2](
                                                    Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                Mat dst_plate;
                                                resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                int size_1 = one_illegal_data.detection_data_secondlane[j].vehicle_plate_images.size() - 1;
                                                one_illegal_data.detection_data_secondlane[j].vehicle_plate_images[size_1].vehicle_plate_image = dst_plate;
                                                //替换车牌图像为卡口图像中的车牌图像
                                                one_output.plate_img = dst_plate;
                                            }

                                            catch (...)
                                            {
                                                return PV_ERROR_EXCEPTION;
                                            }
                                        }
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 3)
                                    {
                                        one_output.plate_color = "蓝牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                    {
                                        one_output.plate_color = "双层黄牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 5)
                                    {
                                        one_output.plate_color = "单层黄牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 6)
                                    {
                                        one_output.plate_color = "绿牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 8)
                                    {
                                        one_output.plate_color = "白牌";
                                    }
                                    if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 9)
                                    {
                                        one_output.plate_color = "双层绿牌";
                                    }
                                }
                                if (one_output.vehicle_images.size() >= 4 && !one_output.plate_img.empty() && (one_output.plate_number.size() == 9 || one_output.plate_number.size() == 10))
                                {
                                    one_illegal_data.output_data_secondlane.push_back(one_output);
                                    one_illegal_data.illegal_trackid_secondlane.push_back(one_illegal_data.detection_data_secondlane[j].track_id);
                                }
                            }
                        }
                    }
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(c)对第一次进入违法区域的车辆进行记录，并记录第一次抓拍时间
        if (is_illegal == 1)
        {

            //(c1.1)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_detect = 0;
            for (int j = 0; j < one_illegal_data.detection_data_secondlane.size(); j++)
            {
                if (one_illegal_data.detection_data_secondlane[j].track_id == bbox->track_id)
                {
                    //对于已经出现违法，但是未形成完整违法数据的车辆，记录其从开始违法到最终违法的违法总次数，总次数大于给定阈值，则判定最终违法
                    one_illegal_data.detection_data_secondlane[j].is_illegal_num = one_illegal_data.detection_data_secondlane[j].is_illegal_num + 1;
                    one_illegal_data.detection_data_secondlane[j].end_nframe     = one_illegal_data.nframe;
                    if ((one_illegal_data.detection_data_secondlane[j].end_nframe - one_illegal_data.detection_data_secondlane[j].start_nframe + 1) <= 5)
                    {
                        if (one_illegal_data.detection_data_secondlane[j].is_illegal_num <
                            (one_illegal_data.detection_data_secondlane[j].end_nframe - one_illegal_data.detection_data_secondlane[j].start_nframe + 1))
                        {
                            //对第一次违法做统计分析，如果连续5帧违法则继续跟踪记录，不需要更新第一次抓拍记录，如果连续5帧出现不违法，则更新第一次抓拍记录
                            if (one_illegal_data.detection_data_secondlane[j].vehicle_images.size() > 0)
                            {
                                one_illegal_data.detection_data_secondlane[j].vehicle_images.clear();
                                one_illegal_data.detection_data_secondlane[j].vehicle_location_box.clear();
                                one_illegal_data.detection_data_secondlane[j].illegal_time.clear();
                                // one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_secondlane[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_secondlane[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data_secondlane[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                                double begin                                                 = (double)cv::getTickCount();
                                double start_time                                            = begin * 1000 / cv::getTickFrequency();
                                one_illegal_data.detection_data_secondlane[j].start_time     = start_time;
                                one_illegal_data.detection_data_secondlane[j].is_illegal_num = 1;
                                one_illegal_data.detection_data_secondlane[j].start_nframe   = one_illegal_data.nframe;
                            }
                        }
                    }
                    flag_temp_detect = 1;
                }
            }

            //(c1.2)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_illegal = 0;
            for (int j = 0; j < one_illegal_data.illegal_trackid_secondlane.size(); j++)
            {
                if (one_illegal_data.illegal_trackid_secondlane[j] == bbox->track_id)
                {
                    flag_temp_illegal = 1;
                }
            }
            //(c2)超过指定时间间隔，如果检测数据违法，会被存入输出数据中并在检测数据中清除，如果检测数据未违法，则会被清除；
            //检测数据已经被清除，如果输出数据中已经存在该track_id, 则不再重复记录
            int flag_temp_output = 0;
            for (int j = 0; j < one_illegal_data.output_data_secondlane.size(); j++)
            {
                if (one_illegal_data.output_data_secondlane[j].track_id == bbox->track_id)
                {
                    flag_temp_output = 1;
                }
            }
            //(c3)车辆第一次进入违法区域，记录进入时间点,并获取对应的车牌号图像
            if (flag_temp_detect == 0 && flag_temp_output == 0 && flag_temp_illegal == 0)
            {
                vehicle_detect_data one_detect;

                //第一次抓拍后车辆数据记录
                double begin          = (double)cv::getTickCount();
                double start_time     = begin * 1000 / cv::getTickFrequency();
                one_detect.start_time = start_time;   //车辆违法第一次抓拍时间
                // check time first
                // one_detect.illegal_time.push_back(getCurrentTimeInt64_save());
                one_detect.illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                one_detect.vehicle_images.push_back(img_origin);   //第一张违法图像
                //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                bayonet_data one_bayonet;
                logger_truck->info("one_illegal_data.algo_para.bayonet_img_path1111 : {}", one_illegal_data.algo_para.bayonet_img_path);
                logger_truck->info("one_illegal_data.algo_para.bayonet_lane_no1111: {}", one_illegal_data.algo_para.bayonet_lane_no);
                logger_truck->info("lane.id1111: {}", lane.id);
                if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                {
                    // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                    one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                    one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                    one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                    one_detect.bayonet_vehicle_images.push_back(one_bayonet);
                }

                one_detect.vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                one_detect.track_id       = bbox->track_id;                                                                                                        //车辆跟踪ID
                one_detect.class_id       = bbox->class_id;                                                                                                        //车辆类型
                one_detect.is_illegal_num = one_detect.is_illegal_num + 1;
                one_detect.start_nframe   = one_illegal_data.nframe;
                //判断右侧车道是否空旷
                int is_empty_lane = is_empty_right_lane(contour_rightlane, boundingbox, frame_to_draw.rows);
                if (is_empty_lane == 0)
                {
                    logger_truck->info("first illegal image");
                    one_illegal_data.detection_data_secondlane.push_back(one_detect);
                }
                else
                {
                    //判断右侧空旷时必须第一次进入画面就违法，否则记录车辆id，舍弃该车辆
                    one_illegal_data.illegal_trackid_secondlane.push_back(one_detect.track_id);
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //记录完整违法车辆id，若图像中该id消失，则清除该id
    auto removed_stracks = one_illegal_data.bytetrack.get_removed_stracks();

    for (auto& bbox : removed_stracks)
    {
        for (vector<int>::iterator it = one_illegal_data.illegal_trackid_secondlane.begin(); it != one_illegal_data.illegal_trackid_secondlane.end(); it++)
        {
            if (*it == bbox->track_id)
            {
                one_illegal_data.illegal_trackid_secondlane.erase(it);
                it--;
            }
        }
        for (vector<vehicle_detect_data>::iterator it_it = one_illegal_data.detection_data_secondlane.begin(); it_it != one_illegal_data.detection_data_secondlane.end(); it_it++)
        {
            if (it_it->track_id == bbox->track_id)
            {
                one_illegal_data.detection_data_secondlane.erase(it_it);
                it_it--;
            }
        }
    }
    //输出结果画框，四合一
    logger_truck->info("start img_merge");
    if (one_illegal_data.output_data_secondlane.size() > 0)
    {
        img_merge(one_illegal_data.output_data_secondlane, 0, lane.laneRegion, one_illegal_data);
    }

    logger_truck->info("end img_merge");

    for (auto& output : one_illegal_data.output_data_secondlane)
    {
        // save result
        json output_json;
        output_json["illegal_image_name"] = output.illegal_image_name;
        output_json["illegal_time"]       = output.illegal_time[3];
        output_json["illegal_place"]      = one_illegal_data.algo_para.camera_place;
        ////////////////////////////////////////////////////
        output_json["plate_number"]     = output.plate_number;
        output_json["plate_color"]      = output.plate_color;
        output_json["vehicle_type"]     = output.vehicle_type;
        output_json["illegal_code"]     = one_illegal_data.algo_para.illegal_code_occupyLane;
        output_json["illegal_behavior"] = "驾驶机动车在高速公路上不按规定车道行驶";
        ////////////////////////////////////////////////////
        output_json["device_number"] = one_illegal_data.algo_para.device_number;
        output_json["camera_id"]     = one_illegal_data.algo_para.camera_id;

        results_json.push_back(output_json);
    }

    one_illegal_data.output_data_secondlane.clear();

    return PV_ERROR_OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//不按规定车道行驶-禁行车道
PV_ERROR_CODE truckDetector::postprocess_occupyleftlane(cv::Mat frame_to_draw, cv::Mat img_origin, laneElements lane, std::vector<YoloV8Box> boundingbox, STracks output_stracks,
                                                        vector<json>& results_json, illegal_data& one_illegal_data, int flag_is_C)
{
    int my_index  = -1;
    int flag_lane = 0;
    for (int jk = 0; jk < one_illegal_data.detection_data_prohibited.size(); jk++)
    {
        if (one_illegal_data.detection_data_prohibited[jk].lane_id == lane.id)
        {
            //已经存在的车道
            flag_lane = 1;
            my_index  = jk;
        }
    }
    //不存在的车道id，添加到one_illegal_data.detection_data_prohibited
    if (flag_lane == 0)
    {
        detection_data_lane one_detection_data_lane;
        one_detection_data_lane.lane_id = lane.id;
        one_illegal_data.detection_data_prohibited.push_back(one_detection_data_lane);
        my_index = one_illegal_data.detection_data_prohibited.size() - 1;
    }

    if (frame_to_draw.empty() || img_origin.empty())
    {
        return PV_ERROR_EXCEPTION;
    }
    logger_truck->info("output_stracks.size(): {}", output_stracks.size());
    //根据检测和跟踪结果获取track_id和预测框，判定是否违法
    for (auto& bbox : output_stracks)
    {
        Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};
        //(a)判断车辆下边界中点是否在多边形区域内
        cv::Point center_point     = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1] + bbox->det_tlwh[3]);
        double    is_inside_center = -1;
        try
        {
            is_inside_center = pointPolygonTest(lane.laneRegion, center_point, true);
        }
        catch (...)
        {
            return PV_ERROR_EXCEPTION;
        }

        logger_truck->info("is_inside_center: {}", is_inside_center);

        int is_illegal = 0;   //判定当前车辆是否违法标志，违法为1，不违法为0
        //(b)判定货车、载客汽车是否进入违法区域，对于违法车辆进行第一次抓拍
        if (bbox->class_id == 1 || bbox->class_id == 2 || bbox->class_id == 0)
        {
            if (is_inside_center > 0)   //车辆下边界中点在区域内
            {
                is_illegal = 1;
            }

            logger_truck->info("is_illegal: {}", is_illegal);

            if (one_illegal_data.detection_data_prohibited.size() > 0 && my_index >= 0)
            {
                logger_truck->info("one_illegal_data.detection_data.size(): {}", one_illegal_data.detection_data_prohibited[my_index].detection_data.size());

                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //(e)对于第一次违法车辆，继续跟踪，等待到达指定的时间间隔，开始第二次抓拍
                //到达时间间隔5秒后，判定车辆是否形成完整违法，如果形成，则进行第二次抓拍并存入输出数据，每条输出数据包含一辆车的完整违法证据，
                //主要包括：2张过程图像，1张车牌图像，车牌号码，车辆在图像中的位置,车辆类型，置信度
                //同一车辆第二次违法判断
                //到达时间间隔5秒，判定是否可以形成完整违法证据，若可以则存入输出数据，若不可以，则删除该检测数据
                for (int j = 0; j < one_illegal_data.detection_data_prohibited[my_index].detection_data.size(); j++)
                {
                    logger_truck->info("one_illegal_data.detection_data[j].is_overtime: {}", one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime);
                    logger_truck->info("one_illegal_data.detection_data[j].vehicle_images.size(): {}", one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size());
                    logger_truck->info("one_illegal_data.algo_para.plate_no: {}", one_illegal_data.algo_para.plate_no);
                    logger_truck->info("one_illegal_data.algo_para.vehicle_coordinate: {}", one_illegal_data.algo_para.vehicle_coordinate);
                    //确定抓取车牌对应的图像即为第四张违法图像

                    // if (center_point.y > lane.plateYmax)
                    if (one_illegal_data.algo_para.plate_no != "" && one_illegal_data.algo_para.vehicle_coordinate != "")
                    {

                        //当出现卡口图像，计算触发卡口图像的车辆位置和违法车辆位置关系
                        //计算违法车辆和卡口车辆的iou
                        double ratio_kakou = GetIOU_rect(one_illegal_data.algo_para.kakou_car_box, bb_truck);
                        logger_truck->info("ratio_kakou: {}", ratio_kakou);
                        logger_truck->info("bbox->track_id: {}", bbox->track_id);
                        logger_truck->info("one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id: {}",
                                           one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id);
                        //第四张违法图像

                        // if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime == 0)
                        if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime == 0 && ratio_kakou > 0.8)
                        {
                            // one_illegal_data.detection_data_prohibited[my_index].detection_data[j].kakou_plate_no = one_illegal_data.algo_para.plate_no;

                            one_illegal_data.detection_data_prohibited[my_index].detection_data[j].end_nframe = one_illegal_data.nframe;

                            //到达时间间隔5秒，同一个track_id的车辆依然在违法区域内，则抓拍第四张图像
                            // if (is_illegal == 1)//第四张图像不需要一定违法
                            {
                                if (bbox->track_id == one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id)   //同一车辆间隔5秒后违法
                                {
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime = 1;   //记录时间超过5秒，如果未抓拍到第四张违法图像，则删除该检测数据
                                    logger_truck->info("fourth illegal image");

                                    // one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.push_back(img_origin);
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_location_box.push_back(
                                        Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));

                                    //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                                    logger_truck->info("one_illegal_data.algo_para.bayonet_img_path:{}", one_illegal_data.algo_para.bayonet_img_path);
                                    bayonet_data one_bayonet;
                                    if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                                    {
                                        // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                                        one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                                        one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                                        one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                                        one_illegal_data.detection_data_prohibited[my_index].detection_data[j].bayonet_vehicle_images.push_back(one_bayonet);
                                    }
                                }
                            }
                        }

                        // if (is_illegal == 1)
                        {
                            if (bbox->track_id == one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id &&
                                one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.size() <= 0)
                            {
                                logger_truck->info("illegal plate image");
                                int yy = 0;
                                //获取违法车辆对应的车牌
                                for (auto& b : boundingbox)
                                {
                                    cv::Rect box_plate = {(int)b.x1, (int)b.y1, (int)(b.x2 - b.x1), (int)(b.y2 - b.y1)};

                                    if (b.class_id == 3 || b.class_id == 4 || b.class_id == 5 || b.class_id == 6 || b.class_id == 8 || b.class_id == 9)
                                    {
                                        Rect_<float> bb_plate;
                                        bb_plate.x      = box_plate.x;
                                        bb_plate.y      = box_plate.y;
                                        bb_plate.width  = box_plate.width;
                                        bb_plate.height = box_plate.height;

                                        //计算车辆和车牌的iou
                                        double ratio_plate = GetIOU_rect(bb_truck, bb_plate);

                                        if (ratio_plate > 0.99 && (bb_plate.y + bb_plate.height) > yy)
                                        {
                                            if (bb_plate.width > 16 && bb_plate.height > 16)
                                            {
                                                try
                                                {
                                                    yy                   = bb_plate.y + bb_plate.height;
                                                    cv::Mat    plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    plate_data one_plate;
                                                    one_plate.vehicle_plate_image = plate_img;
                                                    one_plate.plate_id            = b.class_id;
                                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.push_back(one_plate);
                                                }
                                                catch (...)
                                                {
                                                    return PV_ERROR_EXCEPTION;
                                                }
                                            }
                                            else
                                            {
                                                try
                                                {
                                                    yy                = bb_plate.y + bb_plate.height;
                                                    cv::Mat plate_img = img_origin(Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    Mat     dst_plate;
                                                    resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                    plate_data one_plate;
                                                    one_plate.vehicle_plate_image = dst_plate;
                                                    one_plate.plate_id            = b.class_id;
                                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.push_back(one_plate);
                                                }

                                                catch (...)
                                                {
                                                    return PV_ERROR_EXCEPTION;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    double end      = (double)cv::getTickCount();
                    double end_time = end * 1000 / cv::getTickFrequency();
                    //第二张违法图像
                    if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime == 0)
                    {

                        if (bbox->track_id == one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id)   //同一车辆间隔5秒后违法
                        {
                            //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配,存卡口图不要求当前车辆一定违法
                            logger_truck->info("one_illegal_data.algo_para.bayonet_img_path:{}", one_illegal_data.algo_para.bayonet_img_path);
                            bayonet_data one_bayonet;
                            if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                            {
                                // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                                one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                                one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                                one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                                one_illegal_data.detection_data_prohibited[my_index].detection_data[j].bayonet_vehicle_images.push_back(one_bayonet);
                            }

                            if (is_illegal == 1)
                            {
                                logger_truck->info("second illegal image");
                                // one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save());
                                one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_location_box.push_back(
                                    Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                            }
                        }
                    }

                    if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size() >= 4 &&
                        one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime == 1)
                    {
                        if (is_illegal == 1)
                        {
                            int flag_illegal_trackid = 0;
                            for (int jj = 0; jj < one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.size(); jj++)
                            {
                                if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id == one_illegal_data.detection_data_prohibited[my_index].illegal_trackid[jj])
                                {
                                    flag_illegal_trackid = 1;
                                }
                            }
                            if (bbox->track_id == one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id && flag_illegal_trackid == 0)   //同一车辆间隔5秒后违法
                            {
                                if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size() >= 4 &&
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_overtime == 1 &&
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.size() > 0)
                                {
                                    vehicle_output_data one_output;   //形成完整违法，获取完整输出数据
                                    for (int idx = 0; idx < one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size(); idx++)
                                    {
                                        one_output.vehicle_images.push_back(one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images[idx]);
                                        one_output.vehicle_location_box.push_back(one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_location_box[idx]);
                                    }
                                    one_output.illegal_time = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time;
                                    one_output.track_id     = bbox->track_id;
                                    one_output.start_time   = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].start_time;

                                    //确定车型
                                    if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].class_id == 1)
                                    {
                                        one_output.vehicle_type = "载客汽车";
                                    }
                                    else
                                    {
                                        if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].class_id == 2)
                                        {
                                            one_output.vehicle_type = "载货汽车";
                                        }
                                        else
                                        {
                                            one_output.vehicle_type = "小汽车";
                                        }
                                    }
                                    //根据检测数据中的车牌图像获取输出数据中的车牌图像
                                    one_output.vehicle_plate_images = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images;
                                    //车牌识别
                                    vector<string> results;
                                    if (one_output.vehicle_plate_images.size() > 0)
                                    {
                                        int plate_size       = one_output.vehicle_plate_images.size();
                                        one_output.plate_img = one_output.vehicle_plate_images[plate_size - 1].vehicle_plate_image;

                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                        {
                                            cv::Mat plate_img_split = get_split_merge(one_output.plate_img);
                                            // imwrite("/data/lj/" + to_string(one_illegal_data.nframe) + "plate.jpg", plate_img_split);
                                            platerecognition(plate_img_split, results);
                                        }
                                        else
                                        {
                                            // imwrite("/data/lj/" + to_string(one_illegal_data.nframe) + "plate.jpg", one_output.plate_img);
                                            platerecognition(one_output.plate_img, results);
                                        }
                                        if (results.size() > 0)
                                        {
                                            logger_truck->info("车牌号: {}", results[0]);
                                            logger_truck->info("车牌号-卡口: {}", one_illegal_data.algo_para.plate_no);
                                            // logger_truck->info("车牌号-卡口: {}", one_illegal_data.detection_data_prohibited[my_index].detection_data[j].kakou_plate_no);

                                            if (one_illegal_data.algo_para.plate_no != "")
                                            {
                                                // one_output.plate_number = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].kakou_plate_no;
                                                one_output.plate_number = one_illegal_data.algo_para.plate_no;
                                            }
                                            else
                                            {
                                                // one_output.plate_number = results[0];   //使用自动识别的车牌号
                                                one_output.plate_number = "";
                                            }

                                            /////////////////////////////////////////////////////////////////////////////////////////////////////////
                                            //根据卡口图获取车牌图像

                                            Rect_<float> bb_plate = one_illegal_data.algo_para.kakou_plate_box;

                                            logger_truck->info("bb_plate.x : {}", bb_plate.x);
                                            logger_truck->info("bb_plate.y : {}", bb_plate.y);
                                            logger_truck->info("bb_plate.width : {}", bb_plate.width);
                                            logger_truck->info("bb_plate.height : {}", bb_plate.height);
                                            if (bb_plate.width > 16 && bb_plate.height > 16)
                                            {
                                                try
                                                {
                                                    int     size_2    = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size() - 1;
                                                    cv::Mat plate_img = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images[size_2](
                                                        Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    int size_1 = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.size() - 1;
                                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images[size_1].vehicle_plate_image = plate_img;
                                                    //替换车牌图像为卡口图像中的车牌图像
                                                    one_output.plate_img = plate_img;
                                                }
                                                catch (...)
                                                {
                                                    return PV_ERROR_EXCEPTION;
                                                }
                                            }
                                            else
                                            {
                                                try
                                                {
                                                    int     size_2    = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size() - 1;
                                                    cv::Mat plate_img = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images[size_2](
                                                        Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    Mat dst_plate;
                                                    resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                    int size_1 = one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images.size() - 1;
                                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_plate_images[size_1].vehicle_plate_image = dst_plate;
                                                    //替换车牌图像为卡口图像中的车牌图像
                                                    one_output.plate_img = dst_plate;
                                                }

                                                catch (...)
                                                {
                                                    return PV_ERROR_EXCEPTION;
                                                }
                                            }
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 3)
                                        {
                                            one_output.plate_color = "蓝牌";
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 4)
                                        {
                                            one_output.plate_color = "双层黄牌";
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 5)
                                        {
                                            one_output.plate_color = "单层黄牌";
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 6)
                                        {
                                            one_output.plate_color = "绿牌";
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 8)
                                        {
                                            one_output.plate_color = "白牌";
                                        }
                                        if (one_output.vehicle_plate_images[plate_size - 1].plate_id == 9)
                                        {
                                            one_output.plate_color = "双层绿牌";
                                        }
                                    }
                                    if (one_output.vehicle_images.size() >= 4 && !one_output.plate_img.empty() && (one_output.plate_number.size() == 9 || one_output.plate_number.size() == 10))
                                    {
                                        one_illegal_data.detection_data_prohibited[my_index].output_data.push_back(one_output);
                                        one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.push_back(one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(c)对第一次进入违法区域的车辆进行记录，并记录第一次抓拍时间
        if (is_illegal == 1)
        {
            //(c1.1)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_detect = 0;
            if (one_illegal_data.detection_data_prohibited.size() > 0 && my_index >= 0)
            {
                for (int j = 0; j < one_illegal_data.detection_data_prohibited[my_index].detection_data.size(); j++)
                {
                    if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].track_id == bbox->track_id)
                    {
                        //对于已经出现违法，但是未形成完整违法数据的车辆，记录其从开始违法到最终违法的违法总次数，总次数大于给定阈值，则判定最终违法
                        one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_illegal_num =
                            one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_illegal_num + 1;
                        one_illegal_data.detection_data_prohibited[my_index].detection_data[j].end_nframe = one_illegal_data.nframe;
                        if ((one_illegal_data.detection_data_prohibited[my_index].detection_data[j].end_nframe - one_illegal_data.detection_data_prohibited[my_index].detection_data[j].start_nframe +
                             1) <= 5)
                        {
                            if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_illegal_num <
                                (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].end_nframe -
                                 one_illegal_data.detection_data_prohibited[my_index].detection_data[j].start_nframe + 1))
                            {
                                //对第一次违法做统计分析，如果连续5帧违法则继续跟踪记录，不需要更新第一次抓拍记录，如果连续5帧出现不违法，则更新第一次抓拍记录
                                if (one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.size() > 0)
                                {
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.clear();
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_location_box.clear();
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.clear();
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_images.push_back(img_origin);
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].vehicle_location_box.push_back(
                                        Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                                    double begin                                                                          = (double)cv::getTickCount();
                                    double start_time                                                                     = begin * 1000 / cv::getTickFrequency();
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].start_time     = start_time;
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].is_illegal_num = 1;
                                    one_illegal_data.detection_data_prohibited[my_index].detection_data[j].start_nframe   = one_illegal_data.nframe;
                                }
                            }
                        }
                        flag_temp_detect = 1;
                    }
                }
            }

            //(c1.2)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_illegal = 0;
            if (one_illegal_data.detection_data_prohibited.size() > 0 && my_index >= 0)
            {
                for (int j = 0; j < one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.size(); j++)
                {
                    if (one_illegal_data.detection_data_prohibited[my_index].illegal_trackid[j] == bbox->track_id)
                    {
                        flag_temp_illegal = 1;
                    }
                }
            }

            //(c2)超过指定时间间隔，如果检测数据违法，会被存入输出数据中并在检测数据中清除，如果检测数据未违法，则会被清除；
            //检测数据已经被清除，如果输出数据中已经存在该track_id, 则不再重复记录
            int flag_temp_output = 0;
            if (one_illegal_data.detection_data_prohibited.size() > 0 && my_index >= 0)
            {
                for (int j = 0; j < one_illegal_data.detection_data_prohibited[my_index].output_data.size(); j++)
                {
                    if (one_illegal_data.detection_data_prohibited[my_index].output_data[j].track_id == bbox->track_id)
                    {
                        flag_temp_output = 1;
                    }
                }
            }

            //(c3)车辆第一次进入违法区域，记录进入时间点,并获取对应的车牌号图像
            if (flag_temp_detect == 0 && flag_temp_output == 0 && flag_temp_illegal == 0)
            {
                vehicle_detect_data one_detect;

                //第一次抓拍后车辆数据记录
                double begin          = (double)cv::getTickCount();
                double start_time     = begin * 1000 / cv::getTickFrequency();
                one_detect.start_time = start_time;   //车辆违法第一次抓拍时间
                // check time first
                // one_detect.illegal_time.push_back(getCurrentTimeInt64_save());

                one_detect.illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                one_detect.vehicle_images.push_back(img_origin);   //第一张违法图像
                //判断当前是否有卡口图，把卡口图存入卡口数据，用于后续匹配
                bayonet_data one_bayonet;
                if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.bayonet_lane_no == lane.id)
                {
                    // one_bayonet.bayonet_vehicle_image = one_illegal_data.algo_para.bayonet_img;
                    one_bayonet.bayonet_plate_number = one_illegal_data.algo_para.plate_no;
                    one_bayonet.bayonet_plate_box    = one_illegal_data.algo_para.kakou_plate_box;
                    one_bayonet.bayonet_vehicle_box  = one_illegal_data.algo_para.kakou_car_box;
                    one_detect.bayonet_vehicle_images.push_back(one_bayonet);
                }

                one_detect.vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                one_detect.track_id       = bbox->track_id;                                                                                                        //车辆跟踪ID
                one_detect.class_id       = bbox->class_id;                                                                                                        //车辆类型
                one_detect.is_illegal_num = one_detect.is_illegal_num + 1;
                one_detect.start_nframe   = one_illegal_data.nframe;
                logger_truck->info("one_illegal_data.algo_para.bayonet_img_path:{}", one_illegal_data.algo_para.bayonet_img_path);
                logger_truck->info("first illegal image");

                if (my_index >= 0)
                {
                    one_illegal_data.detection_data_prohibited[my_index].detection_data.push_back(one_detect);
                }
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (one_illegal_data.detection_data_prohibited.size() > 0 && my_index >= 0)
    {
        //记录完整违法车辆id，若图像中该id消失，则清除该id
        auto removed_stracks = one_illegal_data.bytetrack.get_removed_stracks();
        if (flag_is_C == 1)   //对于C类，这里只清除已经违法的id,其他数据匹配后再清除
        {
            for (auto& bbox : removed_stracks)
            {
                for (vector<int>::iterator it = one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.begin();
                     it != one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.end(); it++)
                {
                    if (*it == bbox->track_id)
                    {
                        one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.erase(it);
                        it--;
                    }
                }
            }
        }

        //输出结果画框，四合一
        if (flag_is_C == 0)
        {
            for (auto& bbox : removed_stracks)
            {
                for (vector<int>::iterator it = one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.begin();
                     it != one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.end(); it++)
                {
                    if (*it == bbox->track_id)
                    {
                        one_illegal_data.detection_data_prohibited[my_index].illegal_trackid.erase(it);
                        it--;
                    }
                }
                for (vector<vehicle_detect_data>::iterator it_it = one_illegal_data.detection_data_prohibited[my_index].detection_data.begin();
                     it_it != one_illegal_data.detection_data_prohibited[my_index].detection_data.end(); it_it++)
                {
                    if (it_it->track_id == bbox->track_id)
                    {
                        one_illegal_data.detection_data_prohibited[my_index].detection_data.erase(it_it);
                        it_it--;
                    }
                }
            }

            logger_truck->info("start img_merge");
            if (one_illegal_data.detection_data_prohibited[my_index].output_data.size() > 0)
            {
                img_merge(one_illegal_data.detection_data_prohibited[my_index].output_data, 0, lane.laneRegion, one_illegal_data);
            }
            logger_truck->info("end img_merge");

            for (auto& output : one_illegal_data.detection_data_prohibited[my_index].output_data)
            {
                // save result
                json output_json;
                output_json["illegal_image_name"] = output.illegal_image_name;
                output_json["illegal_time"]       = output.illegal_time[3];
                output_json["illegal_place"]      = one_illegal_data.algo_para.camera_place;
                ////////////////////////////////////////////////////
                output_json["plate_number"]     = output.plate_number;
                output_json["plate_color"]      = output.plate_color;
                output_json["vehicle_type"]     = output.vehicle_type;
                output_json["illegal_code"]     = one_illegal_data.algo_para.illegal_code_occupyLane;
                output_json["illegal_behavior"] = "驾驶机动车在高速公路上不按规定车道行驶";
                ////////////////////////////////////////////////////
                output_json["device_number"] = one_illegal_data.algo_para.device_number;
                output_json["camera_id"]     = one_illegal_data.algo_para.camera_id;

                results_json.push_back(output_json);
            }

            one_illegal_data.detection_data_prohibited[my_index].output_data.clear();
        }
    }

    return PV_ERROR_OK;
}

//不按规定车道行驶C类区间抓拍
PV_ERROR_CODE truckDetector::postprocess_interval_detection(vector<illegal_data> m_illegal_data)
{
    //区间抓拍，两个相机车牌匹配
    vector<vehicle_output_data> first_camera;
    vector<vehicle_output_data> second_camera;
    int                         ID_first_camera  = -1;
    int                         ID_second_camera = -1;
    for (int k1 = 0; k1 < m_illegal_data.size(); k1++)
    {
        if (m_illegal_data[k1].algo_para.camera_index == 0)
        {
            ID_first_camera = k1;
            if (m_illegal_data[k1].detection_data_prohibited.size() > 0)
            {
                first_camera = m_illegal_data[k1].detection_data_prohibited[m_illegal_data[k1].detection_data_prohibited.size() - 1].output_data;
            }
        }
        if (m_illegal_data[k1].algo_para.camera_index == 1)
        {
            ID_second_camera = k1;
            if (m_illegal_data[k1].detection_data_prohibited.size() > 0)
            {
                second_camera = m_illegal_data[k1].detection_data_prohibited[0].output_data;
            }
        }
    }

    if (ID_first_camera >= 0 && ID_second_camera >= 0)
    {
        //车牌匹配
        for (int s = 0; s < second_camera.size(); s++)
        {
            vector<cv::Mat> vehicle_images_temp;
            for (int f = 0; f < first_camera.size(); f++)
            {
                if (second_camera[s].plate_number == first_camera[f].plate_number)   //车牌匹配成功，形成完整违法
                {

                    //每个相机跳选两张图，一共四张图，每个违法的最后一张图像为卡口图
                    if (second_camera.size() > 0)
                    {
                        vehicle_images_temp.push_back(first_camera[f].vehicle_images[first_camera[f].vehicle_images.size() / 2]);
                        vehicle_images_temp.push_back(first_camera[f].vehicle_images[first_camera[f].vehicle_images.size() - 1]);
                        vehicle_images_temp.push_back(second_camera[s].vehicle_images[second_camera[s].vehicle_images.size() / 2]);
                        vehicle_images_temp.push_back(second_camera[s].vehicle_images[second_camera[s].vehicle_images.size() - 1]);
                        second_camera[s].vehicle_images.clear();
                        second_camera[s].vehicle_images = vehicle_images_temp;   //从2个相机的违法图像中挑选4张用于最后合成
                        vector<cv::Point> contour1111;
                        img_merge(second_camera, 0, contour1111, m_illegal_data[0]);
                    }
                }
            }
        }

        //数据清除：第二个相机的数据均可以清除，第一个相机的数据超过3分钟清除
        if (m_illegal_data[ID_first_camera].detection_data_prohibited.size() > 0)
        {
            int my_index = m_illegal_data[ID_first_camera].detection_data_prohibited.size() - 1;
            for (vector<vehicle_detect_data>::iterator it_it = m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].detection_data.begin();
                 it_it != m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].detection_data.end(); it_it++)
            {
                double end      = (double)cv::getTickCount();
                double end_time = end * 1000 / cv::getTickFrequency();
                if ((end_time - it_it->start_time) > 3000)   //大于1分钟可以清除
                {
                    m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].detection_data.erase(it_it);

                    it_it--;
                }
            }

            //清除output_data
            if (m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].output_data.size() > 0)
            {
                for (vector<vehicle_output_data>::iterator it_it = m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].output_data.begin();
                     it_it != m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].output_data.end(); it_it++)
                {
                    double end      = (double)cv::getTickCount();
                    double end_time = end * 1000 / cv::getTickFrequency();
                    if ((end_time - it_it->start_time) > 3000)   //大于1分钟可以清除
                    {
                        m_illegal_data[ID_first_camera].detection_data_prohibited[my_index].output_data.erase(it_it);

                        it_it--;
                    }
                }
            }
        }

        if (m_illegal_data[ID_second_camera].detection_data_prohibited.size() > 0)
        {
            m_illegal_data[ID_second_camera].detection_data_prohibited[0].detection_data.clear();    //第二个相机数据全部清除
            m_illegal_data[ID_second_camera].detection_data_prohibited[0].illegal_trackid.clear();   //第二个相机数据全部清除
            m_illegal_data[ID_second_camera].detection_data_prohibited[0].output_data.clear();       //第二个相机数据全部清除
        }
    }

    return PV_ERROR_OK;
}
//区间测速
PV_ERROR_CODE truckDetector::postprocess_speedmeasurement(vector<illegal_data> m_illegal_data, double speed_distance, double speed_limit)
{
    illegal_data first_camera_data;
    illegal_data sencond_camera_data;
    int          ID_first_camera  = -1;
    int          ID_second_camera = -1;
    if (m_illegal_data.size() >= 2)
    {
        for (int k = 0; k < m_illegal_data.size(); k++)
        {
            if (m_illegal_data[k].algo_para.camera_index == 0 && m_illegal_data[k].algo_para.bayonet_img_path != "")   //该相机为C类区间抓拍第一个相机，包括区间测速和区间C类抓拍
            {
                first_camera_data = m_illegal_data[k];
                ID_first_camera   = k;
                //利用卡口图进行分析，第一个相机的卡口位置为起点，第二个相机的卡口位置为终点
                //存储卡口图和对应的车牌号，同时存储卡口图时间
                vehicle_detect_data one_speed;
                double              begin      = (double)cv::getTickCount();
                double              start_time = begin * 1000 / cv::getTickFrequency();
                one_speed.start_time           = start_time;   //车辆违法第一次抓拍时间

                bayonet_data one_bayonet;
                // one_bayonet.bayonet_vehicle_image = first_camera_data.algo_para.bayonet_img;
                one_bayonet.bayonet_plate_number = first_camera_data.algo_para.plate_no;
                one_speed.bayonet_vehicle_images.push_back(one_bayonet);
                first_camera_data.detection_data_speedmeasurement.push_back(one_speed);   //第一个相机数据
            }
            if (m_illegal_data[k].algo_para.camera_index == 1 && m_illegal_data[k].algo_para.bayonet_img_path != "")   //该相机为C类区间抓拍，包括区间测速和区间C类抓拍
            {
                sencond_camera_data = m_illegal_data[k];
                ID_second_camera    = k;
                //利用卡口图进行分析，第一个相机的卡口位置为起点，第二个相机的卡口位置为终点
                //存储卡口图和对应的车牌号，同时存储卡口图时间
                vehicle_detect_data one_speed;
                double              begin      = (double)cv::getTickCount();
                double              start_time = begin * 1000 / cv::getTickFrequency();
                one_speed.start_time           = start_time;   //车辆违法第一次抓拍时间

                bayonet_data one_bayonet;
                // one_bayonet.bayonet_vehicle_image = sencond_camera_data.algo_para.bayonet_img;
                one_bayonet.bayonet_plate_number = sencond_camera_data.algo_para.plate_no;
                one_speed.bayonet_vehicle_images.push_back(one_bayonet);
                sencond_camera_data.detection_data_speedmeasurement.push_back(one_speed);   //第二个相机数据
            }
        }
        vector<int> remooved_id_first_camera;
        //将两个相机数据进行匹配
        for (int dd = 0; dd < first_camera_data.detection_data_speedmeasurement.size(); dd++)
        {
            //车牌匹配,利用同一辆车在两个相机中出现的时间差和时间的行驶距离计算平均车速
            for (int dd1 = 0; dd1 < sencond_camera_data.detection_data_speedmeasurement.size(); dd1++)
            {
                if (first_camera_data.detection_data_speedmeasurement[dd].bayonet_vehicle_images.size() > 0 &&
                    sencond_camera_data.detection_data_speedmeasurement[dd1].bayonet_vehicle_images.size() > 0)
                {
                    if (first_camera_data.detection_data_speedmeasurement[dd].bayonet_vehicle_images[0].bayonet_plate_number ==
                        sencond_camera_data.detection_data_speedmeasurement[dd1].bayonet_vehicle_images[0].bayonet_plate_number)
                    {
                        remooved_id_first_camera.push_back(dd);
                        double total_time    = sencond_camera_data.detection_data_speedmeasurement[dd1].start_time - first_camera_data.detection_data_speedmeasurement[dd].start_time;
                        total_time           = total_time / 1000 / 60 / 60;
                        double average_speed = speed_distance / total_time;   //距离为公里km，时间单位为小时h
                        if (average_speed > speed_limit)                      //超速
                        {
                            //二合一图像，写明区间距离，限速要求，车辆实际行驶的起点时间和终点时间，平均速度超过限速则为超速
                        }
                    }
                }
            }
        }
        //清除数据：第一个相机数据超时后确定不会超时则立刻清除，第二个相机在完成所有匹配后全部可以清除
        double time_limit = speed_distance / speed_limit;
        if (ID_first_camera >= 0 && ID_second_camera >= 0)
        {
            for (vector<vehicle_detect_data>::iterator it_it = m_illegal_data[ID_first_camera].detection_data_speedmeasurement.begin();
                 it_it != m_illegal_data[ID_first_camera].detection_data_speedmeasurement.end(); it_it++)
            {
                double end      = (double)cv::getTickCount();
                double end_time = end * 1000 / cv::getTickFrequency();
                if ((end_time - it_it->start_time) > time_limit)   //大于给定的时间限制肯定不超速，可以清除
                {
                    m_illegal_data[ID_first_camera].detection_data_speedmeasurement.erase(it_it);
                    it_it--;
                }
            }
            m_illegal_data[ID_second_camera].detection_data_speedmeasurement.clear();   //第二个相机数据全部清除
        }
    }
    return PV_ERROR_OK;
}

// flag_illegal=0不按规定保持车道行驶
// flag_illegal=1占用应急车道行驶
// 违法四合一图像生成
PV_ERROR_CODE truckDetector::img_merge(vector<vehicle_output_data>& output_data, int flag_illegal, vector<cv::Point> contour, illegal_data& one_illegal_data)
{
    // 定义边界填充的参数
    int top, bottom, left, right, fontheight, gap_w, gap_h;
    if (output_data.size() > 0)
    {
        top        = 120;                   // 顶部边界填充的像素数
        bottom     = 0;                     // 底部边界填充的像素数
        left       = 0;                     // 左侧边界填充的像素数
        right      = 0;                     // 右侧边界填充的像素数
        fontheight = 30;                    // 字体高度
        gap_w      = (int)top / 10;         // 左侧边距
        gap_h      = (int)(top - 20) / 3;   // 每行高度，底边边距留20，每行高度平均分配，一共3行
        // 填充类型
        int borderType = cv::BORDER_CONSTANT;   // 这里使用常量填充，还可以选择其他填充方式
        // 填充颜色
        cv::Scalar value = cv::Scalar(0, 0, 0);   // 黑色填充，对于BORDER_CONSTANT类型的填充有效

        //输出结果画框，四合一
        for (int i = 0; i < output_data.size(); i++)
        {
            vector<cv::Mat> out_img_copyMakeBorder;
            if (output_data[i].vehicle_images.size() >= 4 && output_data[i].vehicle_location_box.size() >= 4)
            {
                cv::Ptr<cv::freetype::FreeType2> ft2;
                ft2 = cv::freetype::createFreeType2();
                ft2->loadFontData("stzhongsong.ttf", 0);
                string plate_name = "车牌号: ";
                plate_name        = plate_name + output_data[i].plate_number;

                string plate_color = "车牌颜色: ";
                plate_color        = plate_color + output_data[i].plate_color;

                string vehicle_type = "车型: ";
                vehicle_type        = vehicle_type + output_data[i].vehicle_type;

                int m = 0;
                for (int mj = 0; mj < 4; mj++)
                {
                    if (output_data[i].vehicle_images.size() >= 4 && output_data[i].vehicle_images.size() < 8)
                    {
                        // m = output_data[i].vehicle_images.size() / 3 * mj;
                        if (mj == 0)
                        {
                            m = output_data[i].vehicle_images.size() - 4;
                        }
                        if (mj == 1)
                        {
                            m = output_data[i].vehicle_images.size() - 3;
                        }
                        if (mj == 2)
                        {
                            m = output_data[i].vehicle_images.size() - 2;
                        }
                        if (mj == 3)
                        {
                            m = output_data[i].vehicle_images.size() - 1;
                        }
                    }

                    if (output_data[i].vehicle_images.size() >= 8)
                    {
                        if (mj == 0)
                        {
                            m = output_data[i].vehicle_images.size() / 3;
                        }
                        if (mj == 1)
                        {
                            m = 1 * output_data[i].vehicle_images.size() / 2;
                        }
                        if (mj == 2)
                        {
                            m = 2 * output_data[i].vehicle_images.size() / 3;
                        }
                        if (mj == 3)
                        {
                            m = output_data[i].vehicle_images.size() - 1;
                        }
                    }

                    logger_truck->error("mmm111111111111111:{}", m);
                    logger_truck->error("mmm222222222222222:{}", output_data[i].vehicle_images.size());
                    //将4096*2160原始图像resize为1920*1080
                    resize(output_data[i].vehicle_images[m], output_data[i].vehicle_images[m], Size(1920, 1080), 0, 0, INTER_LINEAR);

                    // 应用边界填充
                    cv::Mat paddedImg;
                    try
                    {
                        cv::copyMakeBorder(output_data[i].vehicle_images[m], paddedImg, top, bottom, left, right, borderType, value);
                    }
                    catch (...)
                    {
                        logger_truck->error("img_merge copyMakeBorder error");
                    }

                    if (paddedImg.cols > 0 && paddedImg.rows > 0 && top > 0)
                    {
                        //第一行
                        string time_time = output_data[i].illegal_time[m];
                        time_time        = "违法时间: " + time_time;
                        ft2->putText(paddedImg, format("%s", time_time.c_str()), Point(gap_w, gap_h), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        string illegal_place = "违法地点: " + one_illegal_data.algo_para.camera_place;
                        ft2->putText(paddedImg, format("%s", illegal_place.c_str()), Point(gap_w + 600, gap_h), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", vehicle_type.c_str()), Point(gap_w + 1410, gap_h), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        //第二行
                        ft2->putText(paddedImg, format("%s", plate_name.c_str()), Point(gap_w, 2 * gap_h + fontheight / 10), 30, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", plate_color.c_str()), Point(gap_w + 300, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        // ft2->putText(paddedImg, format("%s", vehicle_type.c_str()), Point(gap_w + 600, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        if (flag_illegal == 0)
                        {
                            string illegal_code = "违法代码: " + one_illegal_data.algo_para.illegal_code_occupyLane;
                            ft2->putText(paddedImg, format("%s", illegal_code.c_str()), Point(gap_w + 600, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                            ft2->putText(paddedImg, format("%s", "违法行为: 驾驶机动车在高速公路上不按规定车道行驶"), Point(gap_w + 900, 2 * gap_h + fontheight / 10), fontheight,
                                         cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        }
                        if (flag_illegal == 1)
                        {
                            string illegal_code = "违法代码: " + one_illegal_data.algo_para.illegal_code_occupyEmergencyLane;
                            ft2->putText(paddedImg, format("%s", illegal_code.c_str()), Point(gap_w + 600, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                            ft2->putText(paddedImg, format("%s", "违法行为: 占用应急车道行驶"), Point(gap_w + 900, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        }

                        //第三行
                        int         length        = 18;
                        std::string random_string = generate_random_string(length);
                        random_string             = "防伪码: " + random_string;
                        string device_number      = "设备编号: " + one_illegal_data.algo_para.device_number;
                        ft2->putText(paddedImg, format("%s", device_number.c_str()), Point(gap_w, 3 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", random_string.c_str()), Point(gap_w + 600, 3 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        out_img_copyMakeBorder.push_back(paddedImg);
                    }
                }
            }

            // 水平合并
            cv::Mat hconcatImg1, hconcatImg2;
            cv::Mat vconcatImg;
            if (out_img_copyMakeBorder.size() == 4)
            {
                // 垂直合并
                try
                {
                    cv::hconcat(out_img_copyMakeBorder[0], out_img_copyMakeBorder[1], hconcatImg1);
                    cv::hconcat(out_img_copyMakeBorder[2], out_img_copyMakeBorder[3], hconcatImg2);
                    cv::vconcat(hconcatImg1, hconcatImg2, vconcatImg);
                }
                catch (...)
                {
                    logger_truck->error("img_merge hconcat or vconcat error");
                }
            }

            if (output_data[i].vehicle_images.size() >= 4 && !output_data[i].plate_img.empty() && vconcatImg.cols > 0 && vconcatImg.rows > 0)
            {
                // 放大为图像宽高的1/10
                Mat dst;
                try
                {
                    resize(output_data[i].plate_img, dst, Size((int)output_data[i].vehicle_images[0].cols / 10, (int)output_data[i].vehicle_images[0].rows / 10), 0, 0, INTER_LINEAR);
                }
                catch (...)
                {
                    logger_truck->error("img_merge resize error");
                    return PV_ERROR_EXCEPTION;
                }

                Mat imageROI, mask;
                try
                {
                    imageROI = vconcatImg(Rect(1, vconcatImg.rows - dst.rows - 1, dst.cols, dst.rows));
                    cvtColor(dst, mask, COLOR_BGR2GRAY);   //参数0显示为灰度图
                    dst.copyTo(imageROI, mask);
                }
                catch (...)
                {
                    logger_truck->error("img_merge vconcatImg error");
                    return PV_ERROR_EXCEPTION;
                }

                int64_t currentTimeInt64_save_name = getCurrentTimeInt64();
                // 设定压缩参数
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(10);   // 设置压缩质量，范围为0-100，越高质量越好，但文件越大

                if (flag_illegal == 0)
                {
                    output_data[i].illegal_image_name = "/data2/camera-handler/upFiles/TruckOccupyLeftLane_" + to_string(currentTimeInt64_save_name) + ".jpg";
                    string videofilename              = "/data2/camera-handler/upFiles/TruckOccupyLeftLane_" + to_string(currentTimeInt64_save_name) + ".avi";
                    // 压缩图像
                    try
                    {
                        imwrite("/data2/camera-handler/upFiles/TruckOccupyLeftLane_" + to_string(currentTimeInt64_save_name) + ".jpg", vconcatImg, compression_params);
                        // QtConcurrent::run(getvideo, videofilename, output_data[i].vehicle_images);
                        // getvideo(videofilename, output_data[i].vehicle_images);
                    }
                    catch (const cv::Exception& ex)
                    {
                        logger_truck->info("图像压缩失败！");
                        return PV_ERROR_EXCEPTION;
                    }
                }
                if (flag_illegal == 1)
                {
                    output_data[i].illegal_image_name = "/data2/camera-handler/upFiles/CarOccupyEmergenceLane_" + to_string(currentTimeInt64_save_name) + ".jpg";
                    string videofilename              = "/data2/camera-handler/upFiles/CarOccupyEmergenceLane_" + to_string(currentTimeInt64_save_name) + ".avi";
                    // 压缩图像
                    try
                    {
                        imwrite("/data2/camera-handler/upFiles/CarOccupyEmergenceLane_" + to_string(currentTimeInt64_save_name) + ".jpg", vconcatImg, compression_params);
                        // QtConcurrent::run(getvideo, videofilename, output_data[i].vehicle_images);
                        // getvideo(videofilename, output_data[i].vehicle_images);
                    }
                    catch (const cv::Exception& ex)
                    {
                        logger_truck->info("图像压缩失败！");
                        return PV_ERROR_EXCEPTION;
                    }
                }
            }
        }
    }

    return PV_ERROR_OK;
}
//主函数
PV_ERROR_CODE truckDetector::inference_test(std::string para_data, vector<json>& results_json)
{
    logger_truck->info("start inference_test");
    vector<std::string> para;
    Stringsplit(para_data, '&', para);

    logger_truck->info("para_data: {}", para_data);
    int is_C_flag = 0;
    for (int i = 0; i < para.size(); i++)
    {
        QString         jsonString = QString::fromStdString(para[i]);
        QJsonParseError jsonError;
        QJsonDocument   jsonDoc(QJsonDocument::fromJson(para[i].data(), &jsonError));
        if (jsonError.error == QJsonParseError::NoError)
        {
            if (jsonDoc.isNull())
            {
                logger_truck->info("jsonDoc erro");
            }
            // 获取根对象
            QJsonObject     jsonObj   = jsonDoc.object();
            algo_parameters algo_para = ParseParameters(jsonObj);
            int             flag_para = 0;
            for (int j = 0; j < m_illegal_data.size(); j++)
            {
                if (m_illegal_data[j].algo_para.camera_id == algo_para.camera_id)
                {
                    //已经存在的相机id，更新参数
                    flag_para                   = 1;
                    m_illegal_data[j].algo_para = algo_para;
                }
            }
            //不存在的相机id，添加到m_illegal_data
            if (flag_para == 0)
            {
                illegal_data one_camera;
                one_camera.algo_para = algo_para;
                one_camera.camera_id = algo_para.camera_id;
                if (one_camera.algo_para.camera_index == 1)   //判定是否需要开启C类区间抓拍
                {
                    is_C_flag = 1;
                }
                m_illegal_data.push_back(one_camera);
                m_illegal_data[m_illegal_data.size() - 1].bytetrack.init(m_illegal_data[m_illegal_data.size() - 1].params);
            }
        }
    }
    // m_illegal_data.size()为相机数量
    for (int k = 0; k < m_illegal_data.size(); k++)
    {
        logger_truck->info("***************************************************************************************************************************");
        logger_truck->info("camera_id: {}", m_illegal_data[k].algo_para.camera_id);
        logger_truck->info(" nframe: {}", m_illegal_data[k].nframe);
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //卡口数据解析
        Rect_<float> kakou_car_box;
        if (m_illegal_data[k].algo_para.vehicle_coordinate != "")
        {
            //获取卡口图像车辆位置
            // 使用 find 函数查找字符在字符串中的位置
            size_t Y_found = m_illegal_data[k].algo_para.vehicle_coordinate.find('Y');
            size_t W_found = m_illegal_data[k].algo_para.vehicle_coordinate.find('W');
            size_t H_found = m_illegal_data[k].algo_para.vehicle_coordinate.find('H');
            // 计算需要截取的长度
            int X_length = Y_found - 1;
            int Y_length = W_found - Y_found;
            int W_length = H_found - W_found;
            int H_length = m_illegal_data[k].algo_para.vehicle_coordinate.length() - H_found;

            // 使用substr方法截取字符串
            int car_X                                 = std::stoi(m_illegal_data[k].algo_para.vehicle_coordinate.substr(1, X_length));
            int car_Y                                 = std::stoi(m_illegal_data[k].algo_para.vehicle_coordinate.substr(Y_found + 1, Y_length));
            int car_W                                 = std::stoi(m_illegal_data[k].algo_para.vehicle_coordinate.substr(W_found + 1, W_length));
            int car_H                                 = std::stoi(m_illegal_data[k].algo_para.vehicle_coordinate.substr(H_found + 1, H_length));
            kakou_car_box                             = {(float)car_X, (float)car_Y, (float)car_W, (float)car_H};
            m_illegal_data[k].algo_para.kakou_car_box = kakou_car_box;
        }
        Rect_<float> kakou_plate_box;
        if (m_illegal_data[k].algo_para.plate_no_coordinate != "")
        {
            //获取卡口图像车辆位置
            // 使用 find 函数查找字符在字符串中的位置
            size_t Y_found = m_illegal_data[k].algo_para.plate_no_coordinate.find('Y');
            size_t W_found = m_illegal_data[k].algo_para.plate_no_coordinate.find('W');
            size_t H_found = m_illegal_data[k].algo_para.plate_no_coordinate.find('H');
            // 计算需要截取的长度
            int X_length = Y_found - 1;
            int Y_length = W_found - Y_found;
            int W_length = H_found - W_found;
            int H_length = m_illegal_data[k].algo_para.plate_no_coordinate.length() - H_found;
            // 使用substr方法截取字符串
            int plate_X                                 = std::stoi(m_illegal_data[k].algo_para.plate_no_coordinate.substr(1, X_length));
            int plate_Y                                 = std::stoi(m_illegal_data[k].algo_para.plate_no_coordinate.substr(Y_found + 1, Y_length));
            int plate_W                                 = std::stoi(m_illegal_data[k].algo_para.plate_no_coordinate.substr(W_found + 1, W_length));
            int plate_H                                 = std::stoi(m_illegal_data[k].algo_para.plate_no_coordinate.substr(H_found + 1, H_length));
            kakou_plate_box                             = {(float)plate_X, (float)plate_Y, (float)plate_W, (float)plate_H};
            m_illegal_data[k].algo_para.kakou_plate_box = kakou_plate_box;
        }

        m_illegal_data[k].nframe = m_illegal_data[k].nframe + 1;
        cv::Mat img;
        try
        {
            if (m_illegal_data[k].algo_para.img_path != "")
            {
                logger_truck->info("imread  img_path");
                img = imread(m_illegal_data[k].algo_para.img_path);
                vector<std::string> img_time;
                Stringsplit(m_illegal_data[k].algo_para.img_path, '_', img_time);
                img_time[1].erase(img_time[1].length() - 4);   // 从字符串末尾移除指定数量的字符
                m_illegal_data[k].algo_para.img_time = img_time[1];
            }
            else
            {
                if (m_illegal_data[k].algo_para.bayonet_img_path != "")
                {
                    logger_truck->info("imread  unv_img_path");
                    img = imread(m_illegal_data[k].algo_para.bayonet_img_path);
                    vector<std::string> img_name;
                    Stringsplit(m_illegal_data[k].algo_para.bayonet_img_path, '/', img_name);
                    vector<std::string> img_time;
                    Stringsplit(img_name[img_name.size() - 1], '_', img_time);
                    m_illegal_data[k].algo_para.img_time = img_time[0];
                }
            }
        }
        catch (...)
        {
            logger_truck->error("img read error");
            return PV_ERROR_EXCEPTION;
        }

        if (img.empty())
        {
            logger_truck->error("img empty");
            return PV_ERROR_EXCEPTION;
        }

        bm_image    image;
        bm_status_t bmret = cv::bmcv::toBMI(img, &image, true);
        if (bmret != BM_SUCCESS)
        {
            return PV_ERROR_EXCEPTION;
        }

        //(1) 根据划定结果获取所有车道区域
        logger_truck->info(" start get contours");
        vector<laneElements> contour_leftlane_isNoOvertaking;
        vector<laneElements> contour_secondlane_isNoOvertaking;
        vector<laneElements> contour_rightlane;
        vector<laneElements> contour_emergencelane;
        //（1.1）如果当前道路为禁止超车路段,只需要获取所有禁行车道和对应的禁行车型即可
        if (m_illegal_data[k].algo_para.isNoOvertaking == 1)   //禁止超车路段
        {
            //获取所有禁行路段和对应的禁行车型
            for (auto& lane : m_illegal_data[k].algo_para.lanes)
            {
                if (lane.prohibitedVehicles.size() > 0 && lane.id != 100)   //禁行车型数量
                {
                    if (lane.prohibitedVehicles[0] >= 0)   //禁行车型类别为：0，1，2，3，4，-1表示正常通行车道，-2表示超车道
                    {
                        contour_leftlane_isNoOvertaking.push_back(lane);
                    }
                }
                //获取应急车道
                if (lane.id == 100)   //车道号为0的是应急车道
                {
                    contour_emergencelane.push_back(lane);
                }
            }
        }
        else   //(1.2)如果为允许超车路段，需获取禁行车道，超车道和右侧车道
        {

            for (auto& lane : m_illegal_data[k].algo_para.lanes)
            {
                //(a)首先获取禁行车道，存在禁行车型的车道即为禁行车道
                if (lane.prohibitedVehicles.size() > 0 && lane.id != 100)   //禁行车型数量
                {
                    if (lane.prohibitedVehicles[0] >= 0)   //禁行车型类别为：0，1，2，3，4，-1表示右车道，-2表示超车道
                    {
                        contour_leftlane_isNoOvertaking.push_back(lane);
                    }
                }
                //(b)获取超车道

                if (lane.prohibitedVehicles.size() == 1 && lane.id != 100)   //禁行车型数量
                {
                    if (lane.prohibitedVehicles[0] == -2)   //-2表示超车道
                    {
                        contour_secondlane_isNoOvertaking.push_back(lane);
                    }
                }
                //(d)获取所有右侧车道
                if (lane.prohibitedVehicles.size() == 1 && lane.id != 100)   //禁行车型数量
                {
                    if (lane.prohibitedVehicles[0] == -1)   //-1表示右车道
                    {
                        contour_rightlane.push_back(lane);
                    }
                }
                //(d)获取应急车道
                if (lane.id == 100)   //车道号为0的是应急车道
                {
                    contour_emergencelane.push_back(lane);
                }
            }
        }
        /////////////////////////////////////////////////////////获取车牌抓取位置////////////////////////////////////////////////////////////////////
        //(1.3)获取禁行车道对应的车牌检测位置
        for (int i1 = 0; i1 < contour_leftlane_isNoOvertaking.size(); i1++)
        {
            int miny = 100000;
            int maxy = 0;
            for (int i = 0; i < contour_leftlane_isNoOvertaking[i1].laneRegion.size(); i++)
            {
                if (contour_leftlane_isNoOvertaking[i1].laneRegion[i].y > maxy)
                {
                    maxy = contour_leftlane_isNoOvertaking[i1].laneRegion[i].y;
                }
                if (contour_leftlane_isNoOvertaking[i1].laneRegion[i].y < miny)
                {
                    miny = contour_leftlane_isNoOvertaking[i1].laneRegion[i].y;
                }
            }
            int plate_Ymax_leftlane                       = maxy - (maxy - miny) / 10;
            contour_leftlane_isNoOvertaking[i1].plateYmax = plate_Ymax_leftlane;
        }
        //(1.4)获取超车道对应的车牌检测位置
        for (int i2 = 0; i2 < contour_secondlane_isNoOvertaking.size(); i2++)
        {
            int miny = 100000;
            int maxy = 0;
            for (int i = 0; i < contour_secondlane_isNoOvertaking[i2].laneRegion.size(); i++)
            {
                if (contour_secondlane_isNoOvertaking[i2].laneRegion[i].y > maxy)
                {
                    maxy = contour_secondlane_isNoOvertaking[i2].laneRegion[i].y;
                }
                if (contour_secondlane_isNoOvertaking[i2].laneRegion[i].y < miny)
                {
                    miny = contour_secondlane_isNoOvertaking[i2].laneRegion[i].y;
                }
            }
            int plate_Ymax_secondlane                       = maxy - (maxy - miny) / 10;
            contour_secondlane_isNoOvertaking[i2].plateYmax = plate_Ymax_secondlane;
        }

        //(1.5)获取应急车道对应的车牌检测位置
        for (int i3 = 0; i3 < contour_emergencelane.size(); i3++)
        {
            int miny = 100000;
            int maxy = 0;
            for (int i = 0; i < contour_emergencelane[i3].laneRegion.size(); i++)
            {
                if (contour_emergencelane[i3].laneRegion[i].y > maxy)
                {
                    maxy = contour_emergencelane[i3].laneRegion[i].y;
                }
                if (contour_emergencelane[i3].laneRegion[i].y < miny)
                {
                    miny = contour_emergencelane[i3].laneRegion[i].y;
                }
            }
            int plate_Ymax_emergencelane        = maxy - (maxy - miny) / 10;
            contour_emergencelane[i3].plateYmax = plate_Ymax_emergencelane;
        }

        logger_truck->info("end get contours");

        logger_truck->info("start cardetect");
        //(2) yolov8车牌车辆检测
        std::vector<bm_image> batch_imgs;
        batch_imgs.push_back(image);
        std::vector<YoloV8BoxVec> boxes;
        std::vector<YoloV8BoxVec> boxes_is_empty_rightlane;
        try
        {
            cardetect(batch_imgs, boxes);
        }
        catch (...)
        {
            logger_truck->error("cardetect error");
            return PV_ERROR_EXCEPTION;
        }

        logger_truck->info("end cardetect");

        logger_truck->info("start cartrack");
        //(3) bytetrack跟踪
        std::vector<std::vector<BoundingBox>> yolov8_BoundingBoxes;
        for (size_t i = 0; i < batch_imgs.size(); i++)
        {
            logger_truck->info("boxes[i].size(): {}", boxes[i].size());
            std::vector<BoundingBox> yolov8_boxes_BoundingBox;
            YoloV8BoxVec             b_is_empty_rightlane;
            for (auto b : boxes[i])
            {
                //跟踪所有车辆
                if (b.class_id == 0 || b.class_id == 1 || b.class_id == 2)
                {
                    yolov8_boxes_BoundingBox.push_back({(int)b.x1, (int)b.y1, (int)(b.x2 - b.x1), (int)(b.y2 - b.y1), b.score, b.class_id});
                }
                if (b.score > 0.6)
                {
                    b_is_empty_rightlane.push_back(b);
                    // cv::rectangle(img, cv::Point((int)b.x1, (int)b.y1), cv::Point((int)b.x2, (int)b.y2), cv::Scalar(0, 255, 0), 2);
                }
            }
            boxes_is_empty_rightlane.push_back(b_is_empty_rightlane);
            yolov8_BoundingBoxes.push_back(yolov8_boxes_BoundingBox);
        }

        vector<STracks> results_stracks;
        try
        {
            cartrack(yolov8_BoundingBoxes, results_stracks, m_illegal_data[k]);
        }
        catch (...)
        {
            logger_truck->error("cartrack error");
            return PV_ERROR_EXCEPTION;
        }
        logger_truck->info("end cartrack");

        logger_truck->info("start select results_stracks");
        //(4) 挑选跟踪框：只跟踪货车、载客汽车
        vector<STracks> results_stracks_select;
        vector<STracks> results_stracks_select_truck;
        vector<STracks> results_stracks_select_yellow_truck;
        vector<STracks> results_stracks_select_yellow_truck_bus;
        for (size_t i = 0; i < batch_imgs.size(); i++)
        {
            STracks one_stracks_select;         //检测大货车和载客汽车
            STracks one_stracks_select_truck;   //只检测大货车
            // STracks one_stracks_select_yellow_truck;       //只检测黄牌货车
            STracks one_stracks_select_yellow_truck_bus;   //只检测黄牌客车和货车
            for (auto& bbox : results_stracks[i])          //所有车辆
            {
                Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};
                //只跟踪黄牌载客汽车和货车
                if (bbox->class_id == 1 && bbox->score > m_illegal_data[k].algo_para.vehicle_conf)
                {
                    for (auto bb : boxes[i])
                    {
                        if (bb.class_id == 4 || bb.class_id == 5)
                        {
                            cv::Rect_<float> bb_plate = {bb.x1, bb.y1, (bb.x2 - bb.x1), (bb.y2 - bb.y1)};
                            //计算车辆bus和黄牌的iou
                            double ratio_yellow_plate = GetIOU_rect(bb_truck, bb_plate);

                            if (ratio_yellow_plate > 0.9)
                            {
                                one_stracks_select.push_back(bbox);
                                one_stracks_select_yellow_truck_bus.push_back(bbox);   //黄牌客车
                            }
                        }
                    }
                }

                if ((bbox->class_id == 2) && bbox->score > m_illegal_data[k].algo_para.vehicle_conf)
                {
                    one_stracks_select.push_back(bbox);
                    one_stracks_select_truck.push_back(bbox);
                    for (auto bb : boxes[i])
                    {
                        if (bb.class_id == 4 || bb.class_id == 5)
                        {
                            cv::Rect_<float> bb_plate = {bb.x1, bb.y1, (bb.x2 - bb.x1), (bb.y2 - bb.y1)};
                            //计算车辆bus和黄牌的iou
                            double ratio_yellow_plate = GetIOU_rect(bb_truck, bb_plate);

                            if (ratio_yellow_plate > 0.9)
                            {
                                one_stracks_select_yellow_truck_bus.push_back(bbox);   //黄牌货车
                            }
                        }
                    }
                }
            }
            results_stracks_select.push_back(one_stracks_select);
            results_stracks_select_truck.push_back(one_stracks_select_truck);
            // results_stracks_select_yellow_truck.push_back(one_stracks_select_yellow_truck);
            results_stracks_select_yellow_truck_bus.push_back(one_stracks_select_yellow_truck_bus);
        }

        logger_truck->info("end select results_stracks");

        int flag_left_removed_stracks      = 0;
        int flag_emergence_removed_stracks = 0;
        int flag_second_removed_stracks    = 0;

        //(5) 违法判定+证据输出
        for (size_t i = 0; i < batch_imgs.size(); i++)
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //接打电话和不系安全带检测
            if (m_illegal_data[k].algo_para.is_detect_BeltAndPhone == 1 || m_illegal_data[k].algo_para.is_detect_BeltAndPhone == 2 || m_illegal_data[k].algo_para.is_detect_BeltAndPhone == 3)
            {
                auto        removed_stracks = m_illegal_data[k].bytetrack.get_removed_stracks();
                vector<int> removed_track_id;
                //记录完整违法车辆id，若图像中该id消失，则清除该id
                for (auto& bbox : removed_stracks)
                {
                    removed_track_id.push_back(bbox->track_id);
                }
                logger_truck->info("开始！！接打电话不系安全带");
                try
                {
                    m_phone_safetybelt.postprocess(m_illegal_data[k].nframe, img, img, m_illegal_data[k].algo_para.timeInterval, boxes[i], results_stracks[i], results_json, m_platenet,
                                                   removed_track_id, m_illegal_data[k].algo_para);
                }
                catch (...)
                {
                    logger_truck->error(" m_phone_safetybelt.postprocess error");
                    return PV_ERROR_EXCEPTION;
                }

                logger_truck->info("结束！！接打电话不系安全带");
            }
            //////////////////////////////////////////////////////////不按规定车道行驶//////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //(5.1)禁行车道检测，可能存在多条禁行车道
            if (m_illegal_data[k].algo_para.lanes.size() > 0)
            {
                if (contour_leftlane_isNoOvertaking.size() > 0)   //禁行车道
                {
                    logger_truck->info("开始！！不按规定车道行驶---禁行车道");
                    flag_left_removed_stracks = 1;

                    for (int idx = 0; idx < contour_leftlane_isNoOvertaking.size(); idx++)
                    {
                        try
                        {
                            //根据传入的车型类型确定需要检测的车型
                            int car_flag   = 0;
                            int truck_flag = 0;
                            int bus_flag   = 0;

                            if (contour_leftlane_isNoOvertaking[idx].prohibitedVehicles.size() > 0)
                            {
                                //车辆型号：小汽车为0，中型货车为1，大型货车为2，中型客车为3，大型客车为4
                                for (auto& cartype : contour_leftlane_isNoOvertaking[idx].prohibitedVehicles)
                                {
                                    if (cartype == 0)
                                    {
                                        car_flag = 1;
                                    }
                                    if (cartype == 1 || cartype == 2)
                                    {
                                        truck_flag = 1;
                                    }
                                    if (cartype == 3 || cartype == 4)
                                    {
                                        bus_flag = 1;
                                    }
                                }
                            }

                            //所有车型都检测
                            if (car_flag == 1 && truck_flag == 1 && bus_flag == 1)
                            {
                                postprocess_occupyleftlane(img, img, contour_leftlane_isNoOvertaking[idx], boxes[i], results_stracks[i], results_json, m_illegal_data[k], 0);
                            }
                            //只检测货车和客车
                            if (car_flag == 0 && truck_flag == 1 && bus_flag == 1)
                            {
                                postprocess_occupyleftlane(img, img, contour_leftlane_isNoOvertaking[idx], boxes[i], results_stracks_select[i], results_json, m_illegal_data[k], 0);
                            }
                            //只检测货车
                            if (car_flag == 0 && truck_flag == 1 && bus_flag == 0)
                            {
                                postprocess_occupyleftlane(img, img, contour_leftlane_isNoOvertaking[idx], boxes[i], results_stracks_select_truck[i], results_json, m_illegal_data[k], 0);
                            }
                            //只检测黄牌货车和客车
                            if (car_flag == 0 && truck_flag == 0 && bus_flag == 1)
                            {
                                postprocess_occupyleftlane(img, img, contour_leftlane_isNoOvertaking[idx], boxes[i], results_stracks_select_yellow_truck_bus[i], results_json, m_illegal_data[k], 0);
                            }
                        }
                        catch (...)
                        {
                            logger_truck->error("postprocess_occupyleftlane error");
                            return PV_ERROR_EXCEPTION;
                        }
                    }

                    logger_truck->info("结束！！不按规定车道行驶---禁行车道");
                }
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //(5.2)超车道检测,需要判断右侧是否空旷,一般只有一条超车道

                if (contour_secondlane_isNoOvertaking.size() > 0 && contour_rightlane.size() > 0)   //超车道
                {
                    logger_truck->info("开始！！不按规定车道行驶---超车道");
                    flag_second_removed_stracks = 1;
                    for (auto& lane_second : contour_secondlane_isNoOvertaking)
                    {
                        try
                        {
                            //只检测货车
                            postprocess_occupysecondlane(img, img, lane_second, contour_rightlane, boxes_is_empty_rightlane[i], results_stracks_select_truck[i], results_json, m_illegal_data[k]);
                        }
                        catch (...)
                        {
                            logger_truck->error("postprocess_occupyleftlane error");
                            return PV_ERROR_EXCEPTION;
                        }
                    }

                    logger_truck->info("结束！！不按规定车道行驶---超车道");
                }
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //(5.3)应急车道检测
                if (m_illegal_data[k].algo_para.isEmergencyLane == 1 && contour_emergencelane.size() > 0)
                {
                    logger_truck->info("开始！！占用应急车道行驶");
                    flag_emergence_removed_stracks = 1;
                    try
                    {
                        postprocess_occupyemergencelane(img, img, contour_emergencelane[0], boxes[i], results_stracks[i], results_json, m_illegal_data[k]);
                        // postprocess_occupyemergencelane_driving(img, contour_emergencelane[0], boxes[i], results_stracks[i], results_json, m_illegal_data[k]);
                        // m_illegal_data[k].bytetrack.clear_removed_stracks();
                    }
                    catch (...)
                    {
                        logger_truck->error("postprocess_occupyemergencelane error");
                        return PV_ERROR_EXCEPTION;
                    }
                    logger_truck->info("结束！！占用应急车道行驶");
                }
                //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //(5.4)C类区间抓拍
                if (is_C_flag == 1)   //先根据相机配置情况确定是否需要进行C类区间抓拍
                {
                    if (m_illegal_data[k].algo_para.camera_index == 0)   //传入第一个相机的超车道进行检测
                    {
                        postprocess_occupyleftlane(img, img, contour_secondlane_isNoOvertaking[0], boxes[i], results_stracks[i], results_json, m_illegal_data[k], 1);
                    }
                    if (m_illegal_data[k].algo_para.camera_index == 1)   //传入第二个相机的超车道进行检测
                    {
                        postprocess_occupyleftlane(img, img, contour_secondlane_isNoOvertaking[0], boxes[i], results_stracks[i], results_json, m_illegal_data[k], 1);
                    }
                }
            }
        }
        // imwrite("/data2/lj/" + to_string(m_illegal_data[k].nframe) + "output.jpg", img);

        //使用后清除消失的id
        m_illegal_data[k].bytetrack.clear_removed_stracks();
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(5.4)C类区间抓拍
        if (is_C_flag == 1)
        {
            postprocess_interval_detection(m_illegal_data);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(5.5)区间测速
        //        double speed_distance = 10;   //区间距离为10km
        //        double speed_limit    = 80;   //限速80km/h
        //        postprocess_speedmeasurement(m_illegal_data, speed_distance, speed_limit);

        logger_truck->info("end inference_test");
        if (batch_imgs.size() > 0)
        {
            bm_image_destroy(batch_imgs[0]);
        }
        batch_imgs.clear();
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    return PV_ERROR_OK;
}
