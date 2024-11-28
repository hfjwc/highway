#include "cardetector.h"

vector<radar_data_time> radar_data;

int                total_bytes = 0;
int                listenfd, connfd;
struct sockaddr_in servaddr, cliaddr;
socklen_t          len;

int64_t getCurrentTimeInt64()
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
void radar_connect()
{
    // 创建TCP套接字
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd < 0)
    {
        std::cerr << "Error creating socket" << std::endl;
        // return -1;
    }

    // 设置服务器地址信息
    servaddr.sin_family      = AF_INET;
    servaddr.sin_port        = htons(6841);
    servaddr.sin_addr.s_addr = INADDR_ANY;
    // inet_pton(AF_INET, addr, &address.sin_addr);

    // 绑定套接字到地址和端口
    if (bind(listenfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        std::cerr << "Error binding socket" << std::endl;
        close(listenfd);
        // return -1;
    }

    // 监听连接
    if (listen(listenfd, 5) < 0)
    {
        std::cerr << "Error listening" << std::endl;
        close(listenfd);
        // return -1;
    }

    // 接受连接
    len    = sizeof(cliaddr);
    connfd = accept(listenfd, (struct sockaddr*)&cliaddr, &len);
    if (connfd < 0)
    {
        std::cerr << "Error accepting connection" << std::endl;
        close(listenfd);
        // return -1;
    }
}
cardetector::cardetector()
    : m_truckyolov8(NULL)
    , m_platenet(NULL)
{
    //创建保存路径
    string      save_path_log     = "/data2/camera-handler/results/log";
    string      save_path_log_txt = "/data2/camera-handler/results/log/不按规定保持车距行驶.txt";
    struct stat info;
    if (stat(save_path_log.c_str(), &info) != 0)
    {
        cout << "保存路径不存在，创建目录..." << endl;
        mkdir(save_path_log.c_str(), 0777);
    }

    int64_t currentTimeInt64_save_name = getCurrentTimeInt64();
    logger_car                         = spdlog::rotating_logger_mt(to_string(currentTimeInt64_save_name) + "logger_carnokeepdistance", save_path_log_txt, 100 * 1024 * 1024, 2, true);
    logger_car->set_level(spdlog::level::trace);   //跟踪所有日志
    logger_car->flush_on(spdlog::level::trace);    //刷新所有跟踪的日志
    logger_car->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    // 添加终端输出目标
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    logger_car->sinks().push_back(console_sink);

    // 1. yolov8车辆车牌检测模型加载和初始化
    string bmodel_file = "yolov8s_5.10.bmodel";
    // creat handle
    int           dev_id = 0;
    BMNNHandlePtr handle = make_shared<BMNNHandle>(dev_id);
    // load bmodel
    shared_ptr<BMNNContext> bm_ctx = make_shared<BMNNContext>(handle, bmodel_file.c_str());

    // initialize net
    if (m_truckyolov8 == NULL)
        m_truckyolov8 = new YoloV8(bm_ctx);
    float       conf_thresh = 0.2;
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

    //雷达数据转换矩阵
    filename_radar_matrix = "radar_matrix_wanning.xml";

    radar_connect();
}

cardetector::~cardetector()
{
    delete m_truckyolov8;
    delete m_platenet;
    close(connfd);

    close(listenfd);
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
PV_ERROR_CODE cardetector::cardetect(std::vector<bm_image> batch_imgs, std::vector<YoloV8BoxVec>& boxes)
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
PV_ERROR_CODE cardetector::cartrack(std::vector<std::vector<BoundingBox>> yolov8_BoundingBoxes, vector<STracks>& results_stracks, illegal_data& one_illegal_data)
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
PV_ERROR_CODE cardetector::platerecognition(cv::Mat plate_img, vector<string>& results)
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

//不按规定保持车距解析参数
algo_parameters ParseParameters2(QJsonObject& jsonObj)
{
    algo_parameters algo_para;
    algo_para.lanes.clear();
    algo_para.bayonet_img_path    = "";
    algo_para.vehicle_coordinate  = "";
    algo_para.plate_no_coordinate = "";
    algo_para.plate_no            = "";
    algo_para.img_path            = "";
    QJsonArray regionsArray       = jsonObj["lanes"].toArray();
    for (auto regionValue : regionsArray)
    {
        QJsonObject  regionObj = regionValue.toObject();
        laneElements laneEle;

        //        // 解析 carType
        //        QJsonArray carTypeArray = regionObj["carType"].toArray();
        //        for (const QJsonValue& carTypeValue : carTypeArray)
        //        {
        //            laneEle.prohibitedVehicles.push_back(carTypeValue.toString().toInt());
        //        }

        //        // 解析 laneNo
        //        laneEle.id = regionObj["laneNo"].toString().toInt();

        // 解析 coordinates-OriginLine
        QJsonArray coordinatesArray = regionObj["OriginLine"].toArray();
        for (auto coordinateValue : coordinatesArray)
        {
            QJsonArray coordinateArray = coordinateValue.toArray();
            if (coordinateArray.size() == 2)
            {
                cv::Point lanepoint = {coordinateArray[0].toInt(), coordinateArray[1].toInt()};
                laneEle.laneRegion.push_back(lanepoint);
            }
        }

        // 解析 coordinates-midLine
        QJsonArray coordinatesArray1 = regionObj["MidpointLine"].toArray();
        for (auto coordinateValue1 : coordinatesArray1)
        {
            QJsonArray coordinateArray1 = coordinateValue1.toArray();
            if (coordinateArray1.size() == 2)
            {
                cv::Point lanepoint = {coordinateArray1[0].toInt(), coordinateArray1[1].toInt()};
                laneEle.laneRegion.push_back(lanepoint);
            }
        }

        // 解析 coordinates-finishLine
        QJsonArray coordinatesArray2 = regionObj["FinishLine"].toArray();
        for (auto coordinateValue2 : coordinatesArray2)
        {
            QJsonArray coordinateArray2 = coordinateValue2.toArray();
            if (coordinateArray2.size() == 2)
            {
                cv::Point lanepoint = {coordinateArray2[0].toInt(), coordinateArray2[1].toInt()};
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

algo_parameters2 ParseParameters(std::string para)
{
    algo_parameters2 algo_para;
    algo_para.regions.clear();
    QString     qStr  = QString::fromStdString(para);
    QStringList sList = qStr.split(",");

    cv::Point point;
    //获取第一个参数，即区域的数量
    int regionNum = sList[0].toInt();
    //下一部分的起始位置
    int nextPosition = 1;
    for (int i = 0; i < regionNum; i++)
    {
        std::vector<cv::Point> region;
        //获取当前部分区域点的数量
        int pointsNum = sList[nextPosition].toInt();
        //获取当前部分的结束位置
        int regionEnd = nextPosition + pointsNum * 2;
        for (int j = nextPosition + 1; j < regionEnd + 1; j = j + 2)
        {
            point.x = sList[j].toFloat();
            point.y = sList[j + 1].toFloat();
            region.push_back(point);
        }
        algo_para.regions.push_back(region);   //所有的区域集合
        //更新下一部分的起始位置
        nextPosition = regionEnd + 1;
    }

    //解析除区域外的参数
    algo_para.img_path = sList[nextPosition].toStdString();   //图片路径
    algo_para.detect_lane = sList[nextPosition + 1].toInt();   //违法车道，0表示都不需要，1表示只关注最左侧车道，2表示只关注应急车道，3表示关注最左侧和应急车道
    algo_para.timeInterval    = sList[nextPosition + 2].toInt();     //取证时间间隔，单位毫秒
    algo_para.vehicle_type    = sList[nextPosition + 3].toInt();     //需要检测的车辆类型，0表示检测所有车辆，1表示检测大货车、大客车
    algo_para.vehicle_conf    = sList[nextPosition + 4].toFloat();   //车辆检测的置信度
    algo_para.is_detect_plate = sList[nextPosition + 5].toInt();     //是否需要检测车牌，0表示不需要，1表示需要
    algo_para.plate_conf      = sList[nextPosition + 6].toFloat();   //车牌检测的置信度
    algo_para.is_detect_BeltAndPhone = sList[nextPosition + 7].toInt();   //是否需要检测安全带和打电话，0表示都不需要检测，1表示打电话，2表示不系安全带，3表示两种都需要
    algo_para.belt_conf              = sList[nextPosition + 8].toFloat();                  //安全带检测的置信度
    algo_para.phone_conf             = sList[nextPosition + 9].toFloat();                  //打电话检测的置信度
    algo_para.device_number                    = sList[nextPosition + 10].toStdString();   //设备编号，盒子备案后会确定，每台盒子不一样
    algo_para.illegal_code_occupyLane          = sList[nextPosition + 11].toStdString();   //不按规定车道行驶违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_occupyEmergencyLane = sList[nextPosition + 12].toStdString();   //占用应急车道违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_safeDistance        = sList[nextPosition + 13].toStdString();   //未按规定保持车距违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_nobelt              = sList[nextPosition + 14].toStdString();   //不系安全带违法代码,根据算法功能确定，可能会改
    algo_para.illegal_code_phone               = sList[nextPosition + 15].toStdString();   //开车打电话违法代码,根据算法功能确定，可能会改
    algo_para.camera_place                     = sList[nextPosition + 16].toStdString();   //违法地点，根据相机确定
    algo_para.camera_id                        = sList[nextPosition + 17].toStdString();   //相机id

    return algo_para;
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

int LineLineCollisionDetection(FloatRet_ line1, FloatRet_ line2)   //线段交叉测试
{
    if ((line1.x1 > line1.x2 ? line1.x1 : line1.x2) < (line2.x1 < line2.x2 ? line2.x1 : line2.x2) || (line1.y1 > line1.y2 ? line1.y1 : line1.y2) < (line2.y1 < line2.y2 ? line2.y1 : line2.y2) ||
        (line2.x1 > line2.x2 ? line2.x1 : line2.x2) < (line1.x1 < line1.x2 ? line1.x1 : line1.x2) || (line2.y1 > line2.y2 ? line2.y1 : line2.y2) < (line1.y1 < line1.y2 ? line1.y1 : line1.y2))
    {
        return 0;
    }
    if ((((line1.x1 - line2.x1) * (line2.y2 - line2.y1) - (line1.y1 - line2.y1) * (line2.x2 - line2.x1)) *
         ((line1.x2 - line2.x1) * (line2.y2 - line2.y1) - (line1.y2 - line2.y1) * (line2.x2 - line2.x1))) > 0 ||
        (((line2.x1 - line1.x1) * (line1.y2 - line1.y1) - (line2.y1 - line1.y1) * (line1.x2 - line1.x1)) *
         ((line2.x2 - line1.x1) * (line1.y2 - line1.y1) - (line2.y2 - line1.y1) * (line1.x2 - line1.x1))) > 0)
    {
        return 0;
    }
    return 1;
}

int AreaLineCollisionDetection(FloatRet_ box, FloatRet_ line)   //线段矩形相交测试
{
    FloatRet_ line1 = {0};
    line1.x1        = box.x1;
    line1.x2        = box.x2;
    line1.y1        = box.y1;
    line1.y2        = box.y1;

    FloatRet_ line2 = {0};
    line2.x1        = box.x1;
    line2.x2        = box.x2;
    line2.y1        = box.y2;
    line2.y2        = box.y2;

    FloatRet_ line3 = {0};
    line3.x1        = box.x1;
    line3.x2        = box.x1;
    line3.y1        = box.y1;
    line3.y2        = box.y2;

    FloatRet_ line4 = {0};
    line4.x1        = box.x2;
    line4.x2        = box.x2;
    line4.y1        = box.y1;
    line4.y2        = box.y2;

    if (LineLineCollisionDetection(line1, line) || LineLineCollisionDetection(line2, line) || LineLineCollisionDetection(line3, line) || LineLineCollisionDetection(line4, line))
    {
        return 1;
    }
    return 0;
}
PV_ERROR_CODE cardetector::worldtoPixel(std::vector<cv::Point2f> srcPt, std::vector<cv::Point2f>& dstPt)
{
    // 使用FileStorage读取矩阵
    cv::FileStorage file(filename_radar_matrix, cv::FileStorage::READ);
    if (!file.isOpened())
    {
        std::cerr << "Unable to open file for reading: " << filename_radar_matrix << std::endl;
        return PV_ERROR_EXCEPTION;
    }
    cv::Mat affineMatrix;
    file["AffineMatrix"] >> affineMatrix;
    file.release();

    // 使用单应性矩阵进行点的变换
    cv::perspectiveTransform(srcPt, dstPt, affineMatrix);
    return PV_ERROR_OK;
}
//根据雷达获取车辆速度
PV_ERROR_CODE cardetector::get_speed(car_detect_data& one_detect, std::shared_ptr<STrack> bbox, cv::Mat frame_to_draw, radar_data_time radar_data_one)
{
    // RadarSoft one_radar;
    //遍历雷达数据中的所有车辆，将雷达车辆和图像车辆进行匹配
    int nums = static_cast<int>(radar_data_one.radar_output.numObject);
    logger_car->info("get speed get speed get speed get speed");
    logger_car->info("nums: {}", nums);
    for (int r = 0; r < nums; r++)
    {
        std::vector<cv::Point2f> srcPt;   //雷达车辆坐标
        std::vector<cv::Point2f> dstPt;   //转换后的车辆坐标
        srcPt.push_back(cv::Point2f(radar_data_one.radar_output.trj[r].posX, radar_data_one.radar_output.trj[r].posY));

        logger_car->info("srcPt.size(): {}", srcPt.size());
        logger_car->info("dstPt.size()111111: {}", dstPt.size());
        if (srcPt.size() > 0)
        {
            logger_car->info("srcPt[0].x: {}", srcPt[0].x);
            logger_car->info("srcPt[0].y: {}", srcPt[0].y);
        }
        worldtoPixel(srcPt, dstPt);   // dstPt为转换后的像素坐标系的车辆中心点

        //判断中心点是否在车辆Box内，获取box的四个坐标点，按照顺指针或逆时针存入contour_box_car作为轮廓点
        std::vector<cv::Point2f> contour_box_car;
        contour_box_car.push_back(cv::Point2f(bbox->det_tlwh[0], bbox->det_tlwh[1]));   //左上角坐标点
        contour_box_car.push_back(cv::Point2f(bbox->det_tlwh[0], bbox->det_tlwh[1] + bbox->det_tlwh[3]));
        contour_box_car.push_back(cv::Point2f(bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]));   //右下角坐标点
        contour_box_car.push_back(cv::Point2f(bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1]));

        double is_inside_box;

        logger_car->info("dstPt.size(): {}", dstPt.size());
        if (dstPt.size() > 0)
        {
            is_inside_box = pointPolygonTest(contour_box_car, dstPt[0], true);
            line(frame_to_draw, dstPt[0], dstPt[0], Scalar(0, 0, 255), 8);
            logger_car->info("dstPt[0].x: {}", dstPt[0].x);
            logger_car->info("dstPt[0].y: {}", dstPt[0].y);
        }
        logger_car->info("is_inside_box: {}", is_inside_box);
        if (is_inside_box > 0)
        {
            //雷达车辆的中心点在图像车辆Box内,匹配成功，获取车速
            one_detect.speed =
                sqrt(radar_data_one.radar_output.trj[r].velX * radar_data_one.radar_output.trj[r].velX + radar_data_one.radar_output.trj[r].velY * radar_data_one.radar_output.trj[r].velY);
            one_detect.lane_id = radar_data_one.radar_output.trj[r].laneNum;
            logger_car->info("one_detect.speed: {}", one_detect.speed);
            logger_car->info("one_detect.lane_id: {}", one_detect.lane_id);
            printf(" one_detect.lane_id1111: %d\n", one_detect.lane_id);

            // line(frame_to_draw, dstPt[0], dstPt[0], Scalar(0, 0, 255), 3);
            break;
        }
        srcPt.clear();
        dstPt.clear();
    }
    return PV_ERROR_OK;
}
PV_ERROR_CODE cardetector::postprocess_nokeepdistance(cv::Mat frame_to_draw, cv::Mat img_origin, int time_interval, int detect_lane, vector<vector<cv::Point>> lines, vector<vector<cv::Point>> contour,
                                                      std::vector<YoloV8Box> boundingbox, STracks output_stracks, vector<json>& results_json, illegal_data& one_illegal_data)
{
    // 不按规定保持车距
    int mintime_id = -1;
    // 0m车距线
    Point A;
    Point B;
    // 50m车距线
    Point C;
    Point D;
    // 100m车距线
    Point E;
    Point F;
    if (lines.size() >= 3)
    {
        A = lines[0][0];
        B = lines[0][1];

        C = lines[1][0];
        D = lines[1][1];

        E = lines[2][0];
        F = lines[2][1];
    }
    else
    {
        return PV_ERROR_EXCEPTION;
    }
    // 50m区域
    vector<cv::Point> contour_zero_fifty;
    contour_zero_fifty.push_back(A);
    contour_zero_fifty.push_back(B);
    contour_zero_fifty.push_back(D);
    contour_zero_fifty.push_back(C);

    // 100m区域
    vector<cv::Point> contour_zero_100;
    contour_zero_100.push_back(A);
    contour_zero_100.push_back(B);
    contour_zero_100.push_back(D);
    contour_zero_100.push_back(F);
    contour_zero_100.push_back(E);
    contour_zero_100.push_back(C);

    FloatRet_ line_zero = {0};
    line_zero.x1        = A.x;
    line_zero.y1        = A.y;
    line_zero.x2        = B.x;
    line_zero.y2        = B.y;

    FloatRet_ line_zero_1 = {0};
    line_zero_1.x1        = A.x;
    line_zero_1.y1        = A.y;
    line_zero_1.x2        = C.x;
    line_zero_1.y2        = C.y;

    FloatRet_ line_50 = {0};
    line_50.x1        = C.x;
    line_50.y1        = C.y;
    line_50.x2        = D.x;
    line_50.y2        = D.y;

    FloatRet_ line_50_1 = {0};
    line_50_1.x1        = B.x;
    line_50_1.y1        = B.y;
    line_50_1.x2        = D.x;
    line_50_1.y2        = D.y;

    FloatRet_ line_100 = {0};
    line_100.x1        = E.x;
    line_100.y1        = E.y;
    line_100.x2        = F.x;
    line_100.y2        = F.y;

    FloatRet_ line_100_1 = {0};
    line_100_1.x1        = B.x;
    line_100_1.y1        = B.y;
    line_100_1.x2        = F.x;
    line_100_1.y2        = F.y;

    FloatRet_ line_100_2 = {0};
    line_100_2.x1        = A.x;
    line_100_2.y1        = A.y;
    line_100_2.x2        = E.x;
    line_100_2.y2        = E.y;

    line(frame_to_draw, A, B, Scalar(255, 0, 0), 3);
    line(frame_to_draw, D, C, Scalar(255, 0, 0), 3);
    line(frame_to_draw, E, F, Scalar(255, 0, 0), 3);

    if (frame_to_draw.empty() || img_origin.empty())
    {
        return PV_ERROR_EXCEPTION;
    }

    logger_car->info("output_stracks.size(): {}", output_stracks.size());

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////第一次循环////////////////////////////////////////////////////////////////////////////////
    //(1)第一次循环所有跟踪的车辆，检测所有压到0m线的车辆并push到detection_data
    for (auto& bbox : output_stracks)
    {
        cv::rectangle(frame_to_draw, cv::Point((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1]), cv::Point((int)bbox->det_tlwh[0] + bbox->det_tlwh[2], (int)bbox->det_tlwh[1] + bbox->det_tlwh[3]),
                      cv::Scalar(0, 255, 0), 2);
        putText(frame_to_draw, "id: " + to_string(bbox->track_id), cv::Point((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1]), 1, 4, Scalar(0, 0, 255), 4, LINE_8);
        // putText(frame_to_draw, "score: " + to_string(bbox->score), cv::Point((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1] + 50), 1, 4, Scalar(0, 0, 255), 4, LINE_8);

        FloatRet_ car_line_left   = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[0], bbox->det_tlwh[1] + bbox->det_tlwh[3]};
        FloatRet_ car_line_top    = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1]};
        FloatRet_ car_line_right  = {bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]};
        FloatRet_ car_line_bottom = {bbox->det_tlwh[0], bbox->det_tlwh[1] + bbox->det_tlwh[3], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]};

        //(a)判断车辆是否压到0m线，车辆box与0m线是否相交，车辆压到0m线开始判定是否存在违法，如果车辆已经越过0m线，仍然没有违法，则该车辆不再继续判断，违法图要求压到0m线和越过0m线两张图。
        //计算车的四条边界线和0m车距线是否相交
        int is_cross_left   = LineLineCollisionDetection(line_zero, car_line_left);
        int is_cross_top    = LineLineCollisionDetection(line_zero, car_line_top);
        int is_cross_right  = LineLineCollisionDetection(line_zero, car_line_right);
        int is_cross_bottom = LineLineCollisionDetection(line_zero, car_line_bottom);

        //计算车辆的车道号
        //        int       lane_id_currnet        = 0;
        //        cv::Point top_center_point       = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1]);
        //        double    is_inside_top_center   = -1;
        //        double    is_inside_top_center_2 = -1;

        //        is_inside_top_center = pointPolygonTest(contour[0], top_center_point, true);
        //        // is_inside_top_center_2 = pointPolygonTest(contour[1], top_center_point, true);
        //        if (is_inside_top_center > 0)
        //        {
        //            lane_id_currnet = 1;
        //        }
        //        if (is_inside_top_center_2 > 0)
        //        {
        //            lane_id_currnet = 2;
        //        }

        //(b)对第一次压到0m线的车辆进行记录，并记录第一次抓拍时间
        if (is_cross_left == 1 || is_cross_top == 1 || is_cross_right == 1 || is_cross_bottom == 1)
        {
            //(b1.1)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_detect = 0;
            for (int j = 0; j < one_illegal_data.detection_data.size(); j++)
            {
                if (one_illegal_data.detection_data[j].track_id == bbox->track_id)
                {
                    //对于已经出现违法，但是未形成完整违法数据的车辆，记录其从开始违法到最终违法的违法总次数，总次数大于给定阈值，则判定最终违法
                    one_illegal_data.detection_data[j].is_illegal_num = one_illegal_data.detection_data[j].is_illegal_num + 1;
                    one_illegal_data.detection_data[j].end_nframe     = one_illegal_data.nframe;
                    if ((one_illegal_data.detection_data[j].end_nframe - one_illegal_data.detection_data[j].start_nframe + 1) <= 5)
                    {
                        if (one_illegal_data.detection_data[j].is_illegal_num < (one_illegal_data.detection_data[j].end_nframe - one_illegal_data.detection_data[j].start_nframe + 1))
                        {
                            //对第一次违法做统计分析，如果连续5帧违法则继续跟踪记录，不需要更新第一次抓拍记录，如果连续5帧出现不违法，则更新第一次抓拍记录
                            if (one_illegal_data.detection_data[j].vehicle_images.size() > 0)
                            {
                                one_illegal_data.detection_data[j].vehicle_images.clear();
                                one_illegal_data.detection_data[j].vehicle_location_box.clear();
                                one_illegal_data.detection_data[j].illegal_time.clear();
                                one_illegal_data.detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                                one_illegal_data.detection_data[j].vehicle_images.push_back(img_origin);
                                one_illegal_data.detection_data[j].vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));
                                double begin                                      = (double)cv::getTickCount();
                                double start_time                                 = begin * 1000 / cv::getTickFrequency();
                                one_illegal_data.detection_data[j].start_time     = start_time;
                                one_illegal_data.detection_data[j].is_illegal_num = 1;
                                one_illegal_data.detection_data[j].start_nframe   = one_illegal_data.nframe;
                            }
                        }
                    }
                    flag_temp_detect = 1;
                }
            }

            //(b1.2)判定数据是否已经被记录，如果已经记录，则未到达指定时间间隔时，不再重复记录
            int flag_temp_illegal = 0;
            for (int j = 0; j < one_illegal_data.illegal_trackid.size(); j++)
            {
                if (one_illegal_data.illegal_trackid[j] == bbox->track_id)
                {
                    flag_temp_illegal = 1;
                }
            }
            //(b2)超过指定时间间隔，如果检测数据违法，会被存入输出数据中并在检测数据中清除，如果检测数据未违法，则会被清除；
            //检测数据已经被清除，如果输出数据中已经存在该track_id, 则不再重复记录
            int flag_temp_output = 0;
            for (int j = 0; j < one_illegal_data.output_data.size(); j++)
            {
                if (one_illegal_data.output_data[j].track_id == bbox->track_id)
                {
                    flag_temp_output = 1;
                }
            }
            //(b3)车辆第一次进入违法区域，记录进入时间点,并获取对应的车牌号图像
            if (flag_temp_detect == 0 && flag_temp_output == 0 && flag_temp_illegal == 0)
            {
                car_detect_data one_detect;

                //第一次抓拍后车辆数据记录
                double begin          = (double)cv::getTickCount();
                double start_time     = begin * 1000 / cv::getTickFrequency();
                one_detect.start_time = start_time;   //车辆违法第一次抓拍时间

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //雷达返回车辆的中心点位置，匹配到图像中压到0m线的车辆，获取车速，车速大于50km/h,再存入detection_data
                //(1)读取转换矩阵，将雷达车辆的世界坐标系坐标转换为像素坐标系坐标,获取雷达数据的每个车辆的中心点坐标
                //(2)获取压到0m线的车辆的box对应的中心点，该中心点应该在车辆box内，从而获取该车辆速度，车辆速度大于50km/h则存入detection_data，否则舍弃

                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // check time first
                one_detect.illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                one_detect.vehicle_images.push_back(img_origin);                                                                                                   //第一张违法图像
                one_detect.vehicle_location_box.push_back(Rect((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1], (int)bbox->det_tlwh[2], (int)bbox->det_tlwh[3]));   //车辆位置
                one_detect.track_id          = bbox->track_id;                                                                                                     //车辆跟踪ID
                one_detect.class_id          = bbox->class_id;                                                                                                     //车辆类型
                one_detect.is_illegal_num    = one_detect.is_illegal_num + 1;
                one_detect.start_nframe      = one_illegal_data.nframe;
                one_detect.is_cross_zeroline = 1;
                logger_car->info("first illegal image");
                //根据当前帧图像时间和雷达数据时间，获取当前帧图像对应的雷达数据帧
                size_t a        = std::stoi(one_illegal_data.algo_para.img_time.substr(12, 2));
                size_t b        = std::stoi(one_illegal_data.algo_para.img_time.substr(14, 3));
                size_t min_time = 10000;

                for (int ra = 0; ra < radar_data.size(); ra++)
                {
                    if (a == radar_data[ra].second_time)   //秒相同，比较毫秒
                    {
                        if (abs(radar_data[ra].milli_second_time - b) < min_time)
                        {
                            min_time   = abs(radar_data[ra].milli_second_time - b);
                            mintime_id = ra;
                        }
                    }
                    else
                    {
                        if (a < radar_data[ra].second_time)
                        {
                            if (abs((radar_data[ra].second_time - a) * 1000 - b + radar_data[ra].milli_second_time) < min_time)
                            {
                                min_time   = abs((radar_data[ra].second_time - a) * 1000 - b + radar_data[ra].milli_second_time);
                                mintime_id = ra;
                            }
                        }
                        if (a > radar_data[ra].second_time)
                        {
                            if (abs((a - radar_data[ra].second_time) * 1000 - radar_data[ra].milli_second_time + b) < min_time)
                            {
                                min_time   = abs((a - radar_data[ra].second_time) * 1000 - radar_data[ra].milli_second_time + b);
                                mintime_id = ra;
                            }
                        }
                    }
                }
                logger_car->info("min_time:{}", min_time);
                logger_car->info("mintime_id:{}", mintime_id);
                //遍历雷达数据中的所有车辆，将雷达车辆和图像车辆进行匹配，获取车速和车道号
                one_detect.speed = -9999;   //车速初始值
                if (mintime_id > 0 && mintime_id < radar_data.size())
                {
                    get_speed(one_detect, bbox, frame_to_draw, radar_data[mintime_id]);
                }

                // one_detect.lane_id = lane_id_currnet;

                logger_car->info("one_detect.speed:{}", one_detect.speed);
                // imwrite("/data/lj/未按规定保持车距行驶_" + to_string(one_illegal_data.nframe) + "output0.jpg", img_origin);
                if (one_detect.speed > 50)
                {
                    one_illegal_data.detection_data.push_back(one_detect);
                    // imwrite("/data2/lj/" + to_string(one_illegal_data.nframe) + "_" + to_string(one_illegal_data.detection_data.size() - 1) + "output.jpg", img_origin);
                }
            }
        }
        //每次循环需获取违法车辆的最新位置，根据最新位置确定前车，违法车辆的的前车必须在0-50m车道内，且与违法车辆在同一车道内
        for (int j = 0; j < one_illegal_data.detection_data.size(); j++)
        {
            if (bbox->track_id == one_illegal_data.detection_data[j].track_id)
            {
                one_illegal_data.detection_data[j].is_illegal           = 0;
                one_illegal_data.detection_data[j].illegal_car_is_exist = 1;   //在最后存储违法图像时没有判断违法车辆是否存在，这里加一下判断
                one_illegal_data.detection_data[j].illegal_car_y        = bbox->det_tlwh[1] + bbox->det_tlwh[3] / 2;   //违法车辆的最新位置，中心坐标点的y值
            }
        }
    }
    logger_car->info(" one_illegal_data.detection_data.size()-postpostpost:{}", one_illegal_data.detection_data.size());
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////第二次循环////////////////////////////////////////////////////////////////////////////////
    //(2)第二次循环所有跟踪车辆，找出所有压到0m线车辆对应的前车，判定其是否违法
    for (auto& bbox : output_stracks)
    {
        Rect_<float> bb_truck = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[2], bbox->det_tlwh[3]};

        FloatRet_ car_line_left   = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[0], bbox->det_tlwh[1] + bbox->det_tlwh[3]};
        FloatRet_ car_line_top    = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1]};
        FloatRet_ car_line_right  = {bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]};
        FloatRet_ car_line_bottom = {bbox->det_tlwh[0], bbox->det_tlwh[1] + bbox->det_tlwh[3], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]};

        //(a)判断车辆是否压到0m线，车辆box与0m线是否相交，车辆压到0m线开始判定是否存在违法，如果车辆已经越过0m线，仍然没有违法，则该车辆不再继续判断，违法图要求压到0m线和越过0m线两张图。
        //计算车的四条边界线和0m车距线是否相交
        //        int is_cross_left   = LineLineCollisionDetection(line_zero, car_line_left);
        //        int is_cross_top    = LineLineCollisionDetection(line_zero, car_line_top);
        //        int is_cross_right  = LineLineCollisionDetection(line_zero, car_line_right);
        //        int is_cross_bottom = LineLineCollisionDetection(line_zero, car_line_bottom);

        //计算车的四条边界线和50m车距线是否相交
        int is_cross_left_50   = LineLineCollisionDetection(line_50, car_line_left);
        int is_cross_top_50    = LineLineCollisionDetection(line_50, car_line_top);
        int is_cross_right_50  = LineLineCollisionDetection(line_50, car_line_right);
        int is_cross_bottom_50 = LineLineCollisionDetection(line_50, car_line_bottom);

        FloatRet_ car_box = {bbox->det_tlwh[0], bbox->det_tlwh[1], bbox->det_tlwh[0] + bbox->det_tlwh[2], bbox->det_tlwh[1] + bbox->det_tlwh[3]};

        // int is_cross_box=AreaAreaCollisionDetection( box1,  box2); //矩形碰撞检测
        int is_cross_box_1 = AreaLineCollisionDetection(car_box, line_zero);     //线段矩形相交测试AB
        int is_cross_box_2 = AreaLineCollisionDetection(car_box, line_50);       //线段矩形相交测试CD
        int is_cross_box_3 = AreaLineCollisionDetection(car_box, line_zero_1);   //线段矩形相交测试AC
        int is_cross_box_4 = AreaLineCollisionDetection(car_box, line_50_1);     //线段矩形相交测试BD

        int is_cross_box_5 = AreaLineCollisionDetection(car_box, line_100);     //线段矩形相交测试EF
        int is_cross_box_6 = AreaLineCollisionDetection(car_box, line_100_2);   //线段矩形相交测试AE
        int is_cross_box_7 = AreaLineCollisionDetection(car_box, line_100_1);   //线段矩形相交测试BF

        //计算车辆的车道号

        car_detect_data one_detect_temp;
        // one_detect_temp.speed = 60;
        // one_detect_temp.lane_id = 1;
        logger_car->info("second recurrent image");
        logger_car->info("mintime_id222222:{}", mintime_id);
        if (mintime_id > 0 && mintime_id < radar_data.size())
        {
            get_speed(one_detect_temp, bbox, frame_to_draw, radar_data[mintime_id]);
        }

        //        int       lane_id_currnet        = -1;
        //        cv::Point top_center_point       = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1]);
        //        double    is_inside_top_center   = -1;
        //        double    is_inside_top_center_2 = -1;

        //        is_inside_top_center = pointPolygonTest(contour[0], top_center_point, true);
        //        // is_inside_top_center_2 = pointPolygonTest(contour[1], top_center_point, true);
        //        if (is_inside_top_center > 0)
        //        {
        //            lane_id_currnet         = 1;
        //            one_detect_temp.lane_id = lane_id_currnet;
        //        }
        //        if (is_inside_top_center_2 > 0)
        //        {
        //            lane_id_currnet = 2;
        //        }
        logger_car->info("one_detect_temp.lane_id: {}", one_detect_temp.lane_id);
        logger_car->info("bbox->track_id: {}", bbox->track_id);
        //计算车辆是否在50m或100m区域内
        cv::Point center_point        = Point(bbox->det_tlwh[0] + bbox->det_tlwh[2] / 2, bbox->det_tlwh[1] + bbox->det_tlwh[3] / 2);
        double    is_inside_center_50 = -1;
        try
        {
            is_inside_center_50 = pointPolygonTest(contour_zero_fifty, center_point, true);
        }
        catch (...)
        {
            return PV_ERROR_EXCEPTION;
        }
        //////////////////////////////////////////////////////////////////////////////////////////
        double is_inside_center_100 = -1;
        try
        {
            is_inside_center_100 = pointPolygonTest(contour_zero_100, center_point, true);
        }
        catch (...)
        {
            return PV_ERROR_EXCEPTION;
        }

        logger_car->info("is_inside_center_50: {}", is_inside_center_50);
        logger_car->info("is_inside_center_100: {}", is_inside_center_100);

        logger_car->info("is_cross_box_1: {}", is_cross_box_1);
        logger_car->info("is_cross_box_2: {}", is_cross_box_2);
        logger_car->info("is_cross_box_3: {}", is_cross_box_3);
        logger_car->info("is_cross_box_4: {}", is_cross_box_4);

        // int is_illegal = 0;   //判定当前车辆是否违法标志，违法为1，不违法为0
        //(b)压到0m线进行第一次抓拍

        logger_car->info("one_illegal_data.detection_data.size(): {}", one_illegal_data.detection_data.size());

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //(e)判定车辆是否越过0m线，如果越过0m线判定是否违法：0-50m区域内同一车道内是否存在车辆
        for (int j = 0; j < one_illegal_data.detection_data.size(); j++)
        {
            logger_car->info("one_illegal_data.detection_data[j].track_id: {}", one_illegal_data.detection_data[j].track_id);
            //确定车辆违法后，如果当前为卡口图，记录卡口图在detection_data中的下标位置idx
            // if (one_illegal_data.algo_para.plate_no != "" && one_illegal_data.algo_para.vehicle_coordinate != "")
            //此处的卡口图是针对某一辆车，但是在设置is_overtime=1时把所有违法车辆的is_overtime都设为1了，导致违法四合一中的卡口图可能不是对应车辆的卡口图
            if (one_illegal_data.algo_para.bayonet_img_path != "" && one_illegal_data.algo_para.plate_no != "")
            {
                //当出现卡口图像，计算触发卡口图像的车辆位置和违法车辆位置关系
                //计算违法车辆和卡口车辆的iou
                double ratio_kakou = GetIOU_rect(one_illegal_data.algo_para.kakou_car_box, bb_truck);
                logger_car->info("ratio_kakou: {}", ratio_kakou);
                logger_car->info("bbox->track_id: {}", bbox->track_id);
                // if (one_illegal_data.detection_data[j].is_overtime == 0 && ratio_kakou > 0.8)
                if (bbox->track_id == one_illegal_data.detection_data[j].track_id)
                {
                    if (one_illegal_data.detection_data[j].is_overtime == 0 && ratio_kakou > 0.8)
                    {
                        one_illegal_data.detection_data[j].is_overtime = 1;   //违法车辆的卡口图
                    }
                }
            }

            if (bbox->track_id == one_illegal_data.detection_data[j].track_id)   //更新违法车辆的车道号，避免违法车辆变道
            {
                if (one_detect_temp.lane_id != one_illegal_data.detection_data[j].lane_id)   //后车车道发生变化，则舍弃
                {
                    one_illegal_data.detection_data[j].vehicle_images.clear();   //为了防止车辆变道，违法车辆的车道号需要不断更新
                }
            }
            //此处需要判定detection_data内的车辆是否压0m线
            int is_cross_left_illegal_car, is_cross_top_illegal_car, is_cross_right_illegal_car, is_cross_bottom_illegal_car = -1;
            if (bbox->track_id == one_illegal_data.detection_data[j].track_id)
            {
                //判定违法车辆是否压0m线
                is_cross_left_illegal_car   = LineLineCollisionDetection(line_zero, car_line_left);
                is_cross_top_illegal_car    = LineLineCollisionDetection(line_zero, car_line_top);
                is_cross_right_illegal_car  = LineLineCollisionDetection(line_zero, car_line_right);
                is_cross_bottom_illegal_car = LineLineCollisionDetection(line_zero, car_line_bottom);
            }

            logger_car->info("is_cross_left_illegal_car: {}", is_cross_left_illegal_car);
            logger_car->info("is_cross_top_illegal_car: {}", is_cross_top_illegal_car);
            logger_car->info("is_cross_right_illegal_car: {}", is_cross_right_illegal_car);
            logger_car->info("is_cross_bottom_illegal_car: {}", is_cross_bottom_illegal_car);

            logger_car->info("one_illegal_data.detection_data[j].is_cross_zeroline: {}", one_illegal_data.detection_data[j].is_cross_zeroline);
            logger_car->info("one_illegal_data.detection_data[j].is_pass_zeroline: {}", one_illegal_data.detection_data[j].is_pass_zeroline);
            //判定违法车辆是否越过0m线
            if (one_illegal_data.detection_data[j].is_cross_zeroline == 1 && is_cross_left_illegal_car == 0 && is_cross_top_illegal_car == 0 && is_cross_right_illegal_car == 0 &&
                is_cross_bottom_illegal_car == 0 && one_illegal_data.detection_data[j].is_pass_zeroline == 0)   // 0m线与车辆box任一边界相交
            {
                one_illegal_data.detection_data[j].is_pass_zeroline = 1;   //越过0m线
            }
            //判定0-50m区域内同一车道内是否存在车辆
            //若车辆到达0m位置，车速大于50km/h，先判断50m区域内是否存在其他车辆，
            //如果存在，判断车辆所在车道号与0m位置当前车辆是否属于同一车道，如果属于，则在50m区域内未按规定保持车距
            //车辆box与50m区域的任一边界相交或车辆中心点在50m区域的轮廓内,则50m区域内存在车辆，再判定车辆的车道号和track_id，同一车道内，不同的track_id
            //车速大于50km/h小于100km/h，判定前车是否在50m区域内

            if (one_illegal_data.detection_data[j].speed > 50 && one_illegal_data.detection_data[j].speed <= 100)
            {
                if (is_cross_box_1 == 1 || is_cross_box_2 == 1 || is_cross_box_3 == 1 || is_cross_box_4 == 1 || is_inside_center_50 > 0)
                {
                    //同一车道内50m区域里存在其他车辆,该车辆必须在违法车辆前面，且在同一个车辆，违法车辆不超过50m车距线
                    if (bbox->track_id != one_illegal_data.detection_data[j].track_id && one_detect_temp.lane_id == one_illegal_data.detection_data[j].lane_id &&
                        one_illegal_data.detection_data[j].illegal_car_y > (bbox->det_tlwh[1] + bbox->det_tlwh[3] / 2) && one_illegal_data.detection_data[j].illegal_car_y > C.y)
                    {

                        //记录前车数据信息，同一车道前面可能有多辆车，记录所有满足违法条件的前车，任一前车可以和后车形成完整的违法证据，即为违法
                        int flag_pre_car = 0;
                        for (int j1 = 0; j1 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j1++)
                        {

                            if (bbox->track_id == one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_track_id)
                            {
                                //已经存在的前车
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal = 1;
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_speed.push_back(one_detect_temp.speed);
                                flag_pre_car = 1;
                            }
                        }
                        //不存在的前车，添加到illegal_pre_car_detect_data
                        //对于第一次压到0m线的车辆， 上面的前车数据代码不会执行，one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal =0;
                        //第二次循环时one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal =1;
                        if (flag_pre_car == 0)
                        {
                            pre_car_detect_data one_pre_car;
                            one_pre_car.pre_track_id = bbox->track_id;
                            //获取前车速度
                            one_pre_car.pre_car_speed.push_back(one_detect_temp.speed);

                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data.push_back(one_pre_car);
                        }

                        // pre_track_id一旦确定就不再改变，必须保证该pre_track_id可以形成完整违法，才算违法；
                        if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size() > 0)
                        {
                            //每次需要置0并更新，保证违法，lane_id和illegal_car_y每次都更新
                            one_illegal_data.detection_data[j].is_illegal = 1;   //需要保证4张图都违法，如果可以抓到4张图，且第4张图像违法，那么前三张图一定违法，所以需要保证第4张图违法
                        }
                    }
                }
            }
            else
            {
                if (one_illegal_data.detection_data[j].speed > 100)
                {
                    //车速大于100km/h，判定前车是否在100m区域内
                    if (is_cross_box_1 == 1 || is_cross_box_5 == 1 || is_cross_box_6 == 1 || is_cross_box_7 == 1 || is_inside_center_100 > 0)
                    {
                        //同一车道内100m区域里存在其他车辆,该车辆必须在违法车辆前面，且在同一个车辆，违法车辆不超过100m车距线
                        if (bbox->track_id != one_illegal_data.detection_data[j].track_id && one_detect_temp.lane_id == one_illegal_data.detection_data[j].lane_id &&
                            one_illegal_data.detection_data[j].illegal_car_y > (bbox->det_tlwh[1] + bbox->det_tlwh[3] / 2) && one_illegal_data.detection_data[j].illegal_car_y > E.y)
                        {

                            //记录前车数据信息，同一车道前面可能有多辆车，记录所有满足违法条件的前车，任一前车可以和后车形成完整的违法证据，即为违法
                            int flag_pre_car = 0;
                            for (int j1 = 0; j1 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j1++)
                            {
                                if (bbox->track_id == one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_track_id)
                                {
                                    //已经存在的前车，对于第一次压到0m线的车辆，此次循环已经存储了1张违法图像，不再存储第2张
                                    one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal =
                                        1;   //该参数可避免第一次违法后存2次图像，第2张违法图像需要下一次循环判断后再存储
                                    one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_speed.push_back(one_detect_temp.speed);
                                    flag_pre_car = 1;
                                }
                            }
                            //不存在的前车，添加到illegal_pre_car_detect_data
                            if (flag_pre_car == 0)
                            {
                                pre_car_detect_data one_pre_car;
                                one_pre_car.pre_track_id = bbox->track_id;
                                //获取前车速度
                                one_pre_car.pre_car_speed.push_back(one_detect_temp.speed);
                                //对于第一次压到0m线的车辆，此次循环已经存储了1张违法图像，one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal =
                                // 0，此次循环不再存储第2张
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data.push_back(one_pre_car);
                            }

                            // pre_track_id一旦确定就不再改变，必须保证该pre_track_id可以形成完整违法，才算违法；
                            if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size() > 0 && flag_pre_car == 1)
                            {
                                //每次需要置0并更新，保证违法，lane_id和illegal_car_y每次都更新
                                one_illegal_data.detection_data[j].is_illegal = 1;   //需要保证4张图都违法，如果可以抓到4张图，且第4张图像违法，那么前三张图一定违法，所以需要保证第4张图违法
                            }
                        }
                    }
                }
            }

            logger_car->info("is_illegal: {}", one_illegal_data.detection_data[j].is_illegal);
            //使用第二张图像进行车牌识别
            if (one_illegal_data.detection_data[j].vehicle_images.size() == 2 && bbox->track_id == one_illegal_data.detection_data[j].track_id &&
                one_illegal_data.detection_data[j].plate_roi_flag == 0)
            {
                logger_car->info("illegal plate image");
                int yy = 0;   //如果同一辆车检测到多个车牌，使用yy值大的车牌
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                //(4)使用第四张图像获取违法车辆对应的车牌,识别车牌号，确定车型，输出到output_data
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
                                    one_illegal_data.detection_data[j].vehicle_plate_images.push_back(one_plate);
                                    one_illegal_data.detection_data[j].plate_roi_flag = 1;
                                    // imwrite("/data/lj/未按规定保持车距行驶_" + to_string(one_illegal_data.nframe) + "车牌.jpg", plate_img);
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
                                    one_illegal_data.detection_data[j].vehicle_plate_images.push_back(one_plate);
                                    one_illegal_data.detection_data[j].plate_roi_flag = 1;
                                    // imwrite("/data/lj/未按规定保持车距行驶_" + to_string(one_illegal_data.nframe) + "车牌.jpg", plate_img);
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
            //已经获取4张违法图像，进行车牌识别并输出到output_data
            logger_car->info("one_illegal_data.detection_data[j].vehicle_images.size(): {}", one_illegal_data.detection_data[j].vehicle_images.size());
            logger_car->info("bbox->track_id : {}", bbox->track_id);
            logger_car->info("one_illegal_data.detection_data[j].track_id: {}", one_illegal_data.detection_data[j].track_id);
            logger_car->info("one_illegal_data.detection_data[j].plate_recognition_flag: {}", one_illegal_data.detection_data[j].plate_recognition_flag);
            if (one_illegal_data.detection_data[j].vehicle_images.size() >= 4 && bbox->track_id == one_illegal_data.detection_data[j].track_id &&
                one_illegal_data.detection_data[j].plate_recognition_flag == 0)
            {
                logger_car->info("illegal_pre_car_detect_data.size(): {}", one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size());
                //确定车型，识别车牌号
                if (one_illegal_data.detection_data[j].is_cross_zeroline == 1 && one_illegal_data.detection_data[j].is_pass_zeroline == 2)
                {
                    // if (is_cross_left_illegal_car == 0 && is_cross_top_illegal_car == 0 && is_cross_right_illegal_car == 0 && is_cross_bottom_illegal_car == 0)
                    {
                        int flag_illegal_trackid = 0;
                        for (int jj = 0; jj < one_illegal_data.illegal_trackid.size(); jj++)
                        {
                            if (one_illegal_data.detection_data[j].track_id == one_illegal_data.illegal_trackid[jj])
                            {
                                flag_illegal_trackid = 1;
                            }
                        }
                        //必须保证至少有一辆前车出现在4张违法图像中，和后车才能形成完整违法。
                        int flag_illegal_pre_car = 0;
                        for (int j2 = 0; j2 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j2++)
                        {
                            logger_car->info("one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num: {}",
                                             one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num);
                            if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num >= 4)
                            {
                                flag_illegal_pre_car = 1;
                            }
                        }
                        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        //判定是否形成完整违法，输出到output_data,必须有4张违法图像，有车牌图像，有同一个前车出现4次违法
                        logger_car->info("flag_illegal_trackid: {}", flag_illegal_trackid);
                        logger_car->info("one_illegal_data.detection_data[j].vehicle_plate_images.size() : {}", one_illegal_data.detection_data[j].vehicle_plate_images.size());
                        logger_car->info("flag_illegal_pre_car: {}", flag_illegal_pre_car);
                        logger_car->info("one_illegal_data.detection_data[j].vehicle_images.size(): {}", one_illegal_data.detection_data[j].vehicle_images.size());
                        if (flag_illegal_trackid == 0)
                        {
                            if (one_illegal_data.detection_data[j].vehicle_images.size() == 4 && one_illegal_data.detection_data[j].vehicle_plate_images.size() >= 0 && flag_illegal_pre_car == 1)
                            {
                                vehicle_output_data one_output;   //形成完整违法，获取完整输出数据
                                for (int idx = 0; idx < one_illegal_data.detection_data[j].vehicle_images.size(); idx++)
                                {
                                    one_output.vehicle_images.push_back(one_illegal_data.detection_data[j].vehicle_images[idx]);
                                    one_output.vehicle_location_box.push_back(one_illegal_data.detection_data[j].vehicle_location_box[idx]);
                                }
                                one_output.illegal_time = one_illegal_data.detection_data[j].illegal_time;
                                one_output.track_id     = bbox->track_id;
                                //获取车辆速度
                                one_output.speed = one_illegal_data.detection_data[j].speed;

                                //确定车型
                                if (one_illegal_data.detection_data[j].class_id == 1)
                                {
                                    one_output.vehicle_type = "载客汽车";
                                }
                                else
                                {
                                    if (one_illegal_data.detection_data[j].class_id == 2)
                                    {
                                        one_output.vehicle_type = "载货汽车";
                                    }
                                    else
                                    {
                                        one_output.vehicle_type = "小汽车";
                                    }
                                }
                                //根据检测数据中的车牌图像获取输出数据中的车牌图像
                                one_output.vehicle_plate_images = one_illegal_data.detection_data[j].vehicle_plate_images;
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
                                        one_illegal_data.detection_data[j].plate_recognition_flag = 1;
                                    }
                                    else
                                    {
                                        platerecognition(one_output.plate_img, results);
                                        one_illegal_data.detection_data[j].plate_recognition_flag = 1;
                                    }
                                    if (results.size() > 0)
                                    {
                                        logger_car->info("车牌号: {}", results[0]);

                                        // one_output.plate_number = results[0];
                                        //使用卡口图车牌号
                                        if (one_illegal_data.algo_para.plate_no != "")
                                        {
                                            one_output.plate_number = one_illegal_data.algo_para.plate_no;   //使用自动识别的车牌号
                                        }
                                        else
                                        {
                                            // one_output.plate_number = results[0];   //使用自动识别的车牌号
                                            one_output.plate_number = "";
                                        }
                                        logger_car->info("车牌号-卡口: {}", one_illegal_data.algo_para.plate_no);

                                        /////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        //根据卡口图获取车牌图像

                                        Rect_<float> bb_plate = one_illegal_data.algo_para.kakou_plate_box;

                                        logger_car->info("bb_plate.x : {}", bb_plate.x);
                                        logger_car->info("bb_plate.y : {}", bb_plate.y);
                                        logger_car->info("bb_plate.width : {}", bb_plate.width);
                                        logger_car->info("bb_plate.height : {}", bb_plate.height);
                                        if (bb_plate.width > 0 && bb_plate.height > 0)
                                        {
                                            if (bb_plate.width > 16 && bb_plate.height > 16)
                                            {
                                                try
                                                {
                                                    int     size_2 = one_illegal_data.detection_data[j].vehicle_images.size() - 1;
                                                    cv::Mat plate_img =
                                                        one_illegal_data.detection_data[j].vehicle_images[size_2](Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    int size_1 = one_illegal_data.detection_data[j].vehicle_plate_images.size() - 1;
                                                    one_illegal_data.detection_data[j].vehicle_plate_images[size_1].vehicle_plate_image = plate_img;
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
                                                    int     size_2 = one_illegal_data.detection_data[j].vehicle_images.size() - 1;
                                                    cv::Mat plate_img =
                                                        one_illegal_data.detection_data[j].vehicle_images[size_2](Rect(bb_plate.x, bb_plate.y, 11 * bb_plate.width / 10, 12 * bb_plate.height / 10));
                                                    Mat dst_plate;
                                                    resize(plate_img, dst_plate, Size(168, 48), 0, 0, INTER_LINEAR);
                                                    int size_1 = one_illegal_data.detection_data[j].vehicle_plate_images.size() - 1;
                                                    one_illegal_data.detection_data[j].vehicle_plate_images[size_1].vehicle_plate_image = dst_plate;
                                                    //替换车牌图像为卡口图像中的车牌图像
                                                    one_output.plate_img = dst_plate;
                                                }

                                                catch (...)
                                                {
                                                    return PV_ERROR_EXCEPTION;
                                                }
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
                                    logger_car->info("车型: {}", one_output.plate_color);
                                }
                                // if (one_output.vehicle_images.size() >= 4)
                                if (one_output.vehicle_images.size() == 4 && !one_output.plate_img.empty() && (one_output.plate_number.size() == 9 || one_output.plate_number.size() == 10))
                                {
                                    one_illegal_data.output_data.push_back(one_output);
                                    one_illegal_data.illegal_trackid.push_back(one_illegal_data.detection_data[j].track_id);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //(3)已经获取了detection_data的所有前车数据，现在开始存储违法图像
    for (int j = 0; j < one_illegal_data.detection_data.size(); j++)
    {
        cout << " one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(): " << one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size() << endl;
        logger_car->info("jjjjjjjjjjjjjjjjj:{}", j);

        logger_car->info("one_illegal_data.detection_data[j].flag_save：{}", one_illegal_data.detection_data[j].flag_save);
        logger_car->info("one_illegal_data.detection_data[j].is_overtime:{}", one_illegal_data.detection_data[j].is_overtime);
        logger_car->info("one_illegal_data.detection_data[j].is_pass_zeroline:{}", one_illegal_data.detection_data[j].is_pass_zeroline);
        logger_car->info("one_illegal_data.detection_data[j].is_illegal:{}", one_illegal_data.detection_data[j].is_illegal);
        logger_car->info("one_illegal_data.detection_data[j].illegal_car_is_exist:{}", one_illegal_data.detection_data[j].illegal_car_is_exist);
        //车辆压到0m线的时候没有对其前车进行分析，直接存储到detection_data，此处对第一张违法图像是否存在违法前车进行补充分析，保证4张违法图像都包含违法前车。
        //如果已经存入到detection_data中的车辆，前车信息为0,证明第一张违法图像中不存在违法前车，不形成违法，清除这个违法车辆图像
        if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size() == 0)
        {
            one_illegal_data.detection_data[j].vehicle_images.clear();   //存储的图像被清除，该车辆无法形成完整违法
        }
        logger_car->info("one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size():{}", one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size());
        for (int j1 = 0; j1 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j1++)   //此处循环只会进入一次，不会因为多辆前车而存储重复的违法图
        {
            if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j1].pre_car_is_illegal ==
                1)   //对于第一次压到0m线的车辆，此次循环已经存储了1张违法图像，不再存储第2张,第一次时pre_car_is_illegal=0
            {

                //违法车辆存在，且和前车形成违法:车辆未驶过Om线且未出现卡口图时一直抓拍，如果车辆驶过0m线且已经出现卡口图
                //(1)车辆未驶过Om线且未出现卡口图时抓拍的违法图片：作为第二张违法图
                if (one_illegal_data.detection_data[j].is_overtime == 0 && one_illegal_data.detection_data[j].is_pass_zeroline == 0 && one_illegal_data.detection_data[j].flag_save == 0 &&
                    one_illegal_data.detection_data[j].is_illegal == 1 && one_illegal_data.detection_data[j].illegal_car_is_exist == 1)
                {
                    logger_car->info("second illegal image");   //第一次判断违法时已经抓拍了一张违法图
                    one_illegal_data.detection_data[j].illegal_car_is_exist = 0;
                    one_illegal_data.detection_data[j].flag_save            = 1;
                    one_illegal_data.detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                    one_illegal_data.detection_data[j].vehicle_images.push_back(img_origin);
                    one_illegal_data.detection_data[j].is_illegal = 0;

                    //此处循环遍历所有前车，统计所有前车统计信息
                    for (int j2 = 0; j2 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j2++)
                    {
                        if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal == 1)
                        {
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal = 0;   //前车是否在违法区域，每次需要更新
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num =
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num + 1;   //同一辆前车违法次数要达到4次才算完整违法
                        }
                    }
                }
                //(2)违法车辆卡口图：作为第三张违法图
                if (one_illegal_data.detection_data[j].is_overtime == 1)
                {
                    if (one_illegal_data.detection_data[j].is_illegal == 1 && one_illegal_data.detection_data[j].illegal_car_is_exist == 1)
                    {
                        one_illegal_data.detection_data[j].is_overtime          = 2;
                        one_illegal_data.detection_data[j].illegal_car_is_exist = 0;
                        logger_car->info("third illegal image:kakou illegal image");
                        logger_car->info("one_illegal_data.detection_data[j].is_overtime1111111:{}", one_illegal_data.detection_data[j].is_overtime);

                        one_illegal_data.detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                        one_illegal_data.detection_data[j].vehicle_images.push_back(img_origin);
                        one_illegal_data.detection_data[j].is_illegal = 0;

                        for (int j2 = 0; j2 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j2++)
                        {
                            if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal == 1)
                            {
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal = 0;   //前车是否在违法区域，每次需要更新
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num =
                                    one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num + 1;   //同一辆前车违法次数要达到4次才算完整违法
                            }
                        }
                    }
                    else
                    {
                        //卡口图不满足存图条件，则清除该数据
                        one_illegal_data.detection_data[j].vehicle_images.clear();
                    }
                }

                //(3)违法车辆驶过Om线且已经出现卡口图后对应的图：作为第四张违法图
                if (one_illegal_data.detection_data[j].is_pass_zeroline == 1 && one_illegal_data.detection_data[j].is_overtime == 2 && one_illegal_data.detection_data[j].is_illegal == 1 &&
                    one_illegal_data.detection_data[j].illegal_car_is_exist == 1)
                {
                    one_illegal_data.detection_data[j].is_pass_zeroline     = 2;
                    one_illegal_data.detection_data[j].illegal_car_is_exist = 0;
                    logger_car->info("fourth illegal image:pass 0m line illegal image");
                    logger_car->info("one_illegal_data.detection_data[j].is_pass_zeroline11111:{}", one_illegal_data.detection_data[j].is_pass_zeroline);

                    one_illegal_data.detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                    one_illegal_data.detection_data[j].vehicle_images.push_back(img_origin);
                    one_illegal_data.detection_data[j].is_illegal = 0;

                    for (int j2 = 0; j2 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j2++)
                    {
                        if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal == 1)
                        {
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal = 0;   //前车是否在违法区域，每次需要更新
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num =
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num + 1;   //同一辆前车违法次数要达到4次才算完整违法
                        }
                    }
                }
                //(4)为保证一定可以抓拍到四张违法图，在上述存储图像未满足4张时这里再进行存储，保证违法图像数量不小于4
                if (one_illegal_data.detection_data[j].is_pass_zeroline == 2 && one_illegal_data.detection_data[j].is_overtime == 2 && one_illegal_data.detection_data[j].is_illegal == 1 &&
                    one_illegal_data.detection_data[j].illegal_car_is_exist == 1 && one_illegal_data.detection_data[j].vehicle_images.size() < 4)
                {
                    logger_car->info("other illegal image:pass 0m line illegal image");
                    one_illegal_data.detection_data[j].flag_save            = 1;
                    one_illegal_data.detection_data[j].illegal_car_is_exist = 0;
                    for (int jj = 0; jj < one_illegal_data.detection_data[j].vehicle_images.size(); jj++)
                    {
                        // imwrite("/data2/lj/" + to_string(one_illegal_data.nframe) + "_" + to_string(jj) + "output.jpg", one_illegal_data.detection_data[j].vehicle_images[jj]);
                    }
                    one_illegal_data.detection_data[j].illegal_time.push_back(getCurrentTimeInt64_save2(one_illegal_data.algo_para.img_time));
                    one_illegal_data.detection_data[j].vehicle_images.push_back(img_origin);
                    one_illegal_data.detection_data[j].is_illegal = 0;

                    for (int j2 = 0; j2 < one_illegal_data.detection_data[j].illegal_pre_car_detect_data.size(); j2++)
                    {
                        if (one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal == 1)
                        {
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal = 0;   //前车是否在违法区域，每次需要更新
                            one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num =
                                one_illegal_data.detection_data[j].illegal_pre_car_detect_data[j2].pre_car_is_illegal_num + 1;   //同一辆前车违法次数要达到4次才算完整违法
                        }
                    }
                }
            }
        }
        one_illegal_data.detection_data[j].is_illegal           = 0;
        one_illegal_data.detection_data[j].illegal_car_is_exist = 0;
        logger_car->info("one_illegal_data.detection_data[j].vehicle_images:{}", one_illegal_data.detection_data[j].vehicle_images.size());
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //(5)记录完整违法车辆id，若图像中该id消失，则清除该id
    auto removed_stracks = one_illegal_data.bytetrack.get_removed_stracks();

    for (auto& bbox : removed_stracks)
    {
        for (vector<int>::iterator it = one_illegal_data.illegal_trackid.begin(); it != one_illegal_data.illegal_trackid.end(); it++)
        {
            if (*it == bbox->track_id)
            {
                one_illegal_data.illegal_trackid.erase(it);
                it--;
            }
        }
        for (vector<car_detect_data>::iterator it_it = one_illegal_data.detection_data.begin(); it_it != one_illegal_data.detection_data.end(); it_it++)
        {
            logger_car->info("it_it->vehicle_images.size(): {}", it_it->vehicle_images.size());
            if (it_it->track_id == bbox->track_id || it_it->vehicle_images.size() == 0)
            {
                one_illegal_data.detection_data.erase(it_it);
                it_it--;
            }
        }
    }
    if (one_illegal_data.detection_data.size() == 1)
    {
        if (one_illegal_data.detection_data[0].vehicle_images.size() == 0)
        {
            one_illegal_data.detection_data.clear();
        }
    }
    logger_car->info("one_illegal_data.output_data.size(): {}", one_illegal_data.output_data.size());
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //(6)输出结果画框，四合一
    logger_car->info("start img_merge");
    img_merge(one_illegal_data.output_data, 2, contour[0], one_illegal_data);
    logger_car->info("end img_merge");

    for (auto& output : one_illegal_data.output_data)
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
        output_json["illegal_code"]     = one_illegal_data.algo_para.illegal_code_safeDistance;
        output_json["illegal_behavior"] = "机动车未按规定保持车距行驶";
        ////////////////////////////////////////////////////
        output_json["device_number"] = one_illegal_data.algo_para.device_number;
        output_json["camera_id"]     = one_illegal_data.algo_para.camera_id;

        results_json.push_back(output_json);
    }

    one_illegal_data.output_data.clear();
    return PV_ERROR_OK;
}

// flag_illegal=0最左侧车道
// flag_illegal=1应急车道
// flag_illegal=2未按规定保持车距
PV_ERROR_CODE cardetector::img_merge(vector<vehicle_output_data>& output_data, int flag_illegal, vector<cv::Point> contour, illegal_data& one_illegal_data)
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
                string plate_name = "车牌号：";
                plate_name        = plate_name + output_data[i].plate_number;

                string plate_color = "车牌颜色：";
                plate_color        = plate_color + output_data[i].plate_color;

                string vehicle_type = "车型：";
                vehicle_type        = vehicle_type + output_data[i].vehicle_type;

                for (int m = 0; m < output_data[i].vehicle_images.size(); m++)
                {
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
                        logger_car->error("img_merge copyMakeBorder error");
                    }

                    if (paddedImg.cols > 0 && paddedImg.rows > 0 && top > 0)
                    {
                        //第一行
                        string time_time = output_data[i].illegal_time[m];
                        time_time        = "违法时间: " + time_time;
                        ft2->putText(paddedImg, format("%s", time_time.c_str()), Point(gap_w, gap_h), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        string illegal_place = "违法地点: " + one_illegal_data.algo_para.camera_place;
                        ft2->putText(paddedImg, format("%s", illegal_place.c_str()), Point(gap_w + 600, gap_h), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        //第二行
                        ft2->putText(paddedImg, format("%s", plate_name.c_str()), Point(gap_w, 2 * gap_h + fontheight / 10), 30, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", plate_color.c_str()), Point(gap_w + 300, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", vehicle_type.c_str()), Point(gap_w + 600, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        if (flag_illegal == 0)
                        {
                            string illegal_code = "违法代码: " + one_illegal_data.algo_para.illegal_code_occupyLane;
                            ft2->putText(paddedImg, format("%s", illegal_code.c_str()), Point(gap_w + 900, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                            ft2->putText(paddedImg, format("%s", "违法行为: 机动车不按规定车道行驶"), Point(gap_w + 1200, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8,
                                         LINE_AA);
                        }
                        if (flag_illegal == 1)
                        {
                            string illegal_code = "违法代码: " + one_illegal_data.algo_para.illegal_code_occupyEmergencyLane;
                            ft2->putText(paddedImg, format("%s", illegal_code.c_str()), Point(gap_w + 900, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                            ft2->putText(paddedImg, format("%s", "违法行为: 机动车占用应急车道行驶"), Point(gap_w + 1200, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8,
                                         LINE_AA);
                        }
                        if (flag_illegal == 2)
                        {
                            string illegal_code = "违法代码: " + one_illegal_data.algo_para.illegal_code_safeDistance;
                            ft2->putText(paddedImg, format("%s", illegal_code.c_str()), Point(gap_w + 900, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                            ft2->putText(paddedImg, format("%s", "违法行为: 机动车未按规定保持车距行驶"), Point(gap_w + 1200, 2 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1,
                                         8, LINE_AA);
                        }

                        //第三行
                        string illegal_speed = "车速: " + to_string((int)one_illegal_data.output_data[i].speed) + "km/h";
                        ft2->putText(paddedImg, format("%s", illegal_speed.c_str()), Point(gap_w, 3 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);

                        int         length        = 18;
                        std::string random_string = generate_random_string(length);
                        random_string             = "防伪码: " + random_string;
                        string device_number      = "设备编号: " + one_illegal_data.algo_para.device_number;
                        ft2->putText(paddedImg, format("%s", device_number.c_str()), Point(gap_w + 600, 3 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
                        ft2->putText(paddedImg, format("%s", random_string.c_str()), Point(gap_w + 1200, 3 * gap_h + fontheight / 10), fontheight, cv::Scalar(255, 255, 255), -1, 8, LINE_AA);
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
                    logger_car->error("img_merge hconcat or vconcat error");
                }
            }

            // if (output_data[i].vehicle_images.size() == 4 && !output_data[i].plate_img.empty() && vconcatImg.cols > 0 && vconcatImg.rows > 0)
            if (output_data[i].vehicle_images.size() >= 4 && vconcatImg.cols > 0 && vconcatImg.rows > 0)
            {
                // 放大为图像宽高的1/10
                //                Mat dst;
                //                try
                //                {
                //                    resize(output_data[i].plate_img, dst, Size((int)output_data[i].vehicle_images[0].cols / 10, (int)output_data[i].vehicle_images[0].rows / 10), 0, 0, INTER_LINEAR);
                //                }
                //                catch (...)
                //                {
                //                    logger_car->error("img_merge resize error");
                //                    return PV_ERROR_EXCEPTION;
                //                }

                //                Mat imageROI, mask;
                //                try
                //                {
                //                    imageROI = vconcatImg(Rect(vconcatImg.cols - dst.cols - 1, vconcatImg.rows - dst.rows - 1, dst.cols, dst.rows));
                //                    cvtColor(dst, mask, COLOR_BGR2GRAY);   //参数0显示为灰度图
                //                    dst.copyTo(imageROI, mask);
                //                }
                //                catch (...)
                //                {
                //                    logger_car->error("img_merge vconcatImg error");
                //                    return PV_ERROR_EXCEPTION;
                //                }

                int64_t currentTimeInt64_save_name = getCurrentTimeInt64();
                // 设定压缩参数
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(10);   // 设置压缩质量，范围为0-100，越高质量越好，但文件越大

                if (flag_illegal == 0)
                {
                    output_data[i].illegal_image_name = "/data2/camera-handler/upFiles/不按规定车道行驶_" + output_data[i].plate_number + "_" + to_string(currentTimeInt64_save_name) + ".jpg";
                    imwrite("/data2/camera-handler/upFiles/不按规定车道行驶_" + output_data[i].plate_number + "_" + to_string(currentTimeInt64_save_name) + ".jpg", vconcatImg);
                }
                if (flag_illegal == 1)
                {
                    output_data[i].illegal_image_name = "/data2/camera-handler/upFiles/占用应急车道行驶_" + output_data[i].plate_number + "_" + to_string(currentTimeInt64_save_name) + ".jpg";
                    imwrite("/data2/camera-handler/upFiles/占用应急车道行驶_" + output_data[i].plate_number + "_" + to_string(currentTimeInt64_save_name) + ".jpg", vconcatImg);
                }
                if (flag_illegal == 2)
                {
                    output_data[i].illegal_image_name = "/data2/camera-handler/upFiles/CarNoKeepDistance_" + to_string(currentTimeInt64_save_name) + ".jpg";
                    try
                    {
                        imwrite("/data2/camera-handler/upFiles/CarNoKeepDistance_" + to_string(currentTimeInt64_save_name) + ".jpg", vconcatImg, compression_params);
                    }
                    catch (const cv::Exception& ex)
                    {
                        std::cout << "图像压缩失败: " << ex.what() << std::endl;
                        return PV_ERROR_EXCEPTION;
                    }
                }
            }
        }
    }

    return PV_ERROR_OK;
}
// 坐标点按照y值排序
bool comparePointsY(const cv::Point& a, const cv::Point& b)
{
    return a.y < b.y;
}

// 坐标点按照x值排序
bool comparePointsX(const cv::Point& a, const cv::Point& b)
{
    return a.x < b.x;
}

uint64_t qFromBigEndian_64(uint64_t value)
{
    return ((value >> 56) & 0x00000000000000FFULL) | ((value >> 40) & 0x000000000000FF00ULL) | ((value >> 24) & 0x0000000000FF0000ULL) | ((value >> 8) & 0x00000000FF000000ULL) |
           ((value << 8) & 0x000000FF00000000ULL) | ((value << 24) & 0x0000FF0000000000ULL) | ((value << 40) & 0x00FF000000000000ULL) | ((value << 56) & 0xFF00000000000000ULL);
}

std::string getTimestamp(uint64_t t)
{
    time_t     rawtime  = t / 1000;
    struct tm* timeinfo = localtime(&rawtime);
    char       buffer[80];
    strftime(buffer, 80, "%H:%M:%S", timeinfo);
    return std::string(buffer);
}

void clearSocketBuffer(int sockfd)
{
    // 清空socket缓冲区
    std::vector<char> buffer(4096);
    while (true)
    {
        int n = recv(sockfd, buffer.data(), buffer.size(), MSG_DONTWAIT);
        if (n <= 0)
        {
            break;
        }
    }
}
void processMessage(const TrajactoryOutput& info)
{
    // std::cout << "Processing: numObject=" << static_cast<int>(info.numObject) << ", length=" << info.length <<  std::endl;
    std::cout << "Timestamp: " << getTimestamp(qFromBigEndian_64(info.time2)) << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(2));  // 模拟消息处理时间
    // sleep(1);
}

int radar_data_get(int listenfd, int connfd, int total_bytes, TrajactoryOutput& radar_buffer)
{
    // while (true)
    {
        int  n;
        bool received_message = false;

        // 尝试接收消息
        while (total_bytes < sizeof(radar_buffer))
        {
            n = recv(connfd, reinterpret_cast<char*>(&radar_buffer) + total_bytes, sizeof(radar_buffer) - total_bytes, 0);
            if (n < 0)
            {

                if (errno == EWOULDBLOCK || errno == EAGAIN)
                {
                    break;   // 没有新消息，超时
                    return -2;
                }
                else
                {
                    std::cerr << "Error receiving message: " << strerror(errno) << std::endl;
                    close(connfd);
                    close(listenfd);
                    return -2;
                }
            }
            else if (n == 0)
            {
                std::cout << "Connection closed by peer" << std::endl;
                close(connfd);
                close(listenfd);
                return -2;
            }
            else
            {
                total_bytes += n;
            }
        }

        if (total_bytes == sizeof(radar_buffer))
        {
            total_bytes = 0;   // 重置计数器
            processMessage(radar_buffer);

            // 清空接收缓冲区以确保接收最新消息
            clearSocketBuffer(connfd);
        }
    }
}

uint64_t qFromBigEndian_64_radar(uint64_t value)
{
    return ((value >> 56) & 0x00000000000000FFULL) | ((value >> 40) & 0x000000000000FF00ULL) | ((value >> 24) & 0x0000000000FF0000ULL) | ((value >> 8) & 0x00000000FF000000ULL) |
           ((value << 8) & 0x000000FF00000000ULL) | ((value << 24) & 0x0000FF0000000000ULL) | ((value << 40) & 0x00FF000000000000ULL) | ((value << 56) & 0xFF00000000000000ULL);
}

std::string getTimestamp_radar(uint64_t t)
{
    time_t     rawtime  = t / 1000;              // 将时间戳转换为秒
    struct tm* timeinfo = localtime(&rawtime);   // 将秒转换为本地时间
    char       buffer[80];
    // std::cout << "t1:" << t << std::endl;
    // 格式化为时:分:秒
    strftime(buffer, 80, "%H:%M:%S", timeinfo);
    // std::cout << "t2:" << t << std::endl;
    // 提取毫秒部分
    int milliseconds = t % 1000;
    // std::cout << "milliseconds:" << milliseconds << std::endl;
    // 将毫秒部分附加到格式化时间后面
    char result[84];   // 足够容纳 "%H:%M:%S.123"
    snprintf(result, sizeof(result), "%s.%03d", buffer, milliseconds);

    return std::string(result);
}

PV_ERROR_CODE cardetector::inference_test(std::string para_data, vector<json>& results_json)
{
    //获取雷达数据
    logger_car->info("start inference_test");
    TrajactoryOutput radar_buffer;
    int              radar_flag = radar_data_get(listenfd, connfd, total_bytes, radar_buffer);
    if (radar_flag == -2)
    {
        json output_json;
        output_json["radar_status"] = "stop";
        results_json.push_back(output_json);
        return PV_ERROR_EXCEPTION;
    }
    logger_car->info("radar_buffer.res2: {}", radar_buffer.res2);
    if (radar_buffer.res2 == 0)   // 0表示不拥堵，1表示拥堵，默认是0
    {
        logger_car->info("当前车道不拥堵！！！！");
    }
    else
    {
        logger_car->info("当前车道拥堵！！！！");
        return PV_ERROR_EXCEPTION;
    }
    ///////////////////////////////////////////////////////////////////////////////
    //存储雷达数据，以缩小延迟
    if (radar_data.size() < 55)
    {
        radar_data_time one;
        one.radar_output = radar_buffer;
        radar_data.push_back(one);
    }
    else
    {
        radar_data.erase(radar_data.begin());   //删除首元素
        radar_data_time one;
        one.radar_output = radar_buffer;
        radar_data.push_back(one);   //新的雷达数据放到尾部
    }
    logger_car->info(" radar_data.size(): {}", radar_data.size());
    for (int ra = 0; ra < radar_data.size(); ra++)
    {
        string radar_time = getTimestamp_radar(qFromBigEndian_64_radar(radar_data[ra].radar_output.time2));
        // 使用 find 函数查找字符在字符串中的位置
        size_t secondtime_found = radar_time.find('.');
        // logger_car->info("secondtime_found: {}", secondtime_found);
        size_t second                    = std::stoi(radar_time.substr(secondtime_found - 2, 2));
        size_t milli_second              = std::stoi(radar_time.substr(secondtime_found + 1, 3));
        radar_data[ra].second_time       = second;
        radar_data[ra].milli_second_time = milli_second;
        logger_car->info("getTimestamp_radar: {}", radar_time);
        // logger_car->info("radar_data[ra].second_time: {}", radar_data[ra].second_time);
        // logger_car->info("radar_data[ra].milli_second_time: {}", radar_data[ra].milli_second_time);
    }

    vector<std::string> para;
    Stringsplit(para_data, '&', para);
    logger_car->info("para_data: {}", para_data);

    for (int i = 0; i < para.size(); i++)
    {
        QString         jsonString = QString::fromStdString(para[i]);
        QJsonParseError jsonError;
        QJsonDocument   jsonDoc(QJsonDocument::fromJson(para[i].data(), &jsonError));
        if (jsonError.error == QJsonParseError::NoError)
        {
            if (jsonDoc.isNull())
            {
                logger_car->info("jsonDoc erro");
            }
            // 获取根对象
            QJsonObject     jsonObj   = jsonDoc.object();
            algo_parameters algo_para = ParseParameters2(jsonObj);
            // algo_parameters2 algo_para = ParseParameters(para[i]);
            int flag_para = 0;
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
                m_illegal_data.push_back(one_camera);
                m_illegal_data[m_illegal_data.size() - 1].bytetrack.init(m_illegal_data[m_illegal_data.size() - 1].params);
            }
        }
    }

    for (int k = 0; k < m_illegal_data.size(); k++)
    {
        logger_car->info("***********************************************************************************");
        logger_car->info("camera_id: {}", m_illegal_data[k].algo_para.camera_id);
        logger_car->info(" nframe: {}", m_illegal_data[k].nframe);

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
            cout << "car_X: " << car_X << endl;
            cout << "car_Y: " << car_Y << endl;
            cout << "car_W: " << car_W << endl;
            cout << "car_H: " << car_H << endl;
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

        if (m_illegal_data[k].algo_para.bayonet_img_path != "")
        {
            // Mat bayonet_img                         = imread(m_illegal_data[k].algo_para.bayonet_img_path);
            // m_illegal_data[k].algo_para.bayonet_img = bayonet_img;
            //            cv::rectangle(m_illegal_data[k].algo_para.bayonet_img, cv::Point((int)m_illegal_data[k].algo_para.kakou_plate_box.x, (int)m_illegal_data[k].algo_para.kakou_plate_box.y),
            //                          cv::Point((int)(m_illegal_data[k].algo_para.kakou_plate_box.x + m_illegal_data[k].algo_para.kakou_plate_box.width),
            //                                    (int)(m_illegal_data[k].algo_para.kakou_plate_box.y + m_illegal_data[k].algo_para.kakou_plate_box.height)),
            //                          cv::Scalar(0, 0, 255), 5);
        }

        m_illegal_data[k].nframe = m_illegal_data[k].nframe + 1;
        cv::Mat img;
        try
        {
            if (m_illegal_data[k].algo_para.img_path != "")
            {
                logger_car->info("imread  img_path");
                img = imread(m_illegal_data[k].algo_para.img_path);
                logger_car->info("img.clos:{}", img.cols);
                logger_car->info("img.rows:{}", img.rows);
                vector<std::string> img_time;
                Stringsplit(m_illegal_data[k].algo_para.img_path, '_', img_time);
                img_time[1].erase(img_time[1].length() - 4);   // 从字符串末尾移除指定数量的字符
                m_illegal_data[k].algo_para.img_time = img_time[1];
                logger_car->info("m_illegal_data[k].algo_para.img_time:{}", m_illegal_data[k].algo_para.img_time);
            }
            else
            {
                if (m_illegal_data[k].algo_para.bayonet_img_path != "")
                {
                    logger_car->info("imread  unv_img_path");
                    img = imread(m_illegal_data[k].algo_para.bayonet_img_path);
                    logger_car->info("img.clos:{}", img.cols);
                    logger_car->info("img.rows:{}", img.rows);
                    vector<std::string> img_name;
                    Stringsplit(m_illegal_data[k].algo_para.bayonet_img_path, '/', img_name);
                    vector<std::string> img_time;
                    Stringsplit(img_name[img_name.size() - 1], '_', img_time);
                    m_illegal_data[k].algo_para.img_time = img_time[0];
                    logger_car->info("m_illegal_data[k].algo_para.img_time:{}", m_illegal_data[k].algo_para.img_time);
                }
            }
        }
        catch (...)
        {
            logger_car->error("img read error");
            return PV_ERROR_EXCEPTION;
        }

        if (img.empty())
        {
            logger_car->error("img empty");
            return PV_ERROR_EXCEPTION;
        }

        bm_image    image;
        bm_status_t bmret = cv::bmcv::toBMI(img, &image, true);
        if (bmret != BM_SUCCESS)
        {
            return PV_ERROR_EXCEPTION;
        }
        vector<vector<cv::Point>> contour_temp_all;
        vector<cv::Point>         contour_temp;
        contour_temp.push_back(cv::Point(1042, 2055));
        contour_temp.push_back(cv::Point(1607, 1291));
        contour_temp.push_back(cv::Point(2081, 669));
        contour_temp.push_back(cv::Point(2303, 403));
        contour_temp.push_back(cv::Point(2447, 268));
        contour_temp.push_back(cv::Point(2589, 275));
        contour_temp.push_back(cv::Point(2480, 513));
        contour_temp.push_back(cv::Point(2379, 908));
        contour_temp.push_back(cv::Point(2205, 1636));
        contour_temp.push_back(cv::Point(2099, 2046));

        contour_temp_all.push_back(contour_temp);
        // contour_temp_all.push_back(contour_temp_2);
        for (int k = 0; k < contour_temp.size() - 1; k++)
        {
            // line(img, contour_temp[k], contour_temp[k + 1], Scalar(0, 0, 255), 3);
        }

        //(1) 根据划定结果获取车距线
        logger_car->info(" start get contours");
        vector<vector<cv::Point>> lines;
        vector<cv::Point>         line_zero;
        vector<cv::Point>         line_50;
        vector<cv::Point>         line_100;
        cout << "m_illegal_data[k].algo_para.lanes.size(): " << m_illegal_data[k].algo_para.lanes.size() << endl;

        if (m_illegal_data[k].algo_para.lanes.size() > 0)
        {
            cout << "m_illegal_data[k].algo_para.lanes.laneRegion: " << m_illegal_data[k].algo_para.lanes[0].laneRegion << endl;
            if (m_illegal_data[k].algo_para.lanes[0].laneRegion.size() == 4)
            {
                line_zero.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[0]);
                line_zero.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[1]);
                line_50.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[2]);
                line_50.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[3]);
                lines.push_back(line_zero);
                lines.push_back(line_50);
            }
        }
        if (m_illegal_data[k].algo_para.lanes.size() > 0)
        {
            if (m_illegal_data[k].algo_para.lanes[0].laneRegion.size() == 6)
            {
                line_zero.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[0]);
                line_zero.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[1]);
                line_50.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[2]);
                line_50.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[3]);
                line_100.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[4]);
                line_100.push_back(m_illegal_data[k].algo_para.lanes[0].laneRegion[5]);
                lines.push_back(line_zero);
                lines.push_back(line_50);
                lines.push_back(line_100);
            }
        }
        for (int k = 0; k < lines.size(); k++)
        {
            // cout << "lines[k][0]: " << lines[k][0] << endl;
            // cout << "lines[k][1]: " << lines[k][1] << endl;
            // line(img, lines[k][0], lines[k][1], Scalar(0, 0, 255), 3);
        }

        logger_car->info("end get contours");

        logger_car->info("start cardetect");
        //(2) yolov8车牌车辆检测
        std::vector<bm_image> batch_imgs;
        batch_imgs.push_back(image);
        std::vector<YoloV8BoxVec> boxes;
        try
        {
            cardetect(batch_imgs, boxes);
        }
        catch (...)
        {
            logger_car->error("cardetect error");
            return PV_ERROR_EXCEPTION;
        }
        logger_car->info("end cardetect");

        logger_car->info("start cartrack");
        //(3) bytetrack跟踪
        std::vector<std::vector<BoundingBox>> yolov8_BoundingBoxes;
        for (size_t i = 0; i < batch_imgs.size(); i++)
        {
            QString boxes_size_str = QString::number(boxes[i].size());
            logger_car->info("boxes[i].size(): {}", boxes[i].size());
            std::vector<BoundingBox> yolov8_boxes_BoundingBox;
            for (auto b : boxes[i])
            {
                // cv::rectangle(img, cv::Point((int)b.x1, (int)b.y1), cv::Point((int)b.x2, (int)b.y2), cv::Scalar(0, 255, 0), 2);
                //跟踪所有车辆
                if (b.class_id == 0 || b.class_id == 1 || b.class_id == 2)
                {
                    yolov8_boxes_BoundingBox.push_back({(int)b.x1, (int)b.y1, (int)(b.x2 - b.x1), (int)(b.y2 - b.y1), b.score, b.class_id});
                }
            }
            // imwrite(to_string(m_illegal_data[k].nframe) + "output.jpg", img);
            yolov8_BoundingBoxes.push_back(yolov8_boxes_BoundingBox);
        }
        QString yolov8_BoundingBoxes_str = QString::number(yolov8_BoundingBoxes.size());
        logger_car->info("yolov8_BoundingBoxes.size(): {}", yolov8_BoundingBoxes.size());

        vector<STracks> results_stracks;
        try
        {
            cartrack(yolov8_BoundingBoxes, results_stracks, m_illegal_data[k]);
        }
        catch (...)
        {
            logger_car->error("cartrack error");
            return PV_ERROR_EXCEPTION;
        }
        logger_car->info("end cartrack");
        double begin = (double)cv::getTickCount();
        double time1 = begin * 1000 / cv::getTickFrequency();

        //(5) 违法判定+证据输出
        for (size_t i = 0; i < batch_imgs.size(); i++)
        {
            //            for (auto& bbox : results_stracks[i])
            //            {
            //                cv::rectangle(img, cv::Point((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1]), cv::Point((int)bbox->det_tlwh[0] + bbox->det_tlwh[2], (int)bbox->det_tlwh[1] +
            //                bbox->det_tlwh[3]),
            //                              cv::Scalar(0, 255, 0), 2);
            //                putText(img, "id: " + to_string(bbox->track_id), cv::Point((int)bbox->det_tlwh[0], (int)bbox->det_tlwh[1]), 1, 4, Scalar(0, 0, 255), 4, LINE_8);
            //            }
            //挑选可信度大于给定阈值的车辆进行违法分析
            STracks one_stracks_select;
            for (auto& bbox : results_stracks[i])   //所有车辆
            {
                if (bbox->score > m_illegal_data[k].algo_para.vehicle_conf)
                {
                    one_stracks_select.push_back(bbox);
                }
            }
            if (lines.size() > 1)
            {
                postprocess_nokeepdistance(img, img, m_illegal_data[k].algo_para.timeInterval, 1, lines, contour_temp_all, boxes[i], one_stracks_select, results_json, m_illegal_data[k]);
                m_illegal_data[k].bytetrack.clear_removed_stracks();
            }
        }
        imwrite("/data2/camera-handler/results/data/" + to_string(m_illegal_data[k].nframe) + "output.jpg", img);
        logger_car->info("end inference_test");
        double end   = (double)cv::getTickCount();
        double time2 = end * 1000 / cv::getTickFrequency();
        double time  = (end - begin) * 1000 / cv::getTickFrequency();
        cout << "time: " << time << endl;
    }

    memset(&radar_buffer, 0, sizeof(radar_buffer));   // 使用memset将结构体清零

    return PV_ERROR_OK;
}
