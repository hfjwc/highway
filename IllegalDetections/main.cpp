#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <float.h>
#include <stdio.h>
#include <vector>
#include <chrono>
#include "BYTETracker.h"

#include "opencv2/opencv.hpp"
#include "inference.h"
#include <io.h>
#include <thread>
#define socklen_t int
#pragma comment (lib, "ws2_32.lib")



using namespace cv;
using namespace std;
using namespace dnn;

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};
typedef struct
{
    float x1;
    float y1;
    float x2;
    float y2;
}FloatRet_;
//检测数据
struct vehicle_detect_data
{
    cv::Mat vehicle_image;
    vector<cv::Mat> vehicle_plate_images;
    cv::Rect vehicle_location_box;
    int class_id;
    int track_id;
    float vehicle_confidence;
    cv::Rect palte_location;
    clock_t start_time;
    int flag_is = 0;
};

//输出数据
struct vehicle_output_data
{
    vector<cv::Mat> vehicle_image;//包含2张车辆图像
    cv::Mat vehicle_plate_image;//车牌图像
    cv::Rect vehicle_location_box;//车辆位置
    string vehicle_plate;//车牌号
    string vehicle_type;//车型
    float vehicle_confidence;//车辆位置置信度
    float vehicle_plate__confidence;//车牌位置置信度
    float vehicle_plate_recognition_confidence;//车牌号置信度
    int track_id;//跟踪ID
};



static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

class yolox
{
public:
    yolox();
    int detect(cv::Mat &img, std::vector<Object> &detectResults);
private:
    const int INPUT_W = 1088;
    const int INPUT_H = 608;
    const float mean_vals[3] = {0.485, 0.456, 0.406};
    const float norm_vals[3] = {0.229, 0.224, 0.225};
    const int stride_arr[3] = {8, 16, 32}; // might have stride=64 in YOLOX
    std::vector<GridAndStride> grid_strides;

    Mat static_resize(Mat& img);
    void generate_grids_and_stride(std::vector<int>& strides);
    void generate_yolox_proposals(const float* feat_ptr, std::vector<Object>& objects);
    void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked);
    float nms_threshold = 0.7;
    float prob_threshold = 0.1;
    int num_grid;
    int num_class;
    Net net;
};

yolox::yolox()
{
    string model_path = "bytetrack_s.onnx";
    
    bool isCuda = true;
    //cuda,gpu
    
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    
    this->net = readNet(model_path);
 
    std::vector<int> strides(stride_arr, stride_arr + sizeof(stride_arr) / sizeof(stride_arr[0]));
    generate_grids_and_stride(strides);
}

Mat yolox::static_resize(Mat& img) {
    float r = min(INPUT_W / (img.cols*1.0), INPUT_H / (img.rows*1.0));
    // r = std::min(r, 1.0f);
    int unpad_w = r * img.cols;
    int unpad_h = r * img.rows;
    Mat re(unpad_h, unpad_w, CV_8UC3);
    resize(img, re, re.size());
    Mat out(INPUT_H, INPUT_W, CV_8UC3, Scalar(114, 114, 114));
    re.copyTo(out(Rect(0, 0, re.cols, re.rows)));
    return out;
}

void yolox::generate_grids_and_stride(std::vector<int>& strides)
{
    for (int i = 0; i < (int)strides.size(); i++)
    {
        int stride = strides[i];
        int num_grid_w = INPUT_W / stride;
        int num_grid_h = INPUT_H / stride;
        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                GridAndStride gs;
                gs.grid0 = g0;
                gs.grid1 = g1;
                gs.stride = stride;
                grid_strides.push_back(gs);
            }
        }
    }
}

void yolox::generate_yolox_proposals(const float* feat_ptr, std::vector<Object>& objects)
{
    const int num_anchors = grid_strides.size();
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_center = (feat_ptr[0] + grid0) * stride;
        float y_center = (feat_ptr[1] + grid1) * stride;
        float w = exp(feat_ptr[2]) * stride;
        float h = exp(feat_ptr[3]) * stride;
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;

        float box_objectness = feat_ptr[4];
        for (int class_idx = 0; class_idx < num_class; class_idx++)
        {
            float box_cls_score = feat_ptr[5 + class_idx];
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > prob_threshold)
            {
                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = w;
                obj.rect.height = h;
                obj.label = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }

        } // class loop
        feat_ptr += (num_class + 5);

    } // point anchor loop
}

void yolox::nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

int yolox::detect(cv::Mat &srcimg, std::vector<Object>& objects)
{
    float scale = min(INPUT_W / (srcimg.cols*1.0), INPUT_H / (srcimg.rows*1.0));
    Mat img = static_resize(srcimg);
    img.convertTo(img, CV_32F);
	int i = 0, j = 0;
    for (i = 0; i < img.rows; i++)
	{
		float* pdata = (float*)(img.data + i * img.step);
		for (j = 0; j < img.cols; j++)
		{
		    pdata[0] = (pdata[2] / 255.0 - this->mean_vals[0]) / this->norm_vals[0];
			pdata[1] = (pdata[1] / 255.0 - this->mean_vals[1]) / this->norm_vals[1];
			pdata[2] = (pdata[0] / 255.0 - this->mean_vals[2]) / this->norm_vals[2];
			pdata += 3;
        }
    }
    this->net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    this->net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    Mat blob = blobFromImage(img);
	this->net.setInput(blob);
	vector<Mat> outs;
	this->net.forward(outs, this->net.getUnconnectedOutLayersNames());
    this->num_grid = outs[0].size[1];
    this->num_class = outs[0].size[2] - 5;
    const float* out = (float*)outs[0].data;
    std::vector<Object> proposals;
    generate_yolox_proposals(out, proposals);
    // sort all proposals by score from highest to lowest
    qsort_descent_inplace(proposals);

    // apply nms with nms_threshold
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked);

    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x) / scale;
        float y0 = (objects[i].rect.y) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

        // clip
        // x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
        // y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
        // x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
        // y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    return 0;
}




int getFiles(std::string path, std::vector<std::string>& files, std::vector<std::string>& names)
{
    int i = 0;
    intptr_t hFile = 0;
    struct _finddata_t c_file;
    std::string imageFile = path + "*.*";

    if ((hFile = _findfirst(imageFile.c_str(), &c_file)) == -1L)
    {
        _findclose(hFile);
        return -1;
    }
    else
    {
        while (true)
        {
            std::string strname(c_file.name);
            if (std::string::npos != strname.find(".jpg") || std::string::npos != strname.find(".png") || std::string::npos != strname.find(".bmp"))
            {
                std::string fullName = path + c_file.name;

                files.push_back(fullName);

                std::string cutname = strname.substr(0, strname.rfind("."));
                names.push_back(cutname);
            }

            if (_findnext(hFile, &c_file) != 0)
            {
                _findclose(hFile);
                break;
            }
        }
    }

    return 0;
}

int LineLineCollisionDetection(FloatRet_ line1, FloatRet_ line2) //线段交叉测试
{
    if ((line1.x1 > line1.x2 ? line1.x1 : line1.x2) < (line2.x1 < line2.x2 ? line2.x1 : line2.x2) ||
        (line1.y1 > line1.y2 ? line1.y1 : line1.y2) < (line2.y1 < line2.y2 ? line2.y1 : line2.y2) ||
        (line2.x1 > line2.x2 ? line2.x1 : line2.x2) < (line1.x1 < line1.x2 ? line1.x1 : line1.x2) ||
        (line2.y1 > line2.y2 ? line2.y1 : line2.y2) < (line1.y1 < line1.y2 ? line1.y1 : line1.y2))
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
int AreaAreaCollisionDetection(FloatRet_ box1, FloatRet_ box2) //矩形碰撞检测
{
    float centreX1 = (box1.x1 + box1.x2) / 2;
    float centreY1 = (box1.y1 + box1.y2) / 2;
    float width1 = abs(box1.x2 - box1.x1);
    float height1 = abs(box1.y2 - box1.y1);
    float centreX2 = (box2.x1 + box2.x2) / 2;
    float centreY2 = (box2.y1 + box2.y2) / 2;
    float width2 = abs(box2.x2 - box2.x1);
    float height2 = abs(box2.y2 - box2.y1);
    if (abs(centreX1 - centreX2) < (width1 + width2) / 2 &&
        abs(centreY1 - centreY2) < (height1 + height2) / 2)
    {
        return 1;
    }
    return 0;
}
int AreaLineCollisionDetection(FloatRet_ box, FloatRet_ line) //线段矩形相交测试
{
    FloatRet_ line1 = { 0 };
    line1.x1 = box.x1;
    line1.x2 = box.x2;
    line1.y1 = box.y1;
    line1.y2 = box.y1;

    FloatRet_ line2 = { 0 };
    line2.x1 = box.x1;
    line2.x2 = box.x2;
    line2.y1 = box.y2;
    line2.y2 = box.y2;

    FloatRet_ line3 = { 0 };
    line3.x1 = box.x1;
    line3.x2 = box.x1;
    line3.y1 = box.y1;
    line3.y2 = box.y2;

    FloatRet_ line4 = { 0 };
    line4.x1 = box.x2;
    line4.x2 = box.x2;
    line4.y1 = box.y1;
    line4.y2 = box.y2;

    if (LineLineCollisionDetection(line1, line) ||
        LineLineCollisionDetection(line2, line) ||
        LineLineCollisionDetection(line3, line) ||
        LineLineCollisionDetection(line4, line))
    {
        return 1;
    }
    return 0;
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

bool isLineInsidePolygon(vector<cv::Point> contour, const cv::Point2f& start, const cv::Point2f& end) {
    // 检查线段的每个点是否在多边形内部
    float epsilon = 1e-5; // 设置一个小的阈值，用于判断点在多边形边上的情况
    for (float t = 0; t <= 1; t += epsilon) {
        cv::Point2f point = (1 - t) * start + t * end; // 计算线段上的点
        if (cv::pointPolygonTest(contour, point, false) < 0) {
            return false; // 如果有一个点在多边形外部，则线段在多边形外部
        }
    }
    return true; // 所有点都在内部，线段在多边形内部
}


int main()
{
    vector<vehicle_detect_data> detection_data;
    vector<vehicle_output_data> output_data;

    std::string projectBasePath = "D:/work/models/2024.5.10/"; // Set your ultralytics base path

    bool runOnGPU = true;

    //
    // Pass in either:
    //
    // "yolov8s.onnx" or "yolov5s.onnx"
    //
    // To run Inference with yolov8/yolov5 (ONNX)
    //

    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    Inference inf(projectBasePath + "/best.onnx", cv::Size(640, 640), "classes.txt", runOnGPU);

    std::vector<std::string> files;
    std::vector<std::string> names;
    getFiles("./test/", files, names);

    //std::vector<std::string> imageNames;
    //imageNames.push_back(projectBasePath + "/test/20221104_8336.jpg");
    //imageNames.push_back(projectBasePath + "/test/20221104_8339.jpg");

    //for (int i = 0; i < files.size(); ++i)
    VideoCapture cap("D://2024.4.19-E//data//不按规定保持安全车距视频//2//20211020144928865.mp4");

    if (!cap.isOpened())
        return 0;
    int img_w = cap.get(CAP_PROP_FRAME_WIDTH);
    int img_h = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    long nFrame = static_cast<long>(cap.get(CAP_PROP_FRAME_COUNT));
    cout << "Total frames: " << nFrame << ", fps: " << fps << endl;
    VideoWriter writer("demo.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(img_w, img_h));
    int num_frames = 0;
    BYTETracker tracker(fps, 30);
    for (;;)
    {
        //cv::Mat frame = cv::imread(files[i]);
        cv::Mat frame;
        if (!cap.read(frame))
            break;


        num_frames++;
        //if (num_frames % 2 == 0)
        {
            if (frame.empty())
                break;
            //frame = cv::imread("2.png");
            // Inference starts here...
            clock_t start, end;
            float time;
            start = clock();
            std::vector<Detection> output = inf.runInference(frame);
            
            end = clock();
            time = (float)(end - start);//CLOCKS_PER_SEC; 
            //printf("timeCount = %f\n", time);
            //沪蓉高速
            /*Point A = Point(526, 127);
            Point B = Point(576, 112);
            Point C = Point(1894, 643);
            Point D = Point(1798, 833);*/

           /* Point A = Point(426, 536);
            Point B = Point(1187, 552);
            Point C = Point(712, 146);
            Point D = Point(1010, 49);*/
            //不按规定保持车距
            //0m车距线
            Point A = Point(1038, 814);
            Point B = Point(1419, 866);
            //50m车距线
            Point C = Point(1383, 642);
            Point D = Point(1536, 659);
            FloatRet_ line_zero = { 0 };
            line_zero.x1 = 1038;
            line_zero.y1 = 814;
            line_zero.x2 = 1419;
            line_zero.y2 = 866;

            FloatRet_ line_zero_1 = { 0 };
            line_zero.x1 = 1038;
            line_zero.y1 = 814;
            line_zero.x2 = 1383;
            line_zero.y2 = 642;


            FloatRet_ line_50 = { 0 };
            line_50.x1 = 1383;
            line_50.y1 = 642;
            line_50.x2 = 1536;
            line_50.y2 = 659;

            FloatRet_ line_50_1 = { 0 };
            line_50.x1 = 1419;
            line_50.y1 = 866;
            line_50.x2 = 1536;
            line_50.y2 = 659;


          

            /*
            Point A = Point(1509, 575);
            Point B = Point(1543, 578);
            Point C = Point(897, 1077);
            Point D = Point(485, 1076);*/
         
            
            line(frame, A, B, Scalar(0, 0, 255), 3);
           // line(frame, B, D, Scalar(0, 0, 255), 3);
            line(frame, D, C, Scalar(0, 0, 255), 3);
           // line(frame, C, A, Scalar(0, 0, 255), 3);
            vector<cv::Point> contour;
            contour.push_back(A);
            contour.push_back(B);
            contour.push_back(D);
            contour.push_back(C);
            FloatRet_ box1;
            box1.x1 = C.x;
            box1.y1 = C.y;
            box1.x2 = B.x;
            box1.y2 = B.y;
            //cout << "num_frames: " << num_frames << endl;

            FloatRet_ line_left = { 0 };
            line_left.x1 = A.x;
            line_left.y1 = A.y;
            line_left.x2 = B.x;
            line_left.y2 = B.y;

            FloatRet_ line_top = { 0 };
            line_top.x1 = A.x;
            line_top.y1 = A.y;
            line_top.x2 = C.x;
            line_top.y2 = C.y;

            FloatRet_ line_right = { 0 };
            line_right.x1 = B.x;
            line_right.y1 = B.y;
            line_right.x2 = D.x;
            line_right.y2 = D.y;

            FloatRet_ line_bottom = { 0 };
            line_bottom.x1 = C.x;
            line_bottom.y1 = C.y;
            line_bottom.x2 = D.x;
            line_bottom.y2 = D.y;

            /*line(frame, Point(line_left.x1, line_left.y1), Point(line_left.x2, line_left.y2), Scalar(0, 0, 255), 3);
            line(frame, Point(line_top.x1, line_top.y1), Point(line_top.x2, line_top.y2), Scalar(0, 255, 0), 3);
            line(frame, Point(line_right.x1, line_right.y1), Point(line_right.x2, line_right.y2), Scalar(255, 0, 0), 3);
            line(frame, Point(line_bottom.x1, line_bottom.y1), Point(line_bottom.x2, line_bottom.y2), Scalar(125, 125, 125), 3);*/

          

            Rect_<float> bb_leftbox;
            bb_leftbox.x = line_left.x1;
            bb_leftbox.y = line_left.y1;
            bb_leftbox.width  = line_bottom.x2 - line_left.x1;
            bb_leftbox.height = line_bottom.y2 - line_left.y1;
           

            std::vector<Object> objects;
            for (int idx = 0; idx < output.size(); idx++)
            {
                Object one_object;
                one_object.rect = output[idx].box;
                one_object.label = output[idx].class_id;
                one_object.prob = output[idx].confidence;
                if (output[idx].class_id == 0 || output[idx].class_id == 1 || output[idx].class_id == 2)//只跟踪车辆
                {
                    objects.push_back(one_object);
                }
                

            }
            //cout << "objects.size(): " << objects.size() << endl;
            
            vector<STrack> output_stracks = tracker.update(objects);
            for (int m = 0; m < output.size(); m++)
            {
                double max_ratio = 0;
                for (int i = 0; i < output_stracks.size(); i++)
                {
                    vector<float> tlwh = output_stracks[i].tlwh;
                    bool vertical = tlwh[2] / tlwh[3] > 1.6;
                    //output[i].track_id = output_stracks[i].track_id;
                    Rect_<float> bb_output;
                    bb_output.x = output[m].box.x;
                    bb_output.y = output[m].box.y;
                    bb_output.width = output[m].box.width;
                    bb_output.height = output[m].box.height;

                    Rect_<float> bb_output_track;
                    bb_output_track.x = tlwh[0];
                    bb_output_track.y = tlwh[1];
                    bb_output_track.width = tlwh[2];
                    bb_output_track.height = tlwh[3];

                    double ratio_output = GetIOU_rect(bb_output, bb_output_track);
                    if (ratio_output > max_ratio)
                    {
                        output[m].track_id = output_stracks[i].track_id;
                    }


                    //if (tlwh[2] * tlwh[3] > 20 && !vertical)
                    {
                        Scalar s = tracker.get_color(output_stracks[i].track_id);
                        //putText(frame, format("%d", output_stracks[i].track_id), Point(tlwh[0]+ tlwh[2]/2, tlwh[1]+ tlwh[3]/2),
                        //    0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                        //rectangle(frame, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);

                    }
                }
            }

           
           // putText(frame, format("frame: %d num: %d", num_frames, (int)output_stracks.size()),
              //  Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            /*cv::namedWindow("YOLOX", cv::WINDOW_NORMAL);
            cv::resizeWindow("YOLOX", 1600, 900);
            cv::imshow("YOLOX", frame);*/
          
            
            int detections = output.size();
            //std::cout << "Number of detections:" << detections << std::endl;
            for (int j = 0; j < detection_data.size(); j++)
            {
                clock_t end_time = clock();
                if ((end_time - detection_data[j].start_time) > 5000)
                {
                    cout << "end_time - detection_data[j].start_time: " << end_time - detection_data[j].start_time << endl;
                    detection_data[j].flag_is = 1;//记录时间超过5秒，如果未抓拍到第二张违法图像，则删除该检测数据
                }
            }
            for (int i = 0; i < detections; ++i)
            {
                Detection detection = output[i];

                cv::Rect box = detection.box;
                cv::Scalar color = detection.color;

                // Detection box
                Rect_<float> bb_truck;
                bb_truck.x = box.x;
                bb_truck.y = box.y;
                bb_truck.width = box.width;
                bb_truck.height = box.height;

                FloatRet_ line_car = { 0 };
                line_car.x1 = box.x;
                line_car.y1 = box.y + box.height;
                line_car.x2 = box.x + box.width/2;
                line_car.y2 = box.y + box.height;
                //计算iou
                double ratio=GetIOU_rect(bb_leftbox, bb_truck);
                //cout << "ratio: " << ratio << endl;

                FloatRet_ car_line_left = { 0 };
                car_line_left.x1 = box.x;
                car_line_left.y1 = box.y;
                car_line_left.x2 = box.x+ box.width;
                car_line_left.y2 = box.y;

                FloatRet_ car_line_top = { 0 };
                car_line_top.x1 = box.x;
                car_line_top.y1 = box.y;
                car_line_top.x2 = box.x;
                car_line_top.y2 = box.y+ box.height;

                FloatRet_ car_line_right = { 0 };
                car_line_right.x1 = box.x+box.width;
                car_line_right.y1 = box.y;
                car_line_right.x2 = box.x + box.width;
                car_line_right.y2 = box.y + box.height;

                FloatRet_ car_line_bottom = { 0 };
                car_line_bottom.x1 = box.x;
                car_line_bottom.y1 = box.y + box.height;
                car_line_bottom.x2 = box.x + box.width;
                car_line_bottom.y2 = box.y + box.height;

                //计算车的四条边界线和0m车距线是否相交
                int is_cross_left = LineLineCollisionDetection(line_zero, car_line_left);
                int is_cross_top = LineLineCollisionDetection(line_zero, car_line_top);
                int is_cross_right = LineLineCollisionDetection(line_zero, car_line_right);
                int is_cross_bottom = LineLineCollisionDetection(line_zero, car_line_bottom);

                //计算车的四条边界线和50m车距线是否相交
                int is_cross_left_50 = LineLineCollisionDetection(line_50, car_line_left);
                int is_cross_top_50 = LineLineCollisionDetection(line_50, car_line_top);
                int is_cross_right_50 = LineLineCollisionDetection(line_50, car_line_right);
                int is_cross_bottom_50 = LineLineCollisionDetection(line_50, car_line_bottom);
               
                FloatRet_ box2;
                box2.x1 = box.x;
                box2.y1 = box.y;
                box2.x2 = box.x + box.width;;
                box2.y2 = box.y + box.height;

                //int is_cross_box=AreaAreaCollisionDetection( box1,  box2); //矩形碰撞检测
                int is_cross_box_1 = AreaLineCollisionDetection(box2, line_zero); //线段矩形相交测试
                int is_cross_box_2 = AreaLineCollisionDetection(box2, line_50); //线段矩形相交测试
                int is_cross_box_3 = AreaLineCollisionDetection(box2, line_zero_1); //线段矩形相交测试
                int is_cross_box_4 = AreaLineCollisionDetection(box2, line_50_1); //线段矩形相交测试
                

                //判断点是否在多边形区域内
           
                cv::Point start_point = Point(line_car.x1, line_car.y1);
                cv::Point end_point = Point(line_car.x2, line_car.y2);
                double is_inside_start = -1;
                double is_inside_end = -1;
                is_inside_start=pointPolygonTest(contour, start_point, true);
                is_inside_end= pointPolygonTest(contour, end_point, true);
                int flag = 0;

               
                //判定指定车型是否进入违法区域
                if (detection.className == "bus" || detection.className == "truck"|| detection.className == "car")
                {
                     //若车辆到达0m位置，车速大于50km/h，先判断50m区域内是否存在其他车辆，
                     //如果存在，判断车辆所在车道号与0m位置当前车辆是否属于同一车道，如果属于，则在50m区域内未按规定保持车距
                     //车辆box与50m区域的任一边界相交
                    if (is_cross_box_1 == 1|| is_cross_box_2==1|| is_cross_box_3==1|| is_cross_box_4==1)
                        
                    {
                        cv::rectangle(frame, box, color, 2);
                        putText(frame, format("%d", detection.track_id), Point(box.x + box.width / 2, box.y + box.height / 2),
                            0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                        cv::putText(frame, "box  warning  warning!!!", cv::Point(1000, 1000), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 5, 0);
                    }

                    if ((is_cross_left == 1 || is_cross_top == 1 || is_cross_right == 1 || is_cross_bottom == 1))//0m线与车辆box任一边界相交
                    {
                        flag = 1;
                       cv::rectangle(frame, box, color, 2);
                       putText(frame, format("%d", detection.track_id), Point(box.x + box.width / 2, box.y + box.height / 2),
                           0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                       cv::putText(frame, "warning  warning  warning!!!", cv::Point(1000, 1000), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 5, 0);
                    }
                    if ((is_cross_left_50 == 1 || is_cross_top_50 == 1 || is_cross_right_50 == 1 || is_cross_bottom_50 == 1))//50m线与车辆box任一边界相交
                    {
                        flag = 1;
                        cv::rectangle(frame, box, color, 2);
                        putText(frame, format("%d", detection.track_id), Point(box.x + box.width / 2, box.y + box.height / 2),
                            0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                        cv::putText(frame, "warning  warning  warning!!!", cv::Point(1000, 1000), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255), 5, 0);
                    }
                  
                    //到达时间间隔5秒，判定是否可以形成完整违法证据，若可以则存入输出数据，若不可以，则删除该检测数据
                    for (int j = 0; j < detection_data.size(); j++)
                    {
                        clock_t end_time = clock();
                        if ((end_time - detection_data[j].start_time) > 5000)
                        {
                            cout << "end_time - detection_data[j].start_time: " << end_time - detection_data[j].start_time << endl;
                            detection_data[j].flag_is = 1;//记录时间超过5秒，如果未抓拍到第二张违法图像，则删除该检测数据
                           
                            if (flag == 1)
                            {
                                if (detection.track_id == detection_data[j].track_id)//同一车辆间隔5秒后违法
                                {
                                    vehicle_output_data one_output;
                                    one_output.vehicle_image.push_back(detection_data[j].vehicle_image);
                                    one_output.vehicle_image.push_back(frame);
                                    //one_output.vehicle_plate_image = detection_data[j].vehicle_plate_image;
                                    one_output.track_id = detection.track_id;
                                    output_data.push_back(one_output);
                                    imwrite("D://work//vs_codes//yolov8_bytetrack//yolov8_bytetrack//output//"+to_string(num_frames) + "output1.jpg", detection_data[j].vehicle_image);
                                    imwrite("D://work//vs_codes//yolov8_bytetrack//yolov8_bytetrack//output//"+to_string(num_frames) + "output2.jpg", frame);
                                    if (detection_data[j].vehicle_plate_images.size() > 1)
                                    {
                                        imwrite("D://work//vs_codes//yolov8_bytetrack//yolov8_bytetrack//output//" + to_string(num_frames) + "output3.jpg", detection_data[j].vehicle_plate_images[0]);
                                        imwrite("D://work//vs_codes//yolov8_bytetrack//yolov8_bytetrack//output//" + to_string(num_frames) + "output4.jpg", detection_data[j].vehicle_plate_images[1]);
                                    }
                                    else
                                    {
                                        if (detection_data[j].vehicle_plate_images.size() == 1)
                                        {
                                            imwrite("D://work//vs_codes//yolov8_bytetrack//yolov8_bytetrack//output//" + to_string(num_frames) + "output3.jpg", detection_data[j].vehicle_plate_images[0]);
                                        }

                                    }
                                   
                                    
                                    detection_data.erase(detection_data.begin() + j);
                                    
                                }
                            }
                        }

                    }
                    
                    // 记录程序开始时间点
                    if (flag == 1)
                    {
                        
                        int flag_temp_detect = 0;
                        for (int j = 0; j < detection_data.size(); j++)
                        {
                            if (detection_data[j].track_id == detection.track_id)
                            {
                                flag_temp_detect = 1;
                            }
                        }
                        //超过5秒，检测数据已经被清除，如果输出数据中已经存在该track_id,则不再重复记录
                        int flag_temp_output = 0;
                        for (int j = 0; j < output_data.size(); j++)
                        {
                            if (output_data[j].track_id == detection.track_id)
                            {
                                flag_temp_output = 1;
                            }
                        }

                        if (flag_temp_detect == 0 && flag_temp_output==0)
                        {  
                            vehicle_detect_data one_detect;
                            for (int n = 0; n < detections; ++n)
                            {
                                Detection detection_plate = output[n];
                                
                                cv::Rect box_plate = detection_plate.box;
                                if (detection_plate.class_id == 3 || detection_plate.class_id == 4 || detection_plate.class_id == 5 || detection_plate.class_id == 6 || detection_plate.class_id == 7)
                                {
                                    Rect_<float> bb_plate;
                                    bb_plate.x = box_plate.x;
                                    bb_plate.y = box_plate.y;
                                    bb_plate.width = box_plate.width;
                                    bb_plate.height = box_plate.height;
                                  
                                  
                                    //计算车辆和车牌的iou
                                    double ratio_palte = GetIOU_rect(bb_truck,bb_plate);
                                    cout << "ratio_palte: " << ratio_palte << endl;
                                    cout << "detection_plate.class_id: " << detection_plate.class_id << endl;
                                    if (ratio_palte > 0.9)
                                    {
                                        one_detect.vehicle_plate_images.push_back(frame(Rect(bb_plate.x, bb_plate.y, bb_plate.width, bb_plate.height)));
                                    }

                                }
                            }
                           
                            one_detect.start_time = clock();
                            one_detect.vehicle_image = frame;
                            one_detect.vehicle_location_box = box;
                            one_detect.track_id = detection.track_id;
                            one_detect.class_id = detection.class_id;
                            detection_data.push_back(one_detect);
                        }
                       
                    }
                   
                }
                

                // Detection box text
                std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
                cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
                cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

                //cv::rectangle(frame, textBox, color, cv::FILLED);
               // cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0), 2, 0);
            }

            //记录时间超过5秒，如果未抓拍到第二张违法图像，则删除该检测数据
            for (int j = 0; j < detection_data.size(); j++)
            {
              
                if (detection_data[j].flag_is == 1)
                {
                    detection_data.erase(detection_data.begin() + j);
                }
            }
            cout << "detection_data.size(): " << detection_data.size() << endl;
            cout << "output_data.size(): " << output_data.size() << endl;
            // Inference ends here...

            // This is only for preview purposes
            float scale = 0.8;
            cv::resize(frame, frame, cv::Size(frame.cols * scale, frame.rows * scale));
            cv::namedWindow("yolov8 Inference", cv::WINDOW_NORMAL);
            cv::resizeWindow("yolov8 Inference", 1600, 900);
            cv::imshow("yolov8 Inference", frame);
            writer.write(frame);
            if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
            {
                break;
            }
            
            
            //cv::waitKey(10);
        }

    }
    cap.release();
    
}

/*

int main(int argc, char** argv)
{
    if (argc != 2)
    {
       // fprintf(stderr, "Usage: %s [videopath]\n", argv[0]);
        //return -1;
    }

    //const char* videopath = "./rc5.avi";
    //VideoCapture cap("rc5.avi");
    VideoCapture cap("510_511_512_514_1697188854616_video_000013.264");
	if (!cap.isOpened())
		return 0;

	int img_w = cap.get(CAP_PROP_FRAME_WIDTH);
	int img_h = cap.get(CAP_PROP_FRAME_HEIGHT);
    int fps = cap.get(CAP_PROP_FPS);
    long nFrame = static_cast<long>(cap.get(CAP_PROP_FRAME_COUNT));
    cout << "Total frames: " << nFrame << ", fps: "<<fps<<endl;

    VideoWriter writer("demo.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(img_w, img_h));

    Mat img;
    yolox yolox;
    BYTETracker tracker(fps, 30);
    int num_frames = 0;
    int total_ms = 1;
	for (;;)
    {
        if(!cap.read(img))
            break;
        num_frames++;
       

        if (num_frames % 20 == 0)
        {
            cout << "Processing frame " << num_frames << " (" << num_frames * 1000000 / total_ms << " fps)" << endl;
        }
		if (img.empty())
			break;
        //cout<<"Processing frame "<<num_frames<<endl;
        std::vector<Object> objects;
        auto start = chrono::system_clock::now();
        yolox.detect(img, objects);
        vector<STrack> output_stracks = tracker.update(objects);
        auto end = chrono::system_clock::now();
        total_ms = total_ms + chrono::duration_cast<chrono::microseconds>(end - start).count();
        for (int i = 0; i < output_stracks.size(); i++)
		{
			vector<float> tlwh = output_stracks[i].tlwh;
			bool vertical = tlwh[2] / tlwh[3] > 1.6;
			//if (tlwh[2] * tlwh[3] > 20 && !vertical)
			{
				Scalar s = tracker.get_color(output_stracks[i].track_id);
				putText(img, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5), 
                        0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                rectangle(img, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
			}
		}
        putText(img, format("frame: %d fps: %d num: %d", num_frames, num_frames * 1000000 / total_ms, (int)output_stracks.size()),
                Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
        cv::namedWindow("YOLOX", cv::WINDOW_NORMAL);
        cv::resizeWindow("YOLOX", 1600, 900);
        cv::imshow("YOLOX", img);
        writer.write(img);
        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }
        
    }
    cap.release();
    cout << "FPS: " << num_frames * 1000000 / total_ms << endl;

    return 0;
}
*/