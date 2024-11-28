#include "opencv2/freetype.hpp"
#include "truckoccupylane.h"
#include <QCoreApplication>
#include <iostream>
#include <string>
#include <unistd.h>
// using namespace TruckCVLibrary;
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    TruckCVLibrary::Functions myfunctions;
    Truckoccupylane*          truckDetector = myfunctions.CreateDetector();
    /*string bmodel_file = "yolov8s_5.10.bmodel";
    //creat handle
    int dev_id = 0;
    BMNNHandlePtr handle = make_shared<BMNNHandle>(dev_id);
    //PV_ERROR_CODE rc1 = truckDetector->loadmodel(handle, bmodel_file);*/
    // open stream
    // cv::VideoCapture cap("rtsp://admin:Admin_123@192.168.0.59/");
    cv::VideoCapture cap("output2.mp4");
    if (!cap.isOpened())
    {
        cout << "open stream failed!" << endl;
        exit(1);
    }
    // get resolution
    int               w             = int(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int               h             = int(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    float             output_score  = 0.5;
    int               time_interval = 1000;
    vector<cv::Point> contour;
    /*std::string       parameter = "1,9,1122,3,1466,1,1397,214,1429,528,1561,908,1643,1077,713,1077,819,524,959,153,/data/lj/"
                            "1.jpg,2,2000,0,0.8,1,0.8,0,0.8,0.8,5xdad324,4312,46081,4608,1101,1223,江苏省南京市,1|2,11,758,6,596,116,423,263,181,529,3,748,4,1066,706,1076,781,674,881,334,1028,96,"
                            "1116,3,10,1124,9,1016,108,919,263,786,608,716,1074,1631,1069,1491,698,1403,356,1418,93,1474,8,/data2/camera-handler/python-server/images/20240606/"
                            "1798650844510576641_1717667065541.jpg,3,2000,0,0.8,0,0.0,0,0.0,0.0,publicstaticvoidgo,4312,4608,1094,1101,1223,金马路地铁站,2";*/
    /*std::string parameter = "2,11,758,6,596,116,423,263,181,529,3,748,4,"
                            "1066,706,1076,781,674,881,334,1028,96,1116,3,10,1124,9,1016,108,919,263,786,608,716,1074,1631,1069,1491,698,1403,356,1418,93,1474,8,/data/lj/"
                            "1.jpg,3,2000,0,0.8,0,0.0,0,0.0,0.0,publicstaticvoidgo,4312,4608,1094,1101,1223,江苏省南京市,1797456648193921026";*/
    //    std::string parameter =
    //    "2,9,2529,843,2369,999,2136,1416,2019,1939,1976,2149,3303,2133,2983,1566,2933,1183,3013,853,10,2009,856,1843,956,1549,1173,1199,1516,983,1766,976,2146,1963,2133,2086,1519,"
    //                            "2246,1183,2523,853,/data/lj/images/1.jpg,3,2000,0,0.8,1,0.0,3,0.1,0.1,publicstaticvoidgo,4312,4608,1094,1101,1223,江苏省南京市,1797456648193921026";

    std::string parameter = "1,12,1119,3,1031,91,926,238,856,394,776,681,711,1069,1636,1064,1498,784,1406,468,1394,199,1434,58,1466,6,/data/lj/images/"
                            "1.jpg,1,2000,0,0.8,0,0.0,3,0.1,0.1,publicstaticvoidgo,4312,4608,1094,1101,1223,金马路地铁站,1798650844510576641";
    // for (int c = 0; c < 10000; c++)
    while (true)
    {
        // get one frame from decoder
        cv::Mat img;
        cap.read(img);
        usleep(5);
        imwrite("/data/lj/images/1.jpg", img);
        usleep(5);

        //        cv::Mat img1;
        //        cap1.read(img1);
        //        imwrite("/data/lj/2.jpg", img1);
        /*bm_image bmimage;
        bm_status_t bmret = cv::bmcv::toBMI(img, &bmimage, true);
        if (bmret != BM_SUCCESS) {
            return PV_ERROR_EXCEPTION;
        }*/

        double begin = (double)cv::getTickCount();
        double time1 = begin * 1000 / cv::getTickFrequency();
        //(2)detect
        vector<json> results_json;
        // PV_ERROR_CODE rc = truckDetector->inference(img, output_score, time_interval, contour, parameter, results_json);
        // PV_ERROR_CODE rc = myfunctions.inference_test(truckDetector, parameter, results_json);

        PV_ERROR_CODE rc = truckDetector->inference_test(parameter, results_json);

        double end   = (double)cv::getTickCount();
        double time2 = end * 1000 / cv::getTickFrequency();
        double time  = (end - begin) * 1000 / cv::getTickFrequency();
        cout << "time: " << time << endl;
    }
    // delete truckDetector;
    return 0;
}
