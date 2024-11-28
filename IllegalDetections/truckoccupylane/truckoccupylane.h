#ifndef TRUCKOCCUPYLANE_H
#define TRUCKOCCUPYLANE_H

#include "common.h"
#include "json.hpp"
#include "opencv2/opencv.hpp"
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>
using json = nlohmann::json;
using namespace std;
using namespace cv;

class Truckoccupylane
{
public:
    virtual PV_ERROR_CODE inference_test(std::string para_data, vector<json>& results_json){};
};

////////////////////////////////////////////////////////////////
// library functions
////////////////////////////////////////////////////////////////
namespace TruckCVLibrary
{
    class Functions
    {
    public:
        /////////////////////////////////////////////////////////////////////////////////
        // Factory function for creating the detector object
        // static std::shared_ptr<Truckoccupylane> CreateDetector();
        Truckoccupylane*     CreateDetector();
        static PV_ERROR_CODE inference_test(Truckoccupylane* obj, std::string para_data, vector<json>& results_json);
    };

}   // namespace TruckCVLibrary
#endif   // TRUCKOCCUPYLANE_H
