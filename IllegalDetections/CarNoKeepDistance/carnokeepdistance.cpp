#include "carnokeepdistance.h"
#include "cardetector.h"

CarNoKeepDistance* Functions::CreateDetector()
{
    // create object
    //    std::shared_ptr<CarNoKeepDistance> ptr = std::make_shared<cardetector>();
    //    return ptr;
    return new cardetector();
}

PV_ERROR_CODE Functions::inference_test(CarNoKeepDistance* obj, std::string para_data, vector<json>& results_json)
{
    try
    {
        return obj->inference_test(para_data, results_json);
    }
    catch (...)
    {
        return PV_ERROR_EXCEPTION;
    }
}

// namespace CarCVLibrary
