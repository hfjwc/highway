#include "truckoccupylane.h"
#include "truckdetector.h"

namespace TruckCVLibrary
{
    Truckoccupylane* Functions::CreateDetector()
    {
        // create object
        //        std::shared_ptr<Truckoccupylane> ptr = std::make_shared<truckDetector>();
        //        return ptr;
        return new truckDetector();
    }

    PV_ERROR_CODE Functions::inference_test(Truckoccupylane* obj, std::string para_data, vector<json>& results_json)
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

}   // namespace TruckCVLibrary
