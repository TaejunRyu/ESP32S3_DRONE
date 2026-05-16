#pragma once


namespace Sensor{
    class ICM20948;
}
namespace Utils{
    class SharedDataManager;
    class ImuCalibrator;
}

namespace Service{

class SensorTask{
    private:
        Utils::SharedDataManager *_data_manager;
        Sensor::ICM20948         *_icm20948;
    public:
        static void ReadSensorTask(void* pvParameters);
        void StartTask();
};

}//namespace Service