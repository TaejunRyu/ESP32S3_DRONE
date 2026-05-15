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



    public:
        Utils::SharedDataManager *_data_manager;
        Sensor::ICM20948         *_icm20948;
        Utils::ImuCalibrator     *_calibrator;


        static void ReadSensorTask(void* pvParameters);
        void StartTask();


};



}