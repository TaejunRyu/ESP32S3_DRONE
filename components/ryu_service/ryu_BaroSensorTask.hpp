#pragma once


namespace Sensor{
    class ICM20948;
}
namespace Utils{
    class SharedDataManager;
    class ImuCalibrator;
}


namespace Service{

class BaroTask{
    private:
        static constexpr const char* TAG = "BaroTask";



    public:
        static void ReadBaroTask(void *pvParameters);
        void StartTask();
};
}