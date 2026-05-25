

#pragma once
#define MAVLINK_USE_MESSAGE_INFO
#include <esp_err.h>
#include <c_library_v2/common/mavlink.h>
#include <c_library_v2/mavlink_get_info.h>
#include "ryu_Types.hpp"
//#include "ryu_StateManager.hpp"
namespace Controller{
    enum class systemState_e : uint8_t; // 전방 선언
}

namespace Service{
    
class Mavlink{
    private:
        static constexpr const char* TAG = "Mavlink";
    
        struct heartbeat_t{
            volatile uint8_t         base_mode;
            volatile uint32_t        custom_mode;
            volatile Controller::systemState_e   system_status; // 시스템 상태 추가
        };

    private:
        Mavlink() = default; 
        ~Mavlink() = default;
    public:
        static Mavlink& get_instance() {
            static Mavlink instance; 
            return instance;
        }
        Mavlink(const Mavlink&) = delete;
        Mavlink& operator=(const Mavlink&) = delete;
        Mavlink(Mavlink&&) = delete;
        Mavlink& operator=(Mavlink&&) = delete;

        heartbeat_t _heartbeat;

        void send_status_text(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
        void send_mavlink_msg(mavlink_message_t *msg);
        void send_mav_command_ack(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_sysid, uint8_t target_compid);
        uint16_t map_qgc_to_ibus_final(int16_t raw_val, bool is_throttle);
        void handle_mavlink_message(mavlink_message_t *msg);

        void MAV_CMD_NAV_TAKEOFF_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_MESSAGE_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_PREFLIGHT_CALIBRATION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_SET_MESSAGE_INTERVAL_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_PROTOCOL_VERSION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
  
        static void inMessageQueueTask(void * pv);
        void StartTask();
        void on_timer_tick();
        esp_err_t initialize();
        
        bool is_initialized(){ return _initialized;};

    private:
        bool _initialized = false;
};



}