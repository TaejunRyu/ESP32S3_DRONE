#include "ryu_mavlink.h"

#include <cmath>   // float, double용 std::abs
#include <ranges>
#include <string_view>
#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 설정 및 정의
#include <freertos/timers.h>    // 소프트웨어 타이머 API 전용 헤더
//#include <driver/uart.h>
#include <lwip/sockets.h>
#include <esp_timer.h>

#include "ryu_Config.hpp"
#include "ryu_ParamTable.h"
#include "ryu_timer.hpp"
// #include "ryu_pid.h"
#include "ryu_espnow.hpp"
// #include "ryu_buzzer.h"
// #include "ryu_config.h"
#include "ryu_gps.hpp"
// #include "ryu_battery.h"
// #include "ryu_flight_task.h"
// #include "ryu_flight_event.h"
// #include "ryu_utils.h"

namespace Service{

void Mavlink::send_status_text(const char *text, uint8_t severity)
{
    mavlink_message_t msg;
    char buf[50] = {};
    strncpy(buf, text, sizeof(buf) - 1);
    // severity: MAV_SEVERITY_INFO (6), MAV_SEVERITY_WARNING (4), MAV_SEVERITY_CRITICAL (2) 등
    mavlink_msg_statustext_pack(
        _qgcinfo.mavlink.sys_id ,
       _qgcinfo.mavlink.comp_id ,
        &msg,
        severity,
        buf,0,0
    );
    send_mavlink_msg(&msg);
}

void Mavlink::send_mavlink_msg(mavlink_message_t *msg){
    Service::EspNow::get_instance().dispatch_mavlink_msg(msg);
}


void Mavlink::send_mav_command_ack(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_sysid, uint8_t target_compid)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
                    _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,    // FC의 System/Component ID
                    &msg,
                    command,                    // 응답할 명령 번호 
                    result,                     // 결과 (MAV_RESULT_ACCEPTED)
                    progress, result_param2,    // Progress, Result_param2
                    target_sysid, target_compid // Target System/Component (GCS의 ID)
    );
    send_mavlink_msg(&msg);
}


/**
 * @brief QGC raw(1000 단위) -> 퍼센트(100 단위) -> i-BUS(1000~2000) 통합 변환
 */
uint16_t Mavlink::map_qgc_to_ibus_final(int16_t raw_val, bool is_throttle) {
    float percent;
    uint16_t ibus_val;

    if (is_throttle) {
        // 1. 0~1000 -> 0~100 매핑
        percent = raw_val / 10.0f;
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;

        // 2. 0~100 -> 1000~2000 변환
        ibus_val = (uint16_t)(percent * 10.0f) + 1000;
    } else {
        // 1. -1000~1000 -> -100~100 매핑
        percent = raw_val / 10.0f;
        if (percent < -100) percent = -100;
        if (percent > 100) percent = 100;

        // 2. -100~100 -> 1000~2000 변환 (0점 1500)
        ibus_val = (uint16_t)(percent * 5.0f) + 1500;
    }

    return ibus_val;
}



void Mavlink::handle_mavlink_message(mavlink_message_t *msg)
{
    const mavlink_message_info_t *ret_msg= mavlink_get_message_info_by_id(msg->msgid);    
    
    switch (msg->msgid) {      

        case MAVLINK_MSG_ID_MANUAL_CONTROL:{
            // // x, y, z, r 값은 이미 -1000 ~ 1000 (또는 z는 0~1000) 범위입니다.
            // float y = static_cast<float>(mavlink_msg_manual_control_get_x(msg)); // Roll
            // float x = static_cast<float>(mavlink_msg_manual_control_get_y(msg)); // Pitch
            // float z = static_cast<float>(mavlink_msg_manual_control_get_z(msg)); // Throttle
            // float r = static_cast<float>(mavlink_msg_manual_control_get_r(msg)); // Yaw

            // rc_data_t m_rc;

            // // 1. Throttle (0~1000 -> 0~100)
            // m_rc.throttle = z * 0.1f; 

            // // 2. Roll/Pitch/Yaw (-1000~1000 -> -100~100)
            // m_rc.roll  = x * 0.1f;
            // m_rc.pitch = y * 0.1f;
            // m_rc.yaw   = r * 0.1f;

            // // 안전을 위한 범위 제한
            // m_rc.throttle = std::clamp(m_rc.throttle,      0.0f, 100.0f);
            // m_rc.roll     = std::clamp(m_rc.roll,       -100.0f, 100.0f);
            // m_rc.pitch    = std::clamp(m_rc.pitch,      -100.0f, 100.0f);
            // m_rc.yaw      = std::clamp(m_rc.yaw,        -100.0f, 100.0f);
            // m_rc.type     = Service::RC_QGC;
            // m_rc.receive_time = esp_timer_get_time();

            // // 이하의 숫자는 0으로 처리.....
            // // Utils::Apply_DeadZone(m_rc.roll ,2.0f);
            // // Utils::Apply_DeadZone(m_rc.pitch,2.0f);
            // // Utils::Apply_DeadZone(m_rc.yaw  ,3.0f);

            // portENTER_CRITICAL(&_qgc_lock);
            // _qgc_rc_data = m_rc;
            // portEXIT_CRITICAL(&_qgc_lock); 
            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TIME:{
            mavlink_message_t ret_msg;
            mavlink_msg_system_time_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,&ret_msg,    // 보통 1 (Autopilot)                
                0,                                                                      // Param 1: Unix time (us)
                (uint32_t)(esp_timer_get_time() / 1000)                                 // Param 2: Boot time (ms)
            );
            send_mavlink_msg(&ret_msg);
            break;
        }
        // 1. QGC가 "네가 가진 파라미터 다 내놔"라고 할 때 (연결 초기)
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: 
        {               
            mavlink_param_request_list_t req;
            mavlink_msg_param_request_list_decode(msg,&req);
            
            if (req.target_system != _qgcinfo.mavlink.sys_id  || 
                (req.target_component !=_qgcinfo.mavlink.comp_id  && req.target_component != 0)) {
                break;
            }

            auto& p_mgr = Service::ParamMgr::get_instance();

            for (size_t i = 0 ; i < p_mgr.get_param_count() ; i++){
                auto  &par = Service::params[i];

                float val_to_send;
                if (par.type == 6) { // INT32
                    int32_t temp = (int32_t)p_mgr.get_value_by_index(i);
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사i
                } else {
                    val_to_send = p_mgr.get_value_by_index(i);
                } 
                mavlink_message_t msg;
                mavlink_msg_param_value_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, 
                                             par.name.data(), val_to_send, par.type,p_mgr.get_param_count(), i);
                send_mavlink_msg(&msg);
                vTaskDelay(pdMS_TO_TICKS(3));
            }
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {   
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(msg, &req);
            if (req.target_system != _qgcinfo.mavlink.sys_id  || 
                (req.target_component !=_qgcinfo.mavlink.comp_id  && req.target_component != 0)) {
                break;
            }
            if (req.param_index != -1) 
            {
                // 전송부 코드 예시
                float val_to_send;
                auto& p_mgr = Service::ParamMgr::get_instance();

                if (Service::params[req.param_index].type == 6) { // INT32
                    
                    int32_t temp = (int32_t)p_mgr.get_value_by_index(req.param_index);
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사
                } else {
                    val_to_send = p_mgr.get_value_by_index(req.param_index);
                }
                mavlink_message_t msg;
                mavlink_msg_param_value_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, 
                                            Service::params[req.param_index].name.data(),
                                            val_to_send, 
                                            Service::params[req.param_index].type,
                                            p_mgr.get_param_count(),
                                            req.param_index);               
                send_mavlink_msg(&msg);
            } 
            break;
        }
        // 2. QGC에서 특정 값을 수정하고 "자, 이거로 바꿔!"라고 할 때
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t set;
            mavlink_msg_param_set_decode(msg, &set);
            if (set.target_system != _qgcinfo.mavlink.sys_id  || 
                (set.target_component !=_qgcinfo.mavlink.comp_id  && set.target_component != 0)) {
                break;
            }
            // [중요] 변경된 값을 다시 보내줘야 QGC 화면에서 수치가 확정됨
            float val_to_send;
            auto& p_mgr = Service::ParamMgr::get_instance();

            if(size_t index = p_mgr.find_name_index(set.param_id);index != -1){
                if (set.param_type == 6){    
                    int32_t temp = (int32_t)set.param_value; 
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사
                }
                else{
                    val_to_send = (int32_t)set.param_value;
                }
                p_mgr.update_by_index(index,val_to_send);
                // 적용 가능한 PID 계수가 있다면 즉시 동기화

                //Controller::PID::get_instance().sync_pid_from_params();

                mavlink_message_t msg;
                mavlink_msg_param_value_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, 
                                            set.param_id, val_to_send, set.param_type,p_mgr.get_param_count(),index);               
                send_mavlink_msg(&msg);
            }
            break;
        }

        // 3. 시동(ARM)이나 특정 명령을 내릴 때
        case MAVLINK_MSG_ID_COMMAND_LONG: { //76
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);       
            
            // 나에게 온것이 아니면 처리하지 않음.
            if (cmd.target_system != _qgcinfo.mavlink.sys_id  || 
                (cmd.target_component !=_qgcinfo.mavlink.comp_id  && cmd.target_component != 0)) {
                break;
            }
            switch (cmd.command){
                case MAV_CMD_COMPONENT_ARM_DISARM:{ //400
                    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);     
                    if (cmd.param1 > 0.5f && cmd.param1 < 1.5f) {
                        //esp_event_post(Event::SYS_MODE_EVENT_BASE,Event::MODE_ARM,nullptr,0,0);    
                    } else if (cmd.param1 < 0.5f) {
                        //esp_event_post(Event::SYS_MODE_EVENT_BASE,Event::MODE_DISARM,nullptr,0,0);    
                    }
                    ESP_LOGI(TAG,"MAV_CMD_COMPONENT_ARM_DISARM_func Param1: %8.5f",cmd.param1);
                    break;
                }
                case MAV_CMD_NAV_TAKEOFF:{ //22
                    MAV_CMD_NAV_TAKEOFF_func(msg,cmd);
                    break;
                }
                case MAV_CMD_DO_SET_HOME:{ //179
                    MAV_CMD_DO_SET_HOME_func(msg,cmd);
                    break;
                }
                case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:{ //520
                     MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(msg,cmd);
                     break;
                }
                case MAV_CMD_REQUEST_MESSAGE:{  //512
                    MAV_CMD_REQUEST_MESSAGE_func(msg,cmd);    
                    break;
                }
                case MAV_CMD_PREFLIGHT_CALIBRATION:{ //241  RC RADIO CALIBRATION.
                    MAV_CMD_PREFLIGHT_CALIBRATION_func(msg,cmd);
                    break;
                }
                case MAV_CMD_SET_MESSAGE_INTERVAL:{ //511
                    MAV_CMD_SET_MESSAGE_INTERVAL_func(msg,cmd);  
                    break;
                }
                case MAV_CMD_REQUEST_PROTOCOL_VERSION:{ //519
                    MAV_CMD_REQUEST_PROTOCOL_VERSION_func(msg,cmd);
                    break;
                } 
                case MAV_CMD_REQUEST_CAMERA_INFORMATION:{ //521
                    send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,100,0,msg->sysid,msg->compid);            
                    break;
                }
                case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:{
                    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,100,0,msg->sysid,msg->compid);             
                    if (std::abs(cmd.param1 - 1.0f) < 0.01f) {
                        // 1번 소리(시동 성공음 등)를 짧게 울리고 재부팅하면 상태 확인에 좋습니다.
                        //Driver::Buzzer::get_instance().sound_system_off(); 
                        esp_restart();
                    }
                    break;
                }
                default:{
                    ESP_LOGI(TAG, "QGC Message : %s , Command : %d  param1 : %f  param2 : %f ",ret_msg->name , cmd.command, cmd.param1, cmd.param2);
                    send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,100,0,msg->sysid,msg->compid);             

                }
                    
            } // end of switch
            
            break;
        }

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            mavlink_message_t ack_msg;
            mavlink_msg_mission_ack_pack(
                _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,
                &ack_msg,
                msg->sysid, msg->compid,        // 받는 사람 (GCS)
                MAV_MISSION_ACCEPTED,           // 결과: 잘 지웠어!
                MAV_MISSION_TYPE_MISSION        // 어떤 타입의 미션인지
            );
            send_mavlink_msg(&ack_msg);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            mavlink_message_t ack_msg;
            mavlink_msg_mission_count_pack(
                _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,
                &ack_msg,
                msg->sysid, msg->compid,    // 받는 사람 (GCS)
                0,                          // 미션 총 개수
                mavlink_msg_mission_request_list_get_mission_type(msg)        //MAV_MISSION_TYPE_MISSION  //미션 타입 지정
            );
            send_mavlink_msg(&ack_msg);
            break;
        }
        case MAVLINK_MSG_ID_SET_MODE:{
            // 각 모드별로 outer loop, inner loop을 처리하는 기준이 되어진다.            
            // manual, alt control, pos control, offboard, acro, rattitude, stabilize, standby, mission, return, land
            // 모드별 pid 제어가 달라진다.            
            // 기본적인것이 끝나면 처리할것.
            mavlink_set_mode_t  cmd;
            mavlink_msg_set_mode_decode(msg, &cmd);
            
            if( cmd.target_system != _qgcinfo.mavlink.sys_id ) break;
            _heartbeat.base_mode = cmd.base_mode;
            if (_heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
                

                switch (cmd.custom_mode) {
                    case (uint32_t)0x00010000: // Manual                         
                        _heartbeat.custom_mode = (uint32_t)0x00010000; //qgc용                        
                        // ENV::g_sys.flight_mode = ENV::MODE_MANUAL;                //fc용       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x00020000: // Altitude control
                        // GPS가 없거나 끊겼을 때 작동합니다. 
                        // 기체의 고도와 바라보는 방향(Yaw Heading)은 그 자리에 고정(Hold)되지만, 
                        // 수평 위치는 바람에 밀릴 수 있습니다
                        _heartbeat.custom_mode = (uint32_t)0x00020000;
                        // ENV::g_sys.flight_mode = ENV::MODE_ALTCTL;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x00030000: // position control
                        // 가장 대표적인 홀드 모드입니다. 
                        // 키를 놓으면 회전 각도(Yaw Hold)뿐만 아니라 GPS를 기반으로 위도, 경도, 고도까지 그 자리에 칼같이 고정합니다.
                        _heartbeat.custom_mode = (uint32_t)0x00030000;
                        // ENV::g_sys.flight_mode = ENV::MODE_POSCTL;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x00040000: // Offboard
                        _heartbeat.custom_mode = (uint32_t)0x00040000;
                        // ENV::g_sys.flight_mode = ENV::MODE_OFFBOARD;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x00050000: // Acro
                        _heartbeat.custom_mode = (uint32_t)0x00050000;
                        // ENV::g_sys.flight_mode = ENV::MODE_ACRO;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x00060000: // rattitude
                        _heartbeat.custom_mode = (uint32_t)0x00060000;
                        // ENV::g_sys.flight_mode = ENV::MODE_MANUAL;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;    
                    case (uint32_t)0x00070000: // Stabilize
                        _heartbeat.custom_mode = (uint32_t)0x00070000;
                        // ENV::g_sys.flight_mode = ENV::MODE_STABILIZED;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x03040000: // standby
                        // 비행 중 QGC 패널에서 대기(Hold/Loiter) 버튼을 누르면 
                        // 기체가 자동으로 조종권을 가져가서 현재 위치와 방위각을 유지하며 제자리 비행을 합니다.
                        _heartbeat.custom_mode = (uint32_t)0x03040000;
                        // ENV::g_sys.flight_mode = ENV::MODE_MANUAL;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_USER_HOLD_MODE;
                        break;
                    case (uint32_t)0x04040000: // Mission
                        _heartbeat.custom_mode = (uint32_t)0x04040000;
                        // ENV::g_sys.flight_mode = ENV::MODE_MISSION;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break; 
                    case (uint32_t)0x05040000: // Return 
                        _heartbeat.custom_mode = (uint32_t)0x05040000;
                        // ENV::g_sys.flight_mode = ENV::MODE_RTL;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                    case (uint32_t)0x09040000: // Land
                        _heartbeat.custom_mode = (uint32_t)0x09040000;
                        // ENV::g_sys.flight_mode = ENV::MODE_PRECISION_LAND;                       
                        // ENV::g_sys.hold_mode = ENV::flight_hold_mode::MODE_NORMAL;
                        break;
                }
            }
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_SET_MODE custom mode: 0x%08X", cmd.custom_mode);
            break;
        }
        default:{            
           //ESP_LOGI(TAG, "SWITCH default msgid: %u (%s)", msg->msgid, ret_msg ? ret_msg->name : "Unknown");
        }
        
    }    
}



// 파라미터	명칭	설명
// Param 1	Pitch	이륙 시 유지할 최소 피치 각도 (단위: 도, Degree). 기체가 상승하며 앞/뒤로 기울어지는 정도를 제어합니다.
// Param 2	Empty	비어 있음 (사용되지 않음).
// Param 3	Empty	비어 있음 (사용되지 않음).
// Param 4	Yaw	이륙 시 유지할 방향 (단위: 도). 보통 현재 헤딩(방향)을 유지하려면 NaN 혹은 0을 사용합니다.
// Param 5	Latitude	이륙 지점의 위도 (Target Latitude). 0이면 현재 위치를 사용합니다.
// Param 6	Longitude	이륙 지점의 경도 (Target Longitude). 0이면 현재 위치를 사용합니다.
// Param 7	Altitude	이륙 목표 고도 (단위: 미터, m). 지면으로부터의 상대 고도(Relative Altitude)를 의미합니다.
// 각 파라메터의 값에 따라 takeoff시에 고도를 얼마에 유지하면 안정적인 상황에서 대기 상태를 유지할 수 있을지 결정하는데 사용됩니다.
void Mavlink::MAV_CMD_NAV_TAKEOFF_func(mavlink_message_t *msg, mavlink_command_long_t cmd){    
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);    
    // float tmp_yaw = cmd.param4; // yaw 각도 (deg)
    // float tmp_lat = cmd.param5; // 위도 (deg)
    // float tmp_lon = cmd.param6; // 경도 (deg)
    // float tmp_alt = cmd.param7; // 고도 (m)
    ESP_LOGI(TAG,"MAV_CMD_NAV_TAKEOFF_func MSG : (%d), CMD : (%d), PARAM1 :(%f), PARAM4 : (%f), PARAM5 : (%f), PARAM6 : (%f), PARAM7 : (%f)",
        msg->msgid,cmd.command,cmd.param1,cmd.param4,cmd.param5,cmd.param6,cmd.param7);

}


void Mavlink::MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    auto& gps = Sensor::Gps::get_instance();
    if (cmd.param1 == 1){
        // Param 1이 1이면 현재 센서(GPS) 위치를 홈으로 설정
        // ENV::qgc_home_pos.lat = gps.share_gps.lat;
        // ENV::qgc_home_pos.lon = gps.share_gps.lon;
        // ENV::qgc_home_pos.alt = gps.share_gps.alt;
    }
    else
    {
        // Param 1이 0이면 전달받은 파라미터로 설정
        // ENV::qgc_home_pos.lat = cmd.param5;
        // ENV::qgc_home_pos.lon = cmd.param6;
        // ENV::qgc_home_pos.alt = cmd.param7;
    }
    // ENV::qgc_home_pos.is_set = true;

    //처리결과 송싱 해야함...
    
}

void Mavlink::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);

    // 1. ACK 패킹 및 전송
    // 2. 버전 정보(AUTOPILOT_VERSION) 설정
    mavlink_autopilot_version_t version = {};
    // 핵심: 지원하는 기능을 비트로 나열
    version.capabilities = 
                        MAV_PROTOCOL_CAPABILITY_MAVLINK2 | 
                        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                        //MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                        MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
    version.board_version = 1;       
    version.flight_sw_version = 0x010D0000; // v1.13.0 (PX4 스타일 버전 넘버링)
    version.middleware_sw_version = 0x010D0000;
    version.os_sw_version = 0x00000000; // FreeRTOS 등 사용 시 기입 가능
    version.vendor_id = 0x1234; // 필요 시 수정
    version.product_id = 0x5678;
    // 필요 시 여기에 UID 등을 추가로 채웁니다.

    // 3. 버전 정보 패킹 및 전송
    mavlink_message_t ver_msg;
    mavlink_msg_autopilot_version_encode(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &ver_msg, &version);
    send_mavlink_msg(&ver_msg);

}

//QGC에서 MESSAGE : 76 , Command : 512 
void Mavlink::MAV_CMD_REQUEST_MESSAGE_func(mavlink_message_t *msg, mavlink_command_long_t cmd)  
{
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    uint32_t requested_id = static_cast<uint32_t>(cmd.param1);
    switch (requested_id)
    {
    case MAVLINK_MSG_ID_PROTOCOL_VERSION: // 519
        // 1. ACK 패킹 및 전송
        mavlink_message_t ack_msg;
        mavlink_msg_protocol_version_pack(
            _qgcinfo.mavlink.sys_id ,           // 내 FC 시스템 ID (보통 1)
           _qgcinfo.mavlink.comp_id ,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
            &ack_msg,
            200,                 // version: MAVLink 2.0 (200)
            100,                 // min_hw_version: 최소 지원 버전 (100)
            200,                 // max_hw_version: 최대 지원 버전 (200)
            0,                   // spec_version_hash: 보통 0 (또는 git hash의 일부)
            0                    // library_version_hash: 보통 0 (또는 git hash의 일부)
        );
        send_mavlink_msg(&ack_msg);
        break;
    case 280: case 259: case 148: case 435: case 397: case 395:
        send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);
        break;
    default:
        send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);
           ESP_LOGI(TAG, "MAV_CMD_REQUEST_MESSAGE_func QGC Message : %d , Command : %d , Request Id: %d",msg->msgid , cmd.command , requested_id);
    }
    
}

void Mavlink::MAV_CMD_PREFLIGHT_CALIBRATION_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    bool calibrate_gyro   = (cmd.param1 == 1.0f);
    bool calibrate_mag    = (cmd.param2 == 1.0f);
    bool calibrate_level  = (cmd.param3 == 1.0f);
    bool calibrate_accel  = (cmd.param4 == 1.0f);
    bool calibrate_airspeed = (cmd.param5 == 1.0f);
    
    send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);


    // TODO: 센서 캘리브레이션 요청 처리 구현
    if (calibrate_gyro || calibrate_mag || calibrate_level || calibrate_accel || calibrate_airspeed) {
        ESP_LOGI(TAG, "MAV_CMD_PREFLIGHT_CALIBRATION_func: gyro=%d mag=%d level=%d accel=%d airspeed=%d",
                 calibrate_gyro, calibrate_mag, calibrate_level, calibrate_accel, calibrate_airspeed);
    }
}



void Mavlink::MAV_CMD_SET_MESSAGE_INTERVAL_func(mavlink_message_t *msg, mavlink_command_long_t cmd) {
    // Param 1: 메시지 ID (예: MAVLINK_MSG_ID_ATTITUDE 등)
    // Param 2: 전송 간격 (마이크로초, us 단위)
    // -1: 전송 중지
    // 0: 기본 간격 사용
    // 100000: 100,000us = 0.1초 (10Hz)
    uint32_t msg_id = static_cast<uint32_t>(cmd.param1);
    float interval_us = cmd.param2;

    static bool imu_data_sending = false;
    static bool att_data_sending = false;
    static bool local_pos_data_sending = false;
    static bool pos_target_data_sending = false;
    
    send_mav_command_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);


    switch (msg_id) {
        case 83: // MAVLINK_MSG_ID_RAW_IMU
            imu_data_sending = (interval_us > 0.0f);
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_RAW_IMU interval set to %f us, sending: %s", interval_us, imu_data_sending ? "ON" : "OFF");
            break;
        case 31: // MAVLINK_MSG_ID_ATTITUDE_QUATERNION
            att_data_sending = (interval_us > 0.0f);
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_ATTITUDE_QUATERNION interval set to %f us, sending: %s", interval_us, att_data_sending ? "ON" : "OFF");
            break;
        case 32: // MAVLINK_MSG_ID_LOCAL_POSITION_NED
            local_pos_data_sending = (interval_us > 0.0f);
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_LOCAL_POSITION_NED interval set to %f us, sending: %s", interval_us, local_pos_data_sending ? "ON" : "OFF");
            break;
        case 85: // MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
            pos_target_data_sending = (interval_us > 0.0f);
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED interval set to %f us, sending: %s", interval_us, pos_target_data_sending ? "ON" : "OFF");
            break;
        default:
            break;
    }
}

void Mavlink::MAV_CMD_REQUEST_PROTOCOL_VERSION_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    mavlink_message_t ack_msg;
    mavlink_msg_protocol_version_pack(
        _qgcinfo.mavlink.sys_id ,           // 내 FC 시스템 ID (보통 1)
       _qgcinfo.mavlink.comp_id ,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
        &ack_msg,
        200,                 // version: MAVLink 2.0 (200)
        100,                 // min_hw_version: 최소 지원 버전 (100)
        200,                 // max_hw_version: 최대 지원 버전 (200)
        0,                   // spec_version_hash: 보통 0 (또는 라이브러리 생성 해시)
        0                    // library_version_hash: 보통 0
    );
    send_mavlink_msg(&ack_msg);
}



void Mavlink::on_timer_tick()
{
    static uint8_t step = 0;
    mavlink_message_t msg;
    // 10hz로 구분하고 있으므로 매번 처리...
    mavlink_msg_attitude_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, esp_timer_get_time()/1000, 
                                                _attitude.roll          * DEG_TO_RAD, 
                                                _attitude.pitch         * DEG_TO_RAD, 
                                                _attitude.yaw           * DEG_TO_RAD, 
                                                _attitude.roll_speed    * DEG_TO_RAD, 
                                                _attitude.pitch_speed   * DEG_TO_RAD, 
                                                _attitude.yaw_speed     * DEG_TO_RAD 
                                    );
    send_mavlink_msg(&msg);

    static Sensor::Gps::gps_data_t m_gps={};
    static uint32_t last_itow = 0;     // 마지막으로 전송한 iTOW 저장


    // 문제가 발생하면 xSemaphoreTake 이 부분일것 같음.
    switch(step){
        case 1: case 8: case 9:{
            // g_gps 공동 변수에서 읽어와 사용한다.
            // 정확한 데이터 보장이 필요하기 때문에 시간별 다름을 없애는 목적......
            // gps시간이 다르면 복사하고 아니면 이전 데이터 사용

            auto& gps = Sensor::Gps::get_instance();
            if (xSemaphoreTake(gps.xGpsMutex, 0 )== pdTRUE) { 
                if (gps.share_gps.iTOW != last_itow){
                    m_gps       = gps.share_gps;
                    last_itow   = gps.share_gps.iTOW;         
                    m_gps.last_update_tick = xTaskGetTickCount();      
                }
                xSemaphoreGive(gps.xGpsMutex);
            }
        }
    }

    switch (step) {
        case 0:{ // 하트비트 전송
            mavlink_msg_heartbeat_pack(_qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,&msg, 
                                        MAV_TYPE_QUADROTOR, 
                                        //MAV_AUTOPILOT_GENERIC,
                                        MAV_AUTOPILOT_PX4,
                                        _heartbeat.base_mode,  
                                        _heartbeat.custom_mode, 
                                        3);
            send_mavlink_msg(&msg);
            break;
        }
        case 3:{ // 시스템 상태 전송 (배터리, 전압 등)
            uint32_t sensors_present = 
                        MAV_SYS_STATUS_SENSOR_3D_ACCEL | 
                        MAV_SYS_STATUS_SENSOR_3D_ACCEL2 | // 두 번째 IMU 가속도
                        MAV_SYS_STATUS_SENSOR_3D_GYRO |
                        MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                        MAV_SYS_STATUS_SENSOR_3D_MAG |
                        MAV_SYS_STATUS_SENSOR_3D_MAG2| 
                        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE  |
                        MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE|
                        MAV_SYS_STATUS_AHRS|
                        MAV_SYS_STATUS_SENSOR_BATTERY|
                        MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                        ;   // 두 번째 IMU 자이로
            uint32_t sensors_enabled = sensors_present; // 모두 활성화됨
            uint32_t sensors_health  = sensors_present;  // 모두 정상(Healthy)

            // 1. CPU Load 계산 (0 ~ 1000 사이의 값으로 변환)
            // 로그상 1700us / 2500us 라면 약 680이 됨
            // auto& flight = Controller::Flight::get_instance();
            uint16_t load =0;// (uint16_t)((flight.total_us * 1000) / Controller::Flight::INTERVAL_US);
            
            // auto& bat = Driver::Battery::get_instance();
            // 2. 배터리 가짜 데이터 (12.6V, 10.5A, 85% 잔량)
            uint16_t battery_voltage = 0;//(uint16_t)(bat.get_battery_voltage() * 1000.0f); //mv
            int16_t current_battery = 1050;   // [10mA 단위, 즉 10.5A]
            int8_t battery_remaining = 85;    // [%]
            uint16_t comms_drop_rate = 0;     // 통신 패킷 드랍률 (0.01% 단위)
            uint16_t comms_errors = 0;        // 통신 에러 횟수
            
            // 시스템 상태 패킷 구성 예시
            mavlink_msg_sys_status_pack(
                _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, 
                sensors_present, sensors_enabled, sensors_health,         // 센서 상태 비트마스크
                load,        // CPU Load (0~1000)
                battery_voltage, current_battery, battery_remaining, 
                comms_drop_rate, comms_errors, 0, 0, 0, 0,0,0,0
            );
            send_mavlink_msg(&msg);
            break;
        }    
        case 6:{ // 라디오 상태 전송 (RSSI, Noise)
             mavlink_msg_radio_status_pack_chan(
                            _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id ,MAVLINK_COMM_1, &msg, 
                            Service::EspNow::get_instance().current_rssi, // 드론이 받은 브릿지의  신호
                            0,0, Service::EspNow::get_instance().noise_floor, 0, 0, 0);
            send_mavlink_msg(&msg);
            break;
        }
        case 9:{ // gps 정보 
            if (m_gps.home_alt > -9000.0f && m_gps.fixType >= 3) {
                mavlink_msg_gps_raw_int_pack(
                        _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, 
                        esp_timer_get_time() / 1000,               
                        m_gps.fixType,                              // 실제 Fix 타입을 그대로 전달 (0~4)                     
                        static_cast<int32_t>(m_gps.lat * 1e7),      // 위도
                        static_cast<int32_t>(m_gps.lon * 1e7),      // 경도
                        static_cast<int32_t>(m_gps.hMSL), // 해발 고도 (MSL, mm)
                        static_cast<uint16_t>(m_gps.pDOP),          
                        static_cast<uint16_t>(m_gps.pDOP),          // VDOP 대신 pDOP 사용 가능
                        static_cast<uint16_t>(m_gps.gSpeed),        // 지표속도
                        static_cast<uint16_t>(m_gps.headMot),       // 이동방향
                        static_cast<uint8_t>(m_gps.sats),           // 위성수
                        static_cast<int32_t>(m_gps.height),         // alt_ellipsoid (mm 단위 그대로)
                        m_gps.hAcc,                                 // 수평 정확도 (mm)
                        m_gps.vAcc,                                 // 수직 정확도 (mm)
                        m_gps.sAcc,                                 // 속도 정확도 (mm/s)
                        0,                                          // hdg_acc – [degE5] Heading / track uncertainty
                        static_cast<uint16_t>(_attitude.yaw * 100.0f) // yaw (cdeg 단위로 변환)
                    );
                    send_mavlink_msg(&msg);
            }
            break;
        }
        case 1: case 8:{
            //if (m_gps.home_alt > -9000.0f && m_gps.fixType >= 3) {
                //현재고도
                int32_t alt_msl = static_cast<int32_t>(m_gps.hMSL );
                
                // 상대 고도 (Relative) mm 단위
                int32_t alt_rel = static_cast<int32_t>((m_gps.hMSL - m_gps.home_alt) );
                
                mavlink_msg_global_position_int_pack(
                    _qgcinfo.mavlink.sys_id ,_qgcinfo.mavlink.comp_id , &msg, esp_timer_get_time()/1000,
                    static_cast<int32_t>(m_gps.lat * 1e7), 
                    static_cast<int32_t>(m_gps.lon * 1e7),
                    static_cast<int32_t>(alt_msl),      // 해수면 고도
//                    static_cast<int32_t>(ENV::g_altitude.current),      // 이것은 기압계로 측정한 고도 => g_baro.filtered_altitude * 1000.0f),
                    static_cast<int32_t>(alt_rel),      // 이것은 기압계로 측정한 고도 => g_baro.filtered_altitude * 1000.0f),
                    static_cast<int16_t>(m_gps.velN),   // 단위(cm/s) gps에서 데이터를 받아 처리 VGT문장에서 받으면 된다.
                    static_cast<int16_t>(m_gps.velE),
                    static_cast<int16_t>(m_gps.velD),
                    static_cast<int16_t>(_attitude.yaw * 100.0f)
                );

                    send_mavlink_msg(&msg);
            //}
            break;
        }
        case 2:
        case 4:
        case 5:
        case 7:
            break;
    }
    // 0 -> 1 -> 2 순환
    step = (step + 1) % 10;  // 100ms단위로 실행함. 1초를 10개로 나뉘어서 처리.
}

esp_err_t Mavlink::initialize()
{
    _qgcinfo.mavlink.sys_id = 1;
    _qgcinfo.mavlink.comp_id = 1;
    
    _heartbeat ={
            .base_mode =    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | //MAV_MODE_FLAG_TEST_ENABLED    |    // 테스트 모드 (실제 비행에서는 사용 안 함)
                            MAV_MODE_FLAG_STABILIZE_ENABLED  |  // 자세 제어 활성화
                            MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // 원격제어 활성화
            .custom_mode = 0x00070000 // PX4 STABILIZE 모드: 0x00070000 (Main Mode 7) + 0x00000000 (Sub Mode 0)
    };

    // timer의 callback과 연결하여 on_timer_tick를 타이머에의해서 실행함.
    auto& timer = Service::Timer::get_instance();
    timer.set_timer_callback([this](){on_timer_tick();});
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

} // namespace MAV