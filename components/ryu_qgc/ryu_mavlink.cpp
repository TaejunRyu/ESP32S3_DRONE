#include "ryu_mavlink.hpp"

#include <cmath>   // float, double용 std::abs
#include <ranges>
#include <string_view>
#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 설정 및 정의
#include <freertos/timers.h>    // 소프트웨어 타이머 API 전용 헤더
#include <lwip/sockets.h>
#include <esp_timer.h>

#include "ryu_Config.hpp"
#include "ryu_ParamTable.hpp"
#include "ryu_timer.hpp"
#include "ryu_espnow.hpp"
#include "ryu_gps.hpp"
#include "ryu_battery.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_StateManager.hpp"
#include "ryu_Types.hpp"

namespace Service{

void Mavlink::send_status_text(const char *text, uint8_t severity)
{
    mavlink_message_t msg;
    char buf[50] = {};
    strncpy(buf, text, sizeof(buf) - 1);
    // severity: MAV_SEVERITY_INFO (6), MAV_SEVERITY_WARNING (4), MAV_SEVERITY_CRITICAL (2) 등
    mavlink_msg_statustext_pack(
        ConfigMavlink::sys_id ,
       ConfigMavlink::comp_id ,
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
                    ConfigMavlink::sys_id ,ConfigMavlink::comp_id ,    // FC의 System/Component ID
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

            // 이거는 flysky에서 데이터가 들어올때 SharedDataManager에 변수만 체크할수 있도록 변경해야 할것임.
            // 실제 데이터를 참조하는 것은 데이터의 복사하는 과정이 있기때문에 안됨. (수정할것.)
            rc_data_t rc_data = Controller::SharedDataManager::getInstance().get_shared_data<Controller::Data_type::DT_RC_DATA>();
            if (rc_data.type == RC_FLYSKY) {
                // flysky 모드에서는 QGC에서 오는 RC 입력을 무시하도록 처리 (우선순위: flysky > QGC)
                break;
            }

            // x, y, z, r 값은 이미 -1000 ~ 1000 (또는 z는 0~1000) 범위입니다.
            float y = static_cast<float>(mavlink_msg_manual_control_get_x(msg)); // Roll
            float x = static_cast<float>(mavlink_msg_manual_control_get_y(msg)); // Pitch
            float z = static_cast<float>(mavlink_msg_manual_control_get_z(msg)); // Throttle
            float r = static_cast<float>(mavlink_msg_manual_control_get_r(msg)); // Yaw

            rc_data_t m_rc;

            // 1. Throttle (0~1000 -> 0~100)
            m_rc.throttle = z * 0.1f; 

            // 2. Roll/Pitch/Yaw (-1000~1000 -> -100~100)
            m_rc.roll  = x * 0.1f;
            m_rc.pitch = y * 0.1f;
            m_rc.yaw   = r * 0.1f;

            // 안전을 위한 범위 제한
            m_rc.throttle = std::clamp(m_rc.throttle,      0.0f, 100.0f);
            m_rc.roll     = std::clamp(m_rc.roll,       -100.0f, 100.0f);
            m_rc.pitch    = std::clamp(m_rc.pitch,      -100.0f, 100.0f);
            m_rc.yaw      = std::clamp(m_rc.yaw,        -100.0f, 100.0f);
            m_rc.type     = RC_QGC;

            // 이하의 숫자는 0으로 처리.....
            m_rc.roll  = (std::abs(m_rc.roll) < 2.0f)   ? 0.0f : m_rc.roll;
            m_rc.pitch = (std::abs(m_rc.pitch) < 2.0f)  ? 0.0f : m_rc.pitch;
            m_rc.yaw   = (std::abs(m_rc.yaw) < 3.0f)    ? 0.0f : m_rc.yaw;

            Controller::SharedDataManager::getInstance().publish_data<Controller::Data_type::DT_RC_DATA>(m_rc);
            Controller::SharedDataManager::getInstance().set_rc_updated(true); // RC 데이터 업데이트 플래그 세트

            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TIME:{
            mavlink_message_t ret_msg;
            mavlink_msg_system_time_pack(ConfigMavlink::sys_id ,ConfigMavlink::comp_id ,&ret_msg,    // 보통 1 (Autopilot)                
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
            
            if (req.target_system != ConfigMavlink::sys_id  || 
                (req.target_component !=ConfigMavlink::comp_id  && req.target_component != 0)) {
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
                mavlink_msg_param_value_pack(ConfigMavlink::sys_id ,ConfigMavlink::comp_id , &msg, 
                                             par.name.data(), val_to_send, par.type,p_mgr.get_param_count(), i);
                send_mavlink_msg(&msg);
                vTaskDelay(pdMS_TO_TICKS(3));
            }
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {   
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(msg, &req);
            if (req.target_system != ConfigMavlink::sys_id  || 
                (req.target_component !=ConfigMavlink::comp_id  && req.target_component != 0)) {
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
                mavlink_msg_param_value_pack(ConfigMavlink::sys_id ,ConfigMavlink::comp_id , &msg, 
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
            if (set.target_system != ConfigMavlink::sys_id  || 
                (set.target_component !=ConfigMavlink::comp_id  && set.target_component != 0)) {
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
                mavlink_msg_param_value_pack(ConfigMavlink::sys_id ,ConfigMavlink::comp_id , &msg, 
                                            set.param_id, val_to_send, set.param_type,p_mgr.get_param_count(),index);               
                send_mavlink_msg(&msg);
            }
            break;
        }

        // 3. 시동(ARM)이나 특정 명령을 내릴 때
        case MAVLINK_MSG_ID_COMMAND_LONG: { //76
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);                   
            if (cmd.target_system != ConfigMavlink::sys_id  || 
                (cmd.target_component !=ConfigMavlink::comp_id  && cmd.target_component != 0)) {
                break;
            }

            switch (cmd.command){
                case MAV_CMD_COMPONENT_ARM_DISARM:{ //400
                    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);     
                    if (cmd.param1 > 0.5f && cmd.param1 < 1.5f) {
                        Controller::DroneStatusManager::getInstance().setArmed(true); // 시동
                        Controller::DroneStatusManager::getInstance().setSystemState(systemState_e::SYS_STATE_ACTIVE);  
                        _heartbeat.base_mode  |= MAV_MODE_FLAG_SAFETY_ARMED;
                        _heartbeat.system_status = systemState_e::SYS_STATE_ACTIVE; // 시스템 상태를 Active로 업데이트
                    } else if (cmd.param1 < 0.5f) {
                        Controller::DroneStatusManager::getInstance().setArmed(false);
                        Controller::DroneStatusManager::getInstance().setSystemState(systemState_e::SYS_STATE_STANDBY); // 시스템 상태를 Standby로 업데이트
                        _heartbeat.base_mode  &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                        _heartbeat.system_status = systemState_e::SYS_STATE_STANDBY; // 시스템 상태를 Standby로 업데이트
                    }
                    ESP_LOGI(TAG,"_heartbeat.system_status :%d ",_heartbeat.system_status);
                    break;
                }
                case MAV_CMD_NAV_TAKEOFF:{ //22
                    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);    



                    // 파라미터	명칭	설명
                    // Param 1	Pitch	이륙 시 유지할 최소 피치 각도 (단위: 도, Degree). 기체가 상승하며 앞/뒤로 기울어지는 정도를 제어합니다.
                    // Param 2	Empty	비어 있음 (사용되지 않음).
                    // Param 3	Empty	비어 있음 (사용되지 않음).
                    // Param 4	Yaw	이륙 시 유지할 방향 (단위: 도). 보통 현재 헤딩(방향)을 유지하려면 NaN 혹은 0을 사용합니다.
                    // Param 5	Latitude	이륙 지점의 위도 (Target Latitude). 0이면 현재 위치를 사용합니다.
                    // Param 6	Longitude	이륙 지점의 경도 (Target Longitude). 0이면 현재 위치를 사용합니다.
                    // Param 7	Altitude	이륙 목표 고도 (단위: 미터, m). 지면으로부터의 상대 고도(Relative Altitude)를 의미합니다.
                    // 각 파라메터의 값에 따라 takeoff시에 고도를 얼마에 유지하면 안정적인 상황에서 대기 상태를 유지할 수 있을지 결정하는데 사용됩니다.
                    // float tmp_yaw = cmd.param4; // yaw 각도 (deg)
                    // float tmp_lat = cmd.param5; // 위도 (deg)
                    // float tmp_lon = cmd.param6; // 경도 (deg)
                    // float tmp_alt = cmd.param7; // 고도 (m)
                    ESP_LOGI(TAG,"MAV_CMD_NAV_TAKEOFF_func MSG : (%d), CMD : (%d), PARAM1 :(%f), PARAM4 : (%f), PARAM5 : (%f), PARAM6 : (%f), PARAM7 : (%f)",
                        msg->msgid,cmd.command,cmd.param1,cmd.param4,cmd.param5,cmd.param6,cmd.param7);
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
                ConfigMavlink::sys_id ,ConfigMavlink::comp_id ,
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
                ConfigMavlink::sys_id ,ConfigMavlink::comp_id ,
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
            if( cmd.target_system != ConfigMavlink::sys_id ) break;
            _heartbeat.base_mode = cmd.base_mode;
            if (_heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {                
                _heartbeat.custom_mode = (uint32_t)cmd.custom_mode; //qgc용                        
                Controller::DroneStatusManager::getInstance().setFlyingMode((flyingMode_e)cmd.custom_mode); // 내부 상태 매니저에도 반영
            }
            ESP_LOGI(TAG, "MAVLINK_MSG_ID_SET_MODE custom mode: 0x%08X", cmd.custom_mode);
            break;
        }
        default:{            
           //ESP_LOGI(TAG, "SWITCH default msgid: %u (%s)", msg->msgid, ret_msg ? ret_msg->name : "Unknown");
        }
        
    }    
}


void Mavlink::MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_command_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    auto& gps = Sensor::Gps::getInstance();
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
    mavlink_msg_autopilot_version_encode(ConfigMavlink::sys_id ,ConfigMavlink::comp_id , &ver_msg, &version);
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
            ConfigMavlink::sys_id ,           // 내 FC 시스템 ID (보통 1)
           ConfigMavlink::comp_id ,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
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
        ConfigMavlink::sys_id ,           // 내 FC 시스템 ID (보통 1)
       ConfigMavlink::comp_id ,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
        &ack_msg,
        200,                 // version: MAVLink 2.0 (200)
        100,                 // min_hw_version: 최소 지원 버전 (100)
        200,                 // max_hw_version: 최대 지원 버전 (200)
        0,                   // spec_version_hash: 보통 0 (또는 라이브러리 생성 해시)
        0                    // library_version_hash: 보통 0
    );
    send_mavlink_msg(&ack_msg);
}

/**
 * @brief 
 *      1. ESP-NOW 수신 큐에서 MAVLink 메시지 데이터 패킷을 대기 및 수신
 * 
 * @param pv 
 */
void Mavlink::inMessageQueueTask(void *pv)
{
    //auto& mavlink =  Service::Mavlink::get_instance();
    Mavlink *mavlink = static_cast<Mavlink*>(pv);
    static mavlink_status_t status;
    mavlink_message_t msg;
    Service::EspNow::esp_now_data_t pkt;
    
    while (true) {
        if (xQueueReceive(EspNow::get_instance().mavlink_rx_queue, &pkt, portMAX_DELAY)) {                
            for (int i = 0; i < pkt.len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_2, pkt.buffer[i], &msg, &status)) {
                    mavlink->handle_mavlink_message(&msg);
                    // QGC 명령에 따른 상태 업데이트 로직
                    // static bool previous_armed_state = false;
                    // if (previous_armed_state != ENV::g_sys.is_armed) {
                    //     if (ENV::g_sys.is_armed) {
                    //         ENV::g_heartbeat.base_mode   |= MAV_MODE_FLAG_SAFETY_ARMED;
                    //         ENV::g_sys.system_status      = MAV_STATE_ACTIVE;
                    //         // calibrate_ground_pressure(); // 주석 처리 유지: 통신 두절 방지
                    //         ESP_LOGD(TAG,"시동으로 프래그 변환(시동)");
                    //     } else {
                    //         ENV::g_heartbeat.base_mode   &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                    //         ENV::g_sys.system_status      = MAV_STATE_STANDBY;
                    //         ESP_LOGD(TAG,"시동으로 프래그 변환(시동 꺼짐)");
                    //     }
                    //     previous_armed_state = ENV::g_sys.is_armed; // 중복 코드 제거
                    //}
                }
            }
        } // if(xQueueReceive(....))
    } //while(true)
}

void Mavlink::StartTask()
{
    auto res = xTaskCreatePinnedToCore(inMessageQueueTask, "inMessageQueueTask", 8192, this, 15,nullptr, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ inMessageQueueTask Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "inMessageQueueTask task is passed...");
    //return res;
}

void Mavlink::on_timer_tick()
{
    static uint8_t step = 0;
    mavlink_message_t msg;

    // --- 일괄 전송 데이터 안전 패칭 (10Hz 동기화 캡처) ---
    QgcAttitude_t m_att = Controller::SharedDataManager::getInstance().get_shared_data<Controller::Data_type::DT_QGC_ATTITUDE>();
    gps_data_t    m_gps = Controller::SharedDataManager::getInstance().get_shared_data<Controller::Data_type::DT_GPS_DATA>();    

    // 10Hz 타이머 매 틱마다 자세(Attitude) 데이터 상시 전송 (라디안 단위 정방향 송신)
    mavlink_msg_attitude_pack(ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg, esp_timer_get_time()/1000, 
                                m_att.att.roll, 
                                m_att.att.pitch, 
                                m_att.att.yaw, 
                                m_att.speed.x, 
                                m_att.speed.y, 
                                m_att.speed.z  
                            );
    send_mavlink_msg(&msg);

    // 💡 [유틸리티] MAVLink 0~360도 양수 방위각 및 centi-degrees 연산 가공
    float yaw_deg = m_att.att.yaw * (180.0f / M_PI);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f; // 음수 각도 보정 line
    uint16_t heading_centi_deg = (uint16_t)(yaw_deg * 100.0f); // 0 ~ 36000 범위 정수

    // 스케줄러 분기 루프 시작
    switch (step) {
        case 0: { // 1. 하트비트 전송
            // Controller::systemState_e current_state;
            // current_state = Controller::DroneStatusManager::getInstance().getSystemStateValue(); // 시스템 상태 업데이트 (예: 시동 여부 반영)
            
            mavlink_msg_heartbeat_pack(ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg, 
                                        MAV_TYPE_QUADROTOR, 
                                        MAV_AUTOPILOT_PX4, 
                                        _heartbeat.base_mode,  
                                        _heartbeat.custom_mode, 
                                        (uint8_t)_heartbeat.system_status); // 시스템 상태를 현재 드론 상태로 업데이트);
            send_mavlink_msg(&msg);
            break;
        }

        case 1: { // 2. VFR_HUD 전송 (QGC 계기판 고도/속도 연동 마감)
            // 💡 GPS 속도가 cm/s 또는 mm/s 단위인 경우 m/s로 정정 나눗셈 스케일링 필요 (여기서는 원본 유지)
            float qgc_gnd_speed = m_gps.gndSpeed; 

            mavlink_msg_vfr_hud_pack(
                ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg,          
                0.0f,                                     // airspeed (멀티콥터 미사용)
                qgc_gnd_speed,                            // groundspeed (m/s 단위)
                (int16_t)yaw_deg,                         // 💡 수정한 도(Degree) 단위 정수 방위각 (0~360)
                (uint16_t)m_att.base_throttle,            // 고도 PID 베이스 스로틀 (%)
                m_att.alt,                                // 💡 정밀 수직 칼만필터 추정 고도 (float m)
                m_att.v_speed                             // 💡 정밀 수직 칼만필터 추정 상승속도 (float m/s)
            );
            send_mavlink_msg(&msg);
            break;
        }

        case 8: { // 3. 글로벌 위치 정보 전송 (QGC 지도 마커 부드러운 연동 마감)
            int32_t send_lat = 0;
            int32_t send_lon = 0;
            int32_t send_alt_msl = 0;

            if (m_gps.fixType >= 3 && m_gps.horAcc < 4000) { 
                send_lat = static_cast<int32_t>(m_gps.lat);
                send_lon = static_cast<int32_t>(m_gps.lon);
                send_alt_msl = static_cast<int32_t>(m_gps.horMSL); // 이미 GPS 내부가 mm 규격이면 그대로 주입
            }

            // 💡 수직 칼만필터가 뱉은 m 단위 고도를 MAVLink 규격인 mm 단위 int32_t로 변환
            int32_t relative_alt_mm = static_cast<int32_t>(m_att.alt * 1000.0f);

            // 💡 수직 속도(velDown) 역시 NED 링크 상 수직 하강이 + 이므로, 상승(+) 속도 변수에 -1을 곱해 매핑
            int16_t vel_down_cm_s = static_cast<int16_t>(-m_att.v_speed * 100.0f);

            mavlink_msg_global_position_int_pack(
                ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg, 
                esp_timer_get_time() / 1000,    
                send_lat,                       
                send_lon,                       
                send_alt_msl,                   
                relative_alt_mm,                // 💡 mm 단위 정수로 교정 완료
                static_cast<int16_t>(m_gps.velNorth), 
                static_cast<int16_t>(m_gps.velEast),  
                vel_down_cm_s,                  // 💡 우리가 구한 정밀 수직 속도(cm/s)로 완벽 융합
                heading_centi_deg               // 💡 0~36000 범위의 centi-degrees 규격으로 교정 완료
            );
            send_mavlink_msg(&msg);
            break;
        }

        case 3: { // 4. 시스템 상태 전송
            uint32_t sensors_present = MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_ACCEL2 | 
                                       MAV_SYS_STATUS_SENSOR_3D_GYRO  | MAV_SYS_STATUS_SENSOR_3D_GYRO2  |
                                       MAV_SYS_STATUS_SENSOR_3D_MAG   | MAV_SYS_STATUS_SENSOR_3D_MAG2   | 
                                       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE |
                                       MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_BATTERY | MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
            uint32_t sensors_enabled = sensors_present; 
            uint32_t sensors_health  = sensors_present;  

            uint16_t load = (600.0f * 1000.0f) / LOOP_TIME;
            auto& bat = Driver::Battery::get_instance();
            uint16_t battery_voltage   = (uint16_t)(bat.get_battery_voltage() * 1000.0f); 
            int16_t  current_battery   = 1050;   
            int8_t   battery_remaining = 85;    
            uint16_t comms_drop_rate   = 0;     
            uint16_t comms_errors      = 0;        
            
            mavlink_msg_sys_status_pack(
                ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg, 
                sensors_present, sensors_enabled, sensors_health,         
                load, battery_voltage, current_battery, battery_remaining, 
                comms_drop_rate, comms_errors, 0, 0, 0, 0, 0, 0, 0
            );
            send_mavlink_msg(&msg);
            break;
        }    

        case 6: { // 5. 라디오 전파 링크 상태 전송
             mavlink_msg_radio_status_pack_chan(
                            ConfigMavlink::sys_id, ConfigMavlink::comp_id, MAVLINK_COMM_1, &msg, 
                            Service::EspNow::get_instance().current_rssi, 
                            0, 0, Service::EspNow::get_instance().noise_floor, 0, 0, 0);
            send_mavlink_msg(&msg);
            break;
        }

        case 9: { // 6. 원시 GPS 상태 위성 뷰 데이터 전송
            uint16_t mav_dop = static_cast<uint16_t>(m_gps.pDOP * 100.0f); 

            mavlink_msg_gps_raw_int_pack(
                    ConfigMavlink::sys_id, ConfigMavlink::comp_id, &msg, 
                    esp_timer_get_time() / 1000,               
                    m_gps.fixType,                                                   
                    static_cast<int32_t>(m_gps.lat),      
                    static_cast<int32_t>(m_gps.lon),      
                    static_cast<int32_t>(m_gps.horMSL), 
                    mav_dop,                                  
                    mav_dop,                                  
                    static_cast<uint16_t>(m_gps.gndSpeed),        
                    static_cast<uint16_t>(m_gps.headMotion),       
                    static_cast<uint8_t>(m_gps.numSat),           
                    static_cast<int32_t>(m_gps.height),         
                    m_gps.horAcc,                                 
                    m_gps.verAcc,                                 
                    m_gps.speedAcc,                                 
                    0,                                          
                    heading_centi_deg           // 💡 교정 완료된 centi-degrees 규격 주입
                );
                send_mavlink_msg(&msg);
            break;
        }
        
        default:
            break;
    }

    if (++step >= 10) {
        step = 0;
    }
}

esp_err_t Mavlink::initialize()
{
    _heartbeat ={};

    _heartbeat.base_mode    =   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED   |   //MAV_MODE_FLAG_TEST_ENABLED    |    // 테스트 모드 (실제 비행에서는 사용 안 함)
                                MAV_MODE_FLAG_STABILIZE_ENABLED     |   // 자세 제어 활성화
                                //MAV_MODE_FLAG_SAFETY_ARMED          |   // 시동(ARM) 활성화
                                MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;     // 원격제어 활성화
    _heartbeat.custom_mode    =   (uint32_t)flyingMode_e::MODE_STABILIZED; // PX4 STABILIZE 모드: 0x00070000 (Main Mode 7) + 0x00000000 (Sub Mode 0)
    _heartbeat.system_status  =   systemState_e::SYS_STATE_STANDBY;

    // 중요........
    // timer의 callback과 연결하여 on_timer_tick를 타이머에의해서 실행함.
    Service::Timer::get_instance().set_timer_callback([this](){on_timer_tick();});

    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

} // namespace MAV