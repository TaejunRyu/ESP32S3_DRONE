#pragma once
#include <stdint.h>


inline static constexpr float DEG_TO_RAD = 0.0174532925f;
inline static constexpr float RAD_TO_DEG = 57.295779513f; 

inline static constexpr uint16_t LOOP_TIME =  1000;  //  1000 micro second
inline static constexpr float           dt = 0.001f;  // 0.001 second


namespace ConfigEspnow{
    inline static constexpr uint8_t bridge_macaddr[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
    inline static constexpr size_t  channel  = 6;
}

namespace ConfigMavlink{
    inline static constexpr size_t sys_id  = 1;
    inline static constexpr size_t comp_id = 1;
}