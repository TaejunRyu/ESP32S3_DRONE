1. NED 표준체계의 설명 (구현완료)
    1) X축 (NORTH), Y축 (EAST), Z축 (DOWN)
    2) X축을 내릴때  AX(+)  GY(-)  
    3) Y축을 내릴때  AY(+)  GX(+)
    4) AZ (+1)
    5) GZ 오른쪽으로 회전시 (+)
    6) X가 북을 가르킬때 MX 최대값   MY최소값   MZ(+)  

2. (1.)의 규칙에 일치하는 (EKF) kalman filter를 구현하려한다. 
    1) ESP32S3, ESP-IDF V6.1.0
    2) 최대한 ESP32S3의 특수기능(esp-dsp)을 사용. (문제가 발생하면 사용안해도 됨)
    3) Vector3f struct에  x,y,z 를 사용하고 있음 (ryu_Types.hpp에서 정의)
    4) Attitude_t struct에  roll,pitch,yaw 를 사용하고 있음 (ryu_Types.hpp에서 정의)
    5) #pragma once 사용
    6) namespace 는 Filter 이다 
    7) 화일명 ryu_KalmanFilter.hpp ,ryu_KalmanFilter.cpp
    8) class 이름은 KalmanFilter (singleton으로)
    9) (1.NED 표준체계의 설명)에 부합하도록 센서 데이터 형식을 일치함.
    10) 지자계 (mag)도 처리한다.
    11) 쿼터니언 활용 할것.
    12) getEuler함수 구현할것.
    13) NED의 데이터로서 가능하지 않은면 ENU로 변환하는 함수를 두고 ENU로 처리한 다음 NED방식으로 변환하면 어떤가 ?




3. PID Controller 구현 (구현 해야할것)
    1) 위 내용은 NED를 기준으로 처리하고 있다. 
    2) PidController class 구현 (singleton으로)
    3) namespace Controller
    4) filename  ryu_PidController.hpp , ryu_PidController.cpp
    5) 캐스케이드(2중 루프) PID 클래스 구조로 코드를 설계
    6) 
     