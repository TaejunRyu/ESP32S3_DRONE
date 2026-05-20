#pragma once

#include "ryu_Types.hpp"
#include <cmath>
#include <mutex>

namespace Filter {

class KalmanFilter {
public:
    static KalmanFilter& getInstance() {
        static KalmanFilter instance;
        return instance;
    }

    void init(float initRoll = 0.0f, float initPitch = 0.0f, float initYaw = 0.0f);

    void update(const Vector3f& accel,const Vector3f& gyro, const Vector3f& mag,float dt){
        predict(gyro,dt);
        update(accel,mag);
    };

    void predict(const Vector3f& gyro, float dt);
    void update(const Vector3f& accel, const Vector3f& mag);
    
    Attitude_t getEuler() const;

    float getXErr(){return x_err;};
    float getYErr(){return y_err;};
    float getZErr(){return z_err;};

private:
    KalmanFilter();
    ~KalmanFilter() = default;
    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    mutable std::mutex mtx;

    float q[4];
    float P[4][4];


    ImuData  _imudata{};

    // 100% 정상 작동하는 올바른 참조 방식입니다. (대문자 P 적용)
    // 쉽게 보는 법: 기체가 정지해 있을 때 P[1][1], P[2][2], P[3][3]의 값이 너무 커지지 않고 작은 값(예: 0.01 이하)으로 안정적으로 유지되고 있다면, 
    // 자이로의 누적 오차가 가속도계와 지자계에 의해 완벽하게 억제되고 있다는 뜻입니다.
    float& x_err = P[1][1]; // 쿼터니언 q1 (X축 회전 관여)의 추정 오차 성분
    float& y_err = P[2][2]; // 쿼터니언 q2 (Y축 회전 관여)의 추정 오차 성분
    float& z_err = P[3][3]; // 쿼터니언 q3 (Z축 회전 관여)의 추정 오차 성분

    // 파라미터는 확장 칼만 필터(EKF)가 센서 데이터를 신뢰하는 비율을 결정하는 핵심 가중치(공분산 파라미터)입니다.
    // 필터는 기본적으로 자이로스코프를 이용해 자세를 먼저 예측(Predict)하고, 가속도계와 지자계를 이용해 그 예측치의 오차를 보정(Update)합니다. 
    // 이때 "자이로의 예측 값을 더 믿을 것인가, 아니면 가속도계/지자계의 측정 값을 더 믿을 것인가"를 수학적으로 조율하는 저울 역할을 합니다.
    // Q_gyro = 0.001f; (시스템/프로세스 노이즈 공분산)의미: 
    //      필터 내부의 수학적 예측 모델(자이로스코프 데이터)이 가진 자체적인 오차와 불확실성을 뜻합니다.
    //      비행 시 영향: 이 값을 크게 잡으면 필터는 "자이로 센서 데이터에 노이즈나 누적 오차(Drift)가 많다"고 판단합니다. 
    //      따라서 자이로를 기반으로 한 빠른 추적을 덜 신뢰하고, 가속도계와 지자계의 보정 기여도를 강제로 높이게 됩니다. 
    //      반대로 너무 작게 잡으면 가속도계 보정을 무시하고 자이로만 믿다가 수십 초 뒤 자세가 천천히 흐르는 드리프트가 발생합니다. 
    //      현재 설정하신 0.001f는 매우 적절한 표준값입니다.
    // R_accel = 0.10f; (가속도계 측정 노이즈 공분산)의미: 
    //      외부 가속도계 센서가 측정해오는 데이터의 노이즈(불신도)를 뜻합니다.
    //      비행 시 영향: 이 값이 커질수록 필터는 가속도계 데이터를 덜 신뢰합니다.
    //      주석에 적어두신 것처럼 모터가 돌 때 진동 때문에 QGC 인공수평선이 떨린다면 이 값을 올리는 것이 정답입니다. 
    //      값을 올리면 필터가 고주파 진동 노이즈를 "믿지 못할 쓰레기 데이터"로 취급하여 무시하므로 인공수평선이 아주 부드러워집니다.단, 
    //      이 값을 너무 크게 올리면(예: 1.0f 이상) 기체가 실제로 기울어졌을 때 수평을 다시 잡아주는 보정 속도가 느려져 자세가 일시적으로 흐려질 수 있습니다. 
    //      주석의 0.05f ~ 0.10f 범위 내에서 진동이 멈추는 최소한의 값을 찾는 것이 가장 좋습니다.
    // R_mag = 0.05f; (지자계 측정 노이즈 공분산)의미: 
    //      나침반(지자계) 센서가 읽어오는 데이터의 노이즈(불신도)를 뜻합니다.
    //      비행 시 영향: 이 값이 커질수록 필터는 나침반 데이터를 덜 신뢰합니다.
    //      드론은 모터에 강한 전류가 흐를 때 순간적으로 주변 자기장이 왜곡(하드아이언 전자기 간섭)되는 특성이 있습니다.
    //      만약 평소에는 정북을 잘 보다가, 
    //      모터 출력을 올릴 때마다 나침반(Yaw)이 미세하게 좌우로 틀어지는 현상이 발생한다면 이 R_mag 값을 0.10f ~ 0.20f 정도로 조금 높여서 
    //      지자계 왜곡 노이즈를 필터링해 주어야 기체가 요동치지 않고 똑바로 날아갑니다.
    float Q_gyro;
    float R_accel;
    float R_mag;

    void normalizeQuaternion();

    // [수정] 2차원 배열 참조 형식으로 명확히 타입 정의하여 컴파일 에러 차단
    void matrixInversion3x3(const float in[3][3], float out[3][3]);
};

} // namespace Filter
