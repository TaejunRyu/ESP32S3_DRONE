ESPNOW의 콜백에서 큐로 데이터를 보내서 처리루틴을 








 1. 4대 핵심 변수의 명확한 역할 정의 (역할 분담)변수들이 헷갈리지 않으려면, 딱 이 기준으로만 생각하시면 됩니다.

      변수명          한 줄 요약 (핵심 의미)                      비유하자면?                 관리 주체
      is_armed        모터에 전기가 흐르는가? (안전 잠금장치)      자동차 시동 키             조종기/QGC 명령
      is_flying       실제로 하늘에 떠 있는가? (물리적 상태)      바퀴가 땅에서 떨어짐        센서 (고도/가속도)
      flying_mode     조종사가 어떤 방식으로 조종하는가?          수동 기어 vs 자동 기어      조종기 키 / QGC
      system_state    드론 컴퓨터의 전체 생명 주기                상태부팅중/대기/경고/종료   FC (전체 제어 시스템)

 2. 이 변수들이 얽히는 과정 (실제 비행 시나리오 순서)
      드론이 켜져서 하늘을 날고 착륙할 때까지, 이 변수들이 어떤 순서로 옷을 갈아입는지 보시면 흐름이 한눈에 들어옵니다.

      1단계: 전원 켜짐 (부팅 및 교정)드론에 배터리를 연결하면 센서를 체크하고 정렬합니다.
          🖥️ 상태: system_state = SYS_STATE_BOOT ➔ SYS_STATE_CALIBRATING
          🔒 잠금: is_armed = false, is_flying = false (모터 정지)

      2단계: 비행 준비 완료 (대기)
          센서 교정이 끝나고 GPS가 잡혀서 비행할 준비가 끝났습니다. QGC에서 'Arming 가능' 문구가 뜹니다.
          🖥️ 상태: system_state = SYS_STATE_STANDBY (시동 대기)
          🎮 모드: 기본 모드인 flying_mode = MODE_STABILIZED
          🔒 잠금: is_armed = false, is_flying = false

      3 단계: 시동 걸기 (Arming)조종사가 조종기 스틱을 모으거나 QGC에서 시동 명령을 내렸습니다. 모터가 회전하기 시작합니다.
          🔑 시동: is_armed = true로 변경!🖥️ 상태: system_state = SYS_STATE_ACTIVE (활성화됨)으로 변경!
          🔒 비행: is_flying = false (모터는 돌지만 아직 땅에 있음)

      4단계: 이륙 및 비행 (Flying)조종사가 스로틀을 올려 드론이 하늘로 솟구칩니다. 센서가 고도가 높아지는 것을 감지합니다.
          🚀 비행: is_flying = true로 변경!
          🖥️ 상태: system_state = SYS_STATE_ACTIVE 유지
          🎮 모드: 조종사 취향에 따라 flying_mode = MODE_ALTCTL (고도 유지) 등으로 전환 가능
  
      5단계: 착륙 및 시동 꺼짐 (Disarming)비행을 마치고 땅에 안전하게 내려앉았습니다. 모터 회전을 멈춥니다.
          🔒 비행: is_flying = false (착륙 감지)
          🔑 시동: is_armed = false (모터 정지)
          🖥️ 상태: 다시 비행 전 대기 상태인 system_state = SYS_STATE_STANDBY로 복귀
 
 3. 복잡함을 해결하는 핵심 연동 규칙 (상태 머신 구조)코드가 꼬이지 않으려면 "A가 변할 때 B와 C도 규칙에 따라 같이 변한다"는 상호 배타적 규칙을 선언해야 합니다.
          시동이 꺼져 있으면(is_armed == false), 
          절대로 비행 중(is_flying == true)이 될 수 없다.
          
          만약 is_armed가 false가 되면, is_flying도 무조건 false가 되어야 안전합니다.

          시스템 상태(system_state)는 is_armed와 완전히 동기화된다.
          is_armed == true 이면? ➔ 무조건 SYS_STATE_ACTIVE (비행 또는 모터 작동 중)
          is_armed == false 이면? ➔ 무조건 SYS_STATE_STANDBY (또는 에러 시 CRITICAL)비행 모드(flying_mode)는 조종 방식일 뿐, 
          
          기체의 생사(시동 여부)를 결정하지 않는다.Stabilized 모드든, Position Control 모드든 기체가 땅에 서 있으면 똑같이 is_flying = false 상태입니다.
          💡 난해함을 없애기 위한 코드 가독성 제안이 변수들의 의미가 매번 헷갈린다면, 
          DroneStatusManager 내부나 주석에 이 연동 규칙을 명시하거나, 
          상태를 변환할 때 연관된 변수를 세트로 묶어서 변경하는 헬퍼 함수를 만들어 두면 좋습니다.

          예를 들어, 수신단에서 개별 변수를 하나하나 바꾸지 말고, "시동 상태 전환"이라는 하나의 명령으로 묶는 것입니다.cpp 
          헷갈림을 방지하기 위한 상태 전환 추상화 예시
          void changeToArmedState(bool arm) {
              if (arm) {
                  setArmed(true);
                  setSystemState(systemState_e::SYS_STATE_ACTIVE);
                   시동이 걸리는 순간이므로 아직 하늘에 뜨진 않음
                  setFlying(false); 
              } else {
                  setArmed(false);
                  setSystemState(systemState_e::SYS_STATE_STANDBY);
                  setFlying(false);
              }
          }

   
  "모터 상태"(is_armed), 
  "물리적 위치"(is_flying), 
  "조종 기어"(flying_mode), 
  "컴퓨터 상태"(system_state) 로 4가지 영역을 머릿속에서 완전히 찢어서 생각하시면, 얽혀 있던 실타래가 깔끔하게 풀리실 것입니다.




