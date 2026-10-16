# Laya 연결 프로토타입: 로봇 로컬 / PC 원격 추론

## 현재 확인한 범위

- 실제 `convaiinnovations/laya` 체크포인트를 PC CPU에서 실행했습니다. SDK는 `laya==0.4.8`, 모델 revision은 `7b928d828b7b0e022f929d9bd2e44165aa270148`로 고정했습니다.
- 모델 서버는 ROS가 필요 없는 Python HTTP 서버입니다. 로봇의 `laya_bridge`는 작은 상태 JSON을 보내고 실제 모델의 선택과 확률을 받아 `/laya/decision`에 발행합니다.
- 기존 최종 안전 게이트에 선택적으로 연결했습니다. `require_laya:=true`이면 새롭고 충분한 확률의 `continue` 응답도 있어야 주행할 수 있습니다.
- 실물 로봇에 접속하지 못해 SBC 직접 실행 속도, 실제 네트워크 지연, 주행은 검증하지 못했습니다.

**기본 체크포인트가 로봇 상황을 올바르게 판단한다고 확인한 상태는 아닙니다.** 네 가지 간단한 예제를 실제로 추론했을 때 기대한 선택과 일치한 것은 두 개였습니다. 통로가 비어 있어도 `wait`를 골랐습니다. 이 결과는 작은 예제 확인이며 일반적인 정확도 측정이 아닙니다.

따라서 이 버전은 통신과 연결 구조를 확인하는 실험용 프로토타입입니다. 먼저 판단 결과를 관찰·기록하고, 로봇 상황 데이터로 학습·평가한 뒤 제어에 적용해야 합니다. 실제 결과는 [laya_inference_examples.json](verification/laya_inference_examples.json)에 그대로 저장했습니다.

## 구조

```text
로봇의 YOLO 결과 + LiDAR 상태
          ↓
  laya_bridge (로봇, ROS 2)
          ↓ 작은 JSON / HTTP
  Laya 서버 (PC 또는 로봇 로컬)
          ↓ continue / wait / stop + 선택 확률
  /laya/decision
          ↓
  person_safety_gate → 최종 속도 명령
```

입력은 `camera_fresh`, `lidar_fresh`, `person_visible`, `front_clear`, `front_distance_m`입니다. 영상 전체를 전송하지 않습니다. 로봇에서 관측한 상태를 영어 문장으로 구성해 고정된 세 선택지로 질문합니다. Laya가 바퀴 속도를 직접 생성하지 않습니다.

주행 노드는 `/cmd_vel_nav`에 `Twist`를 발행합니다. 이 연결은 특정 경로 계획 알고리즘에 종속되지 않습니다. 다른 주행 스택을 사용하려면 해당 출력 토픽을 연결하고 `/safety/stop` 시 목적지를 취소하는 어댑터도 구현해야 합니다. 기존 A* 실행 예제는 통신 확인용으로 유지했습니다.

## 1. PC에 설치하고 실제 모델 서버 실행

```bash
cd ~/my-car-astar
python3 scripts/setup_laya.py
.venv-laya/bin/python3 scripts/laya_server.py
```

첫 실행은 Hugging Face에서 가중치를 다운로드합니다. 모델 캐시는 `.cache/huggingface`, 추론 환경은 `.venv-laya`입니다. YOLO 환경과 별도로 설치하며, 캐시와 가상 환경은 Git에 올리지 않습니다.

```bash
curl http://127.0.0.1:8765/health
curl http://127.0.0.1:8765/decide \
  -H 'Content-Type: application/json' \
  -d '{"schema_version":1,"request_id":"manual-example-1","state":{"camera_fresh":true,"lidar_fresh":true,"person_visible":false,"front_clear":true,"front_distance_m":2.0}}'
```

서버는 모델 로딩과 초기 추론을 마친 뒤 요청을 받습니다. 한 번에 한 요청을 처리하고 처리 중 추가 요청에는 503을 반환하여 오래된 상태가 대기열에 쌓이지 않게 합니다. 판단 선택과 `probabilities`는 모델의 실제 결과이며 규칙으로 덮어쓰지 않습니다.

`confidence` 필드는 **선택한 항목의 확률**입니다. Laya SDK의 엔트로피 기반 `confidence`를 그대로 확률로 해석하지 않습니다. 확률이 높더라도 로봇 도메인의 판단 정확도가 보장되지는 않습니다.

## 2. 로봇에서 판단 결과만 관찰

로봇에서는 기존 카메라·YOLO·LiDAR를 실행하고, 새 코드를 빌드한 뒤 다음 노드를 실행합니다. 원격 방식이라면 로봇에 Laya나 PyTorch를 추가 설치할 필요가 없습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/my-car-astar/install/local_setup.bash
export ROS_DOMAIN_ID=16
ros2 run my_bot laya_bridge --ros-args \
  -p server_url:=http://127.0.0.1:8765/decide
```

```bash
ros2 topic echo /laya/status
ros2 topic echo /laya/decision
```

이 노드만 실행하면 기존 기본 안전 게이트의 `require_laya`는 false이므로 Laya 판단은 기록·관찰 용도입니다. 실제 로봇에서는 이 단계부터 검증합니다. Gazebo 데이터를 수신할 때는 노드에 `-p use_sim_time:=true`를 추가합니다.

## 3. PC 서버를 로봇의 localhost로 연결

WSL2에서 ROS DDS 멀티캐스트를 직접 연결하기 어려운 경우도 사용할 수 있도록 HTTP 통신을 사용했습니다. PC에서 모델 서버를 실행하고, **PC의 별도 터미널**에서 SSH reverse forwarding을 설정합니다.

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  -R 127.0.0.1:8765:127.0.0.1:8765 robot@ROBOT_IP
```

`ROBOT_IP`는 실제 주소로 바꿉니다. 로봇에서 `curl http://127.0.0.1:8765/health`가 응답하면 로봇의 localhost 요청이 PC의 모델 서버로 전달되는 상태입니다. 서버와 로봇 사이의 시간 동기화 없이 요청 경과 시간으로 판단 응답을 만료시킵니다. 기존 카메라와 ROS 안전 게이트 사이의 시계 조건은 여전히 적용됩니다.

SBC에서 서버까지 직접 실행할 수 있다면 같은 localhost URL을 쓰면 됩니다. 이번 PC 시험에서 가중치 다운로드는 약 0.84GB, 실행 중 서버 RSS는 약 2GB였으며, 네 예제의 CPU 추론은 약 0.50~0.58초였습니다. 이 수치로 Raspberry Pi의 속도·메모리를 보장할 수는 없습니다. 실제 SBC의 아키텍처·RAM과 ROS·YOLO를 함께 실행했을 때의 여유를 확인해야 합니다.

서버는 기본적으로 loopback에만 바인딩합니다. LAN 주소에 노출하려면 `LAYA_API_TOKEN`이 있어야 하며, 클라이언트에도 같은 환경변수를 설정합니다. 기본 사용 방법은 위 SSH 터널입니다.

## 4. 판단을 속도 허용 조건에 연결하는 실험

### Gazebo

PC에서 서버를 먼저 실행한 뒤:

```bash
bash scripts/run_laya_simulation.sh
# GUI 없이 실행
bash scripts/run_laya_simulation.sh gui:=false rviz:=false viewer:=false
```

기본 모델이 `wait`를 선택하거나 선택 확률이 0.7 미만이면 출발하지 않습니다. 이를 통신 오류로 오인하거나 문턱을 낮춰 정상 주행으로 간주하지 않습니다. 아직 학습하지 않은 기본 모델의 실제 판단 결과입니다.

### 실물

실물 관측·학습·평가가 끝난 뒤 기존 bringup 대신 실행하는 구성입니다.

```bash
bash scripts/run_hardware_bringup.sh require_laya:=true \
  laya_server:=http://127.0.0.1:8765/decide
```

이 launch에 `laya_bridge`가 포함되므로 별도 bridge를 중복 실행하지 않습니다. 카메라와 실제 지도 기반 네비게이션은 [실물 실행 안내](HARDWARE.md)를 따릅니다.

### 정지 조건

- 최신 Laya 응답이 없거나 요청 시작으로부터 3초가 지나면 속도 0.
- `wait`, `stop`, 선택 확률 0.7 미만이면 속도 0.
- 최초 준비 중에는 정상 모델 판단을 기다립니다. 한 번 주행 가능 상태가 된 뒤 위 조건이 발생하면 정지를 잠그고 목적지를 취소합니다.
- 통신이 복구되더라도 수동 `/safety/reset`과 새 목적지가 필요합니다. 해제하려면 최신 Laya 허용 결과와 기존 카메라 해제 조건이 모두 충족되어야 합니다.
- Laya가 `continue`를 골라도 기존 사람 검출·영상 끊김 정지 조건을 해제할 수 없습니다.

## 검증 기록

- 기존 24개 + Laya 프로토콜·실제 ROS 게이트 콜백 16개: 총 40개 테스트 통과.
- [실제 모델 예제 결과](verification/laya_inference_examples.json): 네 입력에 실제 체크포인트로 HTTP 추론. 로봇 도메인 학습·정확도 검증은 별도 과제입니다.
- 실제 ROS bridge 전송·정지 유지 시험은 [laya_transport_verification.json](verification/laya_transport_verification.json)에 기록했습니다. 입력 센서는 시험용 메시지였으며 실물 주행 시험이 아닙니다.

## 고도화 방향

Laya를 임무 선택에 쓰는 프로젝트라면 `다음 목적지 선택`, `막힌 임무 보류`, `대기`, `운영자 요청`처럼 가능한 행동을 명확히 정의합니다. 상황·선택·결과 데이터를 모으고 로봇 도메인으로 학습한 뒤, 같은 상황에서 규칙 기반 판단과 비교합니다. 임무 완료율, 사람 개입 횟수, 판단 지연과 통신 장애 대응을 평가할 수 있습니다.

공식 자료: [Laya 코드와 제한 사항](https://github.com/NandhaKishorM/laya), [모델 카드](https://huggingface.co/convaiinnovations/laya). 서버의 선택은 아직 검증된 로봇 제어 정책이 아닙니다.
