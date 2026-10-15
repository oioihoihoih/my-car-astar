# 실물 Waffle Pi 실행

## 확인된 정보와 남은 확인

기존 장비 기록은 **TurtleBot3 Waffle Pi, LDS-03, Raspberry Pi CSI 카메라, ROS_DOMAIN_ID=16**입니다. 이번 작업에서는 로봇에 접속되지 않아 코드를 설치하거나 모터를 조작하지 않았습니다.

기존 `/camera/image_raw` 이미지 검증은 Gazebo 기록입니다. 실물의 카메라 드라이버와 토픽 이름은 연결 후 확인해야 합니다. 입력 이름은 `input_topic:=...`로 바꿀 수 있습니다.

실물 실행은 Ubuntu 22.04 / ROS 2 Humble의 `Twist` 인터페이스를 기준으로 합니다. SBC의 기존 TurtleBot3 bringup, LDS-03와 OpenCR 설정이 정상이어야 합니다.

## 1. 설치

로봇에서 저장소를 복제하고 [README의 설치·빌드](../README.md#2-설치와-빌드)를 수행합니다. 저장소 루트를 하나의 colcon 작업공간으로 사용합니다.

실물 설치 위치가 `~/my-car-astar`라면 모든 터미널에서 다음을 적용합니다.

```bash
cd ~/my-car-astar
source /opt/ros/humble/setup.bash
# TurtleBot3 소스 작업공간을 사용하는 장비라면 추가
# source ~/turtlebot3_ws/install/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=16
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-03
```

## 2. 구동부와 안전 노드: SBC에서 실행

기존 `turtlebot3_bringup robot.launch.py`, teleop, 다른 네비게이션을 종료하고 실행합니다.

```bash
bash scripts/run_hardware_bringup.sh
# 같은 실행
# ros2 launch my_bot tb3_safe_bringup.launch.py
```

이 launch는 기존 TurtleBot3 드라이버가 `/cmd_vel_safe`를 구독하도록 remap합니다. `person_safety_gate`만 해당 토픽에 최종 명령을 보냅니다.

- 게이트는 부팅 시 정지 상태이며, 사람이 보이거나 영상이 끊기면 속도 0과 `/motor_power`의 `false`를 요청합니다.
- 별도 `actuator_watchdog`는 게이트의 20Hz 명령 신호가 0.5초 이상 끊기면 모터 해제를 요청합니다.
- 두 안전 노드는 **SBC에서** 실행합니다. 탐지·주행 프로세스가 다른 PC에 있어도 SBC에 남겨 두어야 합니다.
- 안전 노드는 모터를 자동으로 켜지 않습니다.

## 3. CSI 카메라: 별도 터미널

ROBOTIS Humble 카메라 환경이 설치된 장비에서는:

```bash
ros2 launch turtlebot3_bringup camera.launch.py
```

카메라 launch나 드라이버가 없다면 장비의 libcamera 또는 legacy V4L2 환경에 맞는 설치가 먼저 필요합니다. [ROBOTIS 공식 SBC 카메라 설정](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)을 따르세요. 이번 작업에서 원격 카메라 드라이버를 설치하거나 부팅 설정을 변경하지 않았습니다.

다른 터미널에서 실제 토픽을 확인합니다.

```bash
ros2 topic list -t
ros2 topic info /camera/image_raw -v
ros2 topic hz /camera/image_raw
```

YOLO는 `sensor_msgs/msg/Image`를 받습니다. compressed 토픽은 직접 구독하지 않으므로 raw 영상 토픽을 지정합니다. 실물 토픽이 `/image_raw`이면 이후 `input_topic:=/image_raw`로 실행합니다.

## 4. 실제 지도와 네비게이션

실제 공간에서 만든 `map.yaml`과 해당 이미지 파일을 함께 준비합니다. Gazebo 지도를 실물 주행에 사용하지 않습니다.

```bash
bash scripts/run_hardware_navigation.sh "$HOME/maps/map.yaml" \
  input_topic:=/camera/image_raw
```

기본은 GUI 없는 실행입니다. 디스플레이가 있는 Linux PC에서는 `rviz:=true viewer:=true`를 추가할 수 있습니다. 스크립트 대신 직접 실행하려면:

```bash
ros2 launch my_bot tb3_astar_yolo_hardware.launch.py \
  map:=$HOME/maps/map.yaml input_topic:=/camera/image_raw \
  rviz:=true viewer:=true
```

SBC에서 추론이 느리면 `image_size:=320 max_fps:=2.0`을 시험할 수 있습니다. 다만 검출률과 추론 시간, 촬영 프레임 나이를 실제로 확인해야 합니다. 1초의 정상 감시 주기를 유지하지 못하면 정지를 유지합니다. 감시 시간을 늘려 영상 끊김을 숨기지 않습니다.

WSL2 환경에서는 로봇과 DDS 통신이 되는지 먼저 확인해야 합니다. 기존 장비 기록에 WSL2 멀티캐스트 문제가 있었습니다. SSH로 SBC에서 모든 노드를 실행하거나, 같은 LAN의 Linux PC에서 토픽 통신을 확인한 뒤 분산 실행합니다. 카메라와 게이트를 분산 실행하면 컴퓨터 시계도 동기화해야 합니다.

## 5. 최초 출발과 정지 해제

1. RViz의 **2D Pose Estimate**로 실제 지도에서 현재 위치를 지정합니다.
2. 카메라 영상, `/safety/status`, `/scan`과 위치 추정을 확인합니다.
3. 사람이 시야에 없는 정상 관측이 최소 1초·3프레임 이어진 뒤 해제를 요청합니다.

```bash
ros2 service call /safety/reset std_srvs/srv/Trigger '{}'
```

성공한 경우 운영자가 모터를 켭니다.

```bash
ros2 service call /motor_power std_srvs/srv/SetBool '{data: true}'
```

RViz의 **2D Goal Pose**로 새 목적지를 지정합니다. 정지 전 목적지는 자동으로 재개되지 않습니다.

GUI가 없으면 `/initialpose`에 `PoseWithCovarianceStamped`, `/goal_pose`에 `PoseStamped`를 실제 지도 좌표로 발행합니다. 두 메시지의 `frame_id`는 `map`입니다. 위치를 모르는 상태로 시뮬레이션 예제 좌표를 실물에 보내지 않습니다.

수동 정지:

```bash
ros2 service call /safety/stop_now std_srvs/srv/Trigger '{}'
```

정지 기록은 `~/.ros/my_bot/person_stop_16.json`에 유지됩니다. 파일 삭제나 프로세스 재시작으로 정지 해제를 우회하지 않습니다.

## 한계

실물 통합 시험은 남아 있습니다. YOLO는 사람을 놓칠 수 있고 카메라 밖 사람은 판단할 수 없습니다. 이 소프트웨어는 인증된 물리 비상 정지 장치를 대체하지 않습니다. torque-off는 기계식 브레이크가 아니므로 경사면 정지를 보장하지 않습니다.
