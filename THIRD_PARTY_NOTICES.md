# 외부 에셋과 구성요소

## Gazebo 모델

- `src/my_bot/models/stop_sign`: Cole Biesemeyer와 Nate Koenig의 OSRF Gazebo 모델. 원본 출처와 CC BY 3.0 라이선스는 해당 폴더의 `SOURCE.txt`, `LICENSE`에 있습니다.
- `src/my_bot/models/person_standing`: OSRF Gazebo 모델. 메쉬에는 Marina Kollmitz의 원저자 정보가 포함되어 있습니다. 원본 출처와 라이선스는 해당 폴더의 `SOURCE.txt`, `LICENSE`를 유지했습니다. 데모 월드의 배치와 스케일은 조정했습니다.

## YOLO

`src/my_bot/weights/yolo11n.pt`는 Ultralytics가 배포한 COCO 학습 가중치입니다.

- [가중치 원본](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt)
- [Ultralytics 라이선스](https://www.ultralytics.com/license)

Ultralytics와 가중치의 이용·재배포는 원 제공자의 라이선스 조건을 확인해야 합니다. 이 저장소를 비공개로 만드는 것만으로 외부 라이선스 조건이 변경되지는 않습니다.

## ROS / TurtleBot3

ROS 2, Navigation2, Gazebo와 TurtleBot3 패키지는 시스템 의존성으로 사용하며 각 프로젝트의 라이선스가 적용됩니다. TurtleBot3 로봇 모델은 설치된 `turtlebot3_gazebo`에서 로딩합니다.

이 정리 작업에서 저장소 전체에 새로운 오픈소스 라이선스를 부여하지 않았습니다. 외부 에셋의 원 출처와 라이선스를 보존했습니다.
