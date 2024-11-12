# Modular Pipeline 자율 주행 시뮬레이션

## 1. 프로젝트 개요
Modular Pipeline 프로젝트는 OpenAI Gym 환경에서 자율 주행을 시뮬레이션하여 주요 모듈을 통합적으로 구현한 프로젝트입니다. 자율 주행의 핵심 단계인 Low-level Perception, Scene Parsing, Path Planning, Vehicle Control의 단계로 나뉘어져 있습니다. 주요 모듈은 LaneDetection, waypoint_prediction, target_speed_prediction, LateralController, LongitudinalController로 구성되며, 각각의 모듈이 자율 주행의 다양한 기능을 수행합니다.
<br/>

## 2. 프로젝트 플로우 (구현)
**LaneDetection** <br/> 이미지에서 차선 감지 후 B-Spline Curve로 근사화하여 경로를 생성합니다. 픽셀의 경사와 local maximum을 통해 차선을 추출하고 최적 경로를 감지합니다. <br/>
**waypoint_prediction / target_speed_prediction** <br/> 감지한 차선 기반으로 waypoint와 target speed를 예측합니다. center 모드와 smooth 모드를 통해 waypoint를 생성하고 곡률을 계산하여 속도를 설정합니다.  <br/>
**LateralController** <br/> Stanley Control을 사용하여 횡방향 위치 제어를 수행합니다. 차량의 바퀴 방향과 경로 사이의 오차를 보정하고, 감쇠항을 추가하여 안정성을 높였습니다.  <br/>
**LongitudinalController** <br/> PID Control을 통해 종방향 속도를 제어합니다. PID의 Kp, Ki, Kd 값을 조정하여 target speed에 빠르게 도달하도록 설정하고, 파라미터 조정(Vmax, Vmin, Kv)을 통해 최적의 속도를 찾았습니다.

<br/>

## 3. 성과 및 회고
안정적인 주행을 구현하고, 다양한 설정에서 목표 속도에 빠르게 도달하도록 조정한 결과, total_reward가 향상되었습니다. 특히, 커브가 심한 구간에서도 경로 이탈 없이 안정적인 주행이 가능하게 되었습니다.
초기 파라미터 조정의 어려움이 있었으나, PID 및 속도 파라미터를 반복적으로 실험하며 개선한 경험을 통해 파라미터 튜닝의 중요성을 느꼈습니다.
