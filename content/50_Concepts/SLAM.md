---
tags:
  - concept
parent_concept:
  - "[[Computer vision]]"
  - "[[3D Vision]]"
  - "[[50_Concepts/3D Reconstruction|3D Reconstruction]]"
---
**SLAM (Simultaneous Localization and Mapping)**
- **핵심 역할:** 센서의 움직임을 추정하고 미지의 환경 구조를 재구성하는 데 사용됩니다.
- **다양한 센서 활용:** 시각 데이터 수집용 카메라뿐만 아니라, 레이더(Radar), 소나(Sonar), 라이다(LiDAR) 등 비가시적 데이터 센서를 다양하게 활용할 수 있습니다.
- **IMU(관성 측정 장치) 기반 위치 수집:** 가속도계, 자이로스코프, 자력계가 결합된 IMU를 사용하여 3차원(3D) 공간 내 기기의 기본 위치 데이터를 포착합니다.
- **구조 로봇 적용:** 로봇에 장착될 경우, 로봇의 병진 이동(Translational movement) 및 회전 운동(Rotational movement)을 정밀하게 측정합니다.
- **vSLAM (Visual SLAM):** 여러 SLAM 기법 중 '카메라'를 주된 시각 입력 장치로 활용하는 방식을 뜻합니다.

**vSLAM (Visual Simultaneous Localization and Mapping)**
- **컴퓨터 비전의 핵심 하위 분야:** 카메라 기반의 시각(Visual) 정보를 주력으로 활용하는 SLAM 기술입니다.
- **동시적 위치 추정 및 지도 구축:** 구조 로봇이 미지의 환경을 이동할 때, 실시간으로 주변 지도를 구축(Mapping)하는 동시에 로봇 자신의 위치와 방향(Pose)을 추적(Localization)합니다.
- **다중 데이터 융합(Sensor Fusion):** 오도메트리(Odometry) 센서에서 얻은 추측 항법(Dead Reckoning) 데이터와 카메라의 이미지 데이터를 입력값으로 결합하여 알고리즘의 목표를 달성합니다.
- **실내 환경 적용 및 고정밀도:** GPS 신호가 닿지 않는 실내 공간에서도 원활하게 작동하며, GPS 대비 훨씬 높은 위치 정확도를 제공한다는 강력한 이점이 있습니다. (그림 1. vSLAM 프로세스 차트 참조)

![[90_Resources/92_Images/SLAM_Figure 1 vSLAM process chart.png]]
![[../90_Resources/92_Images/SLAM_Figure 2.png]]
**vSLAM의 3대 핵심 모듈 및 최적화 과정**
- **vSLAM 3대 핵심 모듈:** vSLAM 알고리즘은 크게 **트래킹(Tracking)**, **로컬 매핑(Local Mapping)**, **루프 클로저(Loop Closure)**의 세 가지 주요 모듈로 구성됩니다. (그림 2 참조)
- **위치 초기화 및 추적:** 구조 로봇은 이 모듈들을 활용해 초기 위치를 설정(Initialization)하고, 새로운 센서 데이터를 실시간으로 분석하여 이동 경로를 추적하며 환경 지도를 생성합니다.
- **특징점(Feature) 데이터 누적:** 로봇이 이동함에 따라 손상된 구조물 내의 문, 벽 모서리, 요구조자의 위치 및 자세(Pose) 등 다양한 특징점 측정값이 지속적으로 증가합니다.
- **환경 표현 최적화 (그림 3 참조):** 데이터가 누적될수록 환경 표현(Environment representation)을 최적화하는 과정이 필수적이며, 이는 로봇에게 상당한 연산량(Computational processing capability)을 요구합니다.
- **효율성 vs 정확도 트레이드오프:** 따라서 최적화 알고리즘의 연산 효율성(Efficiency)과 최종 생성되는 지도의 정확도(Accuracy) 사이에서 적절한 균형을 맞추는 것이 시스템 설계의 핵심입니다.
- **주요 최적화 기법:** 이러한 최적화를 위해 주로 사용되는 두 가지 핵심 기술이 **번들 조정(Bundle Adjustment, BA)**과 **키프레임 선택(Keyframe Selection)**입니다.
![[../90_Resources/92_Images/SLAM_Figure 3.png]]
vSLAM의 예외 상황 처리 및 향후 발전 방향
- **트래킹 실패와 재위치화:** 위치 추적(Tracking)이 실패할 경우, 로봇이 스스로 현재 위치를 다시 파악해야 하므로 **재위치화(Relocalization)** 모듈을 실행합니다.
- **누적 오차(Drift) 보정:** 로봇이 공간을 이동함에 따라 오차가 누적되어(Drift) 위치를 잃을 수 있습니다. 이때 **루프 클로저(Loop Closure)** 기법을 활용한 **전역 지도 최적화(Global Map Optimization)**를 수행하여 궤적의 오차를 바로잡습니다.
- **알고리즘의 강건성(Robustness):** 결론적으로 vSLAM은 동적인 환경 변화에 유연하게 대응할 수 있는 **저비용의 강건한 알고리즘**입니다.
- **한계점 고려:** 단, 실제 구조 현장에 적용하기 위해서는 해당 알고리즘의 실제 성능과 장점뿐만 아니라, 기술적 한계(Limitations)까지 명확히 파악하는 과정이 필수적입니다.
- **다중 카메라(Multi-camera) 도입 모색:** 현재 로봇의 성능 향상을 위해 다중 카메라를 장착하는 방안도 검토 중입니다.
- **영상 획득 방식 최적화:** 다중 카메라 시스템이 구현될 경우, 시각 정보의 활용을 극대화하기 위해 기존과는 다른 새로운 **이미지 획득 방식(Image Acquisition Scheme)**을 적용할 수 있습니다.