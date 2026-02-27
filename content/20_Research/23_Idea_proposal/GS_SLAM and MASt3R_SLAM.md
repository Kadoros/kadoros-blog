---
tags:
  - note
parent_concept:
  - "[[../21_Paper_Reviews/02_MASt3R-SLAM/MASt3R-SLAM|MASt3R-SLAM]]"
  - "[[../21_Paper_Reviews/01_GS_SLAM/GS_SLAM|GS_SLAM]]"
---
MASt3R 신경망을 사용하는데 걍 픽셀 샘플링 해서 조금만 포이트 클라우드 찍은후 그걸 주변으로 3D GS배치하면?
- 이미 있어요 [[../21_Paper_Reviews/03_3R-GS/3R-GS|3R-GS]]
- 근디 [[../21_Paper_Reviews/02_MASt3R-SLAM/MASt3R-SLAM|MASt3R-SLAM]] 는 2 jun 2025 , [[../21_Paper_Reviews/03_3R-GS/3R-GS|3R-GS]] 는 5 apr 2025
- 뭔가 [[../21_Paper_Reviews/03_3R-GS/3R-GS|3R-GS]] 가 캐치 하지 못한 부분이 있지 않을까?
	- 있긴하군요
		- **3R-GS는 "실시간 로봇 주행"이 불가능합니다.**
		    - 3R-GS는 데이터를 몽땅 모아놓고 수만 번 반복 학습(Iteration)을 돌려 완벽한 모델을 깎아내는 **"오프라인 최적화 도구"**입니다.
			    - ![[../../90_Resources/92_Images/Pasted image 20260128014953.png]]
			    - ![[../../90_Resources/92_Images/Pasted image 20260128015032.png]]
		    - **결정적 이유:** 모든 카메라 포즈를 하나의 **Global MLP** 로 묶어놔서, 로봇이 이동하며 맵이 커질 때마다 **처음부터 다시 학습**해야 하는 구조입니다. 즉, "끝없이 들어오는 데이터(Online)"를 처리할 수 없습니다.

- 그러면 생각해 보자 MASt3R로 sparse하게 초기화하고 3DGS로 맵을 채우는 게..?
	- 이미 있어요 [[../21_Paper_Reviews/04_MASt3R-GS/MASt3R-GS|MASt3R-GS]] 
	- 그런데 큰 환경에서는 터진다 그리고 정확도가 조금 떨어짐 
	- ![[../../90_Resources/92_Images/Pasted image 20260128020654.png]]
- 그러면 큰 환경에서는 터진다 를 집중 해보자 그러면 [[../21_Paper_Reviews/01_GS_SLAM/GS_SLAM|GS_SLAM]] 이 녀석 에서 내가 생각한것  처럼 위의[[../21_Paper_Reviews/04_MASt3R-GS/MASt3R-GS|MASt3R-GS]] 에 다가 
	- 걍 3dgs들을 어떤 부분을 한 구역으로 나누고 그 구역의 중심(카메라의 이동위치)를  그래프의 노드로 하고 강제로 움직이고 그리고 최적화 하면 될지도 
- 라고 생각 했다는 것을 넣어 보면?
- 음 에메해요 
	- 못찾겠어요 뭔가 이런 논문 뭔가 있을 것 같아요.. 교수님 찬스를 쓰도록 하줘 
- 만약 없다 그러면 석사 과정 선배중에 - Large-scale 3DGS 하는 선배 있으니 가르쳐 달라고 빌어야지뭐 



Hierarchical MASt3R-GS SLAM
Initialization: MASt3R를 이용해 Sparse한 포인트 클라우드와 초기 포즈를 빠르게 확보.
Mapping:구역(Sub-map)별로 3D Gaussian을 배치하여 대규모 환경에서의 연산 부하 방지.
Optimization: 카메라 궤적의 노드를 그래프 형태로 관리하고, 루프 클로징(Loop Closing) 시 그래프 최적화(Pose-graph optimization)를 통해 전체 맵의 정합성을 강제로 맞춤.

3R-GS의 한계: Global MLP 구조로 인한 온라인 처리 불가 및 확장성 결여.
MASt3R-GS의 한계: 단일 맵 구조로 인해 대규모 환경(Large-scale)에서 메모리 및 최적화 속도 저하.
맵을 '파편화(Decentralized Sub-maps)'하고 이를 '그래프(Graph)'로 연결하여 **Scalability**를 확보.

Q1 MASt3R-GS가 대규모 환경에서 붕괴되는 문제를 해결하기 위해, 맵을 **Sub-map 단위의 노드로 분할하고 Pose-graph로 최적화**하는 방식을 생각했습니다.분할 맵 방식을 MASt3R와 결합한 최신 논문이 더 있을까요?

Large-scale 환경에서 3DGS를 배치할 때, 새로운 구역으로 진입할 때마다 **새로운 노드를 생성하고 이전 노드의 Gaussian들을 'Freezing'** 시키는 방식이 현실적으로 실시간성(Real-time) 확보에 도움이 될까요?

MASt3R를 단순 초기화용(Sparse Point)으로만 쓸지, 아니면 Sub-map 간의 **Relative Pose를 정교하게 계산하는 Local Loop Closure** 엔진으로 쓰는 것이 더 효율적일까요?





mast3r 는 암묵적 3d gs 는 명시적인되 이게 될려나?
MASt3R가 뱉어내는 **'Confidence Map(신뢰도 지도)'**

  

뭔가 3d gs gs 들 최적화 할떄 mast3r 에 서 특정 픽셀 가지고만 계산해서 오차 보정 할 수도 있지 않을까?

기존 3DGS SLAM은 보통 **'색상(Photometric Loss)'** , 깊이 로스 에만 매달립니다. 
색상이 좀 틀려도, **MASt3R라는 거대 모델이 알고 있는 '기하학적 진리(Geometric Truth)'** 를 픽셀 단위로 주입하기 때문에 맵이 훨씬 단단(Robust)해집니다

![[../../90_Resources/92_Images/Pasted image 20260128153614.png]]  

로스 함수를 이렇게 변경하면 rgbd 입력을 받는 3d gs 에서 걍 깊이를 측정을 안해도 되지 않을까? 