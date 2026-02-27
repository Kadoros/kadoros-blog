---
tags:
  - concept
parent_concept:
---
- Dense Unconstrained Stereo 3D Reconstruction 


- 두 장의 사진을 인풋으로 -> 하나의 신경망 ->3d point cloud를 생성 아웃풋,각 좌표가 얼마나 정확한지에 대한 **Confidence Map** 

- 모든 광선이 하나의 중심을 지난다는 **중심 카메라 모델(Central Camera Model)** 가정 하나만으로 작동합니다.

- 이거는 하나의 신경망에서 처리
	-  원래 SLAM이나 3D 복원은 '특징점 추출 -> 매칭 -> 기하학적 계산'
		- 그리고 카메라 정보 알아야함
	- 복잡한 기하학적 단계 삭제
	-  카메라 정보 없이 (Unconstrained)
			- 다양한 카메라로 이런 이미지는 이런 랜즈로 찍으면 이런 3d 모양이 나옴
- 각 픽셀이 3D 공간의 어디에 위치하는지(**Pointmap**)를 직접 **회귀(Regression)** 
- 속도느림, 정교한 픽셀 단위 매칭(Matching) 한계([[MASt3R]])으로의 발전