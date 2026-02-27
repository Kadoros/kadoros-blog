---
tags:
  - concept
parent_concept:
---
두가지 시점에서 찍은 rgb 데이터를 가지고 각 픽셀의 깊이를 예측하여 3D point map을 과 그에 대응돼는 **Confidence Map**을  생성 + 더 나아가 특징점 백터와 그 신뢰도를 생성 하는 인공 신경망 

- Matching And Stereo 3D Reconstruction
	- **Matching:** 두 이미지 사이의 동일한 지점을 찾는 기능이 대폭 강화되었습니다.
	- **Stereo:** 두 장의 이미지(Stereo Pair)를 기본 단위로 입력받습니다.
	- **3D Reconstruction:** 입력된 이미지로부터 즉시 빽빽한 3D 점구름(Pointmap)을 생성합니다

- 두 장의 사진을 인풋으로 -> 하나의 신경망 ->3d point cloud를 생성 아웃풋 좌표(X),각 좌표가 얼마나 정확한지에 대한 **Confidence Map** (C) , d-차원의 특징점 벡터($D$)**와 그 신뢰도($Q$)
	- d-차원의 특징점 벡터($D$)**와 그 신뢰도($Q$) 는** MASt3R에만
- [[DUSt3R]] 가 전신 
- 여기서 매칭 성능을 높임
- 