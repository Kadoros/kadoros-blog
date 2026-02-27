---
tags:
  - Paper
  - SLAM
parent_concept:
  - "[[../../../50_Concepts/SLAM|SLAM]]"
  - "[[50_Concepts/3D Reconstruction|3D Reconstruction]]"
status: 2st Reading
date: 2026-01-27
paper_link: https://arxiv.org/abs/2311.11700
pdf: "[[PDF/GS_SLAM_PDF.pdf]]"
code_link:
---
![[../../../90_Resources/92_Images/GS_SLAM_20260221_1101.png]]
# introduction
최근 NeRF 같은 Neural implicit representation을 사용 하는 방법론들이 많이 등장 했지만 volume randering 의 속도 한계로 인해 실시간 최적화 및 고해상도 렌더링에 어려움이 있었다.

본 논문에서는 처음으로 3D Gaussian Splatting를 베이스로 하는 dense map를 만드는 RGB-D SLAM의 접근 방식을 제시한다. 이는 빠른 splatting 렌더링 기술을 이용해서 mapping optimizing, pose tracking, 실시간성, photo-realistic reconstruction 성능을 향상 시킴 

또한 효과적으로 새롭게 관찰된 장면의 기하학적 특징들을 재건하기 위해 adaptive 3D Gaussian expansion strategy를 사용하고 camera pose estimation을 위해 신뢰 할 수 있는 3D Gaussians를 선택 하기 위해서 coarse-to-fine 기술을 사용함 

본 논문의 기술은 Replica 와 TUM-RGBD datasets를 사용하여 tracking과 mapping을 테스트 했을때, 8.43 FPS의 성능을 확인 할 수 있었으며 이는 efficiency와 accuracy사이의 좋은 벨런스를 가진다. 

# Methodology
![[../../../90_Resources/92_Images/GS_SLAM_20260223_1629.png]]
##  1. 3D Gaussian Scene Representation
GS-SLAM은 scene을 캡쳐 하고 reconstruct하기 위해서 3D GS의 집합을 모델링 한다. 각 3D Gaussian $G_i$는 위치 $X_i$, 3D 공분산 $\Sigma_i$, 불투명도 $\Lambda_i$, 그리고 구면 조화 함수(Spherical Harmonics, $Y_i$)로 정의된다.
$$\mathbf G = \{{Gi : (X_i, Σ_i, Λ_i, Y_i)|i = 1, ..., N}\}$$
**3D Gaussian’s covariance**
3D scale vector $S$ 와 the rotation matrix $R$을 사용하여 각각의 3D 공분산 $\Sigma$ 을 표현한다 여기서 the rotation matrix $R$ 는 quaternion 으로 표시됨. 이것은 최적화와 정규화의 장점이 있음 
$$Σ = RSS^T R^T ,$$
**3D-to-2D Gaussian Projection**
camera pose $\mathbf P = \{R, t\}$ 와 the affine approximation of the [[../../../50_Concepts/projective function|projective function]] $\mathbf J$를 사용하여 3D-to-2D Gaussian Projection을 진행 한다. $P^{-1} \Sigma P^{-T}$ 로 3D Gaussian을 카메라 중심의 좌표게로 변환 한다. $J P^{-1} \Sigma P^{-T} J^T$ 로 3D Gaussian 을 2D Gaussian 으로 project한다. $$\Sigma' = J P^{-1} \Sigma P^{-T} J^T$$projective function은 Z로 나누는 연산 때문에 non-linear function이다. 
3D 가우시안을 비선형 함수에 통과시키면 타원 모양이 깨져버려 렌더링이 불가능해진다. 
이를 해결하기 위해 투영 함수를 가우시안 중심점에서 1차 미분하여 선형 함수로 근사(Affine approximation)한다. 이때 사용되는 미분 행렬이 바로 야코비안 $J$이다. 

**$\alpha$-blanding randering**
각 픽셀의 camera Ray 가 반투명한 3D Gaussians를 통과할때 빛의 누적을 계산한다.
$$\hat{C} = \sum_{i \in N} c_i \alpha_i \prod_{j=1}^{i-1} (1 - \alpha_j), \quad  \hat{D} = \sum_{i \in N} d_i \alpha_i \prod_{j=1}^{i-1} (1 - \alpha_j)$$
 $\prod (1 - \alpha_j)$ 항은 Transmittance를 의미하며 앞의 Gaussians로 인해서 막히지 않고 통과될 확률이다. 따라서 뒤에 있는 Gaussian들은 색과 깊이에 영향을 주지 못한다. $\alpha$ 는 각 픽셀에서의 불투명도이고 Gaussian의 중앙에서의 태생적인 불투명도 $\Lambda$ 에서 계산된다. 
 색상($\hat{C}$) 뿐만 아니라 예상 깊이($\hat{D}$) 또한 $\alpha$-blanding 으로 계산된다. 


## 3.2. Adaptive 3D Gaussian Expanding Mapping
L1 Loss 절대 오차를 사용 하여 loss function을 구성 하고 있다. 
$$\mathcal{L}_c = \sum_{m=1}^{HW} \left| C_m - \hat{C}_m \right|, \quad \mathcal{L}_d = \sum_{m=1}^{HW} \left| D_m - \hat{D}_m \right|$$
$\mathcal{L}_c$로 scene의 appareance를 최적화 함과 동시에 $\mathcal{L}_d$로 geometry를 최적화 한다. 또한 이는 모두 keyframe 에 contribute 한 3D Gaussians을 대상으로 함으로써 computing cost를 줄임.



# 요약
- 처음으로  [[../../../50_Concepts/3D Gaussian representation|3D Gaussian representation]] 을 이용한  [[../../../50_Concepts/SLAM|SLAM]] 
- 다른 것들은 3프래임 나올떄 이녀석은 300프래임 넘꼐 나옴 
- 기본적으로 RGB-D 입력을 받음 
	- 만약 이걸 적외선으로 받는 다면 불나면 무용지물
- 메모리가 4배정도 많이씀

# 내 생각
- 만약 적외선으로 d값을 안받는 다면 화제 환경에서도 사용 가능 할 듯 
- 메모리가 많이 쓰이니까 서버에서 돌리는 면 될지도 
- 여기에는 loop closure 나 전역 최적화가 논의 되지 않음
	- 걍 3dgs들을 어떤 부분을 한 구역으로 나누고 그 구역의 중심(카메라의 이동위치)를  그래프의 노드로 하고 강제로 움직이고 그리고 최적화 하면 될지도 
# Sections
