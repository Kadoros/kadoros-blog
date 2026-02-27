---
tags:
  - concept
parent_concept:
---
Interpolation 은 가지고있는 이산적인 데이터를 가지고 연속적인 함수로 만드는 것이다 

<iframe width="560" height="315" src="https://www.youtube.com/embed/Xj129kA3Ci0?si=LV5w6tgvs5MyPS7K" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>





### 1. 이미지 처리 / 딥러닝에서의 보간 (Pixel Interpolation)

#### 1) 최근접 이웃 보간 (Nearest Neighbor Interpolation)

- **원리:** 실수 좌표에서 가장 가까운 **정수 픽셀 하나**의 값을 그대로 가져옵니다.
    
- **장점:** 계산이 가장 빠릅니다.
    
- **단점:** 이미지가 깨져 보입니다(계단 현상, Aliasing).
    
- **미분 관점:** **미분 불가능**합니다. (값이 계단식으로 변하므로 기울기가 0이거나 정의되지 않음). 따라서 딥러닝 학습(Backpropagation)용으로는 부적합합니다.
    

#### 2) [[Bilinear Interpolation|Bilinear Interpolation]]

- **원리:** 1차원 선형 보간을 가로(x)로 한 번, 세로(y)로 한 번 수행합니다. 주변 **4개 픽셀($2 \times 2$)**의 값을 거리 비율에 따라 가중 평균합니다.
    
- **장점:** 적당히 부드럽고 계산 속도도 빠릅니다.
    
- **미분 관점:** **미분 가능(Differentiable)**합니다. 위치($p$)가 아주 조금 변하면 결과값도 부드럽게 변하므로, 아까 보셨던 `arg min` 최적화나 신경망 학습에 표준적으로 사용됩니다.
    

#### 3) 쌍입방 보간 (Bicubic Interpolation)

- **원리:** 주변 **16개 픽셀($4 \times 4$)**을 사용하여 3차 함수(Cubic function)로 곡면을 만들어 값을 추정합니다.
    
- **장점:** Bilinear보다 훨씬 자연스럽고 선명합니다.
    
- **단점:** 계산량이 많아집니다.
    

#### 4) 란초스 보간 (Lanczos Interpolation)

- **원리:** 씽크 함수(Sinc function)를 기반으로 합니다.
    
- **특징:** 이미지 축소/확대 시 퀄리티가 매우 좋지만, 경계면에서 링잉(Ringing, 물결무늬) 현상이 발생할 수 있습니다.
    

---

### 2. 기하학/수학적 보간 (Curve Fitting)

데이터 포인트들을 잇는 선이나 곡선을 그릴 때 사용합니다.

#### 1) 선형 보간 (Linear Interpolation, Lerp)

- **원리:** 두 점 사이를 **직선**으로 연결합니다.
    
- **수식:** $y = y_0 + (y_1 - y_0) \frac{x - x_0}{x_1 - x_0}$
    

#### 2) 다항식 보간 (Polynomial Interpolation)

- **원리:** 모든 점을 지나는 $n$차 다항식을 찾습니다(예: 라그랑주 보간법).
    
- **단점:** 점이 많아지면 끝부분에서 값이 심하게 요동치는 현상(Runge's phenomenon)이 발생합니다.
    

#### 3) 스플라인 보간 (Spline Interpolation)

- **원리:** 구간을 쪼개서 저차 다항식(주로 3차)으로 연결하되, 연결 부위가 매끄럽게(미분 가능하게) 이어지도록 합니다. (예: B-Spline, Bezier Curve)
    
- **특징:** 컴퓨터 그래픽스(CAD, 폰트 디자인)에서 곡선을 그릴 때 필수적입니다.
    

---

### 3. 회전/벡터 보간 (SLAM/Robotics 중요)

이전 질문에서 다룬 **단위 벡터($\|\psi\|=1$)**나 회전 변환과 관련이 깊습니다.

#### 구면 선형 보간 (Spherical Linear Interpolation, SLERP)

- **문제점:** 일반 선형 보간(Lerp)으로 두 단위 벡터 사이를 이으면, 중간 지점의 벡터 길이가 1보다 작아져서(원 안쪽으로 파고듦) 회전 속도가 일정하지 않게 됩니다.
    
- **해결:** **구(Sphere)의 표면을 따라** 최단 거리(호, Arc)로 이동하며 보간합니다.
    
- **용도:** 3D 회전(쿼터니언)을 부드럽게 연결하거나, 카메라의 방향을 보간할 때 사용합니다.
    

---

### 요약: 님께서 보신 코드에서의 상황

님께서 질문하신 코드(`mast3r_slam`)나 수식($\mathbf{p}^* = \operatorname{arg\,min} \dots$) 상황에서는 **"쌍선형 보간 (Bilinear Interpolation)"**이 가장 중요합니다.

1. **실수 좌표 $p$를 다뤄야 함:** 픽셀 사이의 값을 가져와야 함.
    
2. **미분이 되어야 함:** 그래야 `iter_proj` 과정에서 기울기(Gradient)를 타고 최적의 위치로 이동할 수 있음.
    
3. **속도:** SLAM은 빨라야 하므로 Bicubic보다는 Bilinear를 선호함.