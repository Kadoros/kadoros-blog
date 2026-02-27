---
tags:
  - Paper
  - SLAM
parent_concept:
  - "[[../../../50_Concepts/SLAM|SLAM]]"
  - "[[../../../50_Concepts/MASt3R|MASt3R]]"
status: 2nd Reading
date: 2026-01-27
paper_link: https://arxiv.org/abs/2412.12392
pdf: "[[PDF/MASt3R-SLAM_pdf.pdf]]"
code_link:
published: "true"
---
 
# 요약 
- 핵심 
	- [[MASt3R]] 이라는 인공 신경망을 통해서 두 이미지 인풋을 통해 3d point map 과 특징점 백터 그리고 신뢰도 를 각각 만들고 이를 통해서 설계된 실시간  [[../../../50_Concepts/Monocular SLAM|Monocular SLAM]], [[Dense SLAM]]을 구현
- 특징 
	- 많은 카메라 모델에 대하여 [[MASt3R]] 가 학습 되었기에 카메라 모델에 대한 정보가 없어도 가능함, 즉 Intrinsic Matrix ($K$) 없이 구현됨 (Plug-and-play)시스탬
	-  [[MASt3R]] 의 강력한 사전 지식 덕분에 일관된 포즈와 밀집된 3D 형상을 구현 할 수 있음
- 성능
	- 4090환경에서 15fps : 실시간 성능

# 되는 것
- 아무 카메라나 사용해도됨 
	- 즉 K 행열 연산 안해도됨
- 3d 형상이 고품질
- 실시간 처리
- 카메라 캘리브레이션이 있으면 sota 성능

# 안되는 것
- 카메라가 왜곡이 심하면 작동 안됨
- 무거움 
- 실시간 3d 맵 전역 최적화; 포즈는 전역 최적화함
- 어려운 환경 예 저조도 격한 움직임

# output
![[../../../90_Resources/92_Images/MASt3R-SLAM_20260217_0719.png]]
![[../../../90_Resources/92_Images/MASt3R-SLAM_20260217_0720.png]]
# Sections

## Abstract
- This paper present a real-time monocular [[../../../50_Concepts/Dense SLAM|Dense SLAM]] system designed bottom-up from [[../../../50_Concepts/MASt3R|MASt3R]], a two-view 3D reconstruction and matching prior.
- MASt3R의 강력한 능력 지능
- 와일드한 현장에서 강함
- 카메라 정보 필요 없음 (plug-and-play)
	- 만약 있으면 정확도 확 상승
- Efficient Pointmap Matching, 
	- MASt3R는 무거움 이걸 해결함
- Camera Tracking and Local Fusion
- Graph Construction and Loop Closure
	- 카메라 경로 그래프 형태로 , Loop Closure로 오차 잡음
- Second-order Global Optimisation
- 실시간으로 구동  15fps

## 3. Method
### 3.1 Preliminaries

#### 3.1.1.  $X_i^j$ (Pointmap Notation)
$$\mathbf{X}_i^j \in \mathbb{R}^{H \times W \times 3}$$
* $i$: 어떤 이미지의 점들인가
*  $j$: 누구의 눈(좌표계)으로 보고 있는가 

#### 3.1.2.  $Sim(3)$ (Similarity Transform)
$$\mathbf{T} = \begin{bmatrix} s\mathbf{R} & \mathbf{t} \\ 0 & 1 \end{bmatrix}, \quad \mathbf{T} \in Sim(3)$$
*   보통의 SLAM은 회전($\mathbf{R}$)과 이동($\mathbf{t}$)만 있는 $SE(3)$를 쓴다.
* 하지만 MASt3R 같은 신경망이 예측한 3D 지도들은 기하학적 구조는 정확할지 몰라도, "절대적인 크기(Scale)"가 제각각일 때가 많음
*  그래서 수식에 **스케일 인자($s$)를 추가 지도를 늘렸다 줄였다 하며 맞춰감


#### 3.1.3. $\psi(\cdot)$ (Normalization & Generic Camera)
$$\psi(\mathbf{X}) = \frac{\mathbf{X}}{\|\mathbf{X}\|}$$
* ray를 normalize 함 
* 이 덕분에 $K$ 행렬없어도됨
	* 전통적인 방법: 픽셀 -> K 행렬 연산 -> ray
	* MASt3r 방법: 픽셀 -> 포인트맵 예측 -> $\psi(\mathbf{X})$ -> ray



#### 3.1.4. Lie Algebra ($\mathfrak{sim}(3)$) & Left-plus ($\oplus$)
$$\mathbf{T} \leftarrow \tau \oplus \mathbf{T} \triangleq \text{Exp}(\tau) \circ \mathbf{T}, \quad \tau \in \mathfrak{sim}(3)$$
?


####  3.1.5. 신뢰도 (Confidences, $C, Q$)
$$\mathbf{C}_i^i \in \mathbb{R}^{H \times W \times 1}, \quad \mathbf{Q}_i^i \in \mathbb{R}^{H \times W \times 1}$$
* $C$는 3D 형상에 대한 Confidences
* $Q$는 매칭(특징점)에 대한 Confidences
* 최적화할 때 노이즈에 강해짐

---


### 3.2. Pointmap Matching
- 
$$\mathbf{p}^* = \arg \min_{\mathbf{p}} \left\| \psi \left( [\mathbf{X}_i^i]_{\mathbf{p}} \right) - \psi (\mathbf{x}) \right\|^2 . \space \space \space \space \space where \space  x∈ X_{i}^j$$
- ⭐️ ***p는 pixel coordinates이며 이를 최적화 한다
- 주의 여기서 p는 실수이다!!(Sub-pixel Precision)
- 즉 모든 좌표계는 i 에서 찍은 걸로 간주할 때, j위치에 찍은 어떤 포인트와 카메라 중심 선과 대하여 i위치에서 찍은 포인트맵중 p에서의 포인트와 카메라 중심 선를 가져온 후 각각의 선을 단위 벡터로 만든 뒤 
####  3.2.1   $\psi (\mathbf{X}_i^i)  , \psi (\mathbf{x})$ 
@`/mast3r_slam/matching.py`
@`match_iterative_proj`
```python
def match_iterative_proj(X11, X21, D11, D21, idx_1_to_2_init=None):
	#config
	rays_with_grad_img, pts3d_norm, p_init = prep_for_iter_proj (
											X11, X21, idx_1_to_2_init)
```

- **X11, X21**: 두 이미지의 3D Pointmap (MASt3R 예측값). -> $\mathbf{X}_i^i$ and $\mathbf{X}_i^j$ 
- **D11, D21**: 두 이미지의 Feature Descriptor (매칭 정제용). -> 이거 어디니?
- **idx_1_to_2_init**: 이미지 1의 각 픽셀이 이미지 2의 어떤 위치와 매칭될 것인지에 대한 초기 추정값을 담고 있는 정수형 인덱스 텐서
@`/mast3r_slam/matching.py`
@`prep_for_iter_proj`
```python
def prep_for_iter_proj(X11, X21, idx_1_to_2_init):
	#환경 설정 등 
	
	# Ray image
	rays_img = F.normalize(X11, dim=-1)
	# 기울기 정보?
	rays_img = rays_img.permute(0, 3, 1, 2) # (b,c,h,w)
	gx_img, gy_img = img_utils.img_gradient(rays_img)
	rays_with_grad_img = torch.cat((rays_img, gx_img, gy_img), dim=1)
	rays_with_grad_img = rays_with_grad_img.permute(
	0, 2, 3, 1
	).contiguous() # (b,h,w,c)
	
	# 3D points to project
	X21_vec = X21.view(b, -1, 3)
	pts3d_norm = F.normalize(X21_vec, dim=-1)
	
```

- X21_vec = X21.view(b, -1, 3) : 이미지 $j$의 포인트맵 데이터인 `X21`을 한 점($x, y, z$)씩 나열된 벡터 형태(`X21_vec`)로 재구성하는 것 ->  $x∈ X_{i}^j$
- pts3d_norm = F.normalize(X21_vec, dim=-1): 단위 백터로 변환 ->  $\psi (\mathbf{x})$ 
- rays_img = F.normalize(X11, dim=-1) : x11의 생 ray를 구하고 단위 백터로 면환 하는 과정 과정 -> $\psi (\mathbf{X}_i^i)$ 




#### 3.2.2.  $[\mathbf{X}_i^i]_{\mathbf{p}}$
이 것은 Pointmap $\mathbf{X}_i^i$의 실수 좌표 p 위치에서 ray 값을 가져오라 이다.
$[⋅]p$ :  **Sampling**, [[../../../50_Concepts/Interpolation|Interpolation]]

그런데 p는 실수 좌표 라는 것을 기억 해야 한다. 
우리는 식수 좌표에 대한  ray 값이 없다. 
 그 실수 p값을 사용해서 ray 값을 계산 해야 한다. 여기서는 따라서 우리는 [[../../../50_Concepts/Interpolation|Interpolation]] 중에서도 [[../../../50_Concepts/Bilinear Interpolation|Bilinear Interpolation]]을 사용해서 한다 

이것은 gpu의 병열 계산을 위해  [[../../../50_Concepts/CUDA|CUDA]] 언어를 이용하여 [[../../../50_Concepts/GPU Kernel|GPU Kernel]] 을 사용한다

@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
// Bilinear interpolation
int u11 = static_cast<int>(floor(u));
int v11 = static_cast<int>(floor(v));
float du = u - static_cast<float>(u11);
float dv = v - static_cast<float>(v11); 

// Clamping always ensures full bilinear is fine to calculate
float w11 = du * dv; // top left
float w12 = (1.0-du) * dv; // top right
float w21 = du * (1.0-dv); // bottom left
float w22 = (1.0-du) * (1.0-dv); // bottom right 

// NOTE: Pixels are opposite the area calc!
float const* r11 = &rays_img[b][v11+1][u11+1][0]; // bottom right
float const* r12 = &rays_img[b][v11+1][u11][0]; // bottom left
float const* r21 = &rays_img[b][v11][u11+1][0]; // top right
float const* r22 = &rays_img[b][v11][u11][0]; // top left

#pragma unroll
for (int j=0; j<3; j++) {
	r[j] = w11*r11[j] + w12*r12[j] + w21*r21[j] + w22*r22[j];
}
```
1. 좌표의 각 정수 실수 정보를 가져옴
2. 각 가중치를 계산: 
	1. 현재 좌표의 소수점 위치(`du`, `dv`)를 기준으로 4개 영역의 넓이를 계산합니다. (이 넓이는 기하학적으로 **대각선 반대편 픽셀**이 가져가야 할 가중치가 됩니다.)
3. 각 픽셀의 ray 정보를 가져온다 
4. 각 ray와 가중치를 각각 반대 쪽에 있는 것들 끼리 곱한후 더한다 
5. 실수 좌표의 ray값이 반환된다 
	1. 즉 이것이 $[\mathbf{X}_i^i]_{\mathbf{p}}$ 이다. 계산되었다.




#### 3.2.3 $\psi \left( [\mathbf{X}i^i]_{\mathbf{p}} \right)$
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
// Normalize ray
float r_norm = sqrtf(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
float r_norm_inv = 1.0/r_norm;

#pragma unroll
for (int j=0; j<3; j++) {
	r[j] *= r_norm_inv;
}
```
이건 각 길이가 1인 4개의 ray에 가중치를 곱해서 나온 p에 대한 ray가 길이가 1이 아니기 때문에 그에 대한 normalization 해준 것이다 
#### 3.2.4  기하학적 해석
![[../../../90_Resources/92_Images/MASt3R-SLAM_geometric_interpretation.png]]
1. **카메라 원점 (Camera Center):** 좌표계의 원점 O=(0,0,0) 에 위치함.
2. **광선 (Ray):** 원점 O에서 뻗어나가는 화살표.
3. **단위 구 (Unit Sphere):** 원점을 감싸고 있는 반지름 1짜리 구.
4. **투영 :** 3D 공간상의 점 X를 원점과 이었을 때, 선이 **구의 표면과 만나는 점**이 바로ψ(x) 


#### 3.2.5 $\arg \min_{\mathbf{p}} \left\| \psi \left( [\mathbf{X}_i^i]_{\mathbf{p}} \right) - \psi (\mathbf{x}) \right\|^2$

##### 3.2.5.1. How to optimize 
Squared Euclidean Distance를 cost 로 하여  [[../../../50_Concepts/Levenberg Marquardt|Levenberg-Marquardt|]] 알고리즘으로 푼다
즉 Squared Euclidean Distance가 cost fuction인 것이다. 

$$E(\mathbf{p})=\underbrace { \left\| \psi \left( [\mathbf{X}_i^i]_{\mathbf{p}} \right) - \psi (\mathbf{x}) \right\|^2}_{Squared Euclidean Distance}$$

이 식을 사용하여 최적화 하기 위해서 Residual Vector, r 의 형태로 표현한다. 
$$r(\mathbf{p}) = \psi \left( [\mathbf{X}_i^i]_{\mathbf{p}} \right) - \psi (\mathbf{x}) $$@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
for (int j=0; j<3; j++) {
	err[j] = r[j] - pts_3d_norm[b][n][j];
}
```
$$E(\mathbf{p})= \left\| r(\mathbf{p}) \right\|^2 =r^Tr$$@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
float cost = err[0]*err[0] + err[1]*err[1] + err[2]*err[2];
```


최적의 이동량 $\Delta p$를 구하기 위해, **Gauss-Newton** 방법에 Damping Factor $\lambda$를 추가한  [[../../../50_Concepts/Levenberg Marquardt|Levenberg Marquardt]] 공식을 사용한다 
$$(J^T J + \lambda I) \delta = J^T e$$
논문에서는 [[../../../50_Concepts/Analytical Jacobians|Analytical Jacobians]]를 사용한다고 언급되어 있으나, 이는 수식적으로 Pointmap의 공간적 변화율(Spatial Gradient)을 의미한다. @8.2. Rays and Distance 에서 확인 할 수 있다.(사실 뭔말인지 모르겠다) 

최적화 대상은 픽셀 좌표 $\mathbf{p}=(u, v)$이며, 이를 $\mathbf{p}$에 대해 편미분한 [[../../../50_Concepts/Jacobian|Jacobian]] $\mathbf{J}$는, Pointmap 이미지의 공간적 변화율(Spatial Gradient, **Geometric Ray Gradient**) 자체가 Jacobian J 가 된다 
$$\mathbf{J} = \begin{bmatrix} \frac{\partial \mathbf{r}}{\partial u} & \frac{\partial \mathbf{r}}{\partial v} \end{bmatrix}$$
##### 3.2.5.2. why $J = \nabla \mathbf{I}$ (Spatial Gradient, **Geometric Ray Gradient**)??


$$\mathbf{r}(\mathbf{p}) = \underbrace{\psi([\mathbf{X}_i^i]_{\mathbf{p}})}_{\text{현재 픽셀 } \mathbf{p} \text{에서의 Ray 값}} - \underbrace{\psi(\mathbf{x})}_{\text{타겟 Ray 값 (상수)}}$$
$$let \space \mathbf{I}(\mathbf{p}) = \psi([\mathbf{X}_i^i]_{\mathbf{p}})$$
- $\mathbf{I}$는 좌표 $\mathbf{p}$를 넣으면 해당 위치의 Ray 벡터를 뱉어내는 **이미지 함수**!! 색상 값이 아님
- $\mathbf{T} = \psi(\mathbf{x})$는 우리가 찾고자 하는 정답 값이므로 상수(Constant)
$$\mathbf{J} = \frac{\partial \mathbf{r}}{\partial \mathbf{p}} = \frac{\partial}{\partial \mathbf{p}} \left( \mathbf{I}(\mathbf{p}) - \mathbf{T} \right)$$
$$\mathbf{J} = \frac{\partial \mathbf{I}(\mathbf{p})}{\partial \mathbf{p}} - \underbrace{\frac{\partial \mathbf{T}}{\partial \mathbf{p}}}_{0 (\text{상수 미분})}$$
$$\mathbf{J} = \begin{bmatrix} \frac{\partial \mathbf{I}}{\partial u} & \frac{\partial \mathbf{I}}{\partial v} \end{bmatrix}= \nabla \mathbf{I}$$
**이 수식이 의미하는 바는 무엇일까요?**
- $\frac{\partial \mathbf{I}}{\partial u}$: 픽셀 $u$(가로)가 변할 때, 이미지의 값(Ray)이 얼마나 변하는가?
- $\frac{\partial \mathbf{I}}{\partial v}$: 픽셀 $v$(세로)가 변할 때, 이미지의 값(Ray)이 얼마나 변하는가?
이것은 수학적으로 **이미지 그라디언트(Image Gradient, $\nabla \mathbf{I}$)의 정의와 완벽하게 일치


#####  3.2.5.3.  Calculate J
잔차 $\mathbf{r}$은 3차원 Ray의 차이이므로 $x, y, z$ 성분을 가진다.

$$ \mathbf{r}(\mathbf{p}) = \begin{bmatrix} r_x(u, v) \\ r_y(u, v) \\ r_z(u, v) \end{bmatrix} = \begin{bmatrix} \psi_x([\mathbf{X}]_{\mathbf{p}}) - \psi_x(\mathbf{x}_{target}) \\ \psi_y([\mathbf{X}]_{\mathbf{p}}) - \psi_y(\mathbf{x}_{target}) \\ \psi_z([\mathbf{X}]_{\mathbf{p}}) - \psi_z(\mathbf{x}_{target}) \end{bmatrix} $$
이제 $\mathbf{r}$을 $\mathbf{p}=(u, v)$로 편미분하면,  **$3 \times 2$ 행렬**이 나옴
$$
\mathbf{J} = \frac{\partial \mathbf{r}}{\partial \mathbf{p}} = 
\begin{bmatrix} 
\frac{\partial \mathbf{r}}{\partial u} & \frac{\partial \mathbf{r}}{\partial v} 
\end{bmatrix} 
=
\begin{bmatrix} 
\frac{\partial r_x}{\partial u} & \frac{\partial r_x}{\partial v} \\
\frac{\partial r_y}{\partial u} & \frac{\partial r_y}{\partial v} \\
\frac{\partial r_z}{\partial u} & \frac{\partial r_z}{\partial v}
\end{bmatrix}
$$

여기서 각 열(Column) 벡터를 코드 변수와 매핑하면 다음과 같다
*   **첫 번째 열 (Column 1):** $u$ 방향 변화율 $\rightarrow$ **`gx` (3차원 벡터)**
$$\mathbf{g}_x = \frac{\partial \mathbf{r}}{\partial u} = \begin{bmatrix} \frac{\partial r_x}{\partial u} \\ \frac{\partial r_y}{\partial u} \\ \frac{\partial r_z}{\partial u} \end{bmatrix}$$

* **두 번째 열 (Column 2):** $v$ 방향 변화율 $\rightarrow$ **`gy` (3차원 벡터)**$$\mathbf{g}_y = \frac{\partial \mathbf{r}}{\partial v} = \begin{bmatrix} \frac{\partial r_x}{\partial v} \\ \frac{\partial r_y}{\partial v} \\ \frac{\partial r_z}{\partial v} \end{bmatrix}$$
즉, 자코비안 행렬은 **$\mathbf{J} = [\mathbf{g}_x, \mathbf{g}_y]$** 형태


#####  3.2.5.4.   J 계산과 코드 맵핑
$\mathbf{g}_x, \mathbf{g}_y$를 구해야 한다.
@`/mast3r_slam/matching.py`
@`prep_for_iter_proj`
```python
	gx_img, gy_img = img_utils.img_gradient(rays_img)
	rays_with_grad_img = torch.cat((rays_img, gx_img, gy_img), dim=1)
	rays_with_grad_img = rays_with_grad_img.permute(
	0, 2, 3, 1
	).contiguous() # (b,h,w,c)
```
gx_img, gy_img가 ray 미분 값을 가지고 있다. 그리고는 이걸 진짜 ray 값과 합쳐 rays_with_grad_img를 만든다. 

@`/mast3r_slam/image.py`
@`img_gradient(img)`
```python
gx_kernel = (1.0 / 32.0) * torch.tensor(
	[[-3.0, 0.0, 3.0], [-10.0, 0.0, 10.0], [-3.0, 0.0, 3.0]],
	requires_grad=False, device=device, dtype=dtype,
)
gx_kernel = gx_kernel.repeat(c, 1, 1, 1)
  
gy_kernel = (1.0 / 32.0) * torch.tensor(
	[[-3.0, -10.0, -3.0], [0.0, 0.0, 0.0], [3.0, 10.0, 3.0]],
	requires_grad=False, device=device, dtype=dtype,
)
gy_kernel = gy_kernel.repeat(c, 1, 1, 1)
  
```

$$f'(x) \approx \frac{f(x+1) - f(x-1)}{2}$$
이산 데이터에서 미분이란 결국에는 차분이 된다. 

일반적인 Sobel 필터(`1, 2, 1`)보다 중심 픽셀에 더 큰 가중치를 주는 **Scharr Operator**(`3, 10, 3`) 변형을 사용함
**정규화 계수 `1.0 / 32.0`**: 커널 요소의 절댓값 합($3+10+3 + 3+10+3 = 32$)으로 나누어, 미분 값의 스케일이 픽셀 값 변화량과 일치하도록 보정함

- **`gx_kernel` (수평 미분)**:$$\frac{1}{32} \begin{bmatrix} -3 & 0 & 3 \\ -10 & 0 & 10 \\ -3 & 0 & 3 \end{bmatrix}$$
- **`gy_kernel` (수직 미분)**:$$\frac{1}{32} \begin{bmatrix} -3 & -10 & -3 \\ 0 & 0 & 0 \\ 3 & 10 & 3 \end{bmatrix}$$
@`/mast3r_slam/image.py`
@`img_gradient(img)`
```python
gx = F.conv2d(
	F.pad(img, (1, 1, 1, 1), mode="reflect"),
	gx_kernel,
	groups=img.shape[1],
)
  
gy = F.conv2d(
	F.pad(img, (1, 1, 1, 1), mode="reflect"),
	gy_kernel,
	groups=img.shape[1],
)

return gx, gy
```
테두리의 기울기가 튀는 걸 방지 하기 위해서 padding을 넣어준다.
기본값인 groups=1 로 하면 x,y,z값이 가 합쳐짐 그래서 groups=img.shape[1], 즉 3 으로 적용해서 각 x,y,z 채널을 살리며 conv를 진행해준다. 

@`/mast3r_slam/matching.py`
@`match_iterative_proj`
```python
rays_with_grad_img, pts3d_norm, p_init = prep_for_iter_proj(
	X11, X21, idx_1_to_2_init
)

p1, valid_proj2 = mast3r_slam_backends.iter_proj(
	rays_with_grad_img,
	pts3d_norm,
	p_init,
	cfg["max_iter"],
	cfg["lambda_init"],
	cfg["convergence_thresh"],
)
```
구한 기울기 데이터를 cuda backend로 넘겨준다.

@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
#pragma unroll
for (int j=3; j<6; j++) {
	gx[j-3] = w11*r11[j] + w12*r12[j] + w21*r21[j] + w22*r22[j];
}
```
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
#pragma unroll
for (int j=6; j<9; j++) {
	gy[j-6] = w11*r11[j] + w12*r12[j] + w21*r21[j] + w22*r22[j];
}
```

그리고는 cuda backend 에서 interpolation 작업을 해준다.

왜 conv 는 파이토치, interpolation은 cuda?
- Pytorch에서는 이미 gpu에 최적화한 라이브러리가 있음 그래서 ㄱㅊ
- 레지스터에 값 넣고 for 문 돌리는건 cuda가 잘함
#####   3.2.5.5. Levenberg Marquardt($(J^T J + \lambda I) \delta = J^T e$)
###### 3.2.5.5.1 $\mathbf{J}^T \mathbf{J}$
$$
\mathbf{J}^T \mathbf{J} = 
\begin{bmatrix} \mathbf{g}_x^T \\ \mathbf{g}_y^T \end{bmatrix} 
\begin{bmatrix} \mathbf{g}_x & \mathbf{g}_y \end{bmatrix}
=
\begin{bmatrix} 
\mathbf{g}_x \cdot \mathbf{g}_x & \mathbf{g}_x \cdot \mathbf{g}_y \\
\mathbf{g}_y \cdot \mathbf{g}_x & \mathbf{g}_y \cdot \mathbf{g}_y
\end{bmatrix}
$$
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
// Setup system
// J^T J
// A00 = gx dot gx (1행 1열)
float A00 = gx[0]*gx[0] + gx[1]*gx[1] + gx[2]*gx[2];
// A01 = gx dot gy (1행 2열)
float A01 = gx[0]*gy[0] + gx[1]*gy[1] + gx[2]*gy[2];
// A11 = gy dot gy (2행 2열)
float A11 = gy[0]*gy[0] + gy[1]*gy[1] + gy[2]*gy[2];
```

note! A10 = A01 ($\mathbf{g}_y \cdot \mathbf{g}_x = \mathbf{g}_x \cdot \mathbf{g}_y$)


$$\mathbf{J}^T \mathbf{J} = \begin{bmatrix} 
A_{00} & A_{01}\\
A_{10} & A_{11}
\end{bmatrix}$$
###### 3.2.5.5.2 $J^T e$
여기서 e 는 오차이다 즉 e = r(잔차 백터)인 것이다

그리고 $-\mathbf{J}^\mathbf{T} \mathbf{r}$을 구해야 한다. 그래야 최적화가 된다 왜냐? cost function($\mathbf{r}$)의 아래로 가야 하기 때문
$$\mathbf{J}^\mathbf{T} \mathbf{e}= \mathbf{J}^\mathbf{T} \mathbf{r}=
\begin{bmatrix} \mathbf{g}_x^T \\ \mathbf{g}_y^T \end{bmatrix} 
\begin{bmatrix} \mathbf{r} \end{bmatrix}
=
\begin{bmatrix} 
\mathbf{g}_x \cdot \mathbf{r}  \\
\mathbf{g}_y \cdot \mathbf{r} 
\end{bmatrix}
$$
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel()`
```cpp
// - J^T r
float b0 = - (err[0]*gx[0] + err[1]*gx[1] + err[2]*gx[2]);
float b1 = - (err[0]*gy[0] + err[1]*gy[1] + err[2]*gy[2]);
```

$$\mathbf{J}^\mathbf{T} \mathbf{e}=
\begin{bmatrix} 
b_{0} \\
b_{1} 
\end{bmatrix}$$
###### 3.2.5.5.3 $(J^T J + \lambda I)$
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel(... , const float lambda_init, ...)`
```cpp
float lambda = lambda_init;

.
.
.
// LM diagonal
A00 += lambda;
A11 += lambda;
```
 identity matrix는 대각 행열이다 그래서 $J^T J$의 대각 성분에만 $\lambda$를 더한다. 
 - **$\lambda$가 클 때:** 행렬이 단위 행렬에 가까워져 **Gradient Descent**처럼 동작합니다. (안전하게 조금씩 이동)
- **$\lambda$가 작을 때:** 원래 Hessian에 가까워져 **Gauss-Newton**처럼 동작합니다

###### 3.2.5.5.4 $\delta$ 그리고 새로운 픽셀 좌표
$$\delta = \Delta \mathbf{p}= \begin{bmatrix} 
\Delta {u}  \\
\Delta {v} 
\end{bmatrix}$$
$$(J^T J + \lambda I) \delta = J^T e$$
$$\begin{bmatrix} A_{00} & A_{01}\\A_{10} & A_{11} \end{bmatrix} \begin{bmatrix} \Delta {u} \\ \Delta {v} \end{bmatrix} = \begin{bmatrix} b_{0} \\ b_{1}  \end{bmatrix}$$

$$\begin{bmatrix} \Delta u \\ \Delta v \end{bmatrix} = \frac{1}{\det} \begin{bmatrix} A_{11} & -A_{01} \\ -A_{01} & A_{00} \end{bmatrix} \begin{bmatrix} b_0 \\ b_1 \end{bmatrix}$$$$\frac{1}{\det(H)} = \frac{1}{A_{00}A_{11} - A_{01}^2}$$@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel(... , const float lambda_init, ...)`
```cpp
// Solve system
float det_inv = 1.0/(A00*A11 - A01*A01);
float delta_u = det_inv * ( A11*b0 - A01*b1);
float delta_v = det_inv * (-A01*b0 + A00*b1);

// Get new pixel 
float u_new = u + delta_u;
float v_new = v + delta_v;

clamp(u_new, 1, w-2);
clamp(v_new, 1, h-2);
```
현재 위치 $(u, v)$에 계산된 이동량 $(\Delta u, \Delta v)$를 더해서 새로운 위치를 추정합니다.
clamp 함수로 
%%  %%
#####   3.2.5.6. result evaluation 
@3.2.2.  $[\mathbf{X}_i^i]_{\mathbf{p}}$, 3.2.3 $\psi \left( [\mathbf{X}i^i]_{\mathbf{p}} \right)$, 3.2.5.1. How to optimize 에서 확인 할 수 있는 것 처럼 new_cost를 계산한다. 
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel(... ,torch Tensor. converged , ...)`
```cpp
// Update pixel and lambda
if (new_cost < cost) {
  u = u_new;
  v = v_new;
  lambda *= 0.1;
  converged[b][n] = new_cost < cost_thresh;
}
else {
  lambda *= 10.0;
  converged[b][n] = cost < cost_thresh;
}
```
1. 에러가 줄었을 경우
	1. $\mathbf{p}$ 이동 확정
	2. damping pram $\lambda$ 를 줄여 LM 알고리즘을 **Gauss-Newton** 방식에 가까워지게 함
	3. 에러가 목표치(`cost_thresh`)보다 작아졌으면 "수렴 완료"로 표시함
2. 에러가 늘었을 경우
	1. $\mathbf{p}$ 이동 안함
	2. damping pram $\lambda$ 를 늘여 LM 알고리즘을 **Gradient Descent** 방식에 가까워지게 함
	3. 에러가 목표치(`cost_thresh`)보다 작아졌으면 "수렴 완료"로 표시함

#### 3.2.6  Iteration , Output & Usage
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@`iter_proj_kernel(... , torch Tensor. p_new, ...)`
```cpp
// 1. 최적화 루프 시작
for (int i=0; i<max_iter; i++) {
    // ... (Bilinear Interpolation) ...
    // ... (Jacobian & Hessian 계산) ...
    // ... (LM Update: delta_u, delta_v 계산) ...

    // 2. 업데이트 수락 여부 결정 (LM Step)
   // Update pixel and lambda

	if (new_cost < cost) {
		u = u_new;
		v = v_new;
		lambda *= 0.1;
		converged[b][n] = new_cost < cost_thresh;
	}
	
	else {
		lambda *= 10.0;
		converged[b][n] = cost < cost_thresh;
	}
}

// 3. 최종 최적화된 좌표 저장
p_new[b][n][0] = u;
p_new[b][n][1] = v;
```
위 과정을 max_iter번 반복한다 
커널이 최적화된 실수 좌표 p∗와 converged를 반환한다. 
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@iter_proj_cuda()`
```cpp
iter_proj_cuda(...) {
	torch::Tensor converged = torch::zeros({batch_size, n}, opts_bool);
	iter_proj_kernel<<<blocks, threads>>>(...);

	return {p_new, converged};
}
```
이 함수가 p∗와 converged를 반환한다. 

Python 코드(matching.py)로 돌아와서 **3D 거리 기반의 검증**을 수행한다.  
아무리 Ray(방향)가 일치하더라도, 3D 공간상에서 두 점의 거리가 너무 멀면 잘못된 매칭(Outlier)으로 간주하고 버린다.

@/mast3r_slam/matching.py  
@match_iterative_proj
```python
p1, valid_proj2 = mast3r_slam_backends.iter_proj(...)
p1 = p1.long()
```
p* 가 실수 이기에 소수점을 버리고 정수형(Integer)으로 변환
valid_proj2는 converged 배열을 담고 있다 이는 각각 T/F 값을 가지고 있기에 쓸수 있는 녀석을 알 수있다. 


```python
# Check for occlusion based on distances
batch_inds = torch.arange(b, device=device)[:, None].repeat(1, h * w)
dists2 = torch.linalg.norm(
	X11[batch_inds, p1[..., 1], p1[..., 0], :].reshape(b, h, w, 3) - X21
	, dim=-1
)
```
1. batch_inds: 배치 인댁스를 구한다 
2. X11에서 예측한 좌표들에서 X21를 빼서 각 두 3D 점 사이의 유클리드 거리(Distance)를 계산(torch.linalg.norm) -> 0이 목표 

```python
valid_dists2 = (dists2 < cfg["dist_thresh"]).view(b, -1)
valid_proj2 = valid_proj2 & valid_dists2
```
1. dists2에서 일정 거리보다 작은 것들만 valid_dists2에 T/F 정보를 담고 
2. 수렴된녀석(valid_proj2)과   점들의 거리가  일정거리보다 적은 녀석들(valid_dists2)의 기중을 통화 해야만 정답으로 인정해서 T/F 정보를 담음

지금 까지는 Ray 만 썼는데 Descriptor도 사용하여 보정함 
이 부분은 뒤의 @3.2.7 **Feature-based Refinement** 부분임
```python
if cfg["radius"] > 0:
        (p1,) = mast3r_slam_backends.refine_matches(
            D11.half(),
            D21.view(b, h * w, -1).half(),
            p1,
            cfg["radius"],
            cfg["dilation_max"],
        )
```
- **입력:**
    - `D11`, `D21`: MASt3R가 예측한 고차원 특징 벡터(Descriptor). 
    - `p1`: 기하학적으로 찾은 1차 매칭 위치.
    - `radius`: 주변 몇 픽셀을 뒤져볼지 결정하는 탐색 반경.
- **동작 (`refine_matches` CUDA 커널):**
    - 현재 찾은 위치 `p1`을 중심으로 검색
    - `p1`의 주변 픽셀들과 특징 벡터(`D`)의 유사도(Dot Product)를 계산합니다.
    - 유사도가 가장 높은 픽셀로 `p1`을 업데이트(이동)


```python
# Convert to linear index
idx_1_to_2 = pixel_to_lin(p1, w)
return idx_1_to_2, valid_proj2.unsqueeze(-1)
```
1. idx_1_to_2: Correspondence Map 
	1. 예시: `idx_1_to_2[100] = 5000`이라면: 
		1. 이미지 2의 100번째 픽셀은, 이미지 1의 5000번째 픽셀과 같은 점
2. valid_proj2: Validity Mask


- Tracking (Sec ?.?): 현재 프레임과 키프레임 사이의 매칭을 통해 카메라 포즈를 추정한다.
- Loop Closure (Sec ?.?): 방문했던 장소를 다시 찾았을 때, 두 프레임 간의 기하학적 연결 고리를 만든다.
- Global Optimization (Sec ?.?): 구해진 매칭점들을 제약 조건(Constraint)으로 하여 전체 지도의 일관성을 맞추는 Bundle Adjustment를 수행한다.
#### 3.2.7 Feature-based Refinement
앞의 @ 3.2.6  Iteration , Output & Usage에서 언급했던 부분이다. 다시 한번 코드를 살펴보자 

@/mast3r_slam/matching.py  
@match_iterative_proj
``` python
if cfg["radius"] > 0:
        (p1,) = mast3r_slam_backends.refine_matches(
            D11.half(),
            D21.view(b, h * w, -1).half(),
            p1,
            cfg["radius"],
            cfg["dilation_max"],
        )
```

- **입력:**
    - `D11`, `D21`: MASt3R가 예측한 고차원 특징 벡터(Descriptor). 
    - `p1`: 기하학적으로 찾은 1차 매칭 위치.
    - `radius`: 주변 몇 픽셀을 뒤져볼지 결정하는 탐색 반경.
- **동작 (`refine_matches` CUDA 커널):**
    - 현재 찾은 위치 `p1`을 중심으로 검색
    - `p1`의 주변 픽셀들과 특징 벡터(`D`)의 유사도(Dot Product)를 계산합니다.
    - 유사도가 가장 높은 픽셀로 `p1`을 업데이트(이동)

- D11.half() : 데이터 타입의 Precision 을 반으로 줄임(float32 -> float16)
	- 데이터 용량 줄이기 -> 성능 확보
- D21.view(b, h * w, -1).half(): 차원 줄이기 -> Precision 을 반으로 줄임
- cfg\["radius"\]: 탐색 반경 
- cfg\["dilation_max"\]:탐색 간격 


ray 값만 예측하던 dust3r을 전신으로 하는 MASt3R은 additional per-pixel
features을 추가로 예측한다. 따라서 이것을 가지고 matching 성능을 높일 수 있다. 

이제 cuda 를 살펴보자
@`/mast3r_slam/backend/src/matching_kernels.cu` 
@refine_matches_kernel(...)`
1. 초기화
``` cpp
// Get pixel and its features
long u0 = p1[b][n][0];
long v0 = p1[b][n][1];

scalar_t max_score = ::cuda::std::numeric_limits<scalar_t>::min();
long u_new = u0;
long v_new = v0;
```
최고 점수와 최적 위치 초기화
2. Coarse-to-Fine 루프
``` cpp
for (int d=dilation_max; d>0; d--) {
	const int rd = radius*d;
	const int diam = 2*rd + 1;
	for (int i=0; i<diam; i+=d) {
		for (int j=0; j<diam; j+=d) {
			const long u = u0 - rd + i;
			const long v = v0 - rd + j;
```
d를 점점 줄이면서 for 루프를 돌린다 
- d가 클때: 탑색범위(diam)은 넓지만, 픽셀을 d만큼 넘어가며 검색함(i+=d) 
- d가 작을때: 탑색범위(diam)은 작음, 꼼꼼하게 탐색함 


3. 유사도 측정
``` cpp
if (inside_image(u, v, w, h)) { // 이미지 범위 체크
          scalar_t score = 0.0;
          // 특징 벡터(Descriptor) 내적 계산
          // D21: 타겟 점의 특징, D11: 참조 이미지(u,v)의 특징
          for (int k=0; k<fdim; k++) {
            score += D21[b][n][k] * D11[b][v][u][k];
          }

          // 점수가 더 높으면(더 비슷하면) 위치 업데이트
          if (score > max_score) {
            max_score = score;
            u_new = u;
            v_new = v;
          }
        }
      } // end inner loops
    }
    u0 = u_new;
    v0 = v_new;
}
```
1. 내적을 이용해서 각 타겟 점의 특징과 참조 이미지(u,v)의 특징의 유사도를 측정 
2. 만약 스코어가 크다면 새로운 픽셀로 지정한다. 이 지정한 픽셀이 다은 루프때 사용된다
	1. 즉 점점 도 가깝게 픽셀이 움직이는 것인다 그런데 처음에는 큰 보폭 다음은 작은 보폭으로 

#### 3.2.8        $\|\psi_1 - \psi_2\|^2 = 2(1 - \cos \theta), \space \cos \theta=\psi_1^T \psi_2$ 의 수학 증명
$$\psi_1^T \psi_2=각\space 행열(여기서는 벡터)의 \space 내적$$ 이거는 공부 쫌 더 해야할듯 
$$\|\psi_1 - \psi_2\|^2 = (\psi_1 - \psi_2)^T (\psi_1 - \psi_2)$$
$$= \psi_1^T\psi_1 - \psi_1^T\psi_2 - \psi_2^T\psi_1 + \psi_2^T\psi_2$$
$$= 2 - 2(\psi_1^T\psi_2)$$
$$\psi_1^T\psi_2 = \|\psi_1\|\|\psi_2\|\cos\theta = 1 \cdot 1 \cdot \cos\theta$$
$$= 2 - 2\cos\theta$$
$$= 2(1 - \cos\theta)$$

#### 3.2.9   코사인을 이용한 직관 밑 논문 부연 설명

$$\mathbf{p}^* = \arg \min_{\mathbf{p}} \left\| \psi \left( [\mathbf{X}_i^i]_{\mathbf{p}} \right) - \psi (\mathbf{x}) \right\|^2 . \space \space \space \space \space where \space  x∈ X_{i}^j$$
Squared Euclidean Distance를 cost 로 하여  [[../../../50_Concepts/Levenberg Marquardt|Levenberg Marquardt]] 알고리즘으로 푼다

그런데 왜 $$\|\psi_1 - \psi_2\|^2 = 2(1 - \cos \theta), \space \cos \theta=\psi_1^T \psi_2$$라고 하면서 설명을 할까? 코드에서는 쓰이지도 않는데 말이다. 
![[../../../90_Resources/92_Images/MASt3R-SLAM_3.2Matching_Additional_explanation.png]]
이는 gpu가 연산하기 편한  [[Squared Euclidean Distance|Squared Euclidean Distance]] 로 풀지만  두 단위 백터의 각도를 점점 작게 만들어 결국에는 가장 각도 차이가 작은 것을 찾아 낸다 는 알고리즘이 사람의 직관으로는 더 편하기 떄문이며 이 편한 알고리즘이 gpu가 연산하기 좋은 알고리즘과 동치임을 알려주는 것이다. 

읽는 사람 편하게 하기 위함 과 리뷰어들에게 반박 안당하게 




###  3.3. Tracking and Pointmap Fusion
####  3.3.1 Tracking
요약 하자면 이전 프래임들이 예측해 둔 포인트와 지금 키 프래임에서 찍은 포인트의 방향 + 거리의 차이를 분석해서 그 차이를 줄임으로서 카메라 삼각뿔을 움직여 최적화한다.

최적화에서 중요한 에러와 최적화 알고리즘을 정의 한다 
1. 에러$$E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {w(q,\sigma^2)} \right\|_\rho^2$$
	1. 여기서는 [[../../../50_Concepts/Huber Norm|Huber Norm]]를 사용하고 q 즉 Match Confidence에 대한 Weighting Function을 사용하여 민감도를 조절 하고 있다. 
2. 최적화 알고리즘 :Gauss-Newton on Manifold


주의! 에러가 코드에서는 위 수식과는 다르게 다음 과 같이 계산 된다 
$$E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {\sqrt{w(q,\sigma^2)}} \right\|_\rho^2$$
#####  3.3.1.1. Error 
######  3.3.1.1.1 왜 3D point 를 바로 이용 하지 않을까?
 걍 두 3D 점 사이의 거리를 줄이는 것이 가장 직관적이다.
$$ E_p = \sum \| \tilde{\mathbf{X}}_k - \mathbf{T}_{kf} \mathbf{X}_f \|^2 $$
하지만 이거를 쓰지 않는다. 왜냐하면 MASt3R가 예측한 **깊이(Depth, $z$) 값은 노이즈가 심하고 일관성이 떨어지기 때문
깊이가 조금만 틀려도 3D 공간상의 오차는 엄청나게 커져서 최적화를 방해

$$ E_r = \sum \left\| \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f) \right\|^2 $$
*   **의미:** 방향(Direction)만 맞으면 ok
*   **장점:** Robust
######  3.3.1.1.2 $\psi(\mathbf{X})$
일단 여기서 X 행렬은 전체 h * w 의 픽셀을 가지고 있다 그런더 한 픽셀은 \[x,y,z\]을 가지고 있다. 즉  2차원 격자 형태(H,W,3) 인 것임 
$$\psi(\mathbf{X}) = \frac{\mathbf{X}}{\|\mathbf{X}\|_2}$$

@`/mast3r_slam/geometry.py` 
``` python
def point_to_dist(X):
    #기본값이 L2
    d = torch.linalg.norm(X, dim=-1, keepdim=True) 
    return d

def point_to_ray_dist(X, jacobian=False):
	b = X.shape[:-1]
	d = point_to_dist(X)
	d_inv = 1.0 / d
	r = d_inv * X
	rd = torch.cat((r, d), dim=-1) # Dim 4
	if not jacobian: 
		return rd 
	else: 
	# ... (Jacobian 계산 생략) ... 
		return rd, drd_dX
```
여기서 ray 값과 distance 값을 구할 수 있다. 
여기서 r 이 $\psi(\mathbf{X})$이다 
코드는 rd를 반환하는데, 이는 Ray(ψ)와 Distance(d)를 합친 벡터
	논문 본문에는 Ray만 강조되어 있지만, 회전(Rotation)만 최적화되는 퇴화(Degeneracy)를 막기 위해 거리 오차(rd)도 작게 포함한다

######  3.3.1.1.3 $\psi(\tilde{\mathbf{X}}_k)$
@`/mast3r_slam/tracker.py`
``` python
def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
    ...
    rd_k = point_to_ray_dist(Xk, jacobian=False)

    old_cost = float("inf")
    for step in range(self.cfg["max_iters"]):
        ...
```
rd_k: $\psi(\tilde{\mathbf{X}}_k)$ 이다
함수를 콜할 때 이미 있는 기본 맵데이터를 한번 가져와서 연산만 해주는 것이다. 
for 루트 위에 있다. 즉 그냥 한 번 
###### 3.3.1.1.4 $\mathbf{T}_{kf} \mathbf{X}_f$
@`/mast3r_slam/tracker.py`
``` python
def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
    ...
    T_CkCf = T_WCk.inv() * T_WCf
    ...
    for step in range(self.cfg["max_iters"]):
	    Xf_Ck, dXf_Ck_dT_CkCf = act_Sim3(T_CkCf, Xf, jacobian=True)
        ...
```
@`/mast3r_slam/geomatry.py`
``` python
def act_Sim3(X: lietorch.Sim3, pC: torch.Tensor, jacobian=False):
	pW = X.act(pC)
	if not jacobian:
		return pW
	dpC_dt = torch.eye(3, device=pW.device).repeat(*pW.shape[:-1], 1, 1)
	dpC_dR = -skew_sym(pW)
	dpc_ds = pW.reshape(*pW.shape[:-1], -1, 1)
	# view(-1, mdim)
	return pW, torch.cat([dpC_dt, dpC_dR, dpc_ds], dim=-1) 
```
\todo 뭔말이니?
###### 3.3.1.1.5 $\psi(\mathbf{T}_{kf} \mathbf{X}_f)$, $\psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)$, 
@`/mast3r_slam/tracker.py`
``` python
def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
    ...
    #psi(X_k)
	rd_k = point_to_ray_dist(Xk, jacobian=False)
	
	for step in range(self.cfg["max_iters"]):
	    # 2. 현재 점 변환: T_kf * X_f
	    Xf_Ck, dXf_Ck_dT_CkCf = act_Sim3(T_CkCf, Xf, jacobian=True)
	    # 3. 변환된 점을 Ray로: psi(T_kf * X_f)
	    # r = z-h(x)
	    rd_f_Ck, drd_f_Ck_dXf_Ck = point_to_ray_dist(
								    Xf_Ck, jacobian=True)
	    # 4. 에러(Residual) 계산: r = psi(X_k) - psi(T_kf * X_f)
	    r = rd_k - rd_f_Ck
	        ...
```
그래서 구한 $\mathbf{T}_{kf} \mathbf{X}_f$을 normalize 해준뒤 잔차 벡터를 계산해준다. 

###### 3.3.1.1.6 ${w(q,\sigma^2)}$

$$w(\mathbf{q}, \sigma^2) = \begin{cases} \sigma^2 / \mathbf{q} & \mathbf{q} > \mathbf{q}_{min} \\ \infty & \text{otherwise} \end{cases}$$

@`/config/base.yml`
``` yml
tracking:
	sigma_ray: 0.003
	sigma_dist: 1e+1
```
@`/mast3r_slam/tracker.py`
``` python
def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
    sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk) 
    sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk) 
    # 2. Ray(3차원)와 Distance(1차원) 가중치를 합쳐서 4차원 가중치 벡터 생성 
    sqrt_info = torch.cat((sqrt_info_ray.repeat(1, 3), 
						    sqrt_info_dist), dim=1)
```

| 논문 기호                           | 코드 변수명                  | 의미                              |
| :------------------------------ | :---------------------- | :------------------------------ |
| $q$                             | `Qk`                    | MASt3R가 예측한 매칭 신뢰도 (Confidence) |
| $\sigma$                        | `self.cfg["sigma_ray"]` | 광선 방향에 대한 예상 노이즈 표준편차           |
| $q_{min}$                       | `self.cfg["Q_conf"]`    | 신뢰도가 너무 낮은 점을 버리기 위한 임계값        |
| $w(q, \sigma^2)$                | `1 / sqrt_info**2`      | 가중치 함수 (분산의 역수 개념)              |
| $1/\sqrt{w}$                    | `sqrt_info`             | 실제 잔차에 곱해지는 가중치                 |
| $\mathbf{q} > \mathbf{q}_{min}$ | valid                   | 기준선에 알맞은가                       |
3.3.1.1.6.1 sqrt_info
그런대 코드를 보면 조금 이상하다 
$$  \frac{\sqrt{q}}{\sigma} = \sqrt{\frac{q}{\sigma^2}} =  \sqrt{ \frac{1}{w(q,\sigma^2)}}$$
이와 같이 가중치 함수의 역수 제곱근으로 계산하고 있다!! 뭐냐 이거...



3.3.1.1.6.2 valid
valid는 $\mathbf{q} > \mathbf{q}_{min}$ 를 확인 하는 녀석이다. 
만약 valid 가 0이면 sqrt_info = 0이다. 이는 무슨 말일까?
xf
여기서 otherwise 케이스 인것이다. 그리고 이 가중치 함수는 분모에 들어간다 하지만 실제 코드에서는 역수의 제곱근(`sqrt_info`,  $\sqrt{\frac{1}{w}}$})이 곱해진다 . 즉 같은 경우에서 에러는 0이 된다. 

###### 3.3.1.1.7 $E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {\sqrt{w(q,\sigma^2)}} \right\|_\rho^2$
$$E = \sum\left\| \frac{r}{\sqrt{w(q,\sigma^2)}} \right\|_{\rho}^2$$
 $$\frac{1}{\sqrt{w(q, \sigma^2)}} = \frac{1}{\sqrt{\sigma^2 / q}} = \frac{\sqrt{q}}{\sigma}$$
 Huber Loss $\rho(x)$를 미분하여 가중치 함수로 만들면 $w_{huber}(x) = \frac{\rho'(x)}{x}$가 된다 (??)
 - 에러가 작을 때 ($|x| < \delta$): $w_{huber} = 1$
 - 에러가 클 때 ($|x| \ge \delta$): $w_{huber} = \frac{\delta}{|x|}$ (에러를 줄여줌)

우리가 구하고자 하는 최종 가중치 행열은 구 가중치 행열의 곱이다 
$$\mathbf{W} = W_{conf} \cdot W_{huber} = \left( \frac{q}{\sigma^2} \right) \cdot w_{huber}(r_{white})$$

3.3.1.1.7.1 Confidence Weighting (Whitening)
@`/mast3r_slam/tracker.py`
```python
def solve(self, sqrt_info, r, J):
	...
	# sqrt_info는 위에서 계산됨: (1 / sigma) * sqrt(Qk)
	whitened_r = sqrt_info * r
	...
```
*   **수식:** $r_{white} = \frac{\sqrt{q}}{\sigma} \cdot r$
*   **의미:** 잔차 $r$에 네트워크의 신뢰도($q$)와 분산($\sigma$)을 반영하여 스케일링함. 신뢰도가 높으면 잔차를 키워서 중요하게 다루고, 낮으면 0에 가깝게 만듦.

3.3.1.1.7.2. Robust Weighting (Huber)
@`/mast3r_slam/tracker.py`
```python
def solve(self, sqrt_info, r, J):
	...
	robust_sqrt_info = sqrt_info * torch.sqrt(
	    huber(whitened_r, k=self.cfg["huber"])
	)
	...
```
*   **수식:** $\sqrt{W_{total}} = \frac{\sqrt{q}}{\sigma} \cdot \sqrt{w_{huber}(r_{white})}$
*   **설명:**
    *   `huber(...)` 함수는 Huber 가중치 $w_{huber}$를 반환함.
    *   우리는 $Ax=b$ 형태를 만들기 위해 가중치의 **제곱근($\sqrt{\cdot}$)** 이 필요함.
    *   `sqrt_info`($\sqrt{W_{conf}}$)와 `sqrt(huber)`($\sqrt{W_{huber}}$)를 곱해서 **최종 가중치 제곱근**을 만듦.

3.3.1.1.7.3 Weighted Residual Vector Construction (b)
```python
def solve(self, sqrt_info, r, J):
	...
	# b = sqrt(W) * r
	b = (robust_sqrt_info * r).view(-1, 1) 
	...
```
$$ \mathbf{b} = \sqrt{\mathbf{W}} r $$
3.3.1.1.7.4 Objective Function Evaluation (Total Cost)
```python
def solve(self, sqrt_info, r, J):
	...
	cost = 0.5 * (b.T @ b).item()
	...
	return tau_j, cost
```
$$E = \sum\left\| \frac{r}{\sqrt{w(q,\sigma^2)}} \right\|_{\rho}^2$$
  $$E = \sum \rho(r_{white})$$ 
 $$\rho(x) \rightarrow \frac{1}{2} w_{huber}(x) \cdot x^2 \quad (\text{IRLS Approximation})$$ ?????
 $$\mathbf{W} = w_{huber} \cdot W_{conf}$$

$$\tilde{r}=\mathbf{r} \times \sqrt{\mathbf{W}}$$
$$\|\tilde{r}\|^2 = \tilde{r}^T \tilde{r} = (\mathbf{r} \times \sqrt{\mathbf{W}})^T (\mathbf{r} \times \sqrt{\mathbf{W}}) = r^T {\mathbf{W}} r$$
 $$E = \frac{1}{2} \mathbf{b}^T \mathbf{b} = \frac{1}{2} \|\mathbf{b}\|^2$$  $$E = \frac{1}{2} \underbrace{(\sqrt{\mathbf{W}}\mathbf{r})^T}_{\mathbf{b}^T} \underbrace{(\sqrt{\mathbf{W}}\mathbf{r})}_{\mathbf{b}} = \frac{1}{2} \mathbf{r}^T \mathbf{W} \mathbf{r}$$ 여기서 1/2가 왜 들어가지는 지는 모르겠다 던진다. 뭐 미분 할때 에케 되는 것 아닐까?

Gauss-Newton 등 최적화 과정에서 에러 함수를 미분할 때 발생하는 계수 2를 상쇄하여 $J^T r$형태를 깔끔하게 도출하기 위한 수학적 관례(Convention)로 이해함.



#####  3.3.1.2. 최적화  
엄 lie Algabra 가 뭐에요..? 뭐요 exp? ... ㅎ

회전 행열 안깨지게 하면서 최적화 시키는 방법
#### 3.3.2 pointmap fusion
 $$\tilde{\mathbf{X}}_k^k \leftarrow \frac{\tilde{\mathbf{C}}_k^k \tilde{\mathbf{X}}_k^k + \mathbf{C}_f^k (\mathbf{T}_{kf} \mathbf{X}_f^f)}{\tilde{\mathbf{C}}_k^k + \mathbf{C}_f^k}$$
 $$\tilde{\mathbf{C}}_k^k \leftarrow \tilde{\mathbf{C}}_k^k + \mathbf{C}_f^k$$ 
k위치에서 본 정제된 포인트 맵($\tilde{\mathbf{X}}_k^k$)과 f 위치에서의 포인트 맵을 k위치에서 본 포인트맵으로 변환한것  $\mathbf{T}_{kf} \mathbf{X}_f^f$ 이 있을떄 각 accumulate 된 Confidence($\tilde{\mathbf{C}}_k^k$)와 새로운 거의 Confidence ($\mathbf{C}_f^k$)를 이용해서 평균을 낸것을 정제된 포인트맵에 저장함 
또한 accumulate된 Confidence를 업데이트함 

@`/mast3r_slam/tracker.py`
```python
def track(self, frame: Frame):
	...
	# Use pose to transform points to update keyframe
	Xkk = T_CkCf.act(Xkf)
	keyframe.update_pointmap(Xkk, Ckf)
	...
```
@`/mast3r_slam/frame.py`
```python
def update_pointmap(self, X: torch.Tensor, C: torch.Tensor
	iltering_mode = config["tracking"]["filtering_mode"]
	...
	elif filtering_mode == "weighted_pointmap":
		# 논문 Equation (8) 구현 
		# X_new = (C_old * X_old + C_new * X_new) / (C_old + C_new)
		self.X_canon = ((self.C * self.X_canon) + (C * X)) / (self.C + C)
		# C_new = C_old + C_new
		
		self.C = self.C + C
		self.N += 1
	...
```


### 3.4. Graph Construction and Loop Closure
여기서는 키프래임을 선택하는 기준을 이야기하고 또한 loop closure 을 한다 
#### 3.4.1 keyframe selection
@`/mast3r_slam/tracker.py`
```python
def track(self, frame: Frame):
	...
	# Keyframe selection
	# valid matching points들의 합계 
	n_valid = valid_kf.sum()
	#유효 픽셀 비율 %
	match_frac_k = n_valid / valid_kf.numel()
	#현재 프레임이 키프레임의 '서로 다른' 지점들을 얼마나 다양하게 보고 있는가?
	unique_frac_f = (
		torch.unique(idx_f2k[valid_match_k[:, 0]]).shape[0] / valid_kf.numel()
	)
	#인개점보다 match_frac_k,unique_frac_f 중 하나라도 작으면 새 프래임 찍음
	new_kf = min(match_frac_k, unique_frac_f) < self.cfg["match_frac_thresh"]
	...
```
- valid matching points들의 합계 는 이미 outlier and now confidence 등의 요소를 제외함 
- $\frac{\text{유효 매칭 수}}{\text{전체 픽셀 수}}$
- idx_f2k: 현재 프레임(F)의 픽셀이 키프레임(K)의 **어느 픽셀**에 매칭되는지 가리키는 인덱스
- 
@`/mast3r_slam/main.py`
```python
if __name__ == "__main__":
	...
	if mode == Mode.TRACKING:
	add_new_kf, match_info, try_reloc = tracker.track(frame)
	if try_reloc:
		states.set_mode(Mode.RELOC)
	states.set_frame(frame)
...
	if add_new_kf:
		keyframes.append(frame)
		states.queue_global_optimization(len(keyframes) - 1)
		# In single threaded mode, wait for the backend to finish
		while config["single_thread"]:
			with states.lock:
				if len(states.global_optimizer_tasks) == 0:
					break
			time.sleep(0.01)
	...  
```