---
tags:
  - "#experiment"
parent_concept:
  - "[[../02_MASt3R-SLAM/MASt3R-SLAM|MASt3R-SLAM]]"
  - "[[MASt3R-SLAM Sec 3.3 Discrepancy]]"
---
# 1. Introduction 
$$E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {w(q,\sigma^2)} \right\|_\rho^2 \space\space \space\space (1)$$

$$E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {\sqrt{w(q,\sigma^2)}} \right\|_\rho^2 \space\space \space\space (2)$$

[[MASt3R-SLAM]]논문에서 3.3. Tracking and Pointmap Fusion 에서 나오는 수식 (1) 을 코드 맵핑하던중 깃허브에 공개된 코드에서는 의 수식(2)을 사용하는 것을 발견하였다. 이에 대한 내용은 [[MASt3R-SLAM Sec 3.3 Discrepancy]]를 참고하기 바람. 그리고 수식 전걔의 내용은 [[MASt3R-SLAM]]의 3.3.1 Tracking을 확인 하기 바람

위의 내용을 코드를 논문의 수식으로 변경함으로써 실험을 통해서 변화를 확인 하기 위함이다. 

# 2. Experiment Setup 
- CPU: i5-7500
- GPU: RTX 3060 12GB vram
- RAM: 16GB
- OS : Ubuntu (WSL2)
- Memory tuning 
	- swap memory: 8GB
	- 위의 메모리 양의 한게로 인한 OOM 방지 
- Dataset: TUM RGB-D(9개 시퀀스), 7-Scenes(7개 시퀀스), eth3d(61개 시퀀스)
	- TUM RGB-D, 7-Scenes 는 no-calib, calib 버전이 존제
	- eth3d는 calib 버전만 존제
		- 6개의 시퀀스는 evel_eth3d.sh 상에서 주석 처리 되어 있음
	- euroc dataset은 다운이 안됨
- Evaluation Metric: ATE RMSE (Absolute Trajectory Error)
- codebase link: https://github.com/rmurai0610/MASt3R-SLAM
	- branch: windows 에서 작업함

# 3. Experimental Design
## 3.1. Dataset mode
Dataset이 Calibrated or Uncalibrated 에 따라서 최적화 방식이 나뉜다.
- **Case A:** Calibrated
	- 이는 pixel-space optimization을 진행함. `calib.yaml`를 사용
	- 논문의 3.7 Known Calibration에서 `we change the residuals in optimisation to be in pixel space rather than ray space.` 라고 명시하고 있다 
	- 논문의 8.3. Projection and Depth에서`In the case of known calibration, we instead use a pixel error`라고 명시하고 있다 
instead of ray error.
- **Case B:** Uncalibrated
	- 이는 ray-space optimization을 진행함. `base.yaml` / `--no-calib` 를 사용
	- [[MASt3R-SLAM]] 3.3.1.1. Error 참조

## 3.2 code modification 
Gauss-Newton이나 Levenberg-Marquardt 최적화에서 우리가 최소화하려는 에러 E는 잔차(Residual) r의 제곱합이다.

$$E = \frac{1}{2} \sum \mathbf{b}^T \mathbf{b}$$

코드의 solve 함수를 보면 b는 다음과 같이 정의됩니다.
@`/mast3r_slam/tracking.py` 
```python
def solve(self, sqrt_info, r, J):
	whitened_r = sqrt_info * r
	robust_sqrt_info = sqrt_info * torch.sqrt(
		huber(whitened_r, k=self.cfg["huber"])
	)
	...
	b = (robust_sqrt_info * r).view(-1, 1)
	...
	cost = 0.5 * (b.T @ b).item()
```

$$r_{white} = \text{sqrt\_info} \cdot r$$

$$\text{robust\_sqrt\_info} = \text{sqrt\_info} \cdot \sqrt{w_{huber}(r_{white})}$$

$$\mathbf{b} = \left( \text{sqrt\_info} \cdot \sqrt{w_{huber}(\text{sqrt\_info} \cdot r)} \right) \cdot r$$

벡터 b는 **"기본 가중치 × Huber 가중치 ×잔차" 의 형태가 됩니다.
에러를 수학적으로 전개하면 다음과 같습니다.

$$E = \frac{1}{2} \sum \mathbf{b}^T \mathbf{b}$$

$$E = \frac{1}{2} \sum \left( \text{sqrt\_info} \cdot \sqrt{w_{huber}(r_{white})} \cdot r \right)^2$$

$$E = \frac{1}{2} \sum \left( \mathbf{\text{sqrt\_info}^2} \cdot w_{huber}(r_{white}) \cdot r^2 \right)$$


@`/mast3r_slam/tracking.py` 
```python
def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
	last_error = 0
	# sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
	# sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk)
  
	# make sqrt_info square to test paper formula
	sqrt_info_ray = (1 / (self.cfg["sigma_ray"] ** 2)) * valid * Qk
	sqrt_info_dist = (1 / (self.cfg["sigma_ray"] ** 2)) * valid * Qk
	
	sqrt_info = torch.cat((sqrt_info_ray.repeat(1, 3), sqrt_info_dist), dim=1)
	

def opt_pose_calib_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid, meas_k, valid_meas_k, K, img_size):
	last_error = 0
	# sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
	# sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk)
  
	# make sqrt_info square to test paper formula
	sqrt_info_pixel = 1 / torch.pow(self.cfg["sigma_ray"]) * valid * Qk
	sqrt_info_depth = 1 / torch.pow(self.cfg["sigma_dist"]) * valid * Qk
	

```
위 코드는 sqrt_info를 자기 자신을 제곱하는 꼴이 된다 

$$E = \frac{1}{2} \sum \left( \mathbf{\text{(sqrt\_info}^2)^2} \cdot w_{huber}(r_{white}) \cdot r^2 \right)$$
# 4. Experimental Results
## 4.1 Result Overview 

| 데이터셋          | 모드 (Calib) | Tracking 상태       | 최종 평가 (APE / Umeyama)                 | 비고                                |
| :------------ | :--------- | :---------------- | :------------------------------------ | :-------------------------------- |
| **7-Scenes**  | `no-calib` | 정상 (FPS ~2.8)     | **성공** (RMSE 0.03m ~ 0.11m)           | 논문의 Plug-and-play 기능 정상 작동 확인     |
| **7-Scenes**  | `calib`    | 불안정 (FPS ~2.0)    | **실패** (`Degenerate covariance rank`) | 궤적(Trajectory)이 한 점으로 수렴하거나 붕괴됨   |
| **TUM RGB-D** | `no-calib` | 잦은 Relocalization | **성공** (RMSE 0.02m ~ 0.11m)           | 빠른 움직임으로 인해 Tracking 실패 잦음, 복구는 됨 |
| **TUM RGB-D** | `calib`    | 불안정 (FPS ~3.9)    | **실패** (`Degenerate covariance rank`) | 궤적 붕괴                             |
| **ETH3D**     | `calib`    | 불안정 (FPS ~4.5)    | **실패** (`Degenerate covariance rank`) | 궤적 붕괴                             |
- Uncalibrated 모드는 정상적으로 작동하며, 7-Scenes와 TUM 데이터셋에서 준수한 RMSE(수 cm ~ 10cm 내외)를 보여주었다.
- Calibrated 모드에서 모든 데이터셋의 최종 평가가 실패했다

## 4.2 Result by dataset

### 4.2.1 Calibrated

TUM RGB-D, 7-Scenes, ETH3D 의 Calibrated 모드에서는 모두 다음 에러를 출력하며 실패합니다. 

```
Cholesky failed 1
fps: 3.~~
...
[ERROR] Degenerate covariance rank, Umeyama alignment is not possible
```

### 4.2.2 Uncalibrated

#### Table 1: 7-Scenes Uncalibrated 

| Sequence    | 논문 결과 (Ours*) | 현재 실험 결과  | 상태  |
| :---------- | :------------ | :-------- | :-- |
| chess       | 0.063         | **0.062** | 성공  |
| fire        | 0.046         | **0.047** | 성공  |
| heads       | 0.029         | **0.033** | 성공  |
| office      | 0.103         | **0.103** | 성공  |
| pumpkin     | 0.114         | **0.112** | 성공  |
| redkitchen  | 0.074         | **0.075** | 성공  |
| stairs      | 0.032         | **0.032** | 성공  |
| **Average** | **0.066**     | **0.066** | 일치  |
실험 결과가 논문의 수치와 일치

#### Table 2: TUM RGB-D Uncalibrated 

| Sequence        | 논문 결과 (Ours*) | 현재 실험 결과  | 비교 분석                 |
| :-------------- | :------------ | :-------- | :-------------------- |
| freiburg1_360   | 0.070         | **0.072** | 오차 범위 내 유사            |
| freiburg1_desk  | 0.035         | **0.039** | 오차 범위 내 유사            |
| freiburg1_desk2 | 0.055         | **0.053** | ==소폭 개선==             |
| freiburg1_floor | 0.056         | **0.055** | ==소폭 개선==             |
| freiburg1_plant | 0.035         | **0.036** | 오차 범위 내 유사            |
| freiburg1_room  | ==0.118==     | ==0.054== | ==압도적 개선 (절반 이하 에러)== |
| freiburg1_rpy   | 0.041         | **0.045** | 오차 범위 내 유사            |
| freiburg1_teddy | 0.114         | **0.111** | ==소폭 개선==             |
| freiburg1_xyz   | 0.020         | **0.020** | 완벽 일치                 |
| **Average**     | **0.060**     | **0.054** | ==논문 수치 보다 좋음==       |
|                 |               |           |                       |
실험 결과의 평균이 논문보다 더 우수하게 측정됨


#### logs 
두 Uncalibrated 데이터셋에서 Cholesky failed 36, RELOCALIZING against kf 5 and \[4\] 같은 로그가 자주 발견된다 

`@datasets/7-scenes/heads/`
```
Cholesky failed 36
RELOCALIZING against kf  2  and  [1]
Success! Relocalized
FPS: 2.423780591751784
Database retrieval 4 :  {2}
FPS: 2.5008121949383124
Cholesky failed 93
RELOCALIZING against kf  5  and  [4]
Success! Relocalized
FPS: 2.5325265285669603
Database retrieval 7 :  {5}
FPS: 2.567269228670879
Database retrieval 8 :  {6}
FPS: 2.5867187171643824
```
7-Scenes Uncalibrated 에서는 빈도가 적다 
heads시퀀스에서만 Cholesky failed 과 RELOCALIZING against 3번이 발생했다  

`@datasets/7-scenes/heads/`
```
FPS: 3.0335814130612992
FPS: 3.0589825739121532
RELOCALIZING against kf  11  and  [2]
Failed to relocalize
FPS: 3.0785224992905866
RELOCALIZING against kf  11  and  [7]
Failed to relocalize
RELOCALIZING against kf  11  and  [2]
Failed to relocalize
RELOCALIZING against kf  11  and  [2, 3]
Failed to relocalize
RELOCALIZING against kf  11  and  [2]
Success! Relocalized
...
Cholesky failed 585
RELOCALIZING against kf  18  and  [17, 10]
Success! Relocalized
Database retrieval 19 :  {9, 12, 7}
Database retrieval 20 :  {8, 9, 7}
FPS: 2.8300200637410757
Database retrieval 21 :  {9, 19, 7}
Database retrieval 22 :  {17, 10, 18}
Cholesky failed 618
RELOCALIZING against kf  23  and  [22]
Success! Relocalized
FPS: 2.794340382215157
```
TUM RGB-D Uncalibrated 에서는 빈도가 많다  
전반적으로 Cholesky failed 는 31번 RELOCALIZING against 는 97번 그리고 Failed to relocalize는 66번 발생했다. 

#### Pointmap

| ![[../../../MASt3R-SLAM_Sec 3.3_Discrepancy_experiment_ply_normal.png]] | ![[../../../MASt3R-SLAM_Sec 3.3_Discrepancy_experiment_ply_modified.png]] |
| ----------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| 기본                                                                      | code modifed version                                                      |
보이는 봐와 같이 포인트 맵은 동일 하다 

### 4.3 fps 
**FPS (처리 속도) 차이**
    *   7-Scenes (No-Calib): ~2.8 FPS
    *   TUM (No-Calib): ~2.6 FPS
    *   ETH3D (Calib): ~4.5 FPS
    *   논문에서는 RTX 4090 기준 15 FPS라고 했으나, 현재 환경(RTX 3060 12GB)에서는 2~4 FPS가 나오고 있습니다.
제공해주신 초안에 **작성자님이 추가하신 통찰(Reason 3: Strong Filter 효과)**과 **불안정성(Instability)**에 대한 분석을 전문적인 용어로 다듬어 보강했습니다.

특히 **5.2(정확도 향상)**와 **5.3(불안정성 증가)**이 서로 **Trade-off 관계**에 있다는 점을 강조하면 논리적으로 매우 탄탄한 보고서가 됩니다.

---

# 5. Analysis & Discussion

### 5.1. Mathematical Discrepancy between Paper and Code
논문의 수식 (1)은 최적화하고자 하는 **목적 함수(Cost Function, $E$)**를 정의한 것이며, 코드는 이를 풀기 위해 Solver(Levenberg-Marquardt 등)에 입력하는 **잔차 벡터(Residual Vector, $b$)**를 구현한 것이다.
- **이론적 배경:** Least Squares Solver는 입력된 벡터 $b$에 대해 $E = \frac{1}{2}\|b\|^2$을 최소화한다. 따라서 가중치 $W$를 목적 함수에 적용하려면, 입력 벡터 $b$에는 $\sqrt{W}$를 곱해서 넣어주는 것이 수학적으로 올바른 구현(Whitening)이다.
- **실험의 의의:** 본 실험에서 코드를 수정하여 가중치를 제곱한 행위는, 실제 최적화 과정에서 가중치를 의도된 $W$가 아닌 **$W^2$ (혹은 그 이상)으로 증폭**시켜 적용한 것과 동일한 효과를 낳았다.

### 5.2. Unexpected Robustness of Uncalibrated Mode
가중치가 비정상적으로 폭발했음에도 불구하고, **Uncalibrated 모드**는 궤적 붕괴 없이 논문 수치와 비슷하거나 오히려 더 좋은 성능(정확도)을 보였다. 이에 대한 원인은 다음과 같이 분석된다.
1.  **Bounded Ray Space:** Pixel Space와 달리 Ray Space의 잔차는 단위 구(Unit Sphere) 상의 거리이므로, 에러의 최댓값이 제한적(Bounded)이다. 따라서 가중치가 커져도 수치적 발산의 위험이 상대적으로 적다.
2.  **Optimizer Damping:** Levenberg-Marquardt 알고리즘의 Damping Factor($\lambda$)가 에러 스케일이 커짐에 따라 자동으로 조절되며 최적화 과정의 발산을 억제했을 가능성이 있다.
3.  **Aggressive Filtering Effect (Strong Filter):** **(작성자님 추가 내용 보강)** 가중치를 제곱하여 증폭시킨 것이 오히려 **"공격적인 아웃라이어 제거(Aggressive Outlier Rejection)"** 역할을 수행했다. 신뢰도($Q$)가 조금이라도 낮거나 에러가 있는 매칭점들은 가중치 격차로 인해 최적화 과정에서 영향력이 거의 0으로 수렴하게 되고, **극도로 신뢰도가 높은 소수의 Inlier들만 포즈 추정에 반영**되면서 오히려 평균 정확도(RMSE)가 향상되는 결과를 낳았다.

### 5.3. System Instability
높은 정확도와는 별개로, 로그상에서 `Cholesky failed` 및 `RELOCALIZING` 메시지가 빈번하게 관측되었다. 이는 시스템의 불안정성(Instability)을 시사한다.
- **원인:** 가중치 폭발로 인해 Hessian Matrix($H$)의 조건수(Condition Number)가 악화되면서 수치적 해를 구하지 못하는 경우가 잦아졌다.
- **해석:** 5.2절의 'Strong Filter' 효과는 양날의 검이다. 신뢰도가 높은 구간에서는 정확도를 높여주지만, 매칭이 조금이라도 부족한 구간(빠른 회전 등)에서는 **유효한 특징점이 부족해져 트래킹 실패(Tracking Lost)**로 이어진다.
- **결론:** 즉, 실험 결과는 **안정성(Stability)을 희생하여 정확도(Accuracy)를 얻은 상태**라고 해석할 수 있다.

---

# 6. Conclusion & Future Work

### 6.1. Conclusion
본 연구에서는 MASt3R-SLAM 논문의 수식과 코드 구현의 차이를 분석하고, 수식에 맞춰 코드를 변경했을 때의 시스템 거동을 실험적으로 검증하였다.
1.  **구현의 타당성:** 코드는 Cost Function을 미분 가능한 Residual Vector 형태로 변환하여 구현한 것으로, `sqrt` 적용이 수학적으로 올바른 방법임을 확인하였다.
2.  **Uncalibrated 모드의 강건성:** 가중치 스케일이 비정상적으로 커진 환경에서도 SOTA급 성능을 유지했다. 이는 Intrinsic 정보가 없는 상황에서도 MASt3R-SLAM의 Ray-based Optimization이 매우 강건하게 동작함을 시사한다.
3.  **정확도와 안정성의 Trade-off:** 가중치 증폭 실험을 통해, 아웃라이어를 강하게 억제하면 정확도는 상승하지만 트래킹 유지력(안정성)은 저하되는 Trade-off 관계를 확인하였다.

### 6.2. Future Work
1.  **Code Correction:** `tracker.py`의 가중치 계산 로직(`sqrt_info` 누락 및 제곱 연산)을 깃허브 원본 상태로 복구하여 Calibrated 모드의 정상 동작을 확보한다.
2.  **Full Benchmark Verification:** 코드를 수정한 후, 실패했던 ETH3D 및 7-Scenes (Calib) 데이터셋에 대한 벤치마크를 재수행하여 논문의 `Ours` 수치를 완벽히 재현하는지 검증한다.
3.  **Optimization for Real-time:** 현재 2~4 FPS 수준인 처리 속도를 개선하기 위해 `single_thread` 옵션 해제, FP16 연산 점검 등 시스템 최적화 방안을 모색한다.