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
	sqrt_info_ray = 1 / torch.pow(self.cfg["sigma_ray"]) * valid * Qk
	sqrt_info_dist = 1 / torch.pow(self.cfg["sigma_dist"]) * valid * Qk
	
	sqrt_info = torch.cat((sqrt_info_ray.repeat(1, 3), sqrt_info_dist), dim=1)
	

def opt_pose_calib_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid, meas_k, valid_meas_k, K, img_size):
	last_error = 0
	# sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
	# sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk)
  
	# make sqrt_info square to test paper formula
	sqrt_info_ray = 1 / torch.pow(self.cfg["sigma_ray"]) * valid * Qk
	sqrt_info_dist = 1 / torch.pow(self.cfg["sigma_dist"]) * valid * Qk
	
	sqrt_info = torch.cat((sqrt_info_ray.repeat(1, 3), sqrt_info_dist), dim=1)
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

# 5. Analysis & Discussion
#  6. Conclusion & Future Work






제공해주신 MASt3R-SLAM 논문 분석 노트, 코드(`tracker.py`), 그리고 4개의 실험 로그(`7seane`, `eth3d`, `tum`의 calib/no-calib)를 종합적으로 분석한 결과입니다.

현재 실험에서 **가장 치명적인 문제(Calibrated 모드에서의 궤적 붕괴)**가 발생하고 있으며, 그 원인은 작성자님이 `tracker.py`를 수정하면서 발생한 **수학적 가중치(Weight) 계산의 오류**로 강하게 추정됩니다.

아래에 상세한 실험 결과 정리와 원인 분석, 그리고 해결 방안을 정리해 드립니다.

---

### 1. 실험 결과 요약 (Overview)

*   **성공적인 부분:** 카메라 내부 파라미터(K) 없이 구동하는 `--no-calib` 모드는 정상적으로 작동하며, 7-Scenes와 TUM 데이터셋에서 준수한 RMSE(수 cm ~ 10cm 내외)를 보여주었습니다.
*   **문제점:** 카메라 파라미터를 사용하는 **Calibrated 모드에서 모든 데이터셋의 최종 평가가 실패**했습니다.

---

### 2. 핵심 문제 분석: 왜 Calibrated 모드에서 실패했는가?

로그의 마지막에 공통적으로 나타나는 에러는 다음과 같습니다.
> `[ERROR] Degenerate covariance rank, Umeyama alignment is not possible`

이 에러는 SLAM이 추정한 카메라의 궤적(Trajectory)이 3차원 공간상에 제대로 펼쳐지지 않고, **한 점에 머물러 있거나 일직선으로 붕괴(Collapse)되었을 때** 발생합니다. 즉, Tracking 최적화가 완전히 망가졌다는 뜻입니다.

#### 🔍 원인 추적: `tracker.py` 코드 수정의 부작용
제공해주신 `MASt3R-SLAM Sec 3.3 Discrepancy.md` 노트에서 논문의 수식과 코드의 불일치를 지적하셨고, 이에 따라 `tracker.py`를 직접 수정하신 것으로 보입니다.

**수정 전 (원래 코드 - 주석 처리됨):**
```python
# sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
```
**수정 후 (현재 코드):**
```python
sqrt_info_ray = (1 / (self.cfg["sigma_ray"] ** 2)) * valid * Qk
```

**이 수정이 최적화를 망가뜨린 이유 (수학적 증명):**
Gauss-Newton이나 Levenberg-Marquardt 최적화에서 우리가 최소화하려는 에러 $E$는 잔차(Residual) $r$의 제곱합입니다.
$$ E = \frac{1}{2} \sum \mathbf{b}^T \mathbf{b} $$
코드의 `solve` 함수를 보면 $\mathbf{b}$는 다음과 같이 정의됩니다.
```python
b = (robust_sqrt_info * r) # 즉, b = sqrt_info * r
```
따라서 실제 최적화되는 에러 함수는 다음과 같습니다.
$$ E = \frac{1}{2} \sum (\text{sqrt\_info} \cdot r)^2 = \frac{1}{2} \sum (\text{sqrt\_info})^2 \cdot r^2 $$

1.  **원래 코드의 경우:** $(\frac{\sqrt{Q}}{\sigma})^2 \cdot r^2 = \mathbf{\frac{Q}{\sigma^2} r^2}$
    *   이것이 바로 **논문 저자들이 의도한 올바른 가중치(Weighted Least Squares)**입니다. (논문 수식 표기가 헷갈리게 되어 있었을 뿐, 원래 코드가 수학적으로 맞습니다.)
2.  **작성자님이 수정한 코드의 경우:** $(\frac{Q}{\sigma^2})^2 \cdot r^2 = \mathbf{\frac{Q^2}{\sigma^4} r^2}$
    *   $\sigma$는 보통 매우 작은 값입니다 (예: `sigma_ray = 0.003`).
    *   $\sigma^4$는 $8.1 \times 10^{-11}$이라는 극단적으로 작은 값이 됩니다.
    *   이로 인해 가중치가 **비정상적으로 폭발(Explode)**하게 됩니다.

**결과적 현상:**
가중치가 폭발하면서 Hessian 행렬 $H = A^T A$ 의 값들이 `float` 표현 범위를 넘어서거나 조건수(Condition Number)가 극도로 나빠집니다.
이로 인해 로그에서 무수히 많이 보이는 **`Cholesky failed`** (행렬이 Positive Definite가 아님) 에러가 발생하고, 포즈 업데이트(`tau_ij_sim3`)가 0이 되거나 발산하여 카메라가 움직이지 않은 것으로 계산됩니다. 결국 궤적이 점 하나로 붕괴하여 `Umeyama alignment`가 불가능해진 것입니다.

---

### 3. 기타 관찰 사항 (로그 분석)

1.  **No-Calib 모드의 잦은 Relocalization (TUM 데이터셋)**
    *   `tum_no_cal.txt`를 보면 `Cholesky failed`와 `RELOCALIZING against kf...` 메시지가 매우 자주 뜹니다.
    *   TUM 데이터셋은 7-Scenes에 비해 카메라의 회전(Rotation)이 빠르고 모션 블러가 심합니다.
    *   카메라 내부 파라미터(K) 없이 순수하게 MASt3R의 Pointmap 매칭에만 의존하다 보니, 기하학적 제약이 부족하여 Tracking을 자주 놓치는(Lost) 현상입니다. (그래도 Relocalization 모듈이 훌륭하게 작동하여 궤적을 끝까지 이어붙이는 데는 성공했습니다.)
2.  **FPS (처리 속도) 차이**
    *   7-Scenes (No-Calib): ~2.8 FPS
    *   TUM (No-Calib): ~2.6 FPS
    *   ETH3D (Calib): ~4.5 FPS
    *   논문에서는 RTX 4090 기준 15 FPS라고 했으나, 현재 환경에서는 2~4 FPS가 나오고 있습니다. 이는 CPU/GPU 스펙 차이이거나, `single_thread: True` 설정 때문일 수 있습니다.

---

### 4. 해결 방안 및 다음 단계 (Action Items)

**Step 1: `tracker.py` 코드 원복**
수학적 오해로 인해 변경된 가중치 코드를 원래대로 되돌려야 합니다.
`opt_pose_ray_dist_sim3`와 `opt_pose_calib_sim3` 함수 내의 `sqrt_info` 계산을 아래와 같이 수정하세요.

```python
# opt_pose_ray_dist_sim3 내부
sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk)

# opt_pose_calib_sim3 내부
sqrt_info_pixel = 1 / self.cfg["sigma_pixel"] * valid * torch.sqrt(Qk)
sqrt_info_depth = 1 / self.cfg["sigma_depth"] * valid * torch.sqrt(Qk)
```
*(노트 `MASt3R-SLAM Sec 3.3 Discrepancy.md`의 결론을 "논문의 수식 표기가 틀렸고, 깃허브의 원래 코드가 맞다"로 수정하시는 것을 권장합니다.)*

**Step 2: Calibrated 모드 재실험**
코드를 원복한 후 `bash ./scripts/eval_7_scenes.sh` (calib 모드)를 다시 실행해 보세요. `Degenerate covariance rank` 에러가 사라지고 정상적인 RMSE 평가 결과가 출력될 것입니다.

**Step 3: 속도(FPS) 최적화 (선택 사항)**
현재 2~4 FPS로 실시간(15 FPS)에 미치지 못하고 있습니다.
*   `config/base.yaml` 등에서 `single_thread: False`로 변경해 보세요. (백엔드 최적화와 프론트엔드 트래킹이 병렬로 돌아가 속도가 향상될 수 있습니다.)
*   PyTorch의 `half()` (FP16) 연산이 제대로 GPU에서 돌고 있는지 확인이 필요합니다.

