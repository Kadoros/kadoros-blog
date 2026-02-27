---
tags:
  - note
parent_concept:
---



[[MASt3R-SLAM]] 논문의 Tracking 파트(Sec 3.3)에서 정의한 에러 함수(Error Function)와 실제 공식 GitHub의 PyTorch 구현체 사이에 **수학적 모순**을 발견했습니다.

### 논문의 수식 (Paper Equation)
논문에서는 방향(Ray) 에러 $E_r$을 다음과 같이 정의합니다.
$$E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {w(q,\sigma^2)} \right\|_\rho^2$$
여기서 가중치 함수는 $w(q, \sigma^2) = \frac{\sigma^2}{q}$ 입니다. ($q$: 신뢰도, $\sigma$: 노이즈 표준편차)
이를 수식에 그대로 대입하면, 논문이 주장하는 에러는 다음과 같습니다.
$$ E_{paper} = \sum \left\| \frac{q}{\sigma^2} \cdot \mathbf{r} \right\|_\rho^2 = \sum \left( \frac{q^2}{\sigma^4} \right) \mathbf{r}^2 $$

### 실제 코드 구현 (Code Implementation)
하지만 `tracker.py`의 코드를 분석해 보면 완전히 다르게 연산됩니다.
```python
# 1. 가중치의 역수 제곱근 계산
sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)  # sqrt(q) / sigma

# 2. 잔차(r)에 곱하여 Whitening 수행
whitened_r = sqrt_info * r  # (sqrt(q) / sigma) * r

# 3. 최종 Cost 계산 (Huber 제외 시)
b = whitened_r
cost = 0.5 * (b.T @ b)
```
코드가 계산하는 실제 에러는 다음과 같습니다.
$$ E_{code} = \frac{1}{2} \sum \left\| \frac{\sqrt{q}}{\sigma} \cdot \mathbf{r} \right\|^2 = \frac{1}{2} \sum \left( \frac{q}{\sigma^2} \right) \mathbf{r}^2 $$


따라서 논문의 수식은 다음과 같이 **수정**되어야 합니다.

수정 전 (논문)
$$E_r = \sum \left\|\frac{ \mathbf{r}} {w(q,\sigma^2)} \right\|_\rho^2$$

수정 후 (올바른 수식)
$$E_r = \sum \left\|\frac{ \mathbf{r}} {\sqrt{w(q,\sigma^2)}} \right\|_\rho^2 \quad \text{또는} \quad E_r = \frac{1}{2} \sum \mathbf{r}^T \left( \frac{q}{\sigma^2} \right) \mathbf{r}$$

