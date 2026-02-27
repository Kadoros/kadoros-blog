---
tags:
  - concept
  - reinforce
parent_concept:
---
[[Gradient Descent|Gradient Descent]]과 [[Gauss-Newton|Gauss-Newton]]을 합쳐 [[Non-linear Optimization|Non-linear Optimization]]을 수핼하는 알고리즘


제시해주신 수식 $(H + \lambda I)\Delta p = g$는 Levenberg-Marquardt(LM) 알고리즘의 **업데이트 단계(Step Calculation)**를 가장 직관적으로 보여주는 식입니다.

학부 수준을 넘어, **최적화 이론(Optimization Theory) 및 수치해석적 관점**에서 조금 더 전문적인(Professional) 용어와 개념으로 해설해 드리겠습니다.

---

### 1. 수식의 
$$(H + \lambda I)\Delta p = g$$
$$(J^T J + \lambda I) \delta = J^T e$$

- $J$: [[Jacobian|Jacobian]]
- **$H \approx J^T J$**: 손실 함수(Loss Function)의 2차 미분 정보를 담은 **헤시안 행렬(Hessian Matrix)의 근사(Approximation)**입니다.
    
- **$g = J^T e$**: 손실 함수의 1차 미분 정보인 **그라디언트(Gradient) 벡터**입니다.
    
- **$\lambda I$**: **댐핑 항(Damping Term)** 또는 **정칙화(Regularization) 항**입니다.
    
$$where \space \lambda \space is \space big, \space J^T J\delta= J^T e $$
- [[Gradient Descent|Gradient Descent]]
$$where \space \lambda \space is \space small, \space I\delta= J^T e $$
- [[Gauss-Newton|Gauss-Newton]]