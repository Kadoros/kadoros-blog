---
tags:
  - concept
  - reinforce
parent_concept:
---
여러 변수가 얽힌 복잡한 함수가 국소적으로 얼마나 변하는지를 보여주는 행열
다변수 함수판 미분 계수(기울기)

$$u = f(x, y), \space v = g(x, y)$$ 각 입력값 x,y를 조금씩 변화시킬떄 출력 u,v가 어떻게 바뀌는지를 보여주기 위핸 행렬


만약 $u = f(x, y)$, $v = g(x, y)$라는 함수가 있다면, 이들을 각각 편미분해서 행렬로 나열한 것이 **야코비안 행렬($J$)

$$J = \begin{bmatrix} \frac{\partial u}{\partial x} & \frac{\partial u}{\partial y} \\ \frac{\partial v}{\partial x} & \frac{\partial v}{\partial y} \end{bmatrix}$$


비선형 함수가 있을떄 국소 영역만 보면 선형 같다. 그래서 backprapagation 때 선형으로 퉁치는 것 


종류
1. [[Analytical Jacobians|Analytical Jacobians]]
2. [[Numerical Jacobian|Numerical Jacobian]]
3. [[AutoDiff|AutoDiff]]