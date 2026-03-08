---
tags:
  - note
parent_concept:
  - "[[../02_MASt3R-SLAM/MASt3R-SLAM|MASt3R-SLAM]]"
  - "[[MASt3R-SLAM Sec 3.3 Discrepancy]]"
---
# introduction 
[[MASt3R-SLAM]]논문에서 3.3. Tracking and Pointmap Fusion 에서 나오는 수식 $E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {w(q,\sigma^2)} \right\|_\rho^2$ 을 코드 맵핑하던중 깃허브에 공개된 코드에서는 $E_r = \sum \left\|\frac{ \psi(\tilde{\mathbf{X}}_k) - \psi(\mathbf{T}_{kf} \mathbf{X}_f)} {\sqrt{w(q,\sigma^2)}} \right\|_\rho^2$의 수식을 사용하는 것을 발견하였다. 이에 대한 내용은 [[MASt3R-SLAM Sec 3.3 Discrepancy]]를 참고하기 바람  

