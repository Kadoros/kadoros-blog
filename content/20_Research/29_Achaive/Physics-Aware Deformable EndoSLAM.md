---
tags:
  - note
parent_concept:
---
**연구 목표:** PINO(Physics-Informed Neural Operator)를 활용한 강건한 비강체(Deformable) 내시경 SLAM

### 1. 배경 및 문제 제기 (Motivation)
*   **기존 EndoSLAM의 한계:** 장기는 비강체임. 기존 SLAM은 이를 단순한 Optical Flow으로 추적하여, **반사광, 피, 텍스처 소실** 발생 시 기하학적 붕괴가 빈번함. [[20_Research/23_Idea_proposal/Rigid SLAM의 내시경 환경 적응성|Rigid SLAM의 내시경 환경 적응성]]
*   **물리적 제약의 부재:** 현재 모델들은 '조직의 물리적 특성'을 고려하지 않고 순수 시각적 정보에만 의존하여, 장기의 변형이 물리적으로 불가능한 수준까지 찌그러지는 오차를 방지하지 못함.
### 2. 제안 방법 (Proposed Method: PINO-EndoSLAM)
*   **핵심 컨셉:** **PINO를 SLAM의 '물리 엔진'이자 '사전 지식(Prior)'으로 활용.**
*   **Step 1 (PINO 기반 탄성도 매핑):** MRE-Hyper(PINO) 구조를 응용하여, 내시경 영상에서 장기의 변형 데이터를 입력받아 '조직 탄성도 맵(Elasticity Map)'을 실시간 추론.
*   **Step 2 (물리적 제약 조건 생성):** 추론된 탄성도 정보를 기반으로, 장기가 변형될 수 있는 물리적 범위(Physical Constraints)를 수식화하여 SLAM 백엔드의 **최적화 로스 함수($\mathcal{L}_{pde}$)에 통합**.
*   **Step 3 (강건한 최적화):** 시각적 추적(Photometric Error)이 노이즈에 의해 왜곡되어도, 물리적 제약($\mathcal{L}_{pde}$)이 강제로 기하학적 일관성을 유지하도록 유도.
### 3. 기대 효과 및 Novelty
*   **강건성(Robustness):** 반사광이나 텍스처 소실 구간에서도 물리 법칙(탄성 역학)을 기반으로 변형 궤적을 예측하므로 기존 방식 대비 Tracking Lost 방지.
*   **수학적 깊이:** 비선형 최적화(Non-linear Optimization) 과정에서 **'미분 가능한 물리 제약'**을 추가함으로써, 최적화 이론과 머신러닝의 결합을 심도 있게 연구.
*   **차별성:** 단순히 신경망을 덧붙이는 것이 아니라, SLAM의 근간인 팩터 그래프(Factor Graph)에 **물리 법칙을 수학적 제약으로 통합**한다는 점
### 4. 단계별 실행 계획 (Action Plan)
1.  **Phase 1 (기초):** PINO 관련 논문 및 코드(MRE-Hyper 등)의 PDE 로스 구현 원리 완전 분석.
2.  **Phase 2 (실험):** 내시경 데이터셋 혹은 시뮬레이션 환경에서 장기 변형의 물리적 변수(Mass density, Actuator frequency 등)를 매핑하는 파라미터 학습 확인.
3.  **Phase 3 (결합):** SLAM 최적화 백엔드 코드와 PINO 추론 모듈을 결합하여, 변형장(Deformation Field)에 물리적 로스를 추가한 최적화 알고리즘 구현.
