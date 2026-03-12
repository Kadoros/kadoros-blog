---
tags:
  - note
parent_concept:
  - "[[../01_GS_SLAM/GS_SLAM|GS_SLAM]]"
  - "[[SLAM|SLAM]]"
---
**연구 목표:** PINO(Physics-Informed Neural Operator)를 활용한 강건한 비강체(Deformable) 내시경 SLAM

**1. 배경 및 문제 제기 (Motivation)**

- **기존 EndoSLAM의 한계:** 3D Gaussian Splatting(3DGS)을 적용한 최신 비강체 내시경 SLAM(예: NRGS-SLAM) 모델들조차 장기의 변형을 추정할 때 순수 시각적 정보(RGB)에 과도하게 의존함. 이로 인해 내시경 특유의 반사광, 피, 텍스처 소실 발생 시 진짜 변형인지 노이즈인지 구분하지 못하고 기하학적 붕괴가 빈번하게 발생함.
    
- **물리적 제약의 부재:** 현재 모델들은 '조직의 물리적 특성'을 고려하지 않고 통계적/수치적 정규화에만 의존하여 최적화를 수행함. 따라서 장기의 변형이 물리적으로 불가능한 수준까지 찌그러지거나 팽창하는 오차를 원천적으로 방지하지 못함.
    

**2. 제안 방법 (Proposed Method: PINO-NRGS SLAM)**

- **핵심 컨셉:** PINO를 3DGS 기반 SLAM의 변형장(Deformation Field) 최적화를 제어하는 '물리 엔진'이자 '사전 지식(Prior)'으로 활용.
    
- **Step 1 (사전 학습된 PINO Prior 도입):** NRGS-SLAM에서 도출되는 실시간 변위 데이터를 **사전 학습되어 가중치가 고정된(Frozen) PINO 네트워크**에 입력하여 조직 탄성도 맵($\hat{\mu}$)을 추론. 이때 $\hat{\mu}$는 인체 조직의 실제 탄성 범위 내로 제약(Boundary constraints)을 둠.

- **Step 2 (물리적 제약 조건 생성):** 추론된 탄성도와 변위를 PDE 손실 함수에 대입. 변위가 물리적 타당성을 잃을 경우(형태 붕괴 등), **사전 학습된 $\hat{\mu}$와 충돌하여 로스가 발산하도록 설계**하여 SLAM의 변형장을 교정함.
    
    $$\mathcal{L}_{pde\_total} = \|\hat{\mu}\nabla^{2}\hat{u} + (\nabla\hat{u} + \nabla\hat{u}^{T})\nabla\hat{\mu}\|^{2} + \lambda_{div} \|\nabla \cdot \hat{u}\|^{2}$$
    
- **Step 3 :** SLAM 백엔드의 프레임 단위 변형 추정(Per-frame Deformation Estimation) 및 글로벌 번들 조정(Global Deformable Bundle Adjustment) 로스에 $\mathcal{L}_{pde_total}$을 주입하여, 시각적 추적 오류가 발생해도 물리 제약이 강제로 기하학적 일관성을 유지하도록 유도.
    

**3. 기대 효과 및 Novelty**

- **강건성(Robustness):** 반사광이나 텍스처 소실 구간에서도 단순 시각 정보가 아닌 물리 법칙(연속체 역학)을 기반으로 변형 궤적의 타당성을 검증하므로 기존 방식 대비 Tracking Lost를 강력하게 방지.
    
- **수학적 깊이:** 미분 가능한 3D Gaussian 렌더링 파이프라인(Differentiable Rasterization) 비선형 최적화 과정에 '미분 가능한 물리 제약(PyTorch Autograd 활용)'을 통합함으로써, 최적화 이론과 머신러닝의 결합을 심도 있게 연구.
    
- **차별성:** 비강체 3DGS SLAM이 한계로 지적받는 '시각적 모호성(Coupling Ambiguity)'을 신경망 기반의 물리 법칙 제약으로 해결하는 독창적인 접근법.
    

**4. 단계별 실행 계획 (Action Plan)**

- **Phase 1 (기초):** PINO 관련 논문(MRE-Hyper 등)의 PDE 로스 구현 원리를 분석하고, NRGS-SLAM 구조에서 변위 $\hat{u}$ 추출 및 물리 로스 주입 위치(식 20, 22, 25)를 특정.
    
- **Phase 2 (실험):** PyTorch 환경에서 $\hat{u}$를 입력받아 탄성도를 추론하고, Autograd를 통해 공간 미분($\nabla$) 연산이 포함된 커스텀 로스($\mathcal{L}_{pde\_total}$)가 정상적으로 계산 및 역전파(Backprop)되는지 단일 프레임 수준에서 검증.
    
- **Phase 3 (결합):** SLAM 최적화 백엔드 코드와 PINO 추론 모듈을 통합하여, StereoMIS 등 내시경 데이터셋의 조명 반사 구간에서 제안된 모델이 Tracking RMSE를 얼마나 개선하는지 확인.
    
