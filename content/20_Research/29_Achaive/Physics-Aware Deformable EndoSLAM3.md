---
tags:
  - note
parent_concept:
---

# 연구 목표: PINO(Physics-Informed Neural Operator) 기반의 실시간 강건 비강체(Deformable) 내시경 SLAM

## 1. 배경 및 문제 제기 (Motivation)

- **기존 EndoSLAM의 한계:** 장기는 끊임없이 움직이고 변형되는 비강체임에도 불구하고, 기존 모델들은 이를 단순한 Rigid assumption 혹은 Optical Flow에 의존하여 추적합니다. 이로 인해 반사광(Specular highlights), 출혈, 텍스처 소실 발생 시 기하학적 정보가 완전히 붕괴되는 현상이 빈번합니다.
    
- **물리적 제약의 부재:** 3D Gaussian Splatting(3DGS) 기반의 최신 내시경 모델인 EndoGaussian은 195 FPS라는 압도적인 실시간 렌더링 속도를 달성했지만, 물리적 제약(Physics-constraint) 없이 시각적 컬러($\mathcal{L}_{color}$)와 Depth에만 의존합니다. 조직의 탄성 역학(Elasticity)을 고려하지 않기 때문에 장기가 물리적으로 불가능한 수준으로 일그러지거나 찢어지는 현상을 제어하지 못합니다.
    

## 2. 제안 방법 (Proposed Method: PINO-EndoSLAM)

**핵심 컨셉:** 실시간성을 갖춘 EndoGaussian을 베이스라인으로 삼고, HyperDeepONet 구조를 SGT(Spatio-temporal Gaussian Tracking) 모듈에 통합합니다. 단순한 시각적 추적이 아닌 '조직의 물리 법칙을 이해하는' 최적화 엔진을 구축하여 물리적 강건성(Robustness)을 확보합니다.

- **Step 1 (변위 $\hat{u}$ 및 변수 매핑):** EndoGaussian의 4D 인코딩 복셀(HexPlane)과 MLP가 1차적으로 예측하는 가우시안 중심점의 이동량($\Delta \mu_{GS}$)을 초기 변위 $\hat{u}$로 취급합니다. 인체 연조직은 수분이 대부분이므로 밀도 $\rho$는 상수로 취급합니다.
    
- **Step 2 (PINO 기반 탄성도 $\hat{\mu}$ 추론):** 내시경 영상 특징과 HexPlane의 Latent Feature를 DeepONet의 Branch-Trunk 구조에 통과시켜, 별도의 Ground Truth 없이도 장기 부위별 탄성 계수($\hat{\mu}$)를 실시간으로 추론합니다.
    
- **Step 3 (Autograd 기반 물리 제약 조건 생성):** $\hat{u}$가 개별 점이 아닌 연속된 딥러닝 함수 $f(x,y,z,t)$로 정의됨을 이용하여, K-NN 연산 없이 PyTorch의 `autograd`로 공간 편미분값을 도출합니다. 수술 중 연조직의 변형은 관성보다 탄성력이 지배적인 **준정적(Quasi-static)** 상태임을 반영하여 기존의 고주파 연속파($\omega$) 가정을 배제하고, 다음과 같이 형태 붕괴와 부피 팽창을 동시에 막는 손실 함수($\mathcal{L}_{pde}$)를 정의합니다.
    
    $$\mathcal{L}_{pde\_total} = \underbrace{\|\hat{\mu}\nabla^{2}\hat{u} + (\nabla\hat{u} + \nabla\hat{u}^{T})\nabla\hat{\mu}\|^{2}}_{\text{비정상적인 찢어짐(형태 붕괴) 방지}} + \underbrace{\lambda_{div} \|\nabla \cdot \hat{u}\|^{2}}_{\text{비정상적인 부풀림(부피 팽창, 비압축성) 방지}}$$
    
- **Step 4 (강건한 최적화):** 전체 손실 함수 $\mathcal{L}_{total} = \mathcal{L}_{color} + \lambda_{D} \mathcal{L}_{depth} + \gamma \mathcal{L}_{pde_total}$를 통해, 시각 정보가 불확실한 구간(반사광 등)에서도 물리 수식이 가우시안들을 물리적으로 타당한 위치로 강력하게 밀어 넣도록 강제합니다.
    

## 3. 기대 효과 및 학술적 차별성 (Novelty)

- **왜 PINN이 아닌 PINO인가? (실시간성 확보):** 연조직 변형에 연속체 역학(Continuum model)을 적용한 기존 PINN 연구는 단일 씬 학습에 7시간(A100 GPU 기준)이 소요되어 실시간 SLAM에 적용이 불가능합니다. 본 연구는 연산자(Operator) 자체를 학습하는 PINO를 채택하여 Forward Pass 한 번으로 물리 법칙을 실시간 추론합니다.
    
- **단순 메쉬 추적을 넘어선 Dense 3DGS SLAM:** 최근 물리적 모션 특징을 제약 조건으로 넣은 연구가 존재하나 이는 희소 메쉬(Sparse Mesh) 추적 모델에 불과합니다. 비선형 최적화 과정에 연속체 역학 수식을 고화질 3DGS 렌더링 SLAM에 직접 결합하는 것은 본 연구가 최초입니다.
    
- **임상적 킬러 애플리케이션 (Visual Elastography):** 단순한 3D 추적을 넘어, 모델이 학습 과정에서 획득한 각 가우시안의 상대적 탄성도($\hat{\mu}$) 맵을 렌더링에 투영함으로써, **의사가 내시경 화면만으로도 표면 아래에 숨겨진 딱딱한 종양(Tumor)의 위치를 증강현실(AR)처럼 직관적으로 파악**할 수 있는 실시간 진단 보조 도구로 활용될 수 있습니다.
    

## 4. 성능 검증 및 비교군 (Baselines) 설정

- **메인 베이스라인:** **EndoGaussian** (최신 3DGS 모델). 시각적 오차에만 의존하는 모델과 대비하여 준정적 PINO 제약($\mathcal{L}_{pde\_total}$)이 노이즈 강인성 및 형태 유지에 얼마나 기여하는지 증명합니다.
    
- **서브 베이스라인:** **EndoNeRF, EndoSurf, LerPlane** (NeRF 기반 비강체 모델들). 연산 효율성 및 렌더링 속도 측면에서 기존 3D 모델들을 아득히 뛰어넘음을 수치적으로 입증합니다.
    

## 5. 단계별 실행 계획 (Action Plan)

- **Phase 1 (환경 세팅 및 분석):** EndoGaussian 공식 코드를 클론하여 ENDONERF 데이터셋으로 렌더링 데모를 구동하고, SGT 모듈 내 HexPlane 및 MLP 구조를 정밀 분석합니다.
    
- **Phase 2 (구현):** SGT 모듈을 DeepONet 구조로 확장하여 $\hat{\mu}$ 채널을 추가하고, `torch.autograd`를 이용한 준정적 탄성 방정식 및 발산(Divergence) 항 연산 파이프라인을 PyTorch로 구현합니다.
    
- **Phase 3 (검증):** 노이즈 레벨별 트래킹 성공률과 기하학적 정확도를 측정하여 메인/서브 베이스라인과 성능을 비교하고, $\hat{\mu}$ 히트맵의 유효성을 시각적으로 검증합니다.
    
