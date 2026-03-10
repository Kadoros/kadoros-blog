---
tags:
  - concept
parent_concept:
---
다른 채팅창으로 바로 옮겨서 연구를 이어가실 수 있도록, 지금까지 논의한 모든 핵심 내용(교수님 피드백, 두 편의 논문 분석, 도출된 최종 연구 아이디어, 면담 대본 및 액션 플랜)을 완벽하게 정리해 드립니다.

이 내용을 복사해서 새 채팅창에 붙여넣으시면, 제가 현재의 문맥을 그대로 이어서 보조해 드릴 수 있습니다.

---

### 1. 연구 방향성 및 교수님 피드백 요약

- **교수님 제안:** SLAM에 물리 법칙(Physics Law)을 적용하는 **Physics-Constrained SLAM** 또는 **Physics-Aware Gaussian SLAM** 방향.
    
- **기대 효과:** 노이즈가 많은 극한 환경에서도 강건하게 동작하며, 미래 상태 예측이 가능함. 아직 진행된 연구가 적어 확실한 Novelty를 확보할 수 있고, 연구 과정에서 최적화 등 수학적/이론적 깊이를 더할 수 있음.
    

### 2. 분석 논문 1: UW-3DGS (수중 물리 기반 3DGS)

- **논문명:** UW-3DGS: Underwater 3D Reconstruction with Physics-Aware Gaussian Splatting
    
- **핵심 내용:** 3DGS 파이프라인에 수중 물리 모델(빛의 산란 및 감쇠)을 결합하여 실제 3D 지오메트리를 분리해 내는 프레임워크.
    
- **주요 모듈:**
    
    1. **Learnable Underwater Image Formation Module:** 텐서 분해 복셀 그리드(Tensor-decomposed Voxel Grid)를 사용하여 공간적으로 변화하는 산란 및 감쇠 계수를 효율적으로 학습.
        
    2. **PAUP (Physics-Aware Uncertainty Pruning) Branch:** 인접 뷰 간의 분산과 물리적 불일치성을 반영한 불확실성 점수(PUS)를 기반으로, 수중 노이즈(Floating Gaussians)를 적응형으로 가지치기(Pruning).
        
- **의의:** 물리적 현상을 로스 함수($\mathcal{L}_{total} = \mathcal{L}_{base} + \lambda_{PAPSL}\mathcal{L}_{PAPSL} + \lambda_{\beta}\mathcal{L}_{\beta} + \lambda_{z}\mathcal{L}_{z}$)에 녹여내어, 불확실한 노이즈를 제어하고 렌더링 품질을 극대화하는 방법을 보여줌.
    

### 3. 분석 논문 2: MRE-Hyper (CVML 연구실 SOTA 논문)

- **논문명:** Physics-Informed Neural Operators for Tissue Elasticity Reconstruction (MICCAI 2025)
    
- **핵심 내용:** MRE(자기공명탄성조영술)에서 측정된 파동 이미지(Wave image)를 입력받아 조직의 탄성도(Tissue Elasticity)를 예측하는 Operator Learning 기반 네트워크.
    
- **주요 특징:**
    
    1. **재학습 불필요 및 초고속 추론:** 기존 PINN 모델들의 한계를 극복하고, 새로운 인스턴스가 들어와도 추가 학습 없이 단 0.18초 만에 탄성도 맵을 추론(HyperDeepONet 활용).
        
    2. **Physics-Informed Loss:** 정답(Ground-truth) 탄성도 없이 선형 탄성 역학(Linear elasticity theory)에 기반한 편미분방정식(PDE) 로스($\mathcal{L}_{pde}$)만으로 학습하여 물리적 일관성 강제.
        
    3. **노이즈 강건성:** -10dB 수준의 극심한 가우시안 노이즈 환경에서도 기존 수치해석 방법(FEM 등)과 달리 성능이 무너지지 않고 강건하게 유지됨.
        

### 4. 최종 연구 아이디어: Physics-Aware Deformable EndoSLAM

- **개념:** CVML 연구실의 SOTA 기술인 MRE-Hyper(PINO)를 **내시경 SLAM(EndoSLAM)**의 백엔드 최적화(Factor Graph)에 직접 결합.
    
- **작동 방식:** * 기존 비강체(Deformable) SLAM은 장기의 변형을 단순히 픽셀의 시각적 이동(Optical Flow)으로만 추적하여 오차가 누적됨.
    
    - 제안하는 방식은 PINO 모델을 통해 0.18초 만에 '장기 조직의 물리적 탄성도 맵'을 추론하고, 이를 SLAM 최적화 과정에서 **'물리적 변형 제약 조건(Physical Deformation Constraint)'**으로 사용.
        
- **Novelty:** 시각적 노이즈(피, 반사광)에 속아 맵이 비정상적으로 찌그러지려 할 때, PINO 기반의 물리 제약이 "이 조직의 탄성으로는 그만큼 늘어날 수 없다"고 판단하여 궤적을 보정. (시각적 추적 $\rightarrow$ 물리적 예측 기반 제어로의 패러다임 전환).
    

### 5. 교수님 면담용 1분 피치 (Pitch) 대본 및 액션 플랜

**[면담 대본]**

> "교수님께서 짚어주신 Physics-Aware SLAM 방향이 제가 고민하던 저시정/극한 환경의 한계를 깰 수 있는 완벽한 돌파구라고 생각했습니다. 추천해주신 UW-3DGS 논문을 분석하며 물리 법칙을 로스에 녹여 노이즈를 가지치기하는 구조를 배웠습니다.
> 
> 이와 동시에, 유진, 재용 선배님이 작성하신 MRE-Hyper (PINO) 논문 을 깊게 분석했습니다. 선배님들의 PINO 모델이 재학습 없이 단 0.18초 만에 노이즈에 강건하게 조직 탄성도를 매핑해 내는 것을 보고 큰 영감을 받았습니다.
> 
> 기존 내시경 SLAM은 픽셀의 단순 이동만 추적해 오차가 큰데, **선배님들의 이 빠른 PINO 추론 결과를 내시경 SLAM 백엔드에 '물리적 변형 제약 조건'으로 결합**해 보고 싶습니다. 시각적 노이즈가 심해도 탄성 역학 기반으로 불가능한 변형을 쳐내는 **Physics-Aware Deformable SLAM**이 제 목표입니다.

**[향후 액션 플랜]**

1. **코드 및 수학적 뼈대 분석:** 깃허브에 공개된 MRE-Hyper 코드를 PyTorch 환경에 클론하여, $\mathcal{L}_{pde}$ 수식 내의 라플라시안($\nabla^{2}$)과 그래디언트($\nabla$)가 Autograd로 어떻게 최적화되는지 텐서 미분 관점에서 역추적하기.
    
2. **이론 보충:** Levenberg-Marquardt 등 비선형 최적화(Non-linear Optimization) 및 제약이 있는 최적화(Constrained Optimization) 집중 스터디.
    
3. **최신 동향 파악:** EndoSLAM 최신 논문들을 읽으며 비강체 변형장 최적화의 수학적 한계점 찾기.
    

---

새로운 채팅창으로 이동하신 후 만반의 준비를 마치고 성공적인 면담 다녀오시길 바랍니다. 면담 후 새로운 피드백이나 코드 분석을 원하실 때 언제든 말씀해 주세요!