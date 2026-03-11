---
tags:
  - note
parent_concept:
---
## **연구 목표: PINO(Physics-Informed Neural Operator) 기반의 실시간 강건 비강체(Deformable) 내시경 SLAM**

### **1. 배경 및 문제 제기 (Motivation)**

- **기존 EndoSLAM의 한계:** 장기는 끊임없이 움직이고 변형되는 비강체임에도 불구하고, 기존 SLAM은 이를 단순한 **Rigid assumption** 혹은 **Optical Flow**에 의존하여 추적합니다. 이로 인해 반사광(Specular highlights), 출혈, 텍스처 소실 발생 시 기하학적 정보가 완전히 붕괴되는 현상이 빈번합니다.
    
- **물리적 제약의 부재:** 현재의 딥러닝 기반 SLAM 모델들은 시각적 특징 일치(Photometric consistency)에만 매몰되어 있습니다. 조직의 **탄성 역학(Elasticity)**을 고려하지 않기 때문에, 장기가 물리적으로 불가능한 수준으로 일그러지거나 뚫리는 현상을 제어하지 못합니다.
    

### **2. 제안 방법 (Proposed Method: PINO-EndoSLAM)**

**핵심 컨셉:** **HyperDeepONet** 구조를 SLAM 백엔드에 통합하여, 단순한 추적이 아닌 '조직의 물리 법칙을 이해하는' 최적화 엔진을 구축합니다.

- **Step 1 (PINO 기반 실시간 탄성도 추론):** 내시경 영상($u_{sample}$)과 3D 좌표($y_{fourier}$)를 입력으로 하는 **Hyper-u** 및 **Hyper-$\mu$** 모듈을 구축합니다. 이를 통해 별도의 Ground Truth 탄성 정보 없이도 장기 부위별 탄성 계수(Elasticity Map)와 변형장(Deformation Field)을 실시간으로 추론합니다.
    
- **Step 2 (미분 가능한 물리 제약 조건 생성):** 추론된 탄성도($\mu$)를 바탕으로 아래와 같은 **선형 탄성 방정식(Linear Elasticity Equation)**을 SLAM의 Loss Function($\mathcal{L}_{pde}$)으로 정의합니다.
    
    $$\mathcal{L}_{pde} = \|\hat{\mu}\nabla^{2}\hat{u} + (\nabla\hat{u} + \nabla\hat{u}^{T})\nabla\hat{\mu} + \rho\omega^{2}\hat{u}\|^{2}$$
    
- **Step 3 (강건한 팩터 그래프 최적화):** 시각 정보가 불확실한 구간에서도 위 물리 제약이 **Prior**로 작용하여 가우시안(Gaussian) 또는 특징점들이 물리적으로 가능한 궤적 내에서만 움직이도록 강제합니다.
    

### **3. 기대 효과 및 Novelty**

- **강건성(Robustness):** MRE-Hyper가 입증한 **노이즈 강인성(Noise Robustness)**을 슬램에 이식하여, 저화질 내시경 환경에서도 안정적인 트래킹을 유지합니다.
    
- **실시간 일반화(Generalization):** 기존 PINN과 달리 환자별 **재학습(Retraining)이 필요 없는 연산자 학습** 방식을 채택하여, $0.2s$ 내외의 빠른 추론 속도로 실시간 수술 보조가 가능합니다.
    
- **학술적 차별성:** 단순히 딥러닝을 적용하는 것을 넘어, **비선형 최적화(Non-linear Optimization)** 과정에 연속체 역학(Continuum Mechanics) 수식을 직접 결합한 최초의 PINO 기반 EndoSLAM 시도입니다.
    

### **4. 단계별 실행 계획 (Action Plan)**

- **Phase 1 (분석):** MRE-Hyper의 **Hypernetwork**를 통한 가중치 생성 기법과 **PDE Loss**의 PyTorch 구현체를 정밀 분석합니다.
    
- **Phase 2 (구현):** 내시경 영상 특징(Feature Map)을 Hypernet의 입력 $u$로, 3D 가우시안 좌표를 $y$로 매핑하는 인터페이스를 설계합니다.
    
- **Phase 3 (검증):** **BIOQIC Abdomen 데이터셋**과 유사한 환경에서 노이즈 레벨별 트래킹 성공률을 비교하여, 제안 모델의 **$\mu$ Correlation** 및 기하학적 정확도를 검증합니다.
    

### 1. 변수 매핑: MRE의 언어를 SLAM의 언어로

먼저 식 (6)의 물리량들을 가우시안 파라미터로 치환해야 합니다.

|**MRE 변수**|**3DGS 대응 항목**|**의미**|
|---|---|---|
|**변위 ($\hat{u}$)**|**$\Delta \mu_{GS}$ (Deformation)**|각 가우시안 중심점의 원래 위치 대비 이동량입니다.|
|**탄성 ($\hat{\mu}$)**|**Stiffness ($\sigma_{GS}$)**|해당 가우시안이 속한 부위의 딱딱함 정도로, 학습 가능한 파라미터입니다.|
|**밀도 ($\rho$)**|**Opacity ($\alpha$)**|조직의 밀도를 가우시안의 불투명도와 연관 지어 상수로 취급할 수 있습니다.|
|**진동 ($\omega$)**|**Time-step ($\Delta t$)**|내시경 프레임 간의 시간 간격이나 장기의 움직임 주기로 해석합니다.|

---

### 2. PINO 기반 3DGS 손실 함수(Loss) 설계

기존 3DGS의 손실 함수에 **PDE Constraint**를 추가하여 전체 Loss를 구성합니다.

$$\mathcal{L}_{total} = \mathcal{L}_{color} + \lambda_{D} \mathcal{L}_{depth} + \gamma \mathcal{L}_{pde}$$

여기서 **$\mathcal{L}_{pde}$**는 식 (6)을 가우시안 알갱이들 사이의 관계로 이산화(Discretization)한 형태가 됩니다.

$$\mathcal{L}_{pde} = \sum_{i \in GS} \| \sigma_i \nabla^2 (\Delta \mu_i) + \text{Inter-Gaussian Forces} + \rho \omega^2 \Delta \mu_i \|^2$$

- **$\nabla^2 (\Delta \mu_i)$**: KNN(K-Nearest Neighbors)을 사용하여 주변 가우시안들과의 상대적 위치 변화를 계산함으로써 라플라시안(Laplacian)을 근사합니다.
    
- **물리적 해석**: 이 Loss는 가우시안들이 너무 급격하게 튀거나, 탄성이 낮은 부위에서 과하게 변형되는 것을 막아주는 **'물리적 스프링'** 역할을 합니다.
    

---

### 3. 파라미터 업데이트 수식 (Gradient Descent)

이제 역전파(Backpropagation) 시, 가우시안의 위치($\mu_{GS}$)와 하이퍼네트워크의 가중치($\Theta$)는 다음 수식에 의해 동시에 최적화됩니다.

$$\Theta_{next} = \Theta_{now} - \eta \frac{\partial (\mathcal{L}_{color} + \gamma \mathcal{L}_{pde})}{\partial \Theta}$$

$$\mu_{GS, next} = \mu_{GS, now} - \eta \frac{\partial \mathcal{L}_{total}}{\partial \mu_{GS}}$$

- **효과**: 시각 정보($\mathcal{L}_{color}$)가 불확실한 반사광 구간에서도, 물리 제약($\mathcal{L}_{pde}$)의 그라디언트(Gradient)가 가우시안들을 **물리적으로 타당한 위치**로 밀어넣습니다.
    


박사과정 김유진 선배님의 피드백 

1. 나의 논문은 $$\mathcal{L}_{pde} = \|\hat{\mu}\nabla^{2}\hat{u} + (\nabla\hat{u} + \nabla\hat{u}^{T})\nabla\hat{\mu} + \rho\omega^{2}\hat{u}\|^{2}$$에서 $\hat{u}$ ->  $\hat{\mu}$ 이다 그런디 $\hat{u}$ 를 어디서 가져 올것이며  나며지 변수들은? 어서 가져오니? 
2. endoslam의 베이스라인은 뭐임? 
3. endo 쪽에 physics-constrained를 가진 논문 있음? 있어? 그럼 pino 넣어 없어 일단 physics-constrained 해보고
	1. Endo 쪽에 Physics-constrained 논문 있어? yes
		1.  **PhysiXDeform** (IEEE, 2023 등): 내시경 비디오에서 연조직 변형을 예측할 때 물리적 사전 지식(속도, 방향성 변형률 등)을 그래프 네트워크에 제약 조건으로 넣은 연구.
			1. 이건 단순한 희소 메쉬 추적 모델입니다. 저희는 이 물리 제약의 개념을 **3D Gaussian Splatting 기반의 렌더링 SLAM**에 직접 결합하는 최초의 시도
		2. **Predicting 3D soft tissue dynamics from 2D imaging using physics informed neural networks**: 2D 내시경 이미지에서 3D 조직 움직임을 재구성할 때, 연속체 모델(Continuum model)을 딥러닝(RNN)에 물리 제약으로 넣은 연구. 
			1. NVIDIA A100 GPU에서 학습하는 데 약 7시간이 걸렸습니다
4. 그리고 pinn 안다 생각하고 아니면 deeponet ,FNO 중 deeponet이거 중심으로 pino는 physics-informed neunal opeartor for learning partial differnetial equation 이거 읽어보삼 