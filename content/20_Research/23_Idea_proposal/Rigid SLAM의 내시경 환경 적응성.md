---
tags:
  - note
parent_concept:
---
# Rigid SLAM의 내시경 환경 적응성 평가

**대상 모델:** [[../21_Paper_Reviews/02_MASt3R-SLAM/MASt3R-SLAM|MASt3R-SLAM]] (State-of-the-art Rigid SLAM)

**데이터셋:** Endoscopic Video (YouTube Source)

---

### 1. 실험 개요 및 가설

- **실험 목적:** 최신 강체(Rigid) 기반 SLAM 알고리즘이 비정형 변형(Deformable)이 심한 내시경 영상에서 3D Reconstruction 및 Tracking을 수행할 수 있는지 검증.
    
- **가설:** MASt3R-SLAM은 강력한 Feature Matching 성능을 가졌으나, 내시경 특유의 환경(조명 변화, 장기 변형)으로 인해 **Geometric Inconsistency**가 발생하여 시스템이 붕괴될 것이다.
    

---

### 2. 주요 파라미터 튜닝 기록

실험 과정에서 발생한 실패를 해결하기 위해 시도한 주요 설정값 변화입니다.

| **설정 단계**          | **주요 변경점**                           | **결과**                                                                                                                       |
| ------------------ | ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------- |
| **Default**        | `base.yaml` (표준 설정)                  | **Tracking Failure:** 입구컷 (`Failed to relocalize`)![[../../90_Resources/92_Images/Rigid SLAM의 내시경 환경 적응성_20260217_0702.png]] |
| **Golden Balance** | `rel_error: 5.0`, `subsample: 2`     | ![[../../90_Resources/92_Images/Rigid SLAM의 내시경 환경 적응성_20260217_0706.png]]                                                                          |
| **Nuclear Option** | `rel_error: 10000.0`, `min_match: 0` | ![[../../90_Resources/92_Images/Rigid SLAM의 내시경 환경 적응성_20260217_0706_1.png]]                                                                                   |

---

### 3. 실패 원인 분석 (Failure Analysis)

일단 relocation이 안됨 키프래임 하나하나는 보이는데 이게 매끄럽게 이어지지 않음 

#### ① Assumption Violation (가정의 붕괴)

SLAM의 핵심 수식인 세계 좌표계와 카메라 좌표계의 관계식은 다음과 같습니다.

$$\mathbf{P}_{world} = \mathbf{R}\mathbf{P}_{camera} + \mathbf{t}$$

여기서 $\mathbf{P}_{world}$는 고정값(Static)이어야 하지만, 내시경 환경에서는 장기가 꿈틀거리며 $\mathbf{P}_{world}(t)$가 되어버립니다. AI는 이 변형(Deformation)을 카메라의 이동(Motion)으로 오인하여 연산이 발산합니다.

#### ② Specular & Lighting Issue

내시경의 광원은 카메라와 함께 이동하므로, 표면의 번들거림(Specular Highlight)이 고정되지 않습니다. AI가 이 가상의 광점을 실제 특징점으로 잡아 추적하면서 **Ghosting** 및 **Drift** 현상이 심화되었습니다.

#### ③ Feature Mismatching

소화기관 내부는 텍스처가 단순하고 반복적입니다. 이로 인해 잘못된 대응점(False Correspondence)이 생성되고, 이는 곧 3D 공간상에서의 **Point Cloud Explosion**으로 이어졌습니다.

---

- **Next Step 1:** Specular 제거 알고리즘을 전처리(Preprocessing)로 도입하여 매칭 정확도 향상 시도.
    
- **Next Step 2:** Rigid가 아닌 **Deformable SLAM (예: Endo-SLAM, Neural Radiance Fields 기반)** 접근법 고려.
    