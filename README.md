# DWA Constraint-Based Accident Analysis

DASC 2026 - Dynamic Window Approach (DWA) + Constraint-Based XAI를 이용한 드론 사고 사후 분석 시스템

## 개요

이 프로젝트는 드론 운용 중 사고 발생 시, 제약 기반 설명 가능 AI(XAI)를 통해 사고 원인을 역추적하는 연구 프로토타입입니다.

### 주요 특징

- **Dynamic Window Approach (DWA)**: 의사결정 수준 설명 가능성 제공
- **Constraint-Based XAI**: 필연성(inevitability) 및 가능성(feasibility) 분석
- **No Perception/Vision**: 순수 제어 및 제약 기반 접근
- **Headless Simulation**: GUI 없이 시뮬레이션 실행
- **Reproducible Scenarios**: 명확한 ground-truth cause를 가진 3가지 시나리오

## 시스템 아키텍처

```
┌─────────────────┐
│   DWA Generator │  ← 후보 액션 생성
└────────┬────────┘
         │
┌────────▼────────┐
│ Constraint      │  ← 제약 검사 및 가능 집합 감사
│ Auditor         │
└────────┬────────┘
         │
┌────────▼────────┐
│ Decision        │  ← 의사결정 설명 생성
│ Explainer       │
└────────┬────────┘
         │
┌────────▼────────┐
│ JSONL Logger    │  ← 모든 데이터 로깅
└─────────────────┘
```

## 설치 및 환경 설정

### 필수 요구사항

- Windows 11 with WSL2
- Ubuntu 22.04 (WSL2)
- Python 3.10

### 단계별 설치

자세한 설치 가이드는 [setup_env.md](setup_env.md)를 참조하세요.

**빠른 시작**:

```bash
# WSL2 터미널에서
cd /mnt/c/Users/lab/dwa-constraint-accident-analysis

# 가상환경 생성 및 활성화 (WSL 홈 권장)
mkdir -p ~/venvs
python3.10 -m venv ~/venvs/dasc
source ~/venvs/dasc/bin/activate

# 의존성 설치
pip install -r requirements.txt
```

### 설치 검증

```bash
# Smoke test 실행
python scripts/smoke_test.py
```

## 사용 방법

### 1. 시나리오 실행

3가지 재현 가능한 시나리오가 제공됩니다:

**Scenario A: Inevitable Collision (필연적 충돌)**
```bash
python scripts/run_scenario.py --scenario A
```
- Feasible set이 비어있음
- Minimal relaxation 분석 제공

**Scenario B: Wrong Decision (잘못된 결정)**
```bash
python scripts/run_scenario.py --scenario B
```
- Feasible path 존재하지만 cost 가중치로 인해 잘못된 선택

**Scenario C: Delayed Response (지연된 대응)**
```bash
python scripts/run_scenario.py --scenario C
```
- 시간 지연으로 인한 충돌 패턴

### 2. 로그 분석

실행 후 `logs/` 디렉토리에 JSONL 파일이 생성됩니다:

- `ticks_*.jsonl`: 모든 timestep 데이터
- `events_*.jsonl`: 중요 이벤트만 (no feasible actions, near collision 등)

**로그 확인 예시**:

```bash
# Feasible ratio 확인
jq -r '[.timestep, .constraint_audit.feasible_ratio] | @csv' logs/ticks_*.jsonl

# Critical events 확인
jq '.event_type, .timestep' logs/events_*.jsonl
```

## 핵심 컴포넌트

### 1. DWA Candidate Generator (`src/dwa.py`)

- 후보 액션 생성 (velocity, yaw_rate)
- Cost function 기반 순위 매기기
- Deterministic sampling (시드 고정)

### 2. Constraint Auditor (`src/constraints.py`)

제약 조건:
- `min_obstacle_distance`: 최소 장애물 거리
- `max_acceleration`: 최대 가속도
- `max_yaw_rate`: 최대 요 레이트
- `max_velocity`: 최대 속도
- `altitude_bounds`: 고도 범위

기능:
- Feasible set 감사
- Per-constraint violation 통계
- Minimal relaxation 계산 (counterfactual)

### 3. Decision Explainer (`src/explainer.py`)

- 선택 이유 설명
- Cost breakdown
- Top-K 후보 비교
- Feasible alternatives 제시

### 4. JSONL Logger (`src/logger.py`)

- 모든 timestep 로깅
- Critical events만 별도 로깅
- Post-hoc 분석용 구조화된 데이터

## 시나리오 설명

### Scenario A: Inevitable Collision

**설정**:
- 드론: (0, 0, 1)
- 목표: (5, 0, 1)
- 장애물: (2.5, 0, 1) 등 경로 중간에 배치
- 엄격한 제약 조건

**Ground Truth**:
모든 후보 액션이 제약 위반. 충돌이 필연적.

**검증**:
- `n_feasible = 0` 감지
- `minimal_relaxation` 계산으로 필요 완화량 제시

### Scenario B: Wrong Decision

**설정**:
- 장애물이 경로에서 약간 벗어남 (안전 경로 존재)
- Cost function이 `goal_distance`에 과도한 가중치

**Ground Truth**:
안전한 경로가 존재하지만, cost 가중치로 인해 직접 경로 선택 → 제약 위반

**검증**:
- `n_feasible > 0`이지만 선택된 액션이 infeasible
- Decision explanation에 가중치 문제 명시

### Scenario C: Delayed Response

**설정**:
- 드론이 빠르게 이동 중
- Step 30에 장애물 출현 (지연된 감지)

**Ground Truth**:
개별 timestep에서는 feasible actions 존재하지만, 시간 지연으로 인해 충돌

**검증**:
- Step 30 전후 `feasible_ratio` 변화
- Temporal pattern 분석

## 검증 체크리스트

자세한 검증 방법은 [validation.md](validation.md)를 참조하세요.

주요 검증 항목:
1. ✅ 제약 기반 XAI가 필연성을 올바르게 감지
2. ✅ 의사결정 수준 XAI가 선택을 올바르게 설명
3. ✅ 로그가 사고 사후 분석에 충분
4. ✅ 재현 가능성

## 프로젝트 구조

```
dwa-constraint-accident-analysis/
├── README.md                 # 이 파일
├── setup_env.md             # 환경 설정 가이드
├── validation.md            # 검증 체크리스트
├── requirements.txt         # Python 의존성
├── src/
│   ├── __init__.py
│   ├── dwa.py              # DWA 후보 생성
│   ├── constraints.py      # 제약 기반 XAI (핵심)
│   ├── explainer.py        # 의사결정 설명
│   ├── logger.py           # JSONL 로깅
│   └── scenarios.py        # 시나리오 정의
├── scripts/
│   ├── smoke_test.py       # Headless 시뮬레이션 테스트
│   └── run_scenario.py     # 메인 실행 스크립트
└── logs/                   # 로그 파일 (생성됨)
```

## 제약 사항

- **No GUI**: 모든 실행은 headless 모드
- **No Vision**: Perception/vision 모델 사용 안 함
- **No Deep Learning**: 순수 제어 및 제약 기반 접근
- **Deterministic**: 재현 가능성을 위한 시드 고정

## 문제 해결

일반적인 설치 오류는 [setup_env.md](setup_env.md)의 "일반적인 설치 오류 및 해결" 섹션을 참조하세요.

## 참고 자료

- Dynamic Window Approach: 원본 논문
- gym-pybullet-drones: [GitHub](https://github.com/utiasDSL/gym-pybullet-drones)
- Constraint-Based XAI: 제약 기반 설명 가능 AI 이론

## 라이선스

연구 프로토타입 (DASC 2026)
