# Replayable Decision Forensics ConOps for Autonomous Guidance/Planning

DASC 2026 - 자율 유도/플래닝의 사후 판별(불가피성/선택오류)을 위한 운용개념(ConOps) 프로토타입

## 연구 배경

고도 자동화 및 자율 비행은 UAM을 포함해 빠르게 확산되고 있으며, 운용 개념(ConOps)에서도 고도 자동화와 협력 운용을 전제한 시나리오가 제시되고 있습니다. 자동화가 강화될수록 사고·사건 또는 Near-miss 발생 시, 단순히 "무슨 일이 있었는가"를 넘어 **"시스템이 왜 그러한 결정을 내렸는가"**를 운용 및 조사 관점에서 설명할 수 있어야 한다는 요구가 커집니다.

특히 자율 유도/플래닝 계층은 제한된 시간과 정보 하에서 다수의 후보 행동을 비교해 하나를 선택하므로, 사고 이후 해당 결정이 불가피했는지 혹은 선택 오류가 기여했는지를 판별하는 능력이 중요합니다.

## 문제 정의

자율비행 안전 보장은 설계 검증, 런타임 보증, 인증 프레임워크 등 다양한 축으로 논의되지만, 본 연구는 그중 **사고/사건 후 사후 분석(forensics)** 관점에 집중합니다.

자율 유도/플래닝에서 사고 조사에 핵심적인 질문은 두 가지입니다:

1. **불가피성(inevitability)**: 해당 시점의 물리적·운용적 제약 하에서 안전한 회피 행동이 존재했는가?
2. **선택오류(choice error)**: 안전한 대안이 존재했다면, 왜 위험한 행동이 선택되었는가?

## 기존 접근의 한계

1. **기록 가용성 문제**: 기록이 온보드 중심이면 사고·수색·회수 지연 등으로 인해 적시에 확보가 어려울 수 있음
2. **의사결정 근거 증거 부족**: 전통적 기록(FDR/QAR 등)은 상태/명령 기반의 사건 재구성에는 강하지만, 자율 의사결정에서 필요한 "대안 존재 여부, 비용·제약 트레이드오프, 선택 마진"과 같은 직접 증거가 부족

## 제안 ConOps (운용개념)

본 연구는 특정 XAI 기법을 제안하는 것이 아니라, 자율 유도/플래닝 의사결정의 **재현 가능성(replayability)**을 확보하여 사후에 불가피성/선택오류를 판별할 수 있게 하는 **운용개념(ConOps)**을 제안합니다.

### 적용 범위

본 ConOps는 "유도기가 반드시 제약기반"이라고 단정하지 않습니다. 대신 제약이 명시적으로 모델링되거나(최적화/제약해결), 혹은 안전필터·포화·제한 등으로 강제(enforced)되는 자율 유도/플래닝 모듈을 대상으로 합니다.

### 특징 1: Replay Contract (재현 최소 로그 계약)

자율 의사결정을 사후 재현하기 위해 필요한 최소 로그 스키마를 정의합니다. "원시 센서"가 아니라 **대안 존재/비용 분해/제약 마진**을 복원할 수 있는 형태의 기록을 남기는 것을 목표로 합니다:

- **상시 저비트 로깅** (`ticks.jsonl`): 모든 timestep에서 상태, 선택된 행동, DWA 비용, 제약 통계 기록
- **이벤트 구간 스케치 로그** (`events.jsonl`): 중요 이벤트 시점의 ring buffer, Top-K 후보, 최소 완화량 기록

### 특징 2: GCS 모델/제약 동기화 전제

GCS는 인증/운용 승인된 모델·제약 템플릿을 보유하고, 항공기는 임무/모드 변화 시 **유효 파라미터 스냅샷(버전/해시)**을 공유하여 사후 재현 시 모델 불일치를 최소화합니다.

### 특징 3: GCS 기반 사후 재현/판별 절차

이 절차의 목적은 사고 원인 규명이 아니라 **조사 범위를 줄이는 트리아지(triage)**입니다:

- **(i) 불가피성 판별**: feasible set 존재 여부, 지배 제약, 최소 완화량 등으로 판단
- **(ii) 선택오류 판별**: 후보 대안 비교(top-k), 비용 항 분해, 선택 마진으로 판단

## 검증 방법 (Case Study)

DWA 기반 로컬 플래너를 사용하는 case study로 수행:

- **S1**: 제약 포화로 인해 feasible set이 공집합으로 붕괴하는 불가피 사고(feasible=∅)
- **S2**: feasible set이 존재하지만 비용 구조로 인해 위험 행동이 선택되는 선택오류 상황(feasible≠∅)

제안 ConOps가 두 시나리오에서 사후 판별을 안정적으로 수행하는지 평가합니다.

> **참고**: 통신 환경의 패킷 손실/지연은 본 실험에서 모델링하지 않고 향후 과제로 명시합니다.

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

# 가상환경 생성 및 활성화
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

**S1: Inevitable Collision (불가피한 충돌)**
```bash
python scripts/run_scenario.py --scenario s1 --seed 0 --out outputs/ --gui
```
- 제약 포화로 feasible set이 공집합으로 붕괴
- 불가피성 판별: `n_feasible == 0`, 지배 제약, 최소 완화량 분석

**S2: Wrong Decision (선택오류)**
```bash
python scripts/run_scenario.py --scenario s2 --seed 0 --out outputs/ --gui
```
- Feasible set 존재하지만 비용 구조로 인해 위험 행동 선택
- 선택오류 판별: Top-K 후보 비교, 비용 항 분해, 선택 마진 분석

### 2. Replay Contract 로그 분석

실행 후 `outputs/<scenario>/seed_<seed>/run_<run_id>/` 디렉토리에 생성됩니다:

- **`ticks.jsonl`**: 모든 timestep의 상시 저비트 로그
  - 상태, 선택된 행동, DWA 비용 항, 제약 통계, feasible set 통계
- **`events.jsonl`**: 이벤트 구간 스케치 로그
  - Ring buffer (직전 5초)
  - Top-K 후보 상세 정보
  - 최소 완화량 (S1)
  - 선택된 행동 vs 안전한 대안 비교 (S2)

**로그 확인 예시**:

```bash
# Feasible ratio 추이 확인
jq -r '[.timestep, .metrics.feasible_ratio] | @csv' outputs/s1/seed_0/run_*/ticks.jsonl

# 이벤트 트리거 확인
jq '.timestep, .metrics.n_feasible' outputs/s1/seed_0/run_*/events.jsonl
```

### 3. 결과 요약 생성

```bash
python scripts/generate_paper_results.py --outputs outputs
```

이 명령어는:
- `outputs/run_summary.csv`: 모든 실행의 정량적 지표 요약
- `outputs/experimental_results_summary.md`: 논문용 실험 결과 요약

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
- **Feasible set 감사**: 모든 후보에 대한 제약 위반 검사
- **Per-constraint violation 통계**: 각 제약별 위반 비율 및 최악 위반량
- **Minimal relaxation 계산**: feasible set이 비어있을 때 최소 완화량 (counterfactual)

### 3. Operational Logger (`src/operational_logging.py`)

Replay Contract 구현:
- `ticks.jsonl`: 상시 저비트 로깅 (대안 존재/비용 분해/제약 마진 복원 가능)
- `events.jsonl`: 이벤트 구간 스케치 로그 (ring buffer, Top-K, 최소 완화량)

## 시나리오 설명

### S1: Inevitable Collision (불가피성)

**설정**:
- 드론이 높은 전진 속도로 가까운 장애물에 접근
- 엄격한 가속/제동 제한

**Ground Truth**:
모든 후보 액션이 제약 위반. feasible set이 공집합으로 붕괴. 충돌이 필연적.

**판별 증거**:
- `n_feasible == 0`이 충돌 전 >= 0.3초 지속
- 지배 제약: 주로 `max_acceleration`
- 최소 완화량: `delta_accel`이 의미 있게 큼

### S2: Wrong Decision (선택오류)

**설정**:
- 안전한 우회 경로 존재
- Cost 가중치가 goal 진행에 과도하게 집중

**Ground Truth**:
Feasible set은 비어있지 않지만, 비용 구조로 인해 위험한 직진 경로 선택 → 충돌/근접충돌

**판별 증거**:
- 트리거 시점에 `n_feasible > 0`
- Top-K에 더 안전한 feasible 대안 존재
- 선택된 행동은 goal 항 우세로 인해 선택됨
- 결정 마진이 작음 (거의 선택될 뻔했음)

## 논의

### 적용 대상

UAM 등 도심권 자율항공을 주요 적용 대상으로 합니다 (고도 자동화 운용을 전제한 ConOps 논의와 정합).

### 제약 1: "유도만 본다" 비판 대응

본 ConOps는 모든 사고 원인을 규명하지 않습니다. 대신 유도/플래닝 의사결정이 사고에 기여했는지를 판별(불가피성/선택오류)하여 **조사 범위를 축소**합니다.

### 제약 2: 통신 대역폭/로스 비고려

본 ConOps에서 통신은 온보드 기록을 대체하는 것이 아니라, 기록 가용성 및 조사 신속화를 높이는 **레던던시 옵션**으로 다룹니다.

## 주요 기여

1. **Replayable Decision Forensics ConOps**: 자율 유도/플래닝의 사후 판별(불가피성/선택오류)을 위한 운용개념 제안
2. **Replay Contract + 판별 절차**: 최소 로그 스키마와 GCS 기반 재현·판별 프로토콜 정의
3. **Case study 실증**: DWA 기반 시나리오에서 제안 ConOps가 의사결정 판별을 수행함을 경험적으로 입증 (통신 오류 제외)

## 프로젝트 구조

```
dwa-constraint-accident-analysis/
├── README.md                      # 이 파일
├── EXPERIMENTAL_PROCEDURE.md      # 실험 절차 가이드
├── setup_env.md                   # 환경 설정 가이드
├── validation.md                  # 검증 체크리스트
├── requirements.txt               # Python 의존성
├── src/
│   ├── __init__.py
│   ├── dwa.py                    # DWA 후보 생성
│   ├── constraints.py            # 제약 기반 XAI (핵심)
│   ├── operational_logging.py    # Replay Contract 구현
│   └── nlg.py                    # 자연어 요약
├── scenarios/
│   ├── __init__.py
│   ├── s1_inevitable_dynamics.py  # S1 시나리오
│   └── s2_bad_decision.py         # S2 시나리오
├── scripts/
│   ├── smoke_test.py             # Headless 시뮬레이션 테스트
│   ├── run_scenario.py           # 메인 실행 스크립트
│   ├── generate_paper_results.py # 결과 요약 생성
│   └── print_summary.py          # 요약 출력
└── outputs/                      # 실행 결과 (생성됨)
    ├── s1/
    ├── s2/
    ├── run_summary.csv
    └── experimental_results_summary.md
```

## 참고 자료

- Dynamic Window Approach: 원본 논문
- gym-pybullet-drones: [GitHub](https://github.com/utiasDSL/gym-pybullet-drones)
- Constraint-Based XAI: 제약 기반 설명 가능 AI 이론
- EASA ML/AI Guidelines: 운영 데이터 기록 요구사항

## 라이선스

연구 프로토타입 (DASC 2026)
