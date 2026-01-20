# 실험 절차 및 결과 산출 과정

## 1. 실험 설정

### 1.1 환경 설정
```bash
# WSL2에서 실행 (Ubuntu 22.04)
cd /mnt/c/Users/lab/dwa-constraint-accident-analysis
source ~/venvs/dasc/bin/activate
```

### 1.2 실험 시드 및 반복 횟수
- **S1 (Inevitable Dynamics)**: 시드 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 (총 10회)
- **S2 (Bad Decision)**: 시드 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 (총 10회)
- 총 **20개 실행** (시나리오당 10회)

---

## 2. S1 실험 실행

### 2.1 S1 실행 명령어 (시드별 반복)
```bash
# 시드 0
python scripts/run_scenario.py --scenario s1 --seed 0 --out outputs/ --gui

# 시드 1
python scripts/run_scenario.py --scenario s1 --seed 1 --out outputs/ --gui

# 시드 2
python scripts/run_scenario.py --scenario s1 --seed 2 --out outputs/ --gui

# 시드 3
python scripts/run_scenario.py --scenario s1 --seed 3 --out outputs/ --gui

# 시드 4
python scripts/run_scenario.py --scenario s1 --seed 4 --out outputs/ --gui

# 시드 5
python scripts/run_scenario.py --scenario s1 --seed 5 --out outputs/ --gui

# 시드 6
python scripts/run_scenario.py --scenario s1 --seed 6 --out outputs/ --gui

# 시드 7
python scripts/run_scenario.py --scenario s1 --seed 7 --out outputs/ --gui

# 시드 8
python scripts/run_scenario.py --scenario s1 --seed 8 --out outputs/ --gui

# 시드 9
python scripts/run_scenario.py --scenario s1 --seed 9 --out outputs/ --gui
```

### 2.2 S1에서 저장되는 결과

각 실행마다 `outputs/s1/seed_<seed>/run_<run_id>/` 디렉토리에 저장:

#### A. 타임스텝별 로그 (`ticks.jsonl`)
- **저대역폭 로그**: 모든 타임스텝(240Hz)에서 기록
- **포함 데이터**:
  - `timestep`: 시뮬레이션 스텝
  - `state`: 드론 위치, 속도, 자세
  - `selected_action`: 선택된 행동 [vx, vy, vz, yaw_rate]
  - `metrics`:
    - `n_candidates`: 후보 행동 수
    - `n_feasible`: Feasible 후보 수
    - `feasible_ratio`: Feasible 비율
    - `dist_min`: 최소 장애물 거리
    - `min_time_to_collision`: 최소 충돌 예상 시간 (TTC)
    - `decision_margin`: 결정 마진
  - `dwa`:
    - `selected_cost_total`: 선택된 행동의 총 비용
    - `cost_terms`: 비용 항별 분해 (goal_distance, obstacle_penalty, velocity, yaw_rate)
  - `constraints`:
    - `per_constraint_stats`: 제약별 위반 통계
      - `max_acceleration`: 가속도 제약 위반 비율
      - `max_yaw_rate`: 회전율 제약 위반 비율
      - `min_obstacle_distance`: 장애물 거리 제약 위반 비율
    - `selected_slack`: 선택된 행동의 제약 여유도
    - `minimal_relaxation` (N_feasible == 0일 때): 최소 완화량 (Δ_accel, Δ_yaw)

#### B. 이벤트 로그 (`events.jsonl`)
- **트리거 조건**: `N_feasible == 0` 지속 >= 0.3초 OR `TTC < 1.0초` OR 충돌
- **포함 데이터**:
  - `run_id`: 실행 ID
  - `timestep`: 이벤트 발생 스텝
  - `feasible_set_statistics`:
    - `n_candidates`: 총 후보 수
    - `n_feasible`: Feasible 후보 수
    - `feasible_ratio`: Feasible 비율
  - `constraint_violation_summary`:
    - `dominant_constraint`: 우세 제약 (위반 비율이 가장 높은 제약)
    - `per_constraint_stats`: 제약별 상세 통계
  - `minimal_relaxation`:
    - `delta_accel`: 가속도 제약 최소 완화량
    - `delta_yaw`: 회전율 제약 최소 완화량
  - `top_k`: 상위 K개 후보 (각 후보의 비용, feasible 여부, slack)
  - `metrics`:
    - `min_obstacle_distance`: 최소 장애물 거리
    - `min_time_to_collision`: TTC
    - `n_feasible`: Feasible 후보 수
  - `ring_buffer`: 이벤트 발생 직전 5초간의 타임스텝 로그

#### C. 스크린샷 (`frames/`)
- **주기적 캡처**: 1초 간격 (0.5초 간격으로 변경 가능)
  - `s1_step00000_periodic_close.png`: 3인칭 시점
  - `s1_step00000_periodic_drone.png`: 1인칭 시점
- **트리거 시점 캡처**:
  - `s1_step00150_trigger_close.png`: 3인칭 시점
  - `s1_step00150_trigger_drone.png`: 1인칭 시점

#### D. 최종 보고서 (`report.json`)
- `scenario`: 시나리오 이름
- `gt_label`: Ground truth 레이블 ("INEVITABLE_DYNAMICS")
- `predicted_label`: 예측 레이블
- `summary_text`: 자연어 요약 (NLG)

---

## 3. S2 실험 실행

### 3.1 S2 실행 명령어 (시드별 반복)
```bash
# 시드 0
python scripts/run_scenario.py --scenario s2 --seed 0 --out outputs/ --gui

# 시드 1
python scripts/run_scenario.py --scenario s2 --seed 1 --out outputs/ --gui

# 시드 2
python scripts/run_scenario.py --scenario s2 --seed 2 --out outputs/ --gui

# 시드 3
python scripts/run_scenario.py --scenario s2 --seed 3 --out outputs/ --gui

# 시드 4
python scripts/run_scenario.py --scenario s2 --seed 4 --out outputs/ --gui

# 시드 5
python scripts/run_scenario.py --scenario s2 --seed 5 --out outputs/ --gui

# 시드 6
python scripts/run_scenario.py --scenario s2 --seed 6 --out outputs/ --gui

# 시드 7
python scripts/run_scenario.py --scenario s2 --seed 7 --out outputs/ --gui

# 시드 8
python scripts/run_scenario.py --scenario s2 --seed 8 --out outputs/ --gui

# 시드 9
python scripts/run_scenario.py --scenario s2 --seed 9 --out outputs/ --gui
```

### 3.2 S2에서 저장되는 결과

S2도 동일한 구조로 저장되지만, 이벤트 트리거 조건이 다름:

#### A. 타임스텝별 로그 (`ticks.jsonl`)
- S1과 동일한 구조

#### B. 이벤트 로그 (`events.jsonl`)
- **트리거 조건**: `dist_min < 0.5` AND `n_feasible > 0` OR 충돌
- **추가 포함 데이터**:
  - `selected_vs_safer_alternative`:
    - `safer_feasible_alternative_exists`: 더 안전한 feasible 대안 존재 여부
    - `safer_alternative_rank`: 더 안전한 대안의 Top-K 내 순위
    - `decision_margin`: 선택된 행동과 더 안전한 대안 간 비용 차이
    - `selected_action`: 선택된 행동
    - `selected_cost_total`: 선택된 행동의 총 비용
    - `safer_action`: 더 안전한 대안 행동
    - `safer_cost_total`: 더 안전한 대안의 총 비용

#### C. 스크린샷 (`frames/`)
- S1과 동일한 구조

#### D. 최종 보고서 (`report.json`)
- S1과 동일하지만 `gt_label`은 "BAD_DECISION"

---

## 4. 결과 요약 생성

### 4.1 실행 단위 요약 CSV 생성
```bash
python scripts/generate_paper_results.py --outputs outputs --output outputs/experimental_results_summary.md
```

이 명령어는:
1. **`outputs/run_summary.csv`** 생성: 모든 실행의 요약 통계
   - 각 실행별로 다음 지표 저장:
     - `scenario`, `seed`, `gt_label`, `predicted_label`, `correctness`
     - `collided`, `near_miss`
     - `first_time_N_feasible_zero`: N_feasible이 처음 0이 된 스텝
     - `duration_N_feasible_zero`: N_feasible == 0 지속 시간 (초)
     - `dominant_constraint`: 우세 제약 조건
     - `delta_accel`, `delta_yaw`: 최소 완화량
     - `safer_feasible_alternative_exists`: 더 안전한 대안 존재 여부
     - `decision_margin`: 결정 마진
     - `safer_alternative_rank`: 더 안전한 대안의 Top-K 순위

2. **`outputs/experimental_results_summary.md`** 생성: 논문용 요약 문서
   - 시나리오별 정량적 결과
   - RQ1, RQ2, RQ3 평가 요약

---

## 5. 사후 분석을 위한 데이터 활용

### 5.1 RQ1 검증: 불가피성 판별 (S1)
다음 데이터를 확인:
- `ticks.jsonl`에서 `n_feasible == 0` 시작 시점 및 지속 시간
- `events.jsonl`에서 `dominant_constraint` 및 `minimal_relaxation`
- **기대 결과**: 모든 S1 실행에서 `n_feasible == 0`이 사고 전 >= 0.3초 지속, `dominant_constraint`는 주로 `max_acceleration`

### 5.2 RQ2 검증: 회피 가능성 검출 (S2)
다음 데이터를 확인:
- `events.jsonl`에서 `n_feasible > 0` 및 `safer_feasible_alternative_exists`
- **기대 결과**: 모든 S2 실행에서 트리거 시점에 `n_feasible > 0`, `safer_feasible_alternative_exists == true`

### 5.3 RQ3 검증: 결정 근거 설명 (S2)
다음 데이터를 확인:
- `events.jsonl`에서 `top_k` 후보들의 `cost_total` 및 `cost_terms`
- `selected_vs_safer_alternative`의 `decision_margin` 및 `safer_alternative_rank`
- `ticks.jsonl`에서 선택된 행동의 `cost_terms` (goal_distance 항이 우세)
- **기대 결과**: 선택된 행동의 `goal_distance` 비용 항이 우세, `decision_margin`이 작아서 거의 선택될 뻔했음을 보여줌

---

## 6. 결과 검증 체크리스트

### S1 검증
- [ ] 모든 실행에서 `n_feasible == 0`이 사고 전 >= 0.3초 지속
- [ ] `dominant_constraint`가 `max_acceleration` 또는 `max_yaw_rate`
- [ ] `minimal_relaxation.delta_accel`이 의미 있게 큼 (> 10.0)
- [ ] `events.jsonl`에 트리거 이벤트 존재

### S2 검증
- [ ] 트리거 시점에 `n_feasible > 0`
- [ ] `safer_feasible_alternative_exists == true`
- [ ] `top_k`에서 feasible 후보가 존재
- [ ] 선택된 행동의 `cost_terms.goal_distance`가 우세
- [ ] `decision_margin`이 작음 (< 2.0)

---

## 7. 자동화 스크립트 (선택적)

모든 시드를 자동으로 실행하려면:

```bash
# S1 5회 실행
for seed in 0 1 2 3 4; do
    echo "Running S1 seed $seed..."
    python scripts/run_scenario.py --scenario s1 --seed $seed --out outputs/ --gui
done

# S2 5회 실행
for seed in 0 1 2 3 4; do
    echo "Running S2 seed $seed..."
    python scripts/run_scenario.py --scenario s2 --seed $seed --out outputs/ --gui
done

# 결과 요약 생성
python scripts/generate_paper_results.py --outputs outputs --output outputs/experimental_results_summary.md
```

---

## 8. 저장 경로 구조

```
outputs/
├── s1/
│   ├── seed_0/
│   │   ├── run_s1_seed0_20260118_183758/
│   │   │   ├── ticks.jsonl
│   │   │   ├── events.jsonl
│   │   │   ├── report.json
│   │   │   └── frames/
│   │   ├── run_s1_seed0_20260118_190040/
│   │   └── ...
│   ├── seed_1/
│   └── ...
├── s2/
│   ├── seed_0/
│   ├── seed_1/
│   └── ...
├── run_summary.csv
└── experimental_results_summary.md
```

---

이 구조로 실험을 수행하면 논문에서 요구하는 모든 데이터가 저장되고, 사후 분석이 가능해집니다.
