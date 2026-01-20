# 검증 체크리스트 (Validation Checklist)

이 문서는 Replay Contract 기반 사후 판별 시스템의 정확성을 검증하기 위한 체크리스트입니다.

목적: 사고 원인 규명이 아니라 **조사 범위를 줄이는 트리아지** 수행 여부 검증

## 1. 불가피성 판별 검증 (S1)

### 1.1 S1 (Inevitable Collision) 검증

**목표**: Feasible set이 비어있을 때 불가피성(inevitability) 판별

**검증 항목**:

- [ ] `N_feasible == 0` 감지
  ```bash
  # events.jsonl에서 트리거 이벤트 확인
  jq '.metrics.n_feasible' outputs/s1/seed_0/run_*/events.jsonl
  ```

- [ ] Minimal relaxation 계산
  ```bash
  # minimal_relaxation 필드가 있는지 확인
  jq '.minimal_relaxation' outputs/s1/seed_0/run_*/events.jsonl
  ```
  
- [ ] 지배 제약(dominant constraint) 확인
  ```bash
  # dominant_constraint 필드 확인
  jq '.constraint_violation_summary.dominant_constraint' outputs/s1/seed_0/run_*/events.jsonl
  ```

- [ ] 각 제약별 violation_ratio 확인
  - 주로 `max_acceleration` 또는 `max_yaw_rate`가 지배 제약
  - 해당 제약의 violation_ratio가 높음

**기대 결과**:
- 충돌 전 >= 0.3초 동안 `n_feasible == 0` 지속
- `minimal_relaxation.delta_accel`이 의미 있게 큼 (> 10.0)
- 지배 제약이 명확히 식별됨

## 2. 선택오류 판별 검증 (S2)

### 2.1 S2 (Wrong Decision) 검증

**목표**: Feasible set이 존재하지만 선택오류(choice error) 판별

**검증 항목**:

- [ ] `N_feasible > 0` 감지
  ```bash
  # 트리거 시점에 feasible candidates 존재
  jq '.metrics.n_feasible' outputs/s2/seed_0/run_*/events.jsonl
  ```

- [ ] 더 안전한 feasible 대안 존재 확인
  ```bash
  # safer_feasible_alternative_exists 필드 확인
  jq '.selected_vs_safer_alternative.safer_feasible_alternative_exists' outputs/s2/seed_0/run_*/events.jsonl
  ```

- [ ] Top-K 후보 비교 확인
  ```bash
  # Top-K에 feasible 후보가 포함되어 있는지 확인
  jq '.top_k[] | select(.is_feasible == true)' outputs/s2/seed_0/run_*/events.jsonl
  ```

- [ ] 비용 항 분해 확인
  ```bash
  # 선택된 행동의 cost_terms 확인
  jq '.top_k[0].cost_terms' outputs/s2/seed_0/run_*/events.jsonl
  ```

- [ ] 결정 마진 확인
  ```bash
  # decision_margin 확인 (작아야 함)
  jq '.selected_vs_safer_alternative.decision_margin' outputs/s2/seed_0/run_*/events.jsonl
  ```

**기대 결과**:
- 트리거 시점에 `n_feasible > 0`
- `safer_feasible_alternative_exists == true`
- Top-K에 feasible 후보 존재
- 선택된 행동의 `goal_distance` 항이 우세 (비용 구조 문제)
- 결정 마진이 작음 (< 2.0)

## 2. 의사결정 수준 XAI가 선택을 올바르게 설명하는지 검증

### 2.1 선택 이유 (Selection Reason) 검증

**검증 항목**:

- [ ] `explanation['selection_reason']` 필드 존재
- [ ] 이유가 명확하고 해석 가능한지 확인
  ```bash
  # 모든 tick에서 selection_reason 확인
  jq -r '.explanation.selection_reason' logs/ticks_*.jsonl | head -10
  ```

**기대 결과**:
- S1: Feasible set collapse로 인한 필연성 명시
- S2: 비용 구조로 인한 선택오류 명시

### 2.2 Cost Ranking 검증

**검증 항목**:

- [ ] `explanation['cost_ranking']` 존재
- [ ] Ranking이 cost 순서로 정렬되어 있는지
  ```bash
  # Cost ranking 확인
  jq '.explanation.cost_ranking' logs/ticks_*.jsonl | head -50
  ```

- [ ] Top-2 margin 계산 확인
  ```bash
  jq -r '.explanation.top_2_margin' logs/ticks_*.jsonl | head -20
  ```

**기대 결과**:
- 모든 ranking이 cost 오름차순
- Margin이 작을수록 ambiguous decision
- S2에서 feasible 후보가 ranking에 있지만 선택되지 않음

### 2.3 Score Breakdown 검증

**검증 항목**:

- [ ] `explanation['score_breakdown']` 존재
- [ ] 각 cost component 값이 합리적인지
  ```bash
  # Score breakdown 확인
  jq '.explanation.score_breakdown' logs/ticks_*.jsonl | head -50
  ```

**기대 결과**:
- S2에서 `goal_distance`가 매우 높은 가중치 (선택오류의 원인)
- S1에서 제약 위반으로 인한 필연성
- Obstacle proximity가 가까울수록 높은 penalty

## 3. 로그가 사고 사후 분석에 충분한지 검증

### 3.1 Ticks Logging 검증

**검증 항목**:

- [ ] 모든 timestep이 로깅되는지
  ```bash
  # Timestep 연속성 확인
  jq -r '.timestep' logs/ticks_*.jsonl | awk '{if(NR>1 && $1 != prev+1) print "Gap:", prev, "->", $1; prev=$1}'
  ```

- [ ] 필수 필드 존재 확인
  ```bash
  # 필수 필드 체크
  jq -r '[.timestep, .state.position, .selected_action, .constraint_audit.n_feasible] | @csv' logs/ticks_*.jsonl | head -5
  ```

**필수 필드**:
- `timestep`: 시뮬레이션 스텝
- `state.position`: 드론 위치
- `state.velocity`: 드론 속도
- `selected_action`: 선택된 액션
- `constraint_audit.n_total`: 총 후보 수
- `constraint_audit.n_feasible`: 가능한 후보 수
- `constraint_audit.feasible_ratio`: 가능 비율
- `explanation.selection_reason`: 선택 이유

### 3.2 Events Logging 검증

**검증 항목**:

- [ ] Critical events만 로깅되는지
  ```bash
  # 이벤트 타입 확인
  jq -r '.event_type' logs/events_*.jsonl | sort | uniq -c
  ```

- [ ] 이벤트별 필수 정보 확인
  ```bash
  # 각 이벤트 타입별 필드 확인
  jq '.event_type, .additional_info' logs/events_*.jsonl
  ```

**이벤트 타입별 필수 정보**:
- `no_feasible_actions`: `minimal_relaxation` 필드
- `near_collision`: `distance_to_nearest_obstacle`
- `constraint_violation`: `violations` 리스트

### 3.3 Post-Hoc Analysis 가능성 검증

**검증 항목**:

- [ ] JSONL 파일 파싱 가능 여부
  ```bash
  # Python 스크립트로 파싱 테스트
  python -c "
  import json
  with open('logs/ticks_*.jsonl') as f:
      for i, line in enumerate(f):
          data = json.loads(line)
          assert 'timestep' in data
          if i > 10: break
  print('✓ Parsing successful')
  "
  ```

- [ ] 시계열 분석 가능 여부
  - Timestep 순서대로 정렬 가능
  - State 변화 추적 가능
  - Constraint violation 패턴 추적 가능

- [ ] 판별 가능 여부
  - S1: Minimal relaxation로 불가피성 판별
  - S2: Cost breakdown 및 Top-K 비교로 선택오류 판별

## 4. 재현성 검증

### 4.1 시드 고정 검증

**검증 항목**:

- [ ] 동일한 시드로 실행 시 동일한 결과
  ```bash
  # 두 번 실행하고 결과 비교
  python scripts/run_scenario.py --scenario s1 --seed 0 --out outputs/ > run1.log
  python scripts/run_scenario.py --scenario s1 --seed 0 --out outputs/ > run2.log
  diff outputs/s1/seed_0/run_*/ticks.jsonl
  ```

**기대 결과**:
- DWA candidate generation이 동일
- State trajectory가 동일 (약간의 수치 오차 허용)

### 4.2 시나리오 재현성

**검증 항목**:

- [ ] 각 시나리오가 명확한 판별 목적을 가지고 있는지
- [ ] 시나리오 설정이 문서화되어 있는지
  ```bash
  # 시나리오 설정 확인
  python -c "from scenarios import get_scenario; s = get_scenario('s1'); print(s.gt_label)"
  ```

## 5. 자동화된 검증 스크립트

검증을 자동화하는 스크립트를 실행:

```bash
python scripts/validate_results.py --scenario s1 --outputs outputs/
```

**검증 스크립트가 체크하는 항목**:
1. JSONL 파일 형식 유효성
2. 필수 필드 존재 여부
3. Scenario별 기대 결과 일치 여부
4. 통계적 일관성 (예: feasible_ratio 범위)
5. 이벤트 로깅 적절성

## 6. 문제 해결 가이드

### 문제: No feasible actions가 감지되지 않음

**원인**:
- 제약 조건이 너무 느슨함
- 장애물 배치가 충돌을 필연적으로 만들지 않음

**해결**:
- `scenarios/s1_inevitable_dynamics.py`의 `constraint_config` 조정
- 장애물 위치 조정

### 문제: 선택오류가 감지되지 않음

**원인**:
- Cost weights가 충분히 극단적이지 않음
- Feasible path가 너무 명확함

**해결**:
- `scenarios/s2_bad_decision.py`의 `cost_weights` 조정
- 장애물 위치를 경로에 더 가깝게

### 문제: 로그 파일이 생성되지 않음

**원인**:
- `logs/` 디렉토리 권한 문제
- JSON serialization 오류

**해결**:
```bash
mkdir -p outputs
# Python 오류 확인
python scripts/run_scenario.py --scenario s1 --seed 0 --out outputs/ 2>&1 | grep -i error
```
