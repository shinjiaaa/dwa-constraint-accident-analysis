# 검증 체크리스트 (Validation Checklist)

이 문서는 사고 사후 분석 시스템의 정확성을 검증하기 위한 체크리스트입니다.

## 1. 제약 기반 XAI가 필연성을 올바르게 감지하는지 검증

### 1.1 Scenario A (Inevitable Collision) 검증

**목표**: Feasible set이 비어있을 때 필연성 감지

**검증 항목**:

- [ ] `N_feasible == 0` 감지
  ```bash
  # events.jsonl에서 'no_feasible_actions' 이벤트 확인
  grep '"event_type": "no_feasible_actions"' logs/events_*.jsonl
  ```

- [ ] Minimal relaxation 계산
  ```bash
  # minimal_relaxation 필드가 있는지 확인
  grep '"minimal_relaxation"' logs/events_*.jsonl
  ```
  
- [ ] 각 제약별 violation_ratio 확인
  - `min_obstacle_distance` violation_ratio ≈ 1.0 (모든 후보 위반)
  - 다른 제약도 높은 violation_ratio

**기대 결과**:
- 모든 timestep에서 `n_feasible = 0`
- `minimal_relaxation`에 각 제약의 완화 필요량 기록
- 예: `{"min_obstacle_distance": 0.3, "max_velocity": 0.5}`

### 1.2 Scenario B (Wrong Decision) 검증

**목표**: Feasible set이 비어있지 않지만 잘못된 결정

**검증 항목**:

- [ ] `N_feasible > 0` 감지
  ```bash
  # 대부분의 timestep에서 feasible candidates 존재
  grep '"feasible_ratio"' logs/ticks_*.jsonl | head -20
  ```

- [ ] 선택된 액션이 제약 위반인지 확인
  ```bash
  # constraint_violation 이벤트 확인
  grep '"event_type": "constraint_violation"' logs/events_*.jsonl
  ```

- [ ] 대안(feasible alternatives) 제시 확인
  - `explanation['best_feasible_alternatives']` 필드 존재
  - 대안의 cost가 선택된 액션보다 높을 수 있음

**기대 결과**:
- 일부 timestep에서 `n_feasible > 0`이지만 선택된 액션이 infeasible
- Decision explanation에 "wrong decision" 원인 명시
- Cost breakdown에서 goal_distance가 과도하게 높은 가중치

### 1.3 Scenario C (Delayed Response) 검증

**목표**: 시간 지연 패턴 감지

**검증 항목**:

- [ ] Step 30 이전: feasible actions 존재
- [ ] Step 30 이후: feasible actions 감소 또는 제약 위반 증가
  ```bash
  # Step 30 근처의 feasible_ratio 변화 확인
  awk '/"timestep": 2[89]/,/"timestep": 32/' logs/ticks_*.jsonl | grep feasible_ratio
  ```

- [ ] Temporal pattern 분석
  - 초기: 높은 velocity, feasible actions 존재
  - 후기: 제약 위반 증가, 충돌 가능성 증가

**기대 결과**:
- Step 30 이전: `feasible_ratio` 상대적으로 높음
- Step 30 이후: `feasible_ratio` 급격히 감소
- `near_collision` 이벤트 발생

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
- Scenario A: "Selected action violates constraints. This indicates an inevitable collision..."
- Scenario B: "Selected from very small feasible set..." 또는 "Selected based on lowest cost..."
- Scenario C: 일반적인 선택 이유, 후반부에 필연성 언급

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
- Scenario B에서 feasible 후보가 ranking에 있지만 선택되지 않음

### 2.3 Score Breakdown 검증

**검증 항목**:

- [ ] `explanation['score_breakdown']` 존재
- [ ] 각 cost component 값이 합리적인지
  ```bash
  # Score breakdown 확인
  jq '.explanation.score_breakdown' logs/ticks_*.jsonl | head -50
  ```

**기대 결과**:
- Scenario B에서 `goal_distance`가 매우 높은 가중치
- Scenario C에서 초기 `velocity_magnitude`가 높음
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

- [ ] 원인 분석 가능 여부
  - Scenario A: Minimal relaxation로 필연성 증명
  - Scenario B: Cost breakdown으로 잘못된 가중치 식별
  - Scenario C: Temporal pattern으로 지연 감지

## 4. 재현성 검증

### 4.1 시드 고정 검증

**검증 항목**:

- [ ] 동일한 시드로 실행 시 동일한 결과
  ```bash
  # 두 번 실행하고 결과 비교
  python scripts/run_scenario.py --scenario A > run1.log
  python scripts/run_scenario.py --scenario A > run2.log
  diff logs/ticks_*_run1.jsonl logs/ticks_*_run2.jsonl
  ```

**기대 결과**:
- DWA candidate generation이 동일
- State trajectory가 동일 (약간의 수치 오차 허용)

### 4.2 시나리오 재현성

**검증 항목**:

- [ ] 각 시나리오가 명확한 ground truth cause를 가지고 있는지
- [ ] 시나리오 설정이 문서화되어 있는지
  ```bash
  # 시나리오 설정 확인
  python -c "from src.scenarios import ScenarioFactory; s = ScenarioFactory.get_scenario('A'); print(s.ground_truth_cause)"
  ```

## 5. 자동화된 검증 스크립트

검증을 자동화하는 스크립트를 실행:

```bash
python scripts/validate_results.py --scenario A --log-dir logs/
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
- `scenario_a_inevitable_collision()`의 `constraint_config` 조정
- 장애물 위치 조정

### 문제: Wrong decision이 감지되지 않음

**원인**:
- Cost weights가 충분히 극단적이지 않음
- Feasible path가 너무 명확함

**해결**:
- `scenario_b_wrong_decision()`의 `cost_weights` 조정
- 장애물 위치를 경로에 더 가깝게

### 문제: 로그 파일이 생성되지 않음

**원인**:
- `logs/` 디렉토리 권한 문제
- JSON serialization 오류

**해결**:
```bash
mkdir -p logs
chmod 755 logs
# Python 오류 확인
python scripts/run_scenario.py --scenario A 2>&1 | grep -i error
```
