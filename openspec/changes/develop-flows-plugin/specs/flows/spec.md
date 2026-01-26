## ADDED Requirements

### Requirement: Flow Definition Schema
시스템은 JSON 형식의 플로우 정의 파일을 통해 워크플로우를 선언적으로 구성할 수 있어야 한다.

플로우 정의 파일은 다음 구조를 가져야 한다:
- `name`: 플로우 식별자 (kebab-case, 파일명과 일치)
- `version`: 시맨틱 버저닝
- `description`: 플로우 설명 (선택)
- `config`: 글로벌 설정 (timeout, max_retries 등)
- `initial_state`: 시작 노드 이름
- `nodes`: 노드 정의 맵

#### Scenario: 유효한 플로우 정의 로드
- **GIVEN** 유효한 JSON 형식의 플로우 정의 파일이 존재할 때
- **WHEN** `/flow start <flow-name>` 커맨드가 실행되면
- **THEN** 플로우 정의가 파싱되고 유효성이 검증되어야 한다

#### Scenario: 잘못된 스키마 거부
- **GIVEN** 필수 필드가 누락된 플로우 정의 파일이 존재할 때
- **WHEN** 플로우 로드를 시도하면
- **THEN** 명확한 에러 메시지와 함께 로드가 거부되어야 한다

---

### Requirement: Node Definition
각 노드는 type, config, routes를 포함하는 정의를 가져야 한다.

노드 정의 구조:
- `type`: 노드 타입 (agent, tool, command, skill, conditional, parallel, delay, loop, retry, sub-flow, end)
- `config`: 타입별 설정
- `routes`: 결과별 다음 노드 매핑 (결과명 → 노드명 또는 null)

#### Scenario: 노드 타입별 설정 검증
- **GIVEN** 특정 타입의 노드가 정의될 때
- **WHEN** 해당 타입에 필요한 필수 설정이 누락되면
- **THEN** 검증 오류가 발생해야 한다

#### Scenario: 라우트 정의
- **GIVEN** 노드가 실행을 완료했을 때
- **WHEN** 결과에 해당하는 라우트가 정의되어 있으면
- **THEN** 해당 라우트의 다음 노드로 전이해야 한다

---

### Requirement: Agent Node with Arbitrary Results
에이전트 노드는 success/failed 외에 사용자 정의 결과(arbitrary results)를 지원해야 한다.

에이전트 노드 설정:
- `prompt`: 에이전트에게 전달할 프롬프트 (변수 치환 지원)
- `agent_type`: 사용할 에이전트 타입 (general, senior-sw-engineer 등)
- `results`: 가능한 결과 목록 (name, description 쌍)

에이전트에게 결과 선택 가이드가 프롬프트에 주입되어야 한다.
에이전트 응답에서 `[RESULT:name]` 패턴을 파싱하여 결과를 결정한다.

#### Scenario: 사전 정의된 결과 선택
- **GIVEN** results에 `approved`, `rejected`가 정의된 에이전트 노드가 있을 때
- **WHEN** 에이전트가 `[RESULT:approved]`를 출력하면
- **THEN** 노드 결과는 `approved`가 되어야 한다

#### Scenario: 결과 가이드 주입
- **GIVEN** results가 정의된 에이전트 노드가 실행될 때
- **WHEN** 프롬프트가 에이전트에게 전달되면
- **THEN** 결과 선택 가이드가 프롬프트 끝에 추가되어야 한다

---

### Requirement: Blackboard State Management
각 플로우 인스턴스는 독립적인 Blackboard를 가지며, 노드 간 상태 공유 및 영속화를 담당한다.

Blackboard 기능:
- Key-Value 저장소로 동작
- 파일 기반 영속화 (JSON 형식)
- 변수 참조 문법 `${key}` 지원
- 노드 히스토리 관리 (모든 노드의 결과 저장)

**히스토리 관리 정책**:
- 각 노드의 결과는 노드 이름을 키로 하여 저장된다
- 노드 이름은 플로우 내에서 유일하므로 충돌이 발생하지 않는다
- 같은 노드가 여러 번 실행되면 최신 결과로 덮어쓴다
- 모든 노드의 결과가 보존되어 컨텍스트 손실이 방지된다

#### Scenario: 변수 치환
- **GIVEN** Blackboard에 `user_name`이 "Alice"로 저장되어 있을 때
- **WHEN** 프롬프트에 `${user_name}`이 포함되어 있으면
- **THEN** "Alice"로 치환되어야 한다

#### Scenario: 히스토리 참조
- **GIVEN** `analyze` 노드가 실행을 완료했을 때
- **WHEN** 다음 노드의 프롬프트에 `${history.analyze.message}`가 있으면
- **THEN** analyze 노드의 결과 메시지로 치환되어야 한다

#### Scenario: 노드별 결과 저장
- **GIVEN** 플로우에 `step1`, `step2`, `step3` 노드가 순차 실행될 때
- **WHEN** 모든 노드가 완료되면
- **THEN** 각 노드의 결과가 `_results.step1`, `_results.step2`, `_results.step3`에 저장되어야 한다

#### Scenario: 영속화 및 복구
- **GIVEN** 플로우 인스턴스가 실행 중일 때
- **WHEN** 프로세스가 비정상 종료되면
- **THEN** 재시작 시 저장된 상태에서 복구할 수 있어야 한다

---

### Requirement: Flow Manager Singleton
FlowManager는 싱글톤으로 동작하며, 모든 플로우 인스턴스의 라이프사이클을 관리한다.

FlowManager 책임:
- 플로우 정의 로드 및 검증
- 인스턴스 생성 및 관리
- 500ms 간격의 Tick 루프 실행
- 다중 인스턴스 동시 실행 지원

#### Scenario: Tick 루프 관리
- **GIVEN** 실행 중인 플로우 인스턴스가 존재할 때
- **WHEN** 매 500ms마다
- **THEN** 각 인스턴스의 tick() 메서드가 호출되어야 한다

#### Scenario: 인스턴스 정리
- **GIVEN** 플로우 인스턴스가 completed 또는 failed 상태가 되었을 때
- **WHEN** 다음 tick에서
- **THEN** 해당 인스턴스는 관리 대상에서 제거되어야 한다

#### Scenario: 빈 루프 중지
- **GIVEN** 관리 중인 인스턴스가 0개가 되었을 때
- **WHEN** tick 루프가 체크하면
- **THEN** tick 루프가 중지되어야 한다

---

### Requirement: Flow Instance FSM
각 플로우 인스턴스는 독립적인 FSM으로 동작하며, 상태 전이를 관리한다.

FSM 상태:
- `initializing`: 초기화 중
- `running`: 실행 중
- `paused`: 일시 정지 (향후 지원)
- `completed`: 성공적 완료
- `failed`: 실패로 종료

#### Scenario: 상태 전이
- **GIVEN** 현재 노드가 실행을 완료하고 결과가 `success`일 때
- **WHEN** routes에 `success: next_node`가 정의되어 있으면
- **THEN** 현재 상태가 `next_node`로 전이해야 한다

#### Scenario: End 노드 도달
- **GIVEN** 현재 노드가 `type: end`일 때
- **WHEN** 노드가 실행을 완료하면
- **THEN** 인스턴스 상태가 `completed` 또는 `failed`로 변경되어야 한다

---

### Requirement: Flow Command Interface
`/flow` 커맨드를 통해 워크플로우를 제어할 수 있어야 한다.

지원 액션:
- `start <flow-name> [prompt]`: 플로우 시작
- `stop [instance-id]`: 플로우 중단 (ID 생략 시 모두 중단)
- `status [instance-id]`: 상태 조회
- `list`: 사용 가능한 플로우 목록

#### Scenario: 플로우 시작
- **GIVEN** `code-review.json` 플로우가 정의되어 있을 때
- **WHEN** `/flow start code-review "src/ 리뷰해주세요"` 명령이 실행되면
- **THEN** 새 인스턴스가 생성되고 ID가 반환되어야 한다

#### Scenario: 상태 조회
- **GIVEN** 실행 중인 플로우 인스턴스들이 있을 때
- **WHEN** `/flow status` 명령이 실행되면
- **THEN** 각 인스턴스의 ID, 이름, 현재 노드, 상태, 경과 시간이 표시되어야 한다

---

### Requirement: Error Handling and Retry
노드 실행 중 발생하는 오류는 재시도 정책에 따라 처리되어야 한다.

오류 처리 정책:
- 설정된 max_retries까지 재시도
- retry_delay 간격으로 대기 후 재시도
- 최대 재시도 초과 시 failed 결과 반환
- 타임아웃 발생 시 오류로 처리

#### Scenario: 재시도 성공
- **GIVEN** 노드 실행이 일시적 오류로 실패했을 때
- **WHEN** 재시도 횟수가 max_retries 미만이면
- **THEN** retry_delay 후 재시도해야 한다

#### Scenario: 최대 재시도 초과
- **GIVEN** 노드가 max_retries번 모두 실패했을 때
- **WHEN** 마지막 재시도도 실패하면
- **THEN** 노드 결과는 `failed`가 되어야 한다

---

### Requirement: Session Management
각 플로우 인스턴스는 단일 OpenCode 세션을 사용하여 컨텍스트를 유지해야 한다.

세션 관리 정책:
- 인스턴스 초기화 시 새 세션 생성
- 동일 인스턴스 내 모든 에이전트 노드는 같은 세션 사용
- 인스턴스 종료 시 세션 정리

#### Scenario: 세션 재사용
- **GIVEN** 플로우에 연속된 두 에이전트 노드가 있을 때
- **WHEN** 두 노드가 순차적으로 실행되면
- **THEN** 동일한 세션 ID를 사용해야 한다

---

### Requirement: Flow Definition File Location
플로우 정의 파일은 다음 위치에서 순차적으로 검색되어야 한다.

검색 순서 (우선순위 순):
1. 프로젝트 로컬: `.opencode/flows/<name>.json`
2. 글로벌 사용자: `~/.config/opencode/flows/<name>.json`
3. 글로벌 shared: `~/.config/opencode/shared/flows/<name>.json`

#### Scenario: 로컬 우선
- **GIVEN** 동일한 이름의 플로우가 로컬과 글로벌에 모두 존재할 때
- **WHEN** 플로우 로드 시
- **THEN** 로컬 파일이 사용되어야 한다

---

### Requirement: Control Flow Nodes
제어 흐름을 위한 특수 노드들을 지원해야 한다.

지원 노드:
- `conditional`: 조건에 따른 분기
- `loop`: 반복 실행
- `delay`: 시간 대기
- `end`: 플로우 종료

#### Scenario: 조건 분기
- **GIVEN** conditional 노드에 `field: status, operator: eq, value: approved` 조건이 있을 때
- **WHEN** blackboard의 status가 "approved"이면
- **THEN** 해당 조건의 result 결과가 반환되어야 한다

#### Scenario: 루프 실행
- **GIVEN** loop 노드에 max_iterations가 5로 설정되어 있을 때
- **WHEN** 5번째 반복이 완료되면
- **THEN** `max_reached` 결과가 반환되어야 한다
