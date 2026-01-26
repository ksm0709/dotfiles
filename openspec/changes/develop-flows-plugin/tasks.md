## 1. Core Infrastructure

- [x] 1.1 FlowManager 싱글톤 구현
  - [x] getInstance() 패턴
  - [x] initialize() 메서드
  - [x] Tick 루프 (500ms 간격)
  - [x] 인스턴스 맵 관리

- [x] 1.2 FlowInstance FSM 구현
  - [x] 상태 정의 (initializing, running, paused, completed, failed)
  - [x] tick() 메서드
  - [x] 상태 전이 로직
  - [x] 세션 관리 (생성/정리)

- [x] 1.3 Blackboard 구현 (개선된 버전)
  - [x] Key-Value 저장소
  - [x] 파일 기반 영속화
  - [x] 변수 해석 (`${key}` 문법)
  - [x] 노드별 결과 저장 (`_results` 구조)
  - [x] 실행 순서 추적 (`_execution_order`)

- [x] 1.4 개선된 스키마 파서
  - [x] 단축 문법 해석 (run, agent, read 등)
  - [x] routes → on 변환
  - [x] results 배열 → 객체 변환
  - [x] Zod 스키마 업데이트

## 2. Basic Nodes

- [x] 2.1 BaseNode 추상 클래스
  - [x] execute() 추상 메서드
  - [x] 결과 헬퍼 (success, failed, running, result)
  - [x] 변수 해석 헬퍼
  - [x] 타이머 유틸리티

- [x] 2.2 AgentNode 구현
  - [x] 프롬프트 변수 치환
  - [x] 히스토리 컨텍스트 빌드 (전체 노드 결과 참조)
  - [x] 결과 가이드 주입
  - [x] 응답 파싱 (`[RESULT:name]`)
  - [x] 비동기 실행 (running 상태 지원)

- [x] 2.3 CommandNode 구현
  - [x] 명령어 실행
  - [x] expect_exit_code 처리
  - [x] 타임아웃 처리
  - [x] 비동기 실행

- [x] 2.4 ToolNode 구현
  - [x] read 도구
  - [x] write 도구
  - [x] glob 도구
  - [x] bash 도구

- [x] 2.5 EndNode 구현
  - [x] 종료 상태 결정 (success/failed)
  - [x] 최종 메시지 저장

## 3. Control Flow Nodes

- [x] 3.1 ConditionalNode 구현
  - [x] 조건 평가 (eq, ne, gt, lt, contains, exists)
  - [x] 다중 조건 순차 평가
  - [x] 기본 분기

- [x] 3.2 DelayNode 구현
  - [x] 시간 대기
  - [x] Tick 기반 완료 체크

- [x] 3.3 LoopNode 구현
  - [x] 반복 카운터
  - [x] max_iterations 체크
  - [x] while_condition 평가

## 4. Plugin Integration

- [x] 4.1 NodeFactory 구현
  - [x] 노드 타입별 생성
  - [x] 단축 문법 노드 타입 추론

- [x] 4.2 /flow 커맨드 구현
  - [x] start 액션
  - [x] stop 액션
  - [x] status 액션
  - [x] list 액션

- [x] 4.3 플러그인 엔트리포인트
  - [x] FlowManager 초기화
  - [x] 커맨드 등록
  - [x] 이벤트 훅 등록

- [ ] 4.4 에러 핸들링
  - [ ] 재시도 로직
  - [ ] 타임아웃 처리
  - [ ] 글로벌 에러 핸들러

## 5. Testing & Documentation

- [ ] 5.1 단위 테스트
  - [x] Blackboard 테스트
  - [ ] 각 노드 타입 테스트
  - [ ] FlowInstance 테스트

- [ ] 5.2 통합 테스트
  - [ ] 예제 플로우 실행 테스트
  - [ ] 에러 시나리오 테스트
  - [ ] crash recovery 테스트

- [x] 5.3 예제 플로우 작성 (개선된 스키마)
  - [x] code-review.json
  - [x] tdd-cycle.json
  - [x] simple-task.json (간단한 예제)

- [ ] 5.4 문서화
  - [ ] README.md 업데이트
  - [ ] 스키마 레퍼런스
  - [ ] 사용 예시

## 6. Code Refactoring

- [ ] 6.1 기존 코드 개선된 스키마로 업데이트
  - [x] schemas.ts 업데이트
  - [x] Blackboard.ts 히스토리 관리 변경
  - [x] 예제 JSON 파일 업데이트
