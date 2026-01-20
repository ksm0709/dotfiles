# Deep Research Skill Specification

## ADDED Requirements

### Requirement: Iterative Deep Research
시스템은 재귀적으로 리서치를 수행하여 깊이 있는 탐색을 지원해야 합니다.
각 반복에서 이전 학습 내용을 반영하여 새로운 방향을 탐색합니다.

#### Scenario: 기본 재귀 탐색
- **GIVEN** 사용자가 "AI trends 2026" 주제로 리서치 요청
- **AND** depth=2, breadth=3으로 설정
- **WHEN** deep_research() 함수 실행
- **THEN** 최대 2회 재귀 탐색 수행
- **AND** 각 단계에서 3개 URL 수집
- **AND** 이전 learnings가 다음 단계에 반영됨

#### Scenario: 조기 종료 (답변 완성)
- **GIVEN** 첫 번째 탐색에서 충분한 정보 수집
- **WHEN** analyze_results()가 is_complete=True 반환
- **THEN** 남은 depth와 무관하게 탐색 종료
- **AND** 최종 리포트 생성

#### Scenario: 최대 깊이 도달
- **GIVEN** depth=3으로 설정
- **WHEN** 3회 탐색 후에도 is_complete=False
- **THEN** 수집된 정보로 최선의 리포트 생성
- **AND** "추가 탐색 필요" 표시

### Requirement: Learnings-based Direction
시스템은 각 탐색 단계에서 학습 내용과 새로운 방향을 추출해야 합니다.

#### Scenario: Learnings 추출
- **GIVEN** 검색 결과에서 "AI 규제가 2026년부터 강화" 정보 발견
- **WHEN** analyze_results() 호출
- **THEN** learnings 리스트에 해당 정보 추가
- **AND** 다음 create_plan() 호출 시 해당 정보 컨텍스트로 전달

#### Scenario: Directions 추출
- **GIVEN** 검색 결과에서 "구체적인 규제 내용 불명확" 발견
- **WHEN** analyze_results() 호출
- **THEN** directions 리스트에 "AI 규제 세부 내용" 추가
- **AND** 다음 탐색 시 해당 방향 우선 탐색

#### Scenario: 중복 학습 방지
- **GIVEN** 이미 learnings에 동일 정보 존재
- **WHEN** 새 결과에서 같은 정보 발견
- **THEN** 중복 추가하지 않음
- **AND** learnings 최대 20개 유지

### Requirement: Breadth × Depth Parameters
시스템은 탐색의 폭(breadth)과 깊이(depth)를 독립적으로 설정할 수 있어야 합니다.

#### Scenario: 명시적 breadth, depth 설정
- **GIVEN** CLI에서 --breadth 5 --depth 3 지정
- **WHEN** 리서치 실행
- **THEN** 각 단계에서 5개 URL 수집
- **AND** 최대 3회 재귀 탐색

#### Scenario: 하위 호환성 (depth만 지정)
- **GIVEN** CLI에서 --depth 10만 지정 (기존 방식)
- **WHEN** 리서치 실행
- **THEN** 단일 패스로 10개 URL 수집
- **AND** 재귀 탐색 없이 즉시 리포트 생성

#### Scenario: 기본값 동작
- **GIVEN** 파라미터 없이 실행
- **WHEN** 리서치 실행
- **THEN** breadth=5, depth=2로 동작
- **AND** 최대 2회 재귀 탐색

### Requirement: Research State Management
시스템은 리서치 상태를 관리하여 중복 방지 및 컨텍스트 유지를 해야 합니다.

#### Scenario: 방문 URL 중복 방지
- **GIVEN** 이전 단계에서 "example.com/article1" 방문
- **WHEN** 다음 단계 검색 결과에 동일 URL 포함
- **THEN** 해당 URL 스크래핑 건너뜀
- **AND** 새로운 URL만 처리

#### Scenario: 진행 상황 출력
- **GIVEN** depth=3으로 설정
- **WHEN** 각 재귀 단계 시작
- **THEN** "[Depth 1/3] Exploring: {direction}" 형식 출력
- **AND** 현재 learnings 개수 표시
