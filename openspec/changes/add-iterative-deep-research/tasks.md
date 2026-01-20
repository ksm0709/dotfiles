# Tasks: Add Iterative Deep Research

## 1. 테스트 코드 작성 (TDD)
- [x] 1.1 ResearchState 데이터 클래스 테스트
- [x] 1.2 analyze_results() 함수 테스트 (Learnings/Directions 추출)
- [x] 1.3 is_answer_complete() 함수 테스트
- [x] 1.4 deep_research() 재귀 로직 테스트
- [x] 1.5 CLI 파라미터 (--breadth, --depth) 파싱 테스트

## 2. 핵심 기능 구현
- [x] 2.1 ResearchState, AnalysisResult 데이터 클래스 추가
- [x] 2.2 analyze_results() 함수 구현 (LLM 호출)
- [x] 2.3 select_best_direction() 함수 구현
- [x] 2.4 deep_research() 메인 재귀 함수 구현
- [x] 2.5 기존 create_plan()에 learnings 컨텍스트 추가

## 3. CLI 및 통합
- [x] 3.1 argparse에 --breadth 파라미터 추가
- [x] 3.2 --depth 의미 변경 (URL 수 → 재귀 깊이)
- [x] 3.3 하위 호환성 로직 구현
- [x] 3.4 run_research.sh 업데이트 (기존 동작 유지)

## 4. 문서화 및 검증
- [x] 4.1 SKILL.md 워크플로우 다이어그램 업데이트
- [x] 4.2 테스트 코드 통과 확인 (20 tests)
- [x] 4.3 기존 테스트 통과 확인 (18 tests)

## Implementation Notes

### 완료된 구현
- `ResearchState`: 재귀 탐색 상태 관리 (topic, session_id, depth, breadth, learnings, directions, visited_urls, all_results)
- `AnalysisResult`: 결과 분석 구조 (learnings, directions, is_complete, confidence)
- `analyze_results()`: LLM 호출로 결과 분석 및 학습/방향 추출
- `select_best_direction()`: 다음 탐색 방향 선택 (Phase 1: 첫 번째 선택)
- `deep_research()`: 재귀적 깊이 탐색 메인 루프
- `limit_learnings()`: 학습 누적 및 최대 20개 제한
- `create_argument_parser()`: CLI 파서 (--breadth, --depth)

### 하위 호환성
- `--depth`만 지정 시: Classic 모드 (단일 패스, depth가 URL 수로 해석)
- `--breadth` 지정 시: Iterative 모드 (재귀 탐색)

### 상수
- MAX_DEPTH = 5 (최대 재귀 깊이)
- DEFAULT_DEPTH = 2 (기본 재귀 깊이)
- DEFAULT_BREADTH = 5 (기본 URL 수)
- CONFIDENCE_THRESHOLD = 0.7 (조기 종료 기준)
- MAX_LEARNINGS = 20 (최대 학습 개수)
