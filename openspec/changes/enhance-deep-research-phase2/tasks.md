# Tasks: Deep Research Phase 2 Enhancements

## 1. 병렬 스크래핑 (Parallel Scraping)

### 1.1 테스트 코드 (TDD)
- [x] AsyncWebScraper 초기화 테스트
- [x] fetch_all() 병렬 실행 테스트
- [x] Rate limiting (semaphore) 동작 테스트
- [x] 에러 처리 (일부 URL 실패) 테스트
- [x] 동기 폴백 테스트

### 1.2 구현
- [x] aiohttp 기반 AsyncWebScraper 클래스
- [x] Semaphore 기반 동시성 제어
- [x] 기존 WebScraper와 인터페이스 호환
- [x] DeepResearch.execute_plan() 비동기 지원 (통합 예정)
- [x] CLI `--parallel` 플래그 추가

### 1.3 통합
- [x] requirements.txt에 aiohttp 추가
- [ ] run.py에서 조건부 비동기 실행 (다음 단계)
- [ ] 기존 테스트 통과 확인

## 2. 토큰 예산 관리 (Token Budget)

### 2.1 테스트 코드 (TDD)
- [x] TokenBudget 초기화 테스트
- [x] can_continue() 로직 테스트
- [x] add_usage() 누적 테스트
- [x] 예산 초과 시 조기 종료 테스트

### 2.2 구현
- [x] TokenBudget 데이터 클래스
- [ ] llm_client.py에 토큰 사용량 반환 (다음 단계)
- [x] ResearchState에 token_budget 필드 추가
- [ ] deep_research()에서 예산 체크 (다음 단계)

### 2.3 CLI
- [x] `--token-budget` 파라미터 추가
- [ ] 사용량 리포트 출력 (다음 단계)

## 3. 진행률 표시 (Progress UI)

### 3.1 테스트 코드 (TDD)
- [ ] ResearchProgress 초기화 테스트
- [ ] update_depth() 호출 테스트
- [ ] update_urls() 호출 테스트

### 3.2 구현
- [ ] Rich 라이브러리 기반 Progress 클래스
- [ ] Live 업데이트 통합
- [ ] 선택적 import (Rich 없으면 fallback)

### 3.3 통합
- [ ] requirements.txt에 rich 추가 (optional)
- [ ] 기존 print 문 대체

## 4. 중간 결과 저장 (Checkpoint)

### 4.1 테스트 코드 (TDD)
- [ ] CheckpointManager save/load 테스트
- [ ] 재개 후 상태 복원 테스트
- [ ] 체크포인트 없을 때 동작 테스트

### 4.2 구현
- [ ] CheckpointManager 클래스
- [ ] deep_research()에서 각 depth 완료 시 저장
- [ ] CLI `--resume` 플래그 추가

### 4.3 정리
- [ ] 완료된 세션 체크포인트 자동 삭제
- [ ] 오래된 체크포인트 정리 로직

## 5. Source 신뢰도 평가

### 5.1 테스트 코드 (TDD)
- [x] SourceEvaluator.evaluate() 테스트
- [x] 높은 신뢰도 도메인 테스트
- [x] 낮은 신뢰도 패턴 테스트

### 5.2 구현
- [x] SourceEvaluator 클래스
- [ ] execute_plan()에서 신뢰도 필터링 (다음 단계)
- [x] CLI `--min-trust` 파라미터

### 5.3 통합
- [ ] 리포트에 소스 신뢰도 표시

## 6. 문서화

- [x] SKILL.md 업데이트 (새 CLI 옵션)
- [ ] 워크플로우 다이어그램 업데이트
- [x] 예제 명령어 추가

## 우선순위

1. **병렬 스크래핑** - 가장 큰 성능 향상
2. **토큰 예산 관리** - 비용 제어
3. **진행률 표시** - UX 개선
4. **중간 결과 저장** - 안정성
5. **Source 신뢰도** - 품질 향상
