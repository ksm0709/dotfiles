# Spec: Deep Research Phase 2 Enhancements

## ADDED

### REQ-2.1: 병렬 URL 스크래핑
asyncio 기반으로 여러 URL을 동시에 스크래핑하여 리서치 속도를 향상시킨다.

#### Scenario: 5개 URL 병렬 스크래핑
Given 5개의 URL이 주어졌을 때
When --parallel 플래그가 활성화되면
Then 모든 URL이 동시에 페칭되고
And 총 소요 시간이 순차 실행의 1/5 수준이어야 한다

#### Scenario: Rate limiting 준수
Given 동시 연결 제한이 5개이고 지연이 0.5초일 때
When 10개 URL을 병렬 스크래핑하면
Then 동시에 최대 5개만 페칭되고
And 각 요청 사이에 0.5초 지연이 있어야 한다

#### Scenario: 일부 URL 실패 처리
Given 5개 URL 중 2개가 403 에러를 반환할 때
When 병렬 스크래핑을 실행하면
Then 성공한 3개의 결과만 반환되고
And 실패한 URL은 로그에 기록되어야 한다

### REQ-2.2: 토큰 예산 관리
LLM 호출의 토큰 사용량을 추적하고 예산을 초과하면 조기 종료한다.

#### Scenario: 토큰 예산 내 실행
Given 토큰 예산이 100,000이고 현재 사용량이 50,000일 때
When 새 LLM 호출이 10,000 토큰을 사용하면
Then 사용량이 60,000으로 업데이트되고
And 리서치가 계속 진행되어야 한다

#### Scenario: 토큰 예산 초과 시 조기 종료
Given 토큰 예산이 100,000이고 현재 사용량이 95,000일 때
When 새 LLM 호출이 10,000 토큰을 요구하면
Then 리서치가 조기 종료되고
And 현재까지의 결과로 리포트가 생성되어야 한다

### REQ-2.3: 진행률 표시
리서치 진행 상황을 실시간으로 표시한다.

#### Scenario: 깊이 진행률 표시
Given depth=3으로 리서치가 시작되었을 때
When 첫 번째 iteration이 완료되면
Then "Depth 1/3 완료" 형태로 진행률이 표시되어야 한다

#### Scenario: URL 페칭 진행률 표시
Given breadth=5로 5개 URL을 페칭할 때
When 3개 URL이 완료되면
Then "URLs: 3/5" 형태로 진행률이 표시되어야 한다

### REQ-2.4: 중간 결과 저장 (Checkpoint)
각 depth 완료 시 상태를 저장하여 중단 후 재개를 지원한다.

#### Scenario: 체크포인트 저장
Given depth=3 리서치가 진행 중일 때
When 첫 번째 depth가 완료되면
Then ResearchState가 checkpoint.pkl 파일에 저장되어야 한다

#### Scenario: 체크포인트에서 재개
Given 이전에 depth=2에서 중단된 세션이 있을 때
When --resume 플래그로 실행하면
Then depth=2부터 리서치가 재개되어야 한다

### REQ-2.5: Source 신뢰도 평가
소스의 신뢰도를 평가하고 낮은 신뢰도 소스를 필터링한다.

#### Scenario: 높은 신뢰도 도메인 우선
Given arxiv.org, github.com, medium.com 소스가 있을 때
When 신뢰도 평가를 수행하면
Then arxiv.org(0.95), github.com(0.85), medium.com(0.3) 순으로 점수가 매겨져야 한다

#### Scenario: 최소 신뢰도 필터링
Given --min-trust 0.5가 설정되었을 때
When medium.com(0.3) 소스가 발견되면
Then 해당 소스는 스크래핑에서 제외되어야 한다

## MODIFIED

### REQ-1.4 (from Phase 1): deep_research() 함수
- **AS-IS**: 순차 실행, 토큰 추적 없음
- **TO-BE**: 병렬 실행 옵션, 토큰 예산 체크, 체크포인트 저장

## CLI Interface

```bash
# Phase 2 새 옵션
--parallel          # 병렬 스크래핑 활성화
--token-budget N    # 토큰 예산 (기본: 100000)
--resume            # 이전 세션 재개
--min-trust N       # 최소 신뢰도 (기본: 0.3)
```
