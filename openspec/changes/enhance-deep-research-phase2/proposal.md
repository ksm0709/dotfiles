# Proposal: Deep Research Phase 2 Enhancements

## Summary

딥 리서치 Phase 1(Iterative Deep Research)이 완료되었습니다. Phase 2에서는 리서치 결과를 바탕으로 가장 임팩트 있는 개선 사항을 구현합니다.

## Background

### Phase 1 완료 항목
- 재귀적 깊이 탐색 (Iterative Deepening)
- Breadth × Depth 파라미터 분리
- 학습 기반 방향 조정 (Learnings → Directions)
- 조기 종료 (confidence threshold)

### 딥 리서치 조사 결과 (2026-01-21)
리서치에서 발견된 2025-2026년 AI 코딩 어시스턴트 핵심 트렌드:

1. **Multi-Agent Collaboration**: 여러 에이전트가 협력하여 복잡한 작업 수행
2. **Long-Term Memory**: 에피소딕, 시맨틱, 절차적 메모리로 학습 지속
3. **Human-in-the-Loop**: 중요 결정에 인간 피드백 통합
4. **Parallel Processing**: 병렬 처리로 성능 향상
5. **MCP (Model Context Protocol)**: 도구 통합 표준화

## Motivation

Phase 2에서는 리서치 품질과 효율성을 크게 향상시킬 수 있는 기능들을 우선적으로 구현합니다:

| 기능 | 임팩트 | 구현 복잡도 | 우선순위 |
|------|--------|------------|----------|
| 병렬 처리 | High | Medium | 1 |
| 토큰 예산 관리 | High | Low | 2 |
| 진행률 표시 (Progress UI) | Medium | Low | 3 |
| 중간 결과 저장 | Medium | Low | 4 |
| Source 신뢰도 평가 | Medium | Medium | 5 |

## Proposed Changes

### 2.1 병렬 URL 스크래핑 (Parallel Scraping)
- `asyncio` 기반 동시 URL 페칭
- Rate limiting으로 서버 부하 방지
- 예상 속도 향상: 5-10x

### 2.2 토큰 예산 관리 (Token Budget)
- LLM 호출당 토큰 사용량 추적
- 총 예산 초과 시 조기 종료
- 비용 예측 및 제어 가능

### 2.3 진행률 표시 (Progress UI)
- 실시간 진행 상황 표시
- 현재 단계, 남은 깊이, 누적 학습 수 표시
- Rich 라이브러리 활용

### 2.4 중간 결과 저장 (Checkpoint)
- 각 depth 완료 시 상태 저장
- 중단 후 재개 가능
- 긴 리서치 세션 안정성 향상

### 2.5 Source 신뢰도 평가
- 도메인 기반 신뢰도 점수
- 학술 논문, 공식 문서 우선
- 낮은 신뢰도 소스 필터링

## Impact

### 사용자 경험
- 리서치 속도 5-10배 향상 (병렬 처리)
- 비용 예측 및 제어 가능 (토큰 예산)
- 진행 상황 실시간 확인 (Progress UI)

### 시스템 안정성
- 긴 리서치 세션 안정성 향상 (Checkpoint)
- 리소스 사용 최적화 (Rate limiting)

## Out of Scope (Phase 3+)

- Follow-up Question UI
- Multi-Agent Collaboration
- Long-Term Memory Integration
- MCP Tools Integration
