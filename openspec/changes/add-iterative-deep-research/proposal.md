# Change: Add Iterative Deep Research with Learnings-based Direction

## Why

현재 deep-research 스킬은 단일 패스(Plan → Execute → Report)로 작동하여 깊이 있는 리서치가 어렵습니다.
dzhng/deep-research, gpt-researcher 등 선도 오픈소스처럼 재귀적 탐색과 학습 기반 방향 조정이 필요합니다.

## What Changes

1. **재귀적 깊이 탐색 (Iterative Deepening)**
   - 리서치 결과를 분석하여 추가 질문 생성
   - `--depth` 파라미터로 재귀 깊이 조절 (기존 URL 수 → 재귀 깊이)

2. **Breadth × Depth 파라미터 분리**
   - `--breadth`: 각 단계당 검색 쿼리/URL 수 (기존 --depth 역할)
   - `--depth`: 재귀 탐색 깊이 (새로 추가)

3. **학습 기반 방향 조정 (Learnings → Directions)**
   - 각 단계에서 "Learnings"와 "New Directions" 추출
   - 다음 리서치에 이전 학습 내용 반영
   - 답변 완성도 평가 후 조기 종료 가능

## Impact

- Affected specs: `deep-research` (신규)
- Affected code: 
  - `opencode/config/skill/deep-research/scripts/run.py`
  - `opencode/config/skill/deep-research/scripts/run_research.sh`
  - `opencode/config/skill/deep-research/SKILL.md`

## References

- dzhng/deep-research (18.4k stars): Breadth×Depth 재귀 탐색
- gpt-researcher (24.9k stars): Planner+Executor 멀티 에이전트
- jina-ai/node-DeepResearch (5.1k stars): 토큰 예산 기반 반복 탐색
