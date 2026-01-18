# OpenSpec Proposal: 에이전트 시스템 표준화 및 리서치 에이전트 추가

## 1. 배경 및 목적 (Background & Objectives)
현재 `opencode/config/agent` 디렉토리에는 다양한 에이전트 설정이 존재하지만, 표준화된 템플릿이 없어 일관성 유지가 어렵습니다. 또한, 심층 리서치 및 분석을 수행할 전용 에이전트의 필요성이 제기되었습니다.

이 제안은 다음을 목표로 합니다:
1.  **표준화**: `pm.md`를 참조하여 에이전트 생성의 기준이 되는 `agent-template.md`를 정의합니다.
2.  **자동화**: 에이전트 생성을 전담할 `agent-creator.md` 서브 에이전트를 도입합니다.
3.  **기능 확장**: `deep-research`와 `research` 스킬을 활용하는 `research-analyst.md`를 추가하여 분석 역량을 강화합니다.

## 2. 변경 범위 (Scope of Changes)
- **신규 파일**:
    - `opencode/config/agent/agent-template.md`
    - `opencode/config/agent/agent-creator.md`
    - `opencode/config/agent/research-analyst.md`
- **대상 디렉토리**: `opencode/config/agent/` (사용자 요청의 `oprncode` 오타 수정 반영)

## 3. 기대 효과 (Expected Benefits)
- 에이전트 정의의 일관성 확보 및 유지보수 용이성 증대
- 새로운 에이전트 추가 시 작업 효율성 향상
- 전문적인 리서치 및 분석 작업 수행 가능

## 4. 리스크 (Risks)
- 기존 에이전트들과의 형상 불일치 (이번 변경은 신규 생성에만 초점을 맞춤)
