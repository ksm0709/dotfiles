# OpenSpec Proposal: 모든 에이전트에 가이드라인 섹션 적용 및 구조 최적화

## 1. 배경 및 목적 (Background & Objectives)
최신 `agent-template.md`에 `Guidelines` 섹션(Boundary, Commands & Skills, Conventions)이 추가되었습니다. 모든 에이전트 파일이 이 표준 구조를 따르도록 리팩토링하여 에이전트의 행동 지침을 더 구체화하고 일관성을 확보하고자 합니다.

## 2. 변경 범위 (Scope of Changes)
- **대상 파일**: `opencode/config/agent/*.md` (총 10개 파일)
- **주요 변경 사항**:
    - `Guidelines` 섹션 추가 및 하위 항목(`Boundary`, `Commands & Skills`, `Conventions`) 구성.
    - 기존 `Reference` 섹션에 혼재되어 있던 지침들을 `Guidelines`로 재배치.
    - `agent-template.md`의 최신 구조(Mermaid 그래프, Todo 단계 등)와 100% 동기화.

## 3. 기대 효과 (Expected Benefits)
- 에이전트의 제약 사항(Boundary)과 선호 도구(Commands & Skills)가 명확해짐에 따라 작업 정확도 향상.
- 모든 에이전트의 설정 구조가 통일되어 유지보수 효율성 증대.

## 4. 리스크 (Risks)
- 리팩토링 과정에서 에이전트 고유의 특수 지침이 누락될 수 있음 (기존 내용을 꼼꼼히 분석하여 재배치함으로써 방지).
