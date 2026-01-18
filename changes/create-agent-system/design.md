# Design: 에이전트 템플릿 및 신규 에이전트 설계

## 1. 에이전트 템플릿 (`agent-template.md`)
`pm.md`의 구조를 기반으로 하되, 범용적으로 사용할 수 있도록 일반화합니다.

### 구조
- **Frontmatter**: `description`, `mode`, `model`, `tools`, `skills` 등 메타데이터
- **Role**: 에이전트의 역할 정의
- **Core Principles**: 한국어 소통, 품질 준수 등 핵심 원칙
- **Workflow**: Mermaid 차트를 포함한 표준 워크플로우 (Plan -> Do -> Check -> Act)
- **Delegation**: 서브 에이전트 활용 전략 (해당되는 경우)
- **Standards**: 코드 품질 및 테스팅 표준

## 2. 에이전트 생성기 (`agent-creator.md`)
새로운 에이전트 생성을 전담하는 메타 에이전트입니다.

### 역할
- 사용자 요청(자연어)을 분석하여 `agent-template.md`를 기반으로 구체적인 에이전트 파일 생성
- 적절한 도구(Tools)와 스킬(Skills) 추천 및 설정

## 3. 리서치 분석가 (`research-analyst.md`)
심층 연구 및 정보 수집을 담당하는 프라이머리 에이전트입니다.

### 스킬 구성
- `deep-research`: 복잡한 주제에 대한 심층 분석 및 리포트 생성
- `research`: 빠른 사실 확인 및 최신 정보 검색
- `webfetch`: 웹 콘텐츠 직접 조회

### 워크플로우
1. **주제 분석**: 연구 목표 및 범위 설정
2. **계획 수립**: 검색 키워드 및 소스 선정
3. **정보 수집**: `research` 및 `deep-research` 스킬 활용
4. **종합 및 분석**: 수집된 정보의 교차 검증 및 인사이트 도출
5. **리포트 작성**: 마크다운 형식의 최종 결과물 생성

## ADDED Requirements

### `agent-template.md`
- `pm.md`의 워크플로우 섹션을 포함해야 함
- `OpenSpec` 관련 지침을 포함해야 함 (Primary Agent인 경우)

### `research-analyst.md`
- `deep-research`와 `research` 스킬이 활성화되어야 함
- `thinking: high` 설정으로 분석 능력 강화

#### Scenario: 리서치 에이전트 동작 확인
1. 사용자가 "최신 LLM 트렌드 조사해줘"라고 요청
2. `research-analyst`가 `deep-research` 스킬을 호출
3. 검색 계획 수립 후 정보 수집
4. 최종 요약 리포트 출력
