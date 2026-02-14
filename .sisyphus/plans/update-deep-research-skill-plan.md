# Plan: Update `deep-research` Skill (Doc Consistency + Keyless Fallback + Trigger Strategy)

## TL;DR

`deep-research` 스킬 업데이트를 3개 범위로 제한해 진행한다: (1) 문서 일관성 최소 수정, (2) API 키 부재 시 `llm_client`의 `opencode run` 경유 fallback 설계/구현, (3) trigger keyword 전략 추가(하위 호환 유지).

산출물은 구현 작업을 바로 실행할 수 있는 실행 계획이며, 각 항목에 문제 정의, 변경안, 단계, 검증, 롤백/가드레일을 포함한다.

---

## Objectives

1. `skill`/`skills` 경로 표기를 일관화해 문서 오해를 제거한다.
2. API 키가 없을 때 `mock`로 바로 떨어지던 경로를 개선해 `opencode run` fallback(선택적)을 먼저 시도한다.
3. 스킬 활성화 신호를 명시적으로 강화하되, 기존 동작(`name`/`description` 기반 discoverability)을 깨지 않는다.

---

## Scope

### In Scope
- `opencode/config/skills/deep-research/SKILL.md`의 경로/표현 정리
- `opencode/config/skills/deep-research/scripts/load_env.sh`의 경로 주석 정리
- `opencode/config/skills/deep-research/scripts/llm_client.py` keyless fallback 설계 반영
- Trigger keyword 전략을 `SKILL.md` frontmatter/본문 규약으로 명시(하위 호환)
- 위 변경에 필요한 최소 테스트/검증 계획

### Out of Scope
- 검색/스크래핑 파이프라인(`run.py` 실행 로직) 기능 확장
- 다른 스킬 파일 수정
- OpenCode 코어 라우팅/런타임 내부 동작 변경
- `opencode run` 미문서 기능을 사실로 확정하는 단정

---

## Non-Goals

- deep-research 성능 최적화
- 모델/프롬프트 품질 개선
- 새로운 설정 시스템 도입
- 스킬 전체 문서 리라이트

---

## Evidence Baseline (Repository)

- `opencode/config/skills/deep-research/SKILL.md:99` — `~/.config/opencode/skill/deep-research` 구 경로 존재
- `opencode/config/skills/deep-research/SKILL.md:11` — slash-command 전제 문구 존재
- `opencode/config/skills/deep-research/scripts/load_env.sh:6` — `skill/deep-research` 구 경로 존재
- `opencode/config/skills/deep-research/scripts/llm_client.py:59` — GEMINI key 없으면 mock
- `opencode/config/skills/deep-research/scripts/llm_client.py:97` — OPENAI key 없으면 mock
- `opencode/config/skills/deep-research/scripts/llm_client.py:150` — provider fallback이 최종 mock으로 귀결
- `opencode/config/skills/deep-research/scripts/run.py:632` — readiness 미충족 시 mock mode 안내
- `opencode/config/skills/plugin-create-guide/references/skill-structure.md:38` — `metadata`는 예시 있으나 trigger/keywords 표준 필드 명시 없음

---

## Item 1 - Minimal Doc Consistency Fixes

### Problem Statement
- deep-research 문서/스크립트 주석에 `skill` 단수 경로와 과거 slash-command 중심 문구가 남아 있어 현재 구조(`skills`) 및 사용 흐름과 불일치한다.

### Proposed Change
- `SKILL.md`, `load_env.sh`의 경로 표기를 `skills`로 정규화한다.
- slash-command 고정 전제를 제거하고, “직접 skill 호출/자동 선택” 문맥으로 정리한다.
- (범위 제한) 의미 변경 없이 문서 표현만 최소 수정한다.

### Implementation Steps
1. `SKILL.md`의 경로 예시를 `~/.config/opencode/skills/deep-research`로 교체
2. `load_env.sh` usage 주석 경로 동일 교체
3. `SKILL.md`의 slash-command stale 문구를 현재 사용 방식으로 중립화
4. 문구 변경 후 명령 예시가 실제 파일 구조와 일치하는지 재검토

### Verification Steps (Agent-Executable)
- `grep -n "~/.config/opencode/skill/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh`
  - 기대값: 출력 없음
- `grep -n "~/.config/opencode/skills/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh`
  - 기대값: 두 파일에서 매치 존재
- `grep -n "/deep-research" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: stale 문구 제거 기준을 충족(남길 경우는 “하위호환 안내 문맥”으로만 존재)

### Rollback / Guardrails
- Guardrail: 기능 코드 수정 금지(문서/주석 only)
- Rollback: 기존 문구/경로로 단일 커밋 되돌리기 가능해야 함
- Guardrail: deep-research 외 스킬 문서 미수정

---

## Item 2 - Keyless Path Enhancement (`opencode run` before mock)

### Problem Statement
- 현재 `llm_client.py`는 API 키가 없으면 즉시 `mock_response()`를 반환한다. 실제 런타임에서 `opencode run` 경유가 가능하더라도 활용하지 못한다.

### Proposed Change
- keyless 상태에서 `opencode run` fallback을 먼저 시도하고, 실패 시 mock으로 안전 하강한다.
- 하위 호환을 위해 fallback은 **선택적(opt-in)** 으로 시작한다.
  - 제안 플래그: `DEEP_RESEARCH_USE_OPENCODE_FALLBACK=1`
- fallback 우선순위:
  1) API key 기반 provider (기존)
  2) `opencode run` fallback (opt-in + keyless)
  3) mock

### Implementation Steps
1. `llm_client.py`에 keyless 분기용 helper 설계(`opencode run` subprocess 호출, timeout, stdout 파싱)
2. `gemini_complete`/`openai_complete`의 keyless 조기 mock 분기를 helper 경유로 변경
3. 실패 조건 정의: command 없음, timeout, non-zero exit, empty/invalid output
4. 실패 시 즉시 mock으로 fallback하고 민감정보 없는 로그만 남김
5. 기존 key-present 경로는 동작 불변 보장

### Verification Steps (Agent-Executable)
- 단위 테스트 (신규/수정)
  - `pytest -q opencode/config/skills/deep-research/scripts/test_research.py -k "llm_client or mock"`
  - 기대값: keyless fallback 관련 테스트 PASS
- 실패 경로 검증
  - `pytest -q opencode/config/skills/deep-research/scripts -k "fallback and keyless"`
  - 기대값: `opencode run` 실패 시 mock fallback 확인
- 정적 확인
  - `grep -n "DEEP_RESEARCH_USE_OPENCODE_FALLBACK" opencode/config/skills/deep-research/scripts/llm_client.py`
  - 기대값: opt-in 가드 존재

### Rollback / Guardrails
- Guardrail: API key 존재 시 기존 provider 호출 경로 절대 변경 금지
- Guardrail: `run.py` 연구 파이프라인 로직 수정 금지
- Guardrail: fallback 결과가 비정상일 때 예외 전파 대신 mock fallback으로 복귀
- Rollback: helper 호출부 제거 후 기존 keyless immediate mock 경로 복원

---

## Item 3 - Trigger Keyword Strategy (Backward Compatible)

### Problem Statement
- 현재 deep-research `SKILL.md`에는 `trigger`/`keywords` 명시 필드가 없다. 활성화 힌트가 `description` 자연어에만 의존해 discoverability가 약하다.

### Proposed Change
- 미문서 최상위 필드 강제 대신, 하위 호환 가능한 `metadata` 기반 규약을 우선 적용한다.
- 제안:
  - frontmatter `metadata.trigger_keywords` (배열)
  - 본문에 “Activation hints” 섹션 추가(동일 키워드 명시)
- 기존 `name`/`description` 유지로 backward compatibility 보장

### Implementation Steps
1. `SKILL.md` frontmatter에 `metadata` 블록 보강 (`trigger_keywords`)
2. 본문에 activation convention 섹션 추가
3. `/deep-research` 전용 문구는 “예시”로만 남기거나 중립 문구로 대체
4. plugin-create-guide 레퍼런스 관점과 충돌 없는지 문서 기준 점검

### Verification Steps (Agent-Executable)
- frontmatter 검증
  - `grep -n "^metadata:" opencode/config/skills/deep-research/SKILL.md`
  - `grep -n "trigger_keywords" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: 두 항목 모두 존재
- backward compatibility 검증
  - `grep -n "^name: deep-research$" opencode/config/skills/deep-research/SKILL.md`
  - `grep -n "^description:" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: 기존 핵심 필드 유지

### Rollback / Guardrails
- Guardrail: OpenCode 미문서 런타임 파서 동작 가정 금지
- Guardrail: top-level `trigger` 도입은 Open Questions 해소 전 보류
- Rollback: `metadata.trigger_keywords`/본문 힌트 제거 시 기존 discoverability는 유지되어야 함

---

## Task Breakdown (Phased, with Dependencies)

- [ ] T1. Baseline freeze and scope lock
  - 내용: 기준 파일 스냅샷 확인, 3개 범위 외 작업 금지선 명문화
  - Depends on: None
  - Blocks: T2, T3, T6

- [ ] T2. Apply doc consistency patch set
  - 내용: `skill` -> `skills`, slash-command stale text 정리
  - Depends on: T1
  - Blocks: T7

- [ ] T3. Add failing tests for keyless fallback path (TDD-RED)
  - 내용: keyless + opt-in + command fail/success 케이스 테스트 먼저 작성
  - Depends on: T1
  - Blocks: T4

- [ ] T4. Implement `opencode run` keyless fallback (TDD-GREEN)
  - 내용: helper + timeout + error-to-mock fallback
  - Depends on: T3
  - Blocks: T5, T7

- [ ] T5. Refactor and harden fallback behavior (TDD-REFACTOR)
  - 내용: 로깅 정리, 중복 제거, key-present 경로 회귀 확인
  - Depends on: T4
  - Blocks: T7

- [ ] T6. Apply trigger keyword convention (metadata + docs)
  - 내용: `metadata.trigger_keywords` 및 본문 activation hints 추가
  - Depends on: T1
  - Blocks: T7

- [ ] T7. Run verification matrix + regression checks
  - 내용: grep/pytest/정적 점검 일괄 실행
  - Depends on: T2, T5, T6
  - Blocks: T8

- [ ] T8. Final review + rollback notes package
  - 내용: 변경 근거, 위험, 롤백 절차 문서화 후 종료
  - Depends on: T7
  - Blocks: Done

---

## Verification Matrix

| ID | Verify Target | Command | Expected Result |
|---|---|---|---|
| V1 | 구 경로 제거 | `grep -n "~/.config/opencode/skill/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh` | no match |
| V2 | 신 경로 반영 | `grep -n "~/.config/opencode/skills/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh` | match exists |
| V3 | stale slash-command 정리 | `grep -n "/deep-research" opencode/config/skills/deep-research/SKILL.md` | legacy-only context or removed |
| V4 | fallback flag 도입 | `grep -n "DEEP_RESEARCH_USE_OPENCODE_FALLBACK" opencode/config/skills/deep-research/scripts/llm_client.py` | match exists |
| V5 | keyless fallback 테스트 | `pytest -q opencode/config/skills/deep-research/scripts -k "fallback and keyless"` | all pass |
| V6 | 기존 mock fallback 보존 | `pytest -q opencode/config/skills/deep-research/scripts/test_research.py -k "mock"` | all pass |
| V7 | trigger keyword 규약 반영 | `grep -n "trigger_keywords" opencode/config/skills/deep-research/SKILL.md` | match exists |
| V8 | 기본 필드 하위호환 | `grep -n "^name: deep-research$\|^description:" opencode/config/skills/deep-research/SKILL.md` | both exist |

---

## Risks and Mitigations

1. `opencode run` 런타임 미지원/버전 편차
   - Mitigation: opt-in 플래그 + timeout + 실패 즉시 mock
2. trigger 필드 스키마 불명확
   - Mitigation: top-level 필드 강제 금지, `metadata` 기반으로 한정
3. 문서 표현 수정이 실제 사용법 오해 유발
   - Mitigation: 경로/호출 예시를 현재 디렉토리 구조와 교차 검증
4. 회귀 리스크 (key-present 경로)
   - Mitigation: 기존 테스트 + key-present 전용 회귀 테스트 추가

---

## Open Questions

1. `opencode run` fallback의 표준 호출 시그니처/출력 포맷이 버전별로 안정적인가?
2. keyless fallback 기본값은 opt-in(기본 OFF)으로 둘지, command 감지 시 기본 ON으로 둘지?
3. `metadata.trigger_keywords`가 실제 스킬 선택기에서 소비되는지, 아니면 문서 discoverability 용도인지?
4. slash-command 문구를 완전 제거할지, “legacy alias” 안내 문구로 제한 유지할지?
5. `run.py`/`run_research.sh`의 depth 기본값 설명 불일치를 Item 1의 “minimal consistency”에 포함할지?

---

## Acceptance Criteria (Plan Quality)

- [ ] 각 Scope Item에 대해 문제정의/변경안/구현단계/검증/롤백이 모두 명시됨
- [ ] 검증 명령이 모두 실행 가능한 실제 명령으로 작성됨(placeholder 없음)
- [ ] 범위가 3개 요청 항목을 벗어나지 않음
- [ ] 미확정 런타임 가정은 Open Questions로 분리됨
- [ ] 작업 간 의존성이 체크박스 기반으로 명시됨

---

## Execution Order (Concise)

- **P0**: T1 (scope lock) + Open Questions 중 blocker 판별
- **P1**: T2, T3, T4, T5, T6 (핵심 변경)
- **P2**: T7, T8 (검증/회귀/정리)
