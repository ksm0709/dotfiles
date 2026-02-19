# Execution Prompt: update-deep-research (T1~T8)

## Objective
- `.sisyphus/plans/update-deep-research-skill-plan.md`를 구현 실행용으로 따른다.
- 작업 범위는 아래 3개만 허용한다.
  1. 문서 일관성 수정: `skill` -> `skills`, stale slash wording 정리
  2. `llm_client.py`에서 API key 없는 경우 `opencode run` keyless optional fallback 시도 후 mock fallback
  3. `SKILL.md` trigger keyword 추가 전략(`metadata` 기반, 하위 호환 유지)
- 플랜 파일은 read-only다. 절대 수정하지 않는다.

## Target files
- `opencode/config/skills/deep-research/SKILL.md`
- `opencode/config/skills/deep-research/scripts/load_env.sh`
- `opencode/config/skills/deep-research/scripts/llm_client.py`
- `opencode/config/skills/deep-research/scripts/test_research.py` (필요 시 keyless fallback 테스트 보강)

## Ordered steps (T1~T8)
- **T1. Baseline freeze and scope lock**
  - 플랜 파일을 읽고 범위를 3개 항목으로 고정한다.
  - read-only 규칙 확인: `.sisyphus/plans/update-deep-research-skill-plan.md` 수정 금지.

- **T2. Apply doc consistency patch set**
  - `SKILL.md`와 `load_env.sh`의 `~/.config/opencode/skill/deep-research`를 `~/.config/opencode/skills/deep-research`로 변경한다.
  - `SKILL.md`의 slash-command 고정/stale 문구를 현재 문맥(직접 호출/자동 선택) 기준으로 중립화한다.
  - 의미 확장 없이 문구/경로 정합성만 수정한다.

- **T3. Add failing tests for keyless fallback path (TDD-RED)**
  - keyless + opt-in + `opencode run` 성공/실패 케이스를 먼저 테스트로 작성한다.
  - 실패 조건(command 없음, timeout, non-zero exit, empty/invalid output)에서 mock fallback 기대를 명시한다.

- **T4. Implement `opencode run` keyless fallback (TDD-GREEN)**
  - `llm_client.py`에 keyless helper를 추가한다(subprocess, timeout, stdout parsing).
  - API key가 없고 `DEEP_RESEARCH_USE_OPENCODE_FALLBACK=1`일 때만 `opencode run`을 시도한다.
  - 실패 시 예외 전파 대신 즉시 mock fallback으로 안전 하강한다.

- **T5. Refactor and harden fallback behavior (TDD-REFACTOR)**
  - 중복 제거와 로깅 정리(민감정보 출력 금지)를 수행한다.
  - API key가 존재하는 기존 provider 경로 동작 불변을 확인한다.

- **T6. Apply trigger keyword convention (metadata + docs)**
  - `SKILL.md` frontmatter에 `metadata.trigger_keywords`를 추가한다.
  - 본문에 Activation hints를 추가해 동일 키워드를 문서화한다.
  - `name`/`description`은 유지해 backward compatibility를 보장한다.

- **T7. Run verification matrix + regression checks**
  - V1~V8 명령을 순서대로 실행하고 기대 결과를 확인한다.
  - 실패 시 해당 T 단계로 되돌아가 수정 후 재검증한다.

- **T8. Final review + rollback notes package**
  - 변경 근거, 리스크, 롤백 절차를 정리한다.
  - 최종 결과가 범위 3개를 벗어나지 않았는지 확인한다.

## Verification command checklist (V1~V8)
- **V1. 구 경로 제거 확인**
  - `grep -n "~/.config/opencode/skill/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh`
  - 기대값: no match

- **V2. 신 경로 반영 확인**
  - `grep -n "~/.config/opencode/skills/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh`
  - 기대값: match exists

- **V3. stale slash wording 정리 확인**
  - `grep -n "/deep-research" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: legacy-only context 또는 제거

- **V4. fallback opt-in flag 존재 확인**
  - `grep -n "DEEP_RESEARCH_USE_OPENCODE_FALLBACK" opencode/config/skills/deep-research/scripts/llm_client.py`
  - 기대값: match exists

- **V5. keyless fallback 테스트**
  - `pytest -q opencode/config/skills/deep-research/scripts -k "fallback and keyless"`
  - 기대값: all pass

- **V6. 기존 mock fallback 회귀 확인**
  - `pytest -q opencode/config/skills/deep-research/scripts/test_research.py -k "mock"`
  - 기대값: all pass

- **V7. trigger keyword 규약 반영 확인**
  - `grep -n "trigger_keywords" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: match exists

- **V8. 기본 필드 하위 호환 확인**
  - `grep -n "^name: deep-research$\|^description:" opencode/config/skills/deep-research/SKILL.md`
  - 기대값: 둘 다 존재

## Guardrails / out-of-scope
- 플랜 파일 수정 금지: `.sisyphus/plans/update-deep-research-skill-plan.md`는 read-only.
- 허용 범위는 3개 승인 항목만. 검색/스크래핑 파이프라인 기능 확장 금지.
- deep-research 외 다른 스킬 파일 수정 금지.
- OpenCode 코어 라우팅/런타임 변경 금지.
- 미문서 top-level trigger 스키마 단정 금지(`metadata` 보수 전략 유지).
- 불필요한 리팩터링/스타일 변경 금지.

## Definition of done
- T1~T8 완료 및 산출물 검토 완료.
- V1~V8 명령이 모두 기대값을 만족.
- 변경 범위가 3개 승인 항목에만 한정.
- `SKILL.md`/`load_env.sh`/`llm_client.py`/테스트 변경이 의도와 일치.
- 플랜 파일 미변경 상태 유지.
