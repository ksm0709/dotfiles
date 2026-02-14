# Final Review + Rollback Notes: update-deep-research

## Scope completed (3 approved items only)

아래 3개 승인 항목만 반영되었고, 그 외 범위 확장은 확인되지 않았다.

1. 문서 일관성 정리
   - `skill` 경로 표기를 `skills`로 정규화
   - stale slash-command 문구를 레거시/예시 문맥으로 제한
2. keyless optional fallback 추가
   - API key 부재 시 즉시 mock 대신, `DEEP_RESEARCH_USE_OPENCODE_FALLBACK=1`일 때 `opencode run` 시도 후 실패 시 mock fallback
3. trigger keyword 전략 추가
   - `SKILL.md` frontmatter에 `metadata.trigger_keywords` 추가
   - 본문에 Activation hints 추가 (`name`/`description` 유지)

## Files changed

- `opencode/config/skills/deep-research/SKILL.md`
  - `metadata.trigger_keywords` 추가
  - Activation hints 섹션 추가
  - `cd ~/.config/opencode/skill/deep-research` -> `cd ~/.config/opencode/skills/deep-research`
- `opencode/config/skills/deep-research/scripts/load_env.sh`
  - usage 주석 경로 `skill` -> `skills` 정규화
- `opencode/config/skills/deep-research/scripts/llm_client.py`
  - `_OPENCODE_FALLBACK_ENV`, `_OPENCODE_FALLBACK_TIMEOUT_SECONDS` 도입
  - `_fallback_without_api_key`, `_try_opencode_run` 추가
  - keyless 시 `opencode run` opt-in fallback -> mock 순서 적용
- `opencode/config/skills/deep-research/scripts/test_research.py`
  - keyless fallback opt-in on/off, success/failure 테스트 추가
  - `llm_complete` keyless fallback 검증 방식 보강(mock 호출 검증)

## Verification evidence (V1~V8 status)

검증 기준: `.sisyphus/plans/update-deep-research-skill-plan.md`의 V1~V8 커맨드.

| ID | Command | Outcome | Status |
|---|---|---|---|
| V1 | `grep -n "~/.config/opencode/skill/deep-research" opencode/config/skills/deep-research/SKILL.md opencode/config/skills/deep-research/scripts/load_env.sh` | 출력 없음 (legacy 경로 미검출) | PASS |
| V2 | `grep -n "~/.config/opencode/skills/deep-research" ...` | `SKILL.md:112`, `load_env.sh:6` 매치 | PASS |
| V3 | `grep -n "/deep-research" opencode/config/skills/deep-research/SKILL.md` | `11,31,112` 매치. `31`은 레거시/예시 문맥, `112`는 경로 예시 | PASS (legacy-only context 허용) |
| V4 | `grep -n "DEEP_RESEARCH_USE_OPENCODE_FALLBACK" opencode/config/skills/deep-research/scripts/llm_client.py` | `18:_OPENCODE_FALLBACK_ENV = "DEEP_RESEARCH_USE_OPENCODE_FALLBACK"` | PASS |
| V5 | `pytest -q opencode/config/skills/deep-research/scripts -k "fallback and keyless"` | deep-research venv에서 실행, `3 passed, 66 deselected` | PASS |
| V6 | `pytest -q opencode/config/skills/deep-research/scripts/test_research.py -k "mock"` | deep-research venv에서 실행, `4 passed, 9 deselected` | PASS |
| V7 | `grep -n "trigger_keywords" opencode/config/skills/deep-research/SKILL.md` | `15:  trigger_keywords:` 매치 | PASS |
| V8 | `grep -n "^name: deep-research$\|^description:" opencode/config/skills/deep-research/SKILL.md` | `2:name: deep-research`, `3:description: |` 매치 | PASS |

### Test outcomes note

- 테스트 커맨드(V5, V6)는 `opencode/config/skills/deep-research/.venv`를 활성화한 뒤 `pytest`로 실행함.
- 결과: V5 `3 passed, 66 deselected`, V6 `4 passed, 9 deselected`로 모두 통과.

## Risk notes and known limitations

- `opencode run` fallback은 opt-in(`DEEP_RESEARCH_USE_OPENCODE_FALLBACK=1`)이므로 기본 동작에는 영향이 제한적이다.
- keyless fallback 성공 여부는 로컬 런타임의 `opencode` 커맨드 가용성과 응답 포맷에 의존한다.
- V5/V6 pytest 검증이 통과하여, keyless fallback 동작에 대한 테스트 기반 회귀 보장이 확보되었다.
- `SKILL.md`의 `/deep-research` 표기는 레거시/예시 용도로 남아 있어 완전 제거는 의도적으로 수행하지 않았다.

## Rollback procedure (file-by-file restore guidance)

아래 절차는 deep-research 업데이트 범위 파일만 되돌린다.

1. 작업 트리에서 변경 파일 확인
   - `git status --short`
2. 파일별 롤백 (필요한 파일만 선택 실행)
   - `git restore opencode/config/skills/deep-research/SKILL.md`
   - `git restore opencode/config/skills/deep-research/scripts/load_env.sh`
   - `git restore opencode/config/skills/deep-research/scripts/llm_client.py`
   - `git restore opencode/config/skills/deep-research/scripts/test_research.py`
3. 리포트 파일 롤백
   - 추적 전(untracked) 상태면: `rm .sisyphus/reports/update-deep-research-final-review.md`
   - 추적 이후라면: `git restore .sisyphus/reports/update-deep-research-final-review.md`
4. 롤백 확인
   - `git status --short`
   - 필요 시 V1, V2, V4, V7, V8 grep 재확인

## Closeout status

- T8 산출물(최종 리뷰 + 롤백 노트) 작성 완료: `.sisyphus/reports/update-deep-research-final-review.md`
