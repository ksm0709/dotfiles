# Update quick-research Skill Plan

## Objectives

- Deliver a low-risk, reversible improvement plan for `quick-research` only.
- Prioritize backward compatibility for existing usage patterns and dependencies.
- Define phased execution with explicit verification and rollback per item.
- Keep scope minimal and avoid undocumented assumptions about OpenCode internals.

Priority policy:
- MUST: compatibility/safety critical
- SHOULD: activation/discoverability and maintainability improvements
- COULD: optional forward-compatibility prep

## In Scope

- `opencode/config/skills/quick-research/SKILL.md` documentation and activation guidance updates.
- `opencode/config/skills/quick-research/scripts/load_env.sh` usage example path correction.
- `opencode/config/skills/quick-research/scripts/run.py` import-time failure strategy planning.
- `opencode/config/skills/quick-research/scripts/test_search.py` test isolation planning for missing dependency cases.
- Compatibility note and migration guardrails for `duckduckgo_search` -> `ddgs` ecosystem direction.

## Out of Scope

- Any production code changes in this deliverable (plan-only output).
- Deep-research feature parity work (venv orchestration, parallel crawler, etc.).
- CI pipeline redesign, repository-wide skill refactor, or non-quick-research feature additions.
- Undocumented runtime behavior claims about slash-command resolver internals.

## Evidence Baseline

- Outdated skill directory name `research/` in structure example: `opencode/config/skills/quick-research/SKILL.md:21`.
- Outdated path `~/.config/opencode/skill/research` appears in quick-start and CLI examples: `opencode/config/skills/quick-research/SKILL.md:33`, `opencode/config/skills/quick-research/SKILL.md:51`.
- Slash-command wording is presented as mandatory trigger style: `opencode/config/skills/quick-research/SKILL.md:11`.
- `load_env.sh` usage comment contains outdated path: `opencode/config/skills/quick-research/scripts/load_env.sh:6`.
- `run.py` performs import at module load and exits immediately on missing package: `opencode/config/skills/quick-research/scripts/run.py:17-22`.
- `test_search.py` repeatedly imports `search` from `run`, inheriting module-level import failure risk: `opencode/config/skills/quick-research/scripts/test_search.py:25`, `opencode/config/skills/quick-research/scripts/test_search.py:42`, `opencode/config/skills/quick-research/scripts/test_search.py:61`, `opencode/config/skills/quick-research/scripts/test_search.py:75`.
- `deep-research` includes `metadata.trigger_keywords`: `opencode/config/skills/deep-research/SKILL.md:14-20`.
- `deep-research` includes explicit "Activation hints" section: `opencode/config/skills/deep-research/SKILL.md:27-31`.
- Additional resiliency edge case (input parsing) exists in `run.py`: `opencode/config/skills/quick-research/scripts/run.py:73`.
- External compatibility signal (provided baseline): PyPI ecosystem direction indicates migration pressure from `duckduckgo_search` naming toward `ddgs`.

## Improvement Items

### I1. Path and usage documentation alignment

- Priority: MUST
- Problem: quick-research docs and shell usage examples still reference legacy `skill/research` path.
- Proposal: normalize examples to current `skills/quick-research` location and add one canonical invocation pattern.
- Steps:
  1. Replace outdated path references in `SKILL.md` and `load_env.sh` comments.
  2. Keep examples minimal and copy-paste safe.
  3. Add a short backward-compatibility note for users with old local symlink layouts.
- Verification:
  - `python - <<'PY'\nfrom pathlib import Path\nfiles=[\n 'opencode/config/skills/quick-research/SKILL.md',\n 'opencode/config/skills/quick-research/scripts/load_env.sh'\n]\nneedle='~/.config/opencode/skill/research'\nhits=[]\nfor f in files:\n    t=Path(f).read_text(encoding='utf-8')\n    if needle in t:\n        hits.append(f)\nprint('OK' if not hits else 'FOUND:'+','.join(hits))\nraise SystemExit(0 if not hits else 1)\nPY`
- Rollback:
  - Revert only modified doc/comment lines in the two files if path update causes confusion.

### I2. Activation wording and metadata parity (minimal)

- Priority: SHOULD
- Problem: quick-research lacks `metadata.trigger_keywords` and activation hints guidance while deep-research provides them.
- Proposal: add lightweight activation hints and trigger keywords without changing runtime behavior assumptions.
- Steps:
  1. Add `metadata.trigger_keywords` in frontmatter for quick-research.
  2. Add a short "Activation hints" section with examples.
  3. Keep slash-command wording non-exclusive ("can be used" instead of "must use").
- Verification:
  - `python - <<'PY'\nfrom pathlib import Path\nt=Path('opencode/config/skills/quick-research/SKILL.md').read_text(encoding='utf-8')\nrequired=['metadata:','trigger_keywords:','Activation hints']\nmissing=[x for x in required if x not in t]\nprint('OK' if not missing else 'MISSING:'+','.join(missing))\nraise SystemExit(0 if not missing else 1)\nPY`
- Rollback:
  - Remove only the added metadata/hints block; retain path fixes.

### I3. run.py import-time failure isolation

- Priority: MUST
- Problem: module import exits process (`sys.exit(1)`) when dependency is missing, causing cascading failures for tests and module consumers.
- Proposal: move dependency loading to runtime boundary (lazy import/check) and return structured error paths instead of exit during import.
- Steps:
  1. Introduce runtime dependency check function.
  2. Ensure module import itself is side-effect free.
  3. Keep CLI behavior user-friendly with clear install guidance on execution path.
  4. Preserve `search(query, max_results=5)` function signature.
- Verification:
  - `python - <<'PY'\nimport importlib.util, pathlib\np=pathlib.Path('opencode/config/skills/quick-research/scripts/run.py')\nspec=importlib.util.spec_from_file_location('qr_run', p)\nm=importlib.util.module_from_spec(spec)\nspec.loader.exec_module(m)\nprint('OK' if hasattr(m, 'search') else 'MISSING_SEARCH')\nraise SystemExit(0 if hasattr(m,'search') else 1)\nPY`
  - `python opencode/config/skills/quick-research/scripts/test_search.py --skip-network`
- Rollback:
  - Restore previous import strategy if runtime behavior regresses, but keep testability notes documented.

### I4. Test architecture resilience for missing dependency

- Priority: MUST
- Problem: tests importing `run` directly are brittle when dependency import path fails.
- Proposal: split tests into dependency-present and dependency-missing scenarios, with import-safe harness.
- Steps:
  1. Keep unit tests mocked and deterministic.
  2. Add explicit test case for missing dependency handling without process crash.
  3. Keep optional network test lane separated (`--skip-network` default in automation).
- Verification:
  - `python opencode/config/skills/quick-research/scripts/test_search.py --skip-network`
  - `python - <<'PY'\nimport subprocess,sys\ncmd=[sys.executable,'opencode/config/skills/quick-research/scripts/test_search.py','--skip-network']\nres=subprocess.run(cmd, capture_output=True, text=True)\nprint('OK' if res.returncode==0 else 'FAIL')\nraise SystemExit(res.returncode)\nPY`
- Rollback:
  - Revert only new/changed tests if false positives appear; keep baseline tests intact.

### I5. Dependency naming compatibility strategy (`duckduckgo_search` and `ddgs`)

- Priority: COULD
- Problem: ecosystem naming drift may break future installs if only one package name is supported.
- Proposal: define staged dual-support strategy in docs/tests before any hard switch.
- Steps:
  1. Document compatibility policy: legacy first, optional new-name support.
  2. Add acceptance checks for both package-name assumptions where feasible.
  3. Defer hard deprecation until data from real environments is collected.
- Verification:
  - `python - <<'PY'\nimport importlib.util\nmods=['duckduckgo_search','ddgs']\nstatus={m: bool(importlib.util.find_spec(m)) for m in mods}\nprint(status)\n# informational check; no hard fail\nPY`
- Rollback:
  - Keep legacy-only support note if dual-path introduces ambiguity.

## Task Breakdown

- [ ] T1 (MUST) Freeze baseline and validate evidence references in plan notes.
  - Depends on: none
  - Blocks: T2, T3, T4, T5
- [ ] T2 (MUST) Apply documentation path alignment plan (I1).
  - Depends on: T1
  - Blocks: T6
- [ ] T3 (SHOULD) Add activation metadata/hints plan details (I2).
  - Depends on: T1
  - Blocks: T6
- [ ] T4 (MUST) Define run.py import isolation implementation tasks (I3).
  - Depends on: T1
  - Blocks: T6, T7
- [ ] T5 (MUST) Define test resilience tasks and scenarios (I4).
  - Depends on: T1
  - Blocks: T7
- [ ] T6 (MUST) Assemble doc-level verification scripts and acceptance criteria.
  - Depends on: T2, T3, T4
  - Blocks: T8
- [ ] T7 (COULD) Add optional dependency naming compatibility lane (I5).
  - Depends on: T4, T5
  - Blocks: T8
- [ ] T8 (MUST) Final review: rollback readiness, low-complexity check, and scope lock.
  - Depends on: T6, T7
  - Blocks: completion

Dependency summary:
- Critical path: T1 -> T4 -> T6 -> T8
- Parallelizable: T2/T3/T4/T5 can proceed in parallel after T1.

## Verification Matrix (Executable Commands)

| ID | Purpose | Command | Expected Result | Phase |
|---|---|---|---|---|
| V1 | Plan file presence | `test -f .sisyphus/plans/update-quick-research-skill-plan.md` | exit code 0 | P0 |
| V2 | Mandatory sections completeness | `python - <<'PY'\nfrom pathlib import Path\np=Path('.sisyphus/plans/update-quick-research-skill-plan.md')\nt=p.read_text(encoding='utf-8')\nrequired=['## Objectives','## In Scope','## Out of Scope','## Evidence Baseline','## Improvement Items','## Task Breakdown','## Verification Matrix (Executable Commands)','## Risks and Mitigations','## Open Questions','## Execution Order (P0/P1/P2)']\nmissing=[x for x in required if x not in t]\nprint('OK' if not missing else 'MISSING:'+','.join(missing))\nraise SystemExit(0 if not missing else 1)\nPY` | prints `OK` | P0 |
| V3 | Legacy path removed where targeted | `python - <<'PY'\nfrom pathlib import Path\nfiles=['opencode/config/skills/quick-research/SKILL.md','opencode/config/skills/quick-research/scripts/load_env.sh']\nneedle='~/.config/opencode/skill/research'\nbad=[f for f in files if needle in Path(f).read_text(encoding='utf-8')]\nprint('OK' if not bad else 'FOUND:'+','.join(bad))\nraise SystemExit(0 if not bad else 1)\nPY` | prints `OK` | P1 |
| V4 | Frontmatter includes trigger keywords | `python - <<'PY'\nfrom pathlib import Path\nt=Path('opencode/config/skills/quick-research/SKILL.md').read_text(encoding='utf-8')\nok=('metadata:' in t and 'trigger_keywords:' in t)\nprint('OK' if ok else 'MISSING')\nraise SystemExit(0 if ok else 1)\nPY` | prints `OK` | P1 |
| V5 | run.py import safety | `python - <<'PY'\nimport importlib.util, pathlib\np=pathlib.Path('opencode/config/skills/quick-research/scripts/run.py')\nspec=importlib.util.spec_from_file_location('qr_run', p)\nm=importlib.util.module_from_spec(spec)\nspec.loader.exec_module(m)\nprint('OK' if hasattr(m,'search') else 'MISSING_SEARCH')\nraise SystemExit(0 if hasattr(m,'search') else 1)\nPY` | prints `OK` | P1 |
| V6 | Offline tests stable | `python opencode/config/skills/quick-research/scripts/test_search.py --skip-network` | all selected tests pass | P1 |
| V7 | Optional dual-package visibility check | `python - <<'PY'\nimport importlib.util\nmods=['duckduckgo_search','ddgs']\nprint({m: bool(importlib.util.find_spec(m)) for m in mods})\nPY` | prints import availability map | P2 |

## Risks and Mitigations

- Risk: Scope creep into deep-research parity work.
  - Mitigation: enforce Out-of-Scope list and quick-research-only file boundary.
- Risk: Breaking behavior while fixing import-time failure.
  - Mitigation: preserve public signature and add import-safety/offline tests before behavior changes.
- Risk: Documentation drift reappears.
  - Mitigation: include path lint checks (V3) in verification matrix.
- Risk: Dependency transition uncertainty (`duckduckgo_search` vs `ddgs`).
  - Mitigation: staged compatibility policy (COULD), no immediate hard switch.
- Risk: Network-dependent tests create flaky acceptance.
  - Mitigation: default acceptance on `--skip-network`; keep network lane optional/informational.

## Open Questions

- OQ1: Canonical command guidance: keep `/quick-search` only or mention alias forms for compatibility?
- OQ2: Should dual-package support (`duckduckgo_search` + `ddgs`) become SHOULD instead of COULD for near-term resilience?
- OQ3: For dependency-missing behavior, should CLI return non-zero with message while library API returns empty list or typed error?

## Execution Order (P0/P1/P2)

### P0 - Safety and baseline lock (MUST)

- Confirm evidence and section completeness (T1, V1, V2).
- Freeze scope boundaries and rollback policy.

### P1 - Core compatibility and resilience plan (MUST/SHOULD)

- I1 documentation path fixes and compatibility note (T2, V3).
- I2 activation metadata/hints parity (T3, V4).
- I3 import-time failure isolation plan (T4, V5).
- I4 test resilience plan with offline-first verification (T5, V6).
- Consolidate acceptance checks (T6).

### P2 - Optional forward-compatibility hardening (COULD)

- I5 dual-package transition policy and informational checks (T7, V7).
- Final readiness review and rollback confirmation (T8).

## Acceptance Criteria

- The plan document includes all required sections and executable verification commands.
- Every improvement item contains: problem, proposal, steps, verification, rollback, and priority (MUST/SHOULD/COULD).
- Task breakdown includes dependencies and checkboxes.
- Execution order explicitly separates P0/P1/P2.
- Scope remains limited to quick-research plus compatibility references only.
