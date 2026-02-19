# Draft: deep-research skill update planning

## Requirements (confirmed)
- Deliverable is planning document only; no production code edits.
- Scope includes exactly three items:
  1. Minimal doc consistency fixes (path normalization `skill` -> `skills` and slash-command stale text cleanup).
  2. `llm_client` keyless fallback enhancement using `opencode run` path before mock.
  3. Trigger keyword support/addition strategy for better skill activation with backward compatibility.
- Plan must include: Objectives, Scope, Non-Goals, Task Breakdown, Verification Matrix, Risks, Open Questions.
- Plan must include phased tasks with checkboxes and dependencies, ending with P0/P1/P2 execution order.

## Technical Decisions
- Planning artifact path: `.sisyphus/plans/update-deep-research-skill-plan.md`.
- Evidence-driven planning only from repository files listed by user.
- No undocumented OpenCode runtime feature will be asserted as fact; unknowns captured in Open Questions.

## Research Findings
- `opencode/config/skills/deep-research/SKILL.md` contains stale path `~/.config/opencode/skill/deep-research` and slash-command specific phrase (`/deep-research`).
- `opencode/config/skills/deep-research/scripts/load_env.sh` usage comment also references `~/.config/opencode/skill/deep-research/...`.
- `opencode/config/skills/deep-research/scripts/llm_client.py` currently falls back directly to `mock_response()` when API keys are missing.
- `opencode/config/skills/deep-research/scripts/run_research.sh` and `load_env.sh` indicate keyless operation currently enters mock mode.
- `opencode/config/skills/plugin-create-guide/references/skill-structure.md` documents `metadata` as optional and does not define standard `trigger`/`keywords` fields.

## Scope Boundaries
- INCLUDE: planning details for the 3 requested items only.
- EXCLUDE: implementation changes, unrelated skills, broader architecture changes.

## Open Questions
- Whether `opencode run` is available and stable in all target environments (local/global install contexts).
- Whether `opencode run` invocation requires auth/session setup beyond current deep-research script assumptions.
- Whether skill discovery currently reads any custom frontmatter keys (e.g., `metadata.tags`, future `keywords`) or only `name/description`.

## Metis Review Findings
- Potential additional doc consistency mismatch identified: `run.py` help text and `run_research.sh` option comments may not align with actual defaults (`DEFAULT_DEPTH`).
- Risk: defining undocumented top-level frontmatter keys (`trigger`, `keywords`) may conflict with parser expectations; safer additive path is `metadata` convention.
- Risk: `opencode run` fallback can degrade UX if latency is high; plan should include timeout + immediate mock fallback.
- Guardrail: fallback enhancement must only run when API keys are unavailable; existing key-present behavior must remain unchanged.
- Guardrail: no scope expansion into run pipeline/search/scraper behavior changes.
