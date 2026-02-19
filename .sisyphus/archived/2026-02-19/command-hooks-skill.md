# Create `command-hooks` Skill for OpenCode Command Hooks Plugin

## TL;DR

> **Quick Summary**: Create a comprehensive skill that guides agents through interactively adding shell command hooks using the `opencode-command-hooks` plugin. The skill uses the Question tool to walk users through hook configuration step-by-step.
> 
> **Deliverables**:
> - `opencode/config/skills/command-hooks/SKILL.md` — Main skill file with interactive workflow
> - `opencode/config/skills/command-hooks/references/hook-types.md` — Complete hook type schema and options reference
> - `opencode/config/skills/command-hooks/references/examples.md` — Curated collection of real-world use cases
> - `opencode/config/skills/command-hooks/references/troubleshooting.md` — Common issues and debugging guide
> 
> **Estimated Effort**: Medium
> **Parallel Execution**: YES - 2 waves
> **Critical Path**: Task 1 → Task 2 → Task 3,4,5 (parallel) → Task 6

---

## Context

### Original Request
Create a new skill under `opencode/config` for the `opencode-command-hooks` plugin. The skill should:
- Help users add hooks interactively with Question tool options
- Cover basic usage through best practices
- Provide maximum reference material for agents applying hooks

### Interview Summary
**Key Discussions**:
- Skill name: `command-hooks` (concise, follows kebab-case convention)
- Workflow design: Full interactive — Question tool at every decision point
- References: Domain-based split (hook-types, examples, troubleshooting)
- Language: English body, Korean triggers in description

**Research Findings**:
- Full source code analysis of `opencode-command-hooks` v0.3.0 completed (index.ts, executor.ts, types/hooks.ts, config/agent.ts, config/global.ts, config/markdown.ts, config/merge.ts, execution/shell.ts, execution/template.ts, schemas.ts)
- Two config methods: global JSONC (`.opencode/command-hooks.jsonc`) + per-agent Markdown frontmatter
- Hook types: Tool hooks (`before`/`after`), Session hooks (`session.start`/`session.idle`/`session.end`)
- Filtering: tool name, toolArgs (exact match, array support), callingAgent, slashCommand
- Output: `inject` (session context), `toast` (UI notification with variant/duration)
- Template variables: `{id}`, `{agent}`, `{tool}`, `{cmd}`, `{stdout}`, `{stderr}`, `{exitCode}`
- Non-blocking execution, sequential command running, 30k char output truncation
- Config precedence: markdown > global (same ID), duplicate IDs within source = error
- Plugin installation: `"plugin": ["opencode-command-hooks"]` in opencode.json
- 19 existing skills in `opencode/config/skills/` follow `SKILL.md` + `references/` pattern

### Self-Review Gap Analysis
**Identified Gaps** (addressed in plan):
- Plugin prerequisite (opencode.json registration) must be covered in skill
- Boundary with `plugin-create-guide` skill must be explicit
- Question tool usage patterns need concrete examples in SKILL.md
- allowed-tools must include read, write, edit, bash, question for the agent to modify config files
- Agent workflow for modifying opencode.json, command-hooks.jsonc, and agent .md files must be detailed

---

## Work Objectives

### Core Objective
Create a skill that transforms any agent into an expert at configuring `opencode-command-hooks`, capable of interactively guiding users through hook creation with structured Question tool prompts.

### Concrete Deliverables
- `opencode/config/skills/command-hooks/SKILL.md` (main skill ~300-400 lines)
- `opencode/config/skills/command-hooks/references/hook-types.md` (complete schema reference)
- `opencode/config/skills/command-hooks/references/examples.md` (use case collection)
- `opencode/config/skills/command-hooks/references/troubleshooting.md` (debugging guide)

### Definition of Done
- [x] All 4 files exist and are valid markdown
- [x] SKILL.md has valid YAML frontmatter with `name: command-hooks` and comprehensive `description`
- [x] SKILL.md under 500 lines (progressive disclosure principle)
- [x] Interactive workflow uses concrete Question tool patterns
- [x] All hook types (tool before/after, session start/idle/end) documented
- [x] Both config methods (JSONC + frontmatter) covered with examples
- [x] All template variables documented with usage examples
- [x] References are correctly linked from SKILL.md

### Must Have
- Complete interactive workflow using Question tool for hook creation
- All hook types and their configuration options
- Both JSON and Markdown frontmatter configuration methods
- Template variable reference with all 7 variables
- Real-world use cases (validation gates, notifications, session hooks)
- Plugin installation prerequisites
- Configuration precedence rules

### Must NOT Have (Guardrails)
- **No plugin development guide** — That's `plugin-create-guide` skill's territory
- **No OpenCode SDK/API internals** — Only user-facing configuration
- **No TypeScript code examples for writing plugins** — Only YAML/JSON config
- **No duplicate content** — SKILL.md and references must not overlap; SKILL.md has workflow, references have data
- **No generic AI slop** — Every example must be a real, working configuration snippet
- **No over-documentation** — Follow progressive disclosure: keep SKILL.md lean, put details in references

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed. No exceptions.

### Test Decision
- **Infrastructure exists**: N/A (markdown files, no test framework needed)
- **Automated tests**: None (documentation skill, not code)
- **Framework**: N/A

### QA Policy
Every task includes agent-executed QA scenarios.
Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **File validation**: Use Bash to verify file existence, YAML frontmatter parsing, line counts
- **Content verification**: Use Grep to verify key sections/content exist
- **Skill loading**: Use Bash to verify skill can be detected by opencode

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — create directory + main SKILL.md):
├── Task 1: Create skill directory structure [quick]
├── Task 2: Write SKILL.md main file [writing]

Wave 2 (After Wave 1 — references in parallel):
├── Task 3: Write references/hook-types.md [writing]
├── Task 4: Write references/examples.md [writing]
├── Task 5: Write references/troubleshooting.md [writing]

Wave 3 (After Wave 2 — verification):
├── Task 6: Final verification and symlink setup [quick]

Wave FINAL (After ALL tasks — independent review):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Content quality review (unspecified-high)
├── Task F3: Scope fidelity check (deep)
```

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| 1 | — | 2, 3, 4, 5 | 1 |
| 2 | 1 | 3, 4, 5, 6 | 1 |
| 3 | 1, 2 | 6 | 2 |
| 4 | 1, 2 | 6 | 2 |
| 5 | 1, 2 | 6 | 2 |
| 6 | 3, 4, 5 | F1-F3 | 3 |
| F1-F3 | 6 | — | FINAL |

### Agent Dispatch Summary

- **Wave 1**: 2 tasks — T1 → `quick`, T2 → `writing`
- **Wave 2**: 3 tasks — T3,T4,T5 → `writing` (parallel)
- **Wave 3**: 1 task — T6 → `quick`
- **FINAL**: 3 tasks — F1 → `oracle`, F2 → `unspecified-high`, F3 → `deep`

---

## TODOs

- [x] 1. Create skill directory structure

  **What to do**:
  - Create directory: `opencode/config/skills/command-hooks/`
  - Create subdirectory: `opencode/config/skills/command-hooks/references/`
  - Verify both directories exist

  **Must NOT do**:
  - Create any files beyond directories (other tasks handle file creation)
  - Create unnecessary subdirectories (no scripts/, assets/)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Simple directory creation, single command
  - **Skills**: []
    - No specialized skills needed for mkdir

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 1 (with Task 2, but Task 2 depends on this)
  - **Blocks**: Tasks 2, 3, 4, 5
  - **Blocked By**: None (can start immediately)

  **References**:

  **Pattern References**:
  - `opencode/config/skills/bear-python-pro/` — Example skill with references/ subdirectory structure
  - `opencode/config/skills/subtask2/` — Another skill with references/ subdirectory

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Directory structure created correctly
    Tool: Bash
    Preconditions: opencode/config/skills/ exists
    Steps:
      1. Run: ls -la opencode/config/skills/command-hooks/
      2. Run: ls -la opencode/config/skills/command-hooks/references/
    Expected Result: Both directories exist and are accessible
    Failure Indicators: "No such file or directory" error
    Evidence: .sisyphus/evidence/task-1-dir-created.txt

  Scenario: No extra files or directories created
    Tool: Bash
    Preconditions: Directories just created
    Steps:
      1. Run: find opencode/config/skills/command-hooks/ -type f | wc -l
    Expected Result: 0 files (directories only at this stage)
    Evidence: .sisyphus/evidence/task-1-no-extra-files.txt
  ```

  **Commit**: NO (groups with Task 6)

- [x] 2. Write SKILL.md — Main skill file with interactive workflow

  **What to do**:
  - Create `opencode/config/skills/command-hooks/SKILL.md`
  - Write YAML frontmatter with `name: command-hooks` and comprehensive `description` that includes:
    - What the skill does (guide hook creation for opencode-command-hooks plugin)
    - When to use it (trigger phrases in English and Korean): "add a hook", "set up command hooks", "configure hooks", "훅 추가", "커맨드 훅", "hook 설정"
  - Write the main body covering:

    **Section 1: Overview** (~20 lines)
    - What opencode-command-hooks plugin does
    - Two configuration methods: JSON and Markdown frontmatter
    - Prerequisites: plugin must be in opencode.json

    **Section 2: Interactive Hook Creation Workflow** (~150 lines)
    - This is the CORE of the skill — a step-by-step conversational workflow using Question tool
    - Step 1: Ask hook category (Question tool: Tool Hook vs Session Hook)
    - Step 2: Based on category:
      - Tool Hook: Ask phase (Question: before vs after)
      - Session Hook: Ask event (Question: session.start vs session.idle vs session.end)
    - Step 3: Ask trigger conditions (Question for tool hooks: which tool to match, with common options like "task", "write", "read", "*")
    - Step 4: For tool hooks, ask about toolArgs filtering (Question: yes/no, then specifics like subagent_type)
    - Step 5: Ask what command(s) to run (free text, offer common patterns)
    - Step 6: Ask output method (Question: inject only, toast only, both, none)
    - Step 7: Ask configuration location (Question: global JSONC vs agent markdown frontmatter)
    - Step 8: Generate the configuration snippet, show to user for review
    - Step 9: Apply the configuration to the appropriate file

    **Section 3: Configuration Quick Reference** (~40 lines)
    - JSON config location: `.opencode/command-hooks.jsonc`
    - Markdown frontmatter format (simplified `hooks:` block)
    - Configuration precedence rules (brief, link to references)

    **Section 4: Template Variables** (~20 lines)
    - Table of all 7 template variables with descriptions
    - Brief usage note for inject and toast strings

    **Section 5: References Navigation** (~20 lines)
    - Link to `references/hook-types.md` — "For complete schema of all hook types and options"
    - Link to `references/examples.md` — "For real-world use case examples"
    - Link to `references/troubleshooting.md` — "For debugging and common issues"

  - Keep TOTAL under 400 lines (target ~300)

  **Must NOT do**:
  - Exceed 500 lines in SKILL.md
  - Duplicate reference content — link to references/ files instead
  - Include TypeScript/plugin development code
  - Use placeholder text or TODO markers
  - Include generic descriptions — every example must be a real, working config snippet

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Primary task is technical documentation writing with structured markdown
  - **Skills**: [`skill-creator`]
    - `skill-creator`: Provides guidelines for writing effective SKILL.md files — frontmatter, progressive disclosure, body structure

  **Parallelization**:
  - **Can Run In Parallel**: NO (must complete before Wave 2 starts)
  - **Parallel Group**: Wave 1 (sequential after Task 1)
  - **Blocks**: Tasks 3, 4, 5, 6
  - **Blocked By**: Task 1

  **References (CRITICAL — Be Exhaustive)**:

  **Pattern References**:
  - `opencode/config/skills/command-creator/SKILL.md` — BEST pattern match: conversational workflow using Question tool, phase-based structure (Understand → Inputs → Review). Copy this workflow design pattern.
  - `opencode/config/skills/plugin-create-guide/SKILL.md` — Shows plugin-related skill structure with Korean/English mixing in description. Use as boundary reference (don't overlap with this skill's content).
  - `opencode/config/skills/gh-cli/SKILL.md` — Example of a comprehensive CLI reference skill. Good pattern for concise quick-reference tables.

  **API/Type References (from plugin source code)**:
  - Plugin README: `https://github.com/shanebishop1/opencode-command-hooks` — Canonical documentation for all config options
  - Hook type schema (from source `src/types/hooks.ts`):
    - `ToolHook`: `{ id, when: { phase, tool?, callingAgent?, slashCommand?, toolArgs? }, run, inject?, toast? }`
    - `SessionHook`: `{ id, when: { event, agent? }, run, inject?, toast? }`
    - `AgentHooks` (frontmatter): `{ before?: AgentHookEntry[], after?: AgentHookEntry[] }`
    - `AgentHookEntry`: `{ run, inject?, toast? }`
    - `Toast`: `{ title?, message, variant?: "info"|"success"|"warning"|"error", duration? }`
  - Template variables (from source `src/execution/template.ts`): `{id}`, `{agent}`, `{tool}`, `{cmd}`, `{stdout}`, `{stderr}`, `{exitCode}`
  - Config schema (from source `src/schemas.ts`): `{ truncationLimit?: number, tool?: ToolHook[], session?: SessionHook[] }`
  - Config file location: `.opencode/command-hooks.jsonc` (searched upward from cwd, source `src/config/global.ts`)
  - Agent config path resolution (from `src/config/agent.ts`):
    - Project: `.opencode/agent/{name}.md`
    - User: `~/.config/opencode/agent/{name}.md`
  - Config precedence (from `src/config/merge.ts`): markdown hooks with same ID replace global hooks

  **External References**:
  - Skill Creator guidelines: `~/.config/opencode/skills/skill-creator/SKILL.md` — Follow frontmatter and body writing guidelines

  **WHY Each Reference Matters**:
  - `command-creator/SKILL.md`: The Question tool workflow pattern in this skill is EXACTLY what we need to replicate. Copy the conversational phase structure (Phase 1, 2, 3) and adapt it for hook creation.
  - Plugin source types: These are the authoritative definitions of every config option. The skill must accurately represent all fields.
  - Config paths: The skill must tell users exactly where files go — these paths come directly from source code.

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: SKILL.md has valid frontmatter and is under 500 lines
    Tool: Bash
    Preconditions: File created
    Steps:
      1. Run: wc -l opencode/config/skills/command-hooks/SKILL.md
      2. Run: head -5 opencode/config/skills/command-hooks/SKILL.md
      3. Verify line 1 is "---" (frontmatter start)
      4. Run: grep "name: command-hooks" opencode/config/skills/command-hooks/SKILL.md
      5. Run: grep "description:" opencode/config/skills/command-hooks/SKILL.md
    Expected Result: Line count < 500, frontmatter starts with ---, name and description fields present
    Failure Indicators: Line count >= 500, missing frontmatter delimiters, missing name/description
    Evidence: .sisyphus/evidence/task-2-frontmatter-valid.txt

  Scenario: Interactive workflow section exists with Question tool patterns
    Tool: Bash (grep)
    Preconditions: SKILL.md created
    Steps:
      1. Run: grep -c "question" opencode/config/skills/command-hooks/SKILL.md (case insensitive)
      2. Run: grep -c "options" opencode/config/skills/command-hooks/SKILL.md
      3. Run: grep -c "Tool Hook\|Session Hook" opencode/config/skills/command-hooks/SKILL.md
    Expected Result: "question" appears 5+ times, "options" appears 3+ times, both hook types mentioned
    Failure Indicators: Zero or very low mention counts
    Evidence: .sisyphus/evidence/task-2-workflow-sections.txt

  Scenario: References are correctly linked
    Tool: Bash (grep)
    Preconditions: SKILL.md created
    Steps:
      1. Run: grep "references/hook-types.md" opencode/config/skills/command-hooks/SKILL.md
      2. Run: grep "references/examples.md" opencode/config/skills/command-hooks/SKILL.md
      3. Run: grep "references/troubleshooting.md" opencode/config/skills/command-hooks/SKILL.md
    Expected Result: All 3 reference files are linked from SKILL.md
    Failure Indicators: Any reference file not mentioned
    Evidence: .sisyphus/evidence/task-2-references-linked.txt

  Scenario: No TypeScript plugin code in SKILL.md
    Tool: Bash (grep)
    Preconditions: SKILL.md created
    Steps:
      1. Run: grep -c "import.*Plugin\|export.*Plugin\|@opencode-ai/plugin" opencode/config/skills/command-hooks/SKILL.md
    Expected Result: 0 matches (no plugin development code)
    Failure Indicators: Any matches found
    Evidence: .sisyphus/evidence/task-2-no-plugin-code.txt
  ```

  **Commit**: NO (groups with Task 6)

- [x] 3. Write references/hook-types.md — Complete hook type schema reference

  **What to do**:
  - Create `opencode/config/skills/command-hooks/references/hook-types.md`
  - This is the AUTHORITATIVE schema reference — every option, every type, every field
  - Include table of contents at top (file will be 100+ lines)
  - Sections:
    
    **1. Tool Hook Schema** (~60 lines)
    - Full JSON schema with all fields and their types
    - `when` object: `phase` (before|after), `tool` (string|string[]), `callingAgent` (string|string[]), `slashCommand` (string|string[]), `toolArgs` (Record<string, string|string[]>)
    - `run`: string | string[] — sequential execution, failures don't block
    - `inject`: string — template with placeholders
    - `toast`: { title?, message, variant?, duration? }
    - Each field with: type, required/optional, default, description, example value

    **2. Session Hook Schema** (~40 lines)
    - Full JSON schema
    - `when` object: `event` (session.created|session.idle|session.end|session.start), `agent` (string|string[])
    - Same run/inject/toast as tool hooks
    - Note: `session.start` is alias for `session.created`

    **3. Markdown Frontmatter Schema** (~30 lines)
    - Simplified `hooks:` format for agent .md files
    - `before`: AgentHookEntry[], `after`: AgentHookEntry[]
    - Each entry: { run, inject?, toast? }
    - Auto-generated IDs: `{agentName}-{phase}-{index}`
    - Implicit scoping: markdown hooks auto-scope to the defining agent

    **4. Toast Configuration** (~20 lines)
    - All variant options: info, success, warning, error
    - Duration in milliseconds
    - Title is optional (defaults to "OpenCode Command Hook")

    **5. Template Variables** (~30 lines)
    - Complete table: placeholder, description, availability (tool/session), example value
    - All 7 variables: {id}, {agent}, {tool}, {cmd}, {stdout}, {stderr}, {exitCode}
    - Note: unavailable values replaced with empty string
    - Note: for array `run`, inject/toast use LAST command's output

    **6. Global Config Options** (~20 lines)
    - `truncationLimit`: number, default 30000, positive integer
    - `tool`: ToolHook[]
    - `session`: SessionHook[]
    - File location: `.opencode/command-hooks.jsonc`
    - JSONC format (comments allowed)

    **7. Configuration Precedence Rules** (~20 lines)
    - Global config loaded from `.opencode/command-hooks.jsonc` (searched upward from cwd)
    - Markdown hooks converted with auto-generated IDs
    - Same ID: markdown wins (replaces global)
    - Duplicate IDs within same source: error
    - Global config cached to avoid repeated reads

  **Must NOT do**:
  - Include workflow instructions (that's SKILL.md's job)
  - Include use case narratives (that's examples.md's job)
  - Include plugin TypeScript internals

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Technical reference documentation requiring precision and structure
  - **Skills**: []
    - No specialized skills needed beyond writing capability

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 4, 5)
  - **Blocks**: Task 6
  - **Blocked By**: Tasks 1, 2

  **References (CRITICAL)**:

  **Pattern References**:
  - `opencode/config/skills/gh-cli/SKILL.md` — Pattern for comprehensive CLI/API reference tables

  **API/Type References (AUTHORITATIVE — from plugin source)**:
  - `src/types/hooks.ts` (full content provided in Task 2 references) — Complete TypeScript interfaces for ALL hook types
  - `src/schemas.ts` — Zod schemas providing validation rules and constraints
  - `src/config/markdown.ts` — Markdown frontmatter parsing logic, `convertToCommandHooksConfig` shows how frontmatter maps to internal format
  - `src/config/merge.ts` — Precedence rules implementation
  - `src/config/global.ts` — Global config loading and JSONC parsing
  - `src/execution/template.ts` — Template variable interpolation logic (shows exact placeholder names)
  - `src/execution/shell.ts` — Command execution showing truncation behavior (DEFAULT_TRUNCATE_LIMIT = 30_000)
  - Plugin README: `https://github.com/shanebishop1/opencode-command-hooks` — User-facing documentation

  **WHY Each Reference Matters**:
  - `types/hooks.ts` is the single source of truth for ALL config fields. Every table in this reference must match these types exactly.
  - `schemas.ts` Zod schemas show validation constraints (e.g., id must be non-empty string, phase must be "before"|"after")
  - `markdown.ts` shows how simplified frontmatter format maps to full config (auto-ID generation pattern: `{agentName}-{phase}-{index}`)
  - `merge.ts` defines exact precedence rules that must be documented accurately
  - `shell.ts` defines the default truncation limit (30,000) that users should know about

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: All hook types documented
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -c "ToolHook\|Tool Hook" opencode/config/skills/command-hooks/references/hook-types.md
      2. Run: grep -c "SessionHook\|Session Hook" opencode/config/skills/command-hooks/references/hook-types.md
      3. Run: grep "truncationLimit" opencode/config/skills/command-hooks/references/hook-types.md
    Expected Result: Both hook types documented, truncationLimit mentioned
    Failure Indicators: Missing hook type documentation
    Evidence: .sisyphus/evidence/task-3-hook-types-complete.txt

  Scenario: All 7 template variables listed
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -c "{id}\|{agent}\|{tool}\|{cmd}\|{stdout}\|{stderr}\|{exitCode}" opencode/config/skills/command-hooks/references/hook-types.md
    Expected Result: All 7 variables appear in the file (count >= 7)
    Failure Indicators: Count < 7
    Evidence: .sisyphus/evidence/task-3-template-vars.txt

  Scenario: Precedence rules documented
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -i "precedence\|priority\|override" opencode/config/skills/command-hooks/references/hook-types.md
    Expected Result: At least 2 matches (precedence rules section exists)
    Evidence: .sisyphus/evidence/task-3-precedence.txt
  ```

  **Commit**: NO (groups with Task 6)

- [x] 4. Write references/examples.md — Real-world use case collection

  **What to do**:
  - Create `opencode/config/skills/command-hooks/references/examples.md`
  - Include table of contents at top
  - Curate a comprehensive collection of REAL, WORKING configuration snippets
  - Every example must include BOTH JSON and Markdown frontmatter versions where applicable
  - Categories:

    **1. Validation Gates** (~50 lines)
    - Run typecheck + lint + test after engineer/debugger subagents
    - Run lint after specific file writes
    - Run build check after any write operation

    **2. Quality Assurance** (~40 lines)
    - Auto-run tests after any task tool call
    - Enforce formatting on specific file patterns
    - Schema validation after database migration changes

    **3. Notifications & Monitoring** (~40 lines)
    - Toast notification for build status
    - Desktop notification when session completes
    - Log session start/end events

    **4. Context Injection** (~40 lines)
    - Inject git status before coding tasks
    - Inject test coverage report after test runs
    - Inject lint errors for agent to fix

    **5. Session Lifecycle** (~30 lines)
    - Run setup scripts on session start
    - Clean temporary files on session idle
    - Environment check on session start

    **6. Selective Filtering** (~40 lines)
    - Hook only on specific subagent types (engineer, debugger)
    - Hook on specific tool arguments (file path matching)
    - Hook on specific slash commands
    - Combining multiple filters

    **7. Advanced Patterns** (~30 lines)
    - Chaining multiple commands (array format)
    - Using template variables in inject messages for structured output
    - Custom truncation limits for verbose command output
    - Colocating hooks with agent definitions in markdown

  **Must NOT do**:
  - Include schema documentation (that's hook-types.md)
  - Include troubleshooting content (that's troubleshooting.md)
  - Use hypothetical/non-working examples
  - Include TypeScript plugin code

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Technical documentation with many code examples
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 3, 5)
  - **Blocks**: Task 6
  - **Blocked By**: Tasks 1, 2

  **References (CRITICAL)**:

  **Pattern References**:
  - `opencode/config/skills/command-creator/SKILL.md:180-330` — Examples section pattern with code blocks and usage descriptions

  **API/Type References**:
  - Plugin README examples section: `https://github.com/shanebishop1/opencode-command-hooks` — Contains 6 official examples to include and expand upon:
    1. Validate after task (typecheck + lint + test)
    2. Tests after any task
    3. Lint after specific write
    4. Toast for build status
    5. Session lifecycle hooks (start + idle)
    6. Filter by tool arguments (playwright + localhost)
  - All hook schemas from `src/types/hooks.ts` — For ensuring examples use valid field names and types

  **WHY Each Reference Matters**:
  - Plugin README examples are the CANONICAL starting point — include all of them, then ADD more creative use cases
  - Hook type schemas ensure examples use correct field names (e.g., `callingAgent` not `agent` in when clause)

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: All 7 example categories present
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -c "^##\|^###" opencode/config/skills/command-hooks/references/examples.md
      2. Run: grep -i "validation\|quality\|notification\|injection\|session\|filter\|advanced" opencode/config/skills/command-hooks/references/examples.md | wc -l
    Expected Result: At least 7 major sections, all 7 categories represented
    Failure Indicators: Fewer than 7 sections
    Evidence: .sisyphus/evidence/task-4-categories.txt

  Scenario: Both JSON and YAML formats shown
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -c '```jsonc\|```json' opencode/config/skills/command-hooks/references/examples.md
      2. Run: grep -c '```yaml\|```markdown' opencode/config/skills/command-hooks/references/examples.md
    Expected Result: Both JSON (3+ blocks) and YAML/markdown (3+ blocks) format examples present
    Failure Indicators: Only one format type
    Evidence: .sisyphus/evidence/task-4-dual-format.txt

  Scenario: No broken JSON snippets
    Tool: Bash
    Preconditions: File created
    Steps:
      1. Manually verify 2-3 JSON code blocks have matching braces/brackets
      2. Check for common errors: trailing commas in last array element (allowed in JSONC), mismatched quotes
    Expected Result: All JSON snippets are syntactically valid JSONC
    Evidence: .sisyphus/evidence/task-4-json-valid.txt
  ```

  **Commit**: NO (groups with Task 6)

- [x] 5. Write references/troubleshooting.md — Debugging guide

  **What to do**:
  - Create `opencode/config/skills/command-hooks/references/troubleshooting.md`
  - Cover common issues users encounter with the plugin
  - Sections:

    **1. Installation Issues** (~20 lines)
    - Plugin not loading: check opencode.json `"plugin": ["opencode-command-hooks"]`
    - Missing dependency: `bun install` / check package.json
    - Version compatibility

    **2. Hook Not Triggering** (~30 lines)
    - Wrong phase (before vs after)
    - Tool name mismatch (exact match required)
    - toolArgs not matching (exact string match)
    - Session event name mismatch (session.start vs session.created)
    - Hook in wrong config file (global vs markdown)
    - callingAgent not matching the subagent name

    **3. Configuration Errors** (~25 lines)
    - Duplicate hook IDs within same source
    - Invalid YAML frontmatter syntax
    - JSONC parse errors (malformed comments)
    - Missing required fields (id, when, run)
    - Invalid phase value (must be "before" or "after")
    - Invalid event value

    **4. Output Issues** (~20 lines)
    - Inject not showing in session: check inject template syntax
    - Toast not appearing: check toast.message is set
    - Output truncated: increase truncationLimit
    - Template variables showing as empty: variable unavailable in context
    - {stdout}/{stderr} empty: command produced no output

    **5. Debugging Techniques** (~25 lines)
    - Enable debug logging: `OPENCODE_HOOKS_DEBUG=true`
    - Check OpenCode logs for hook matching messages
    - Verify config is loaded: look for "Loaded global config" / "Loaded agent config" log lines
    - Test commands manually in terminal first
    - Use simple echo commands to verify hook triggers before complex scripts

    **6. Best Practices** (~30 lines)
    - Keep commands fast (hooks add latency)
    - Use non-destructive commands in before hooks
    - Always test commands outside hooks first
    - Use toast for user awareness, inject for agent awareness
    - Prefer markdown frontmatter for agent-specific hooks (colocation)
    - Use global JSONC for cross-cutting concerns (project-wide validation)
    - Set appropriate truncationLimit for verbose outputs
    - Use meaningful hook IDs for debugging

  **Must NOT do**:
  - Include schema definitions (that's hook-types.md)
  - Include complete examples (that's examples.md)
  - Suggest modifying plugin source code

  **Recommended Agent Profile**:
  - **Category**: `writing`
    - Reason: Technical troubleshooting documentation
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with Tasks 3, 4)
  - **Blocks**: Task 6
  - **Blocked By**: Tasks 1, 2

  **References (CRITICAL)**:

  **Pattern References**:
  - `opencode/config/skills/plugin-create-guide/SKILL.md:329-345` — Troubleshooting section pattern for plugin-related issues

  **API/Type References**:
  - `src/config/global.ts` — Config file search logic (walking up directory tree, 20 dir max depth). Shows exact error messages for parse failures.
  - `src/config/markdown.ts` — Frontmatter extraction logic. Shows what happens with malformed YAML (returns empty config, logs warning).
  - `src/config/merge.ts:findDuplicateIds()` — Duplicate ID detection logic
  - `src/schemas.ts` — Zod validation constraints (what makes a hook config invalid)
  - `src/executor.ts:matches()` — Matching logic (wildcard "*", array matching, undefined = match all). Critical for understanding WHY a hook might not trigger.
  - `src/logging.ts` — Debug logging control (look for `OPENCODE_HOOKS_DEBUG` environment variable reference)
  - `src/execution/shell.ts` — Truncation behavior and default limit (30,000 chars)
  - `src/index.ts` — Plugin initialization and error handling (what happens when plugin fails to initialize)

  **WHY Each Reference Matters**:
  - `executor.ts:matches()` is the KEY to debugging "hook not triggering" issues — it shows exactly how matching works (undefined patterns match all, "*" matches all, arrays check inclusion)
  - `global.ts` config loading shows what error messages users might see and what causes them
  - `schemas.ts` shows exactly what validation constraints exist — directly maps to "Configuration Errors" section

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: All troubleshooting categories present
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep -c "^##" opencode/config/skills/command-hooks/references/troubleshooting.md
      2. Run: grep -i "installation\|not triggering\|configuration error\|output\|debug\|best practice" opencode/config/skills/command-hooks/references/troubleshooting.md | wc -l
    Expected Result: At least 6 major sections, all categories covered
    Evidence: .sisyphus/evidence/task-5-categories.txt

  Scenario: Debug logging mentioned
    Tool: Bash (grep)
    Preconditions: File created
    Steps:
      1. Run: grep "OPENCODE_HOOKS_DEBUG" opencode/config/skills/command-hooks/references/troubleshooting.md
    Expected Result: Debug environment variable documented
    Failure Indicators: No mention of debug logging
    Evidence: .sisyphus/evidence/task-5-debug-logging.txt
  ```

  **Commit**: NO (groups with Task 6)

- [x] 6. Final verification and installation setup

  **What to do**:
  - Verify all 4 files exist and are valid markdown
  - Verify SKILL.md is under 500 lines
  - Verify YAML frontmatter is correct (name matches directory name)
  - Verify all 3 reference files are linked from SKILL.md
  - Verify no content overlap between SKILL.md and reference files
  - Run the install script if available, or verify the skill would be picked up by the opencode/install_agents.sh or similar setup mechanism
  - Check that the skill directory matches the pattern of other skills in `opencode/config/skills/`

  **Must NOT do**:
  - Modify any existing skills
  - Change installation scripts without understanding their full impact

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Verification and validation only, no creation
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3 (solo)
  - **Blocks**: F1, F2, F3
  - **Blocked By**: Tasks 3, 4, 5

  **References**:

  **Pattern References**:
  - `opencode/config/skills/` — Directory listing to compare structure with other skills
  - `opencode/install_agents.sh` or `opencode/install-config.sh` — Installation script that may need awareness of new skill

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: All files exist and have content
    Tool: Bash
    Preconditions: All previous tasks complete
    Steps:
      1. Run: wc -l opencode/config/skills/command-hooks/SKILL.md
      2. Run: wc -l opencode/config/skills/command-hooks/references/hook-types.md
      3. Run: wc -l opencode/config/skills/command-hooks/references/examples.md
      4. Run: wc -l opencode/config/skills/command-hooks/references/troubleshooting.md
    Expected Result: All 4 files exist with non-zero line counts; SKILL.md < 500 lines
    Failure Indicators: Any file missing or empty, SKILL.md >= 500 lines
    Evidence: .sisyphus/evidence/task-6-file-verification.txt

  Scenario: YAML frontmatter valid
    Tool: Bash
    Preconditions: SKILL.md exists
    Steps:
      1. Run: head -3 opencode/config/skills/command-hooks/SKILL.md
      2. Verify first line is "---"
      3. Run: grep "^name: command-hooks$" opencode/config/skills/command-hooks/SKILL.md
      4. Run: grep "^description:" opencode/config/skills/command-hooks/SKILL.md
    Expected Result: Valid frontmatter with correct name and description
    Evidence: .sisyphus/evidence/task-6-frontmatter.txt

  Scenario: Directory structure matches other skills
    Tool: Bash
    Preconditions: All files created
    Steps:
      1. Run: find opencode/config/skills/command-hooks/ -type f | sort
      2. Compare with: find opencode/config/skills/bear-python-pro/ -type f | sort
    Expected Result: Structure follows SKILL.md + references/ pattern
    Evidence: .sisyphus/evidence/task-6-structure-match.txt

  Scenario: No content duplication between SKILL.md and references
    Tool: Bash (grep)
    Preconditions: All files exist
    Steps:
      1. Extract unique multi-word phrases from hook-types.md tables
      2. Check if those same tables appear verbatim in SKILL.md
    Expected Result: Schema tables exist ONLY in hook-types.md, not duplicated in SKILL.md
    Evidence: .sisyphus/evidence/task-6-no-duplication.txt
  ```

  **Commit**: YES
  - Message: `feat(opencode): add command-hooks skill for opencode-command-hooks plugin`
  - Files: `opencode/config/skills/command-hooks/SKILL.md`, `opencode/config/skills/command-hooks/references/hook-types.md`, `opencode/config/skills/command-hooks/references/examples.md`, `opencode/config/skills/command-hooks/references/troubleshooting.md`
  - Pre-commit: `wc -l opencode/config/skills/command-hooks/SKILL.md` (verify < 500)

---

## Final Verification Wave

> 3 review agents run in PARALLEL. ALL must APPROVE. Rejection → fix → re-run.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, check content). For each "Must NOT Have": search all 4 files for forbidden patterns — reject with file:line if found. Check evidence files exist in .sisyphus/evidence/. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [x] F2. **Content Quality Review** — `unspecified-high`
  Review all 4 markdown files for: valid YAML frontmatter, correct markdown syntax, working internal links between SKILL.md and references, no placeholder text, no TODO markers, consistent formatting, accurate configuration snippets matching plugin v0.3.0 source.
  Output: `Files [N clean/N issues] | Links [N valid/N broken] | VERDICT`

- [x] F3. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual file content. Verify 1:1 — everything in spec was built (no missing), nothing beyond spec was built (no creep). Check "Must NOT do" compliance. Verify no overlap with `plugin-create-guide` skill content.
  Output: `Tasks [N/N compliant] | Scope [CLEAN/N issues] | VERDICT`

---

## Commit Strategy

- **1**: `feat(opencode): add command-hooks skill for opencode-command-hooks plugin` — all 4 files in `opencode/config/skills/command-hooks/`

---

## Success Criteria

### Verification Commands
```bash
# Verify all files exist
ls -la opencode/config/skills/command-hooks/SKILL.md
ls -la opencode/config/skills/command-hooks/references/hook-types.md
ls -la opencode/config/skills/command-hooks/references/examples.md
ls -la opencode/config/skills/command-hooks/references/troubleshooting.md

# Verify SKILL.md line count < 500
wc -l opencode/config/skills/command-hooks/SKILL.md

# Verify YAML frontmatter has required fields
head -20 opencode/config/skills/command-hooks/SKILL.md | grep "name: command-hooks"
head -20 opencode/config/skills/command-hooks/SKILL.md | grep "description:"
```

### Final Checklist
- [x] All "Must Have" present
- [x] All "Must NOT Have" absent
- [x] SKILL.md under 500 lines
- [x] All 4 files are valid markdown
- [x] References correctly linked from SKILL.md
