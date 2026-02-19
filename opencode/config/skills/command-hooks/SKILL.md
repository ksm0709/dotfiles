---
name: command-hooks
description: |-
  Guide users through interactively adding shell command hooks using the opencode-command-hooks plugin.
  Use this to automate tasks before or after tool executions, or on session events.

  Trigger phrases:
  - "add a hook", "set up command hooks", "configure hooks"
  - "훅 추가", "커맨드 훅", "hook 설정"
---

# Command Hooks

Automate shell commands triggered by OpenCode tool executions or session events.

<core_approach>

**Hook configuration is conversational and step-by-step.**

- MUST NOT guess configuration—use the `question` tool to guide the user.
- SHOULD offer common patterns (e.g., running tests after a write).
- MUST show the final configuration snippet for review before applying.

</core_approach>

<overview>

The `opencode-command-hooks` plugin (v0.3.0) allows you to define declarative shell command hooks.

- **Non-blocking**: Hook failures or long-running commands will not crash your session.
- **Two Config Methods**:
  1. **Global JSONC**: `.opencode/command-hooks.jsonc` (Project-wide)
  2. **Agent Markdown**: Frontmatter in agent `.md` files (Agent-specific)
- **Prerequisite**: Ensure `"plugin": ["opencode-command-hooks"]` is in your `opencode.json`.

</overview>

<workflow>

## Phase 1: Understand the Hook

Use the `question` tool to determine the hook's purpose.

1. **Ask hook CATEGORY** — Use `question` with options:
   - **Tool Hook**: "Runs before/after tool executions like task, write, read"
   - **Session Hook**: "Runs on session lifecycle events: start, idle, end"

2. **Determine the Trigger Phase/Event:**
   - **Tool Hook** → Ask **PHASE** using `question`: "before" vs "after"
   - **Session Hook** → Ask **EVENT** using `question`: "session.start", "session.idle", "session.end"

## Phase 2: Inputs & Configuration

3. **Identify the Trigger (Tool Hooks only):**
   - Ask **TRIGGER tool name** using `question` with common options:
     - `task` (subagent creation)
     - `write` (file writes)
     - `read` (file reads)
     - `*` (all tools)
   - Ask about **TOOL ARGS filtering** using `question`:
     - "Yes, filter by arguments" (e.g., `subagent_type: ["engineer"]`)
     - "No, match all calls"

4. **Define the Action:**
   - **"What command(s) should run?"** (Free text, offer patterns like `npm test` or `npm run lint`).
   - **Ask OUTPUT method** using `question`:
     - `inject`: "Inject into session for agent to read"
     - `toast`: "UI notification only"
     - `both`: "Inject + toast"
     - `none`: "Run silently"

5. **Ask CONFIG location** using `question`:
   - **Global JSONC**: `.opencode/command-hooks.jsonc` (Project-wide)
   - **Agent Markdown**: Frontmatter in agent `.md` file (Agent-specific)

## Phase 3: Review & Apply

6. **Generate and Show Snippet:**
   - Present the JSONC or YAML snippet to the user for review.
   - Ask: "Does this look correct? Should I apply it?"

7. **Apply Configuration:**
   - Use the `Edit` or `Write` tool to update the chosen configuration file.

</workflow>

<reference>

## Configuration Quick Reference

### JSONC (`.opencode/command-hooks.jsonc`)
```jsonc
{
  "tool": [
    {
      "id": "run-tests-after-write",
      "when": {
        "phase": "after",
        "tool": "write"
      },
      "run": "npm test",
      "toast": { "message": "Tests running...", "variant": "info" }
    }
  ]
}
```

### Markdown Frontmatter
```yaml
hooks:
  after:
    - run: ["npm run lint", "npm run typecheck"]
      inject: "Check results: {stdout}"
```

**Precedence**: Markdown hooks override global hooks with the same ID. See [Hook Types & Schema](references/hook-types.md) for full rules.

## Template Variables

| Placeholder | Description | Example |
|---|---|---|
| `{id}` | Hook ID | `validate-after-task` |
| `{agent}` | Calling agent name | `engineer` |
| `{tool}` | Tool name | `task` |
| `{cmd}` | Executed command | `npm test` |
| `{stdout}` | Command stdout (truncated) | test output |
| `{stderr}` | Command stderr (truncated) | error output |
| `{exitCode}` | Exit code | `0` or `1` |

*Note: Truncation defaults to 30,000 characters.*

*For array `run`, inject/toast templates use the **last command's** output.*

## References Navigation

- [Hook Types & Schema](references/hook-types.md): Complete technical specification.
- [Real-world Examples](references/examples.md): Common use cases and patterns.
- [Troubleshooting](references/troubleshooting.md): Debugging and best practices.

</reference>
