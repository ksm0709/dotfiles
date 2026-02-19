# Command Hook Schema Reference (v0.3.0)

This document provides the authoritative schema reference for all hook types and configuration options in the `opencode-command-hooks` plugin.

## Table of Contents
- [Tool Hook Schema](#section-1-tool-hook-schema)
- [Session Hook Schema](#section-2-session-hook-schema)
- [Markdown Frontmatter Schema](#section-3-markdown-frontmatter-schema)
- [Toast Configuration](#section-4-toast-configuration)
- [Template Variables](#section-5-template-variables)
- [Global Config Options](#section-6-global-config-options)
- [Configuration Precedence Rules](#section-7-configuration-precedence-rules)

---

## Section 1: Tool Hook Schema

Tool hooks trigger based on the execution of specific tools by agents.

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `id` | `string` | Yes | — | Unique hook identifier (non-empty) |
| `when.phase` | `"before"` \| `"after"` | Yes | — | Execution phase |
| `when.tool` | `string` \| `string[]` | No | `"*"` (all) | Tool name(s) to match |
| `when.callingAgent` | `string` \| `string[]` | No | `"*"` (all) | Agent name(s) to match |
| `when.slashCommand` | `string` \| `string[]` | No | `"*"` (all) | Slash command(s) to match |
| `when.toolArgs` | `Record<string, string \| string[]>` | No | — | Tool argument filters (exact match) |
| `run` | `string` \| `string[]` | Yes | — | Command(s) to execute sequentially |
| `inject` | `string` | No | — | Template for session injection |
| `toast` | `object` | No | — | Toast notification config |

### Example: Tool Hook
```jsonc
{
  "id": "validate-after-task",
  "when": {
    "phase": "after",
    "tool": "task",
    "callingAgent": "engineer"
  },
  "run": "npm test",
  "toast": {
    "message": "Task completed. Tests running...",
    "variant": "info"
  }
}
```

---

## Section 2: Session Hook Schema

Session hooks trigger based on lifecycle events of an OpenCode session.

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `id` | `string` | Yes | — | Unique hook identifier |
| `when.event` | `"session.created"` \| `"session.idle"` \| `"session.end"` \| `"session.start"` | Yes | — | Session lifecycle event |
| `when.agent` | `string` \| `string[]` | No | `"*"` (all) | Agent name(s) to match |
| `run` | `string` \| `string[]` | Yes | — | Command(s) to execute |
| `inject` | `string` | No | — | Template for session injection |
| `toast` | `object` | No | — | Toast notification config |

**Note:** `session.start` is an alias for `session.created`.

### Example: Session Hook
```jsonc
{
  "id": "init-workspace",
  "when": {
    "event": "session.created",
    "agent": "architect"
  },
  "run": ["git pull", "npm install"],
  "inject": "Workspace initialized for {agent}."
}
```

---

## Section 3: Markdown Frontmatter Schema

Agents can define hooks directly in their `.md` instruction files using a simplified format.

```yaml
hooks:
  before:
    - run: "string" | ["string", ...]
      inject: "string"  # optional
      toast:             # optional
        message: "string"
        variant: "info"
  after:
    - run: ...
```

- **Auto-generated IDs:** Hooks defined in markdown receive IDs in the format `{agentName}-{phase}-{index}` (e.g., `engineer-after-0`).
- **Implicit Scoping:** These hooks are automatically scoped to the defining agent via `callingAgent` and default to `tool: ["task"]`.
- **Agent File Locations:**
  - Project: `.opencode/agent/{name}.md`
  - User: `~/.config/opencode/agent/{name}.md`

---

## Section 4: Toast Configuration

Configuration for visual notifications shown in the UI.

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `title` | `string` | No | `"OpenCode Command Hook"` | Toast title |
| `message` | `string` | Yes | — | Toast message (supports template variables) |
| `variant` | `"info"` \| `"success"` \| `"warning"` \| `"error"` | No | `"info"` | Visual style |
| `duration` | `number` | No | — | Duration in milliseconds |

---

## Section 5: Template Variables

Placeholders available in `run`, `inject`, and `toast` strings.

| Placeholder | Description | Availability | Example |
|---|---|---|---|
| `{id}` | Hook ID | Tool + Session | `validate-after-task` |
| `{agent}` | Agent name | Tool + Session | `engineer` |
| `{tool}` | Tool name | Tool only | `task` |
| `{cmd}` | Executed command | Tool + Session | `npm test` |
| `{stdout}` | Command stdout (truncated) | Tool + Session | `All tests passed` |
| `{stderr}` | Command stderr (truncated) | Tool + Session | `Error: missing semicolon` |
| `{exitCode}` | Exit code | Tool + Session | `0` |

**Notes:**
- Unavailable values are replaced with an empty string.
- For array `run`, templates use the **last command's** output.
- Both `inject` and `toast` strings support these placeholders.

---

## Section 6: Global Config Options

Global configuration is stored in `.opencode/command-hooks.jsonc`. This file supports comments. The plugin searches upward from the current working directory (up to 20 levels).

| Option | Type | Default | Description |
|---|---|---|---|
| `truncationLimit` | `number` | `30000` | Max chars to capture from command output |
| `tool` | `ToolHook[]` | `[]` | Array of tool execution hooks |
| `session` | `SessionHook[]` | `[]` | Array of session lifecycle hooks |

---

## Section 7: Configuration Precedence Rules

1. Global configuration is loaded from `.opencode/command-hooks.jsonc`.
2. Markdown hooks are converted to the internal format with auto-generated IDs.
3. If a markdown hook and a global hook share the same `id`, the **markdown hook wins** (replaces the global hook).
4. Duplicate IDs within the **same source** (e.g., two hooks with the same ID in the global config) are treated as errors.
5. Global configuration is cached to avoid repeated file reads during a session.
6. `truncationLimit` can only be set in the global config; markdown hooks cannot override this value.
