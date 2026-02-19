# Command Hooks Examples

This document provides a curated collection of real, working configuration snippets for the `opencode-command-hooks` plugin v0.3.0. These examples demonstrate how to automate validation, quality assurance, and context management within your OpenCode environment.

## Table of Contents

1. [Validation Gates](#1-validation-gates)
2. [Quality Assurance](#2-quality-assurance)
3. [Notifications & Monitoring](#3-notifications--monitoring)
4. [Context Injection](#4-context-injection)
5. [Session Lifecycle](#5-session-lifecycle)
6. [Selective Filtering](#6-selective-filtering)
7. [Advanced Patterns](#7-advanced-patterns)

---

## 1. Validation Gates

Validation gates ensure that code changes meet specific standards before or after an agent performs a task.

### Example 1a: Run typecheck + lint + test after engineer/debugger subagents (JSON)

This hook triggers after any `task` tool call made by an `engineer` or `debugger` subagent.

```jsonc
{
  "tool": [
    {
      "id": "validate-after-task",
      "when": {
        "phase": "after",
        "tool": "task",
        "toolArgs": { "subagent_type": ["engineer", "debugger"] }
      },
      "run": ["npm run typecheck", "npm run lint", "npm test"],
      "inject": "Validation (exit {exitCode})\n\n{stdout}\n{stderr}",
      "toast": {
        "title": "Validation",
        "message": "exit {exitCode}",
        "variant": "info",
        "duration": 5000
      }
    }
  ]
}
```

### Example 1b: Same as markdown frontmatter (YAML)

When defining hooks directly in an agent's `.md` file, use the `hooks` key in the frontmatter.

```yaml
hooks:
  after:
    - run: ["npm run typecheck", "npm run lint", "npm test"]
      inject: "Validation (exit {exitCode})\n\n{stdout}\n{stderr}"
      toast:
        title: "Validation"
        message: "exit {exitCode}"
        variant: "info"
```

### Example 1c: Lint after specific file write (JSON)

Trigger a linting command only when a specific critical file is modified.

```jsonc
{
  "tool": [
    {
      "id": "lint-src-index",
      "when": {
        "phase": "after",
        "tool": "write",
        "toolArgs": { "path": "src/index.ts" }
      },
      "run": ["npm run lint"],
      "inject": "Lint (exit {exitCode})\n\n{stdout}\n{stderr}"
    }
  ]
}
```

---

## 2. Quality Assurance

Automate repetitive testing and build checks to maintain high code quality.

### Example 2a: Auto-run tests after any task tool call

Ensure that every major task performed by an agent is followed by a test run.

```jsonc
{
  "tool": [
    {
      "id": "tests-after-task",
      "when": { "phase": "after", "tool": "task" },
      "run": ["npm test"],
      "inject": "Tests (exit {exitCode})\n\n{stdout}\n{stderr}"
    }
  ]
}
```

### Example 2b: Run build check after any write

Verify that the project still builds successfully after any file modification.

```jsonc
{
  "tool": [
    {
      "id": "build-after-write",
      "when": { "phase": "after", "tool": "write" },
      "run": ["npm run build"],
      "toast": {
        "title": "Build",
        "message": "exit {exitCode}",
        "variant": "info",
        "duration": 3000
      }
    }
  ]
}
```

---

## 3. Notifications & Monitoring

Keep track of long-running processes or session states with notifications.

### Example 3a: Toast notification for build status

A standalone toast notification configuration for build results.

```jsonc
{
  "tool": [
    {
      "id": "build-status-toast",
      "when": { "phase": "after", "tool": "bash", "toolArgs": { "command": "npm run build" } },
      "toast": {
        "title": "Build Status",
        "message": "Build finished with exit code {exitCode}",
        "variant": "info",
        "duration": 5000
      }
    }
  ]
}
```

### Example 3b: Desktop notification when session idles (macOS)

Use system commands to send desktop notifications when the OpenCode session becomes idle.

```jsonc
{
  "session": [
    {
      "id": "notify-idle",
      "when": { "event": "session.idle" },
      "run": ["osascript -e 'display notification \"Session idle\" with title \"OpenCode\"'"],
      "toast": { "message": "Session idle", "variant": "info" }
    }
  ]
}
```

---

## 4. Context Injection

Automatically provide the agent with relevant system state or documentation.

### Example 4a: Inject git status before coding tasks

Give the agent visibility into the current git state before it starts a new task.

```jsonc
{
  "tool": [
    {
      "id": "git-status-before-task",
      "when": { "phase": "before", "tool": "task" },
      "run": ["git status --short"],
      "inject": "Current git status:\n{stdout}"
    }
  ]
}
```

### Example 4b: Inject test coverage after test runs (markdown)

Automatically append coverage reports to the agent's context after running tests.

```yaml
hooks:
  after:
    - run: ["npm test -- --coverage"]
      inject: "Coverage report:\n{stdout}"
```

---

## 5. Session Lifecycle

Manage environment setup and cleanup at the start and end of sessions.

### Example 5a: Session start + idle hooks

Perform initialization when a session starts and log when it goes idle.

```jsonc
{
  "session": [
    {
      "id": "session-start",
      "when": { "event": "session.start" },
      "run": ["echo 'New session started'"],
      "toast": { "title": "Session", "message": "started", "variant": "info" }
    },
    {
      "id": "session-idle",
      "when": { "event": "session.idle" },
      "run": ["echo 'Session idle'"]
    }
  ]
}
```

---

## 6. Selective Filtering

Fine-tune when hooks are triggered based on specific tool arguments or agent types.

### Example 6a: Hook only on specific subagent types

You can filter by the `subagent_type` argument passed to the `task` tool.

```jsonc
{
  "tool": [
    {
      "id": "engineer-only-hook",
      "when": {
        "phase": "after",
        "tool": "task",
        "toolArgs": { "subagent_type": ["engineer", "debugger"] }
      },
      "run": ["echo 'Running for engineer or debugger'"]
    }
  ]
}
```

### Example 6b: Hook on specific calling agents

Alternatively, you can filter by the name of the agent that is calling the tool using `callingAgent`.

```jsonc
{
  "tool": [
    {
      "id": "specialized-agent-hook",
      "when": {
        "phase": "before",
        "callingAgent": ["specialized-coder", "architect"]
      },
      "run": ["echo 'Preparing environment for specialized agent'"]
    }
  ]
}
```

### Example 6c: Hook on specific tool arguments (playwright + localhost)

Trigger a hook only when the agent navigates to a local development server.

```jsonc
{
  "tool": [
    {
      "id": "playwright-localhost",
      "when": {
        "phase": "after",
        "tool": "playwright_browser_navigate",
        "toolArgs": { "url": "http://localhost:3000" }
      },
      "run": ["echo 'Agent navigated to localhost'"],
      "toast": { "message": "Agent used {tool} to access localhost" }
    }
  ]
}
```

---

## 7. Advanced Patterns

Complex configurations for power users.

### Example 7a: Chaining multiple commands

Run a sequence of commands in a single hook.

```jsonc
{
  "tool": [
    {
      "id": "complex-cleanup",
      "when": { "phase": "after", "tool": "task" },
      "run": [
        "rm -rf ./tmp",
        "npm run clean",
        "git checkout -- package-lock.json"
      ]
    }
  ]
}
```

### Example 7b: Custom truncation limit

Increase the amount of command output that can be injected into the context.

```jsonc
{
  "truncationLimit": 5000,
  "tool": [
    {
      "id": "large-output-hook",
      "when": { "phase": "after", "tool": "bash" },
      "run": ["cat large_log_file.log"],
      "inject": "Log content:\n{stdout}"
    }
  ]
}
```

### Example 7c: Colocating hooks with agent definitions

A complete example of an agent markdown file with embedded hooks.

```markdown
---
name: specialized-coder
description: An agent specialized in React development.
hooks:
  after:
    - run: ["npm run lint"]
      inject: "Lint results (exit {exitCode}):\n{stdout}"
---

# Specialized Coder

You are an expert React developer. Always ensure that the code you write follows the project's linting rules.
```
