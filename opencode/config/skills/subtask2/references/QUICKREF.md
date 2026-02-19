# Subtask2 Quick Reference

One-page syntax cheat sheet for rapid lookup.

## Frontmatter Directives

```yaml
---
# Basic subtask
subtask: true

# With return chain
subtask: true
return: Next instruction

# With multiple returns
subtask: true
return:
  - Step 1
  - Step 2
  - /command

# With loop
loop:
  max: 10
  until: "condition description"

# With parallel execution
parallel:
  - /command1
  - /command2
  - command: /cmd3
    arguments: custom args

# Combined
subtask: true
parallel:
  - /cmd1 {as:result1}
  - /cmd2 {as:result2}
return:
  - Synthesize $RESULT[result1] and $RESULT[result2]
  - /next-step
---
```

## Inline Syntax

### Parameter Overrides

```
{model:provider/model-name}
{agent:agent-name}
{loop:N}
{loop:N && until:condition}
{as:result-name}
{return:prompt}
{return:/command}
```

### Combining Parameters

```
{model:anthropic/claude-sonnet-4 && agent:build}
{model:openai/gpt-4o && loop:5 && until:tests pass}
{model:anthropic/claude-opus-4.5 && agent:plan && as:design}
```

### Multiple Returns (use ||)

```
{return:step1 || step2 || /command}
```

### Multiple Parallel (use || for args)

```
/command main || arg1 || arg2 || arg3
```

## Context Variables

```
$TURN[n]          # Last n messages
$TURN[:3]         # 3rd message from end
$TURN[:2:5:8]     # Messages at indices 2, 5, 8
$TURN[*]          # All messages
$RESULT[name]     # Named result output
$ARGUMENTS        # Command arguments
```

## Common Patterns

### Simple Subtask
```
/subtask do something
```

### With Model Override
```
/subtask {model:anthropic/claude-opus-4.5} complex analysis
```

### With Loop
```
/subtask {loop:5 && until:perfect} refine this code
```

### With Return Chain
```
/subtask {return:review || implement || test} create prototype
```

### Parallel Models
```yaml
parallel:
  - /task {model:model1 && as:name1}
  - /task {model:model2 && as:name2}
return: Compare $RESULT[name1] and $RESULT[name2]
```

## Command Examples

```bash
# Chain returns
/plan design system
  -> return: implement
  -> return: test

# Loop until done
/fix {loop:10 && until:all tests pass}

# Parallel research
/research {parallel:/docs || /codebase}

# Model A/B test
/plan {model:gpt-4o && as:gpt}
/plan {model:claude-sonnet && as:claude}
/synthesize compare $RESULT[gpt] and $RESULT[claude]

# Context-aware
/implement considering $TURN[5]
```

## Syntax Cheat Sheet

| Goal | Syntax |
|------|--------|
| Make it a subtask | `subtask: true` in frontmatter |
| Chain next action | `return: prompt` or `return: /command` |
| Multiple actions | `return: [action1, action2, /cmd]` |
| Repeat N times | `{loop:N}` |
| Repeat until condition | `{loop:N && until:description}` |
| Run in parallel | `parallel: [/cmd1, /cmd2]` |
| Use specific model | `{model:provider/name}` |
| Use specific agent | `{agent:name}` |
| Capture output | `{as:name}` |
| Reference output | `$RESULT[name]` |
| Pass context | `$TURN[n]` |
| Ad-hoc subtask | `/subtask {params} prompt` |
| Combine params | `{param1 && param2 && param3}` |
| Multiple returns | `{return:step1 \|\| step2}` |
| Custom args in parallel | `command: /cmd` with `arguments: args` |

## Installation Reminder

```json
{
  "plugins": ["@spoons-and-mirrors/subtask2@latest"]
}
```

## Priority Rules

1. **Inline** > **Frontmatter** > **Config** > **Defaults**
2. **Pipe args** (`||`) > **Frontmatter args** > **Inherit main args**
3. **Explicit return** > **Config generic_return** > **Built-in default** > **OpenCode generic**

## Loop Control Tags

```
<subtask2 loop="continue"/>   # Keep looping
<subtask2 loop="break"/>      # Exit loop
```
