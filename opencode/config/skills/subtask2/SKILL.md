---
name: subtask2
description: Interactive workflow for creating powerful OpenCode /commands with subtask2 orchestration features. Guides users through designing commands with return chains, loops, parallel execution, context passing, and result capture. Use when users want to CREATE new /commands that orchestrate multiple subagents, need iterative or parallel workflows, want to automate complex multi-step processes, or when they ask to create a subtask, command with chaining, parallel commands, loop commands, or multi-step automation.
---

# Subtask Creator: Build Powerful /commands

This skill guides you through creating sophisticated OpenCode commands using subtask2 features. Unlike basic commands, subtask2-enabled commands can chain actions, loop until conditions are met, run tasks in parallel, and intelligently pass context between steps.

**When to use this skill:**
- User says: "Create a subtask that..."
- User says: "Make a command with return chain..."
- User says: "I want parallel commands..."
- User says: "Create loop command..."
- User wants multi-step workflow automation
- User needs to orchestrate multiple subagents
- User asks about advanced command patterns

**Not for:** Simple one-shot commands (use `command-creator` skill instead)

## Interactive Creation Workflow

MUST follow this conversational workflow to create subtask2 commands. DO NOT jump directly to writing files - gather requirements first.

### Phase 1: Understand the Task (ASK USER)

**Question 1:** "What repetitive task or workflow do you want to automate with a /command?"

Listen for:
- Multi-step processes (research → plan → implement)
- Need for parallel execution (multiple models/agents)
- Iterative refinement (try until quality criteria met)
- Result passing between steps

**Question 2:** "Can you walk me through what should happen step by step?"

Example good answers:
- "Review PR, then run tests, fix issues, and commit"
- "Research from docs AND codebase in parallel, then plan"
- "Generate with 3 models, compare, pick best, implement"
- "Fix tests → check → if failing, retry (loop)"

### Phase 2: Determine Execution Mode (DECISION TREE)

Based on user's description, guide them through this decision tree:

```
Does this need background execution or orchestration?
├── YES → Must use subtask2 features
│   └── Does it trigger actions AFTER completion?
│       ├── YES → Needs `return:` chain
│       │   └── Multiple sequential steps?
│       │       ├── YES → Array: `return: [step1, step2, step3]`
│       │       └── NO → String: `return: next action`
│       └── NO → Simple `subtask: true`
│   
│   └── Does it repeat until quality criteria met?
│       ├── YES → Needs `loop:`
│       └── NO → No loop needed
│   
│   └── Does it run independent tasks simultaneously?
│       ├── YES → Needs `parallel:`
│       └── NO → Sequential execution
│
└── NO → Regular command (use command-creator skill)
```

**Explain to user:** "Based on what you described, this needs [subtask/return/loop/parallel] because [reason]."

### Phase 3: Design Return Chain (IF NEEDED)

If return chain identified:

**Question 3:** "What should happen after each step completes?"

Map out the sequence with user:
```
Step 1: [Subtask] → Result: [what it produces]
Step 2: [Next action using result?]
Step 3: [Next action?]
...
Final: [Completion action]
```

**Return types to explain:**
- **Prompt return**: `return: "Review output and identify issues"` (sent as user message)
- **Command return**: `return: /fix-issues` (executes another command)
- **Array returns**: `return: [step1, step2, step3]` (sequential workflow)

### Phase 4: Choose Context Strategy

**Question 4:** "What information does this command need as input?"

Options table:

| Input Type | Syntax | When to Use |
|------------|--------|-------------|
| User arguments | `$ARGUMENTS` | Input passed to /command |
| Conversation | `$TURN[5]` | Needs previous discussion |
| Files | `@filepath` | Read config/schema/code |
| Shell output | ``!`cmd` `` | Dynamic context (git status) |
| Previous results | `$RESULT[name]` | From earlier subtask |

**Insert placeholders ONLY ONCE per command.** Use dedicated XML blocks.

### Phase 5: Plan Result Handling (IF PARALLEL/MULTI-STEP)

**Question 5:** "Do you need to capture outputs from steps for later synthesis?"

If yes, explain capture/refer pattern:
```yaml
# Capture
/subtask {as:research-results} research the topic

# Reference later
/subtask synthesize $RESULT[research-results]
```

**Naming guidance:** Use descriptive names like `{as:security-audit}` not `{as:result1}`

### Phase 6: Validate Design

**Checklist before creating:**
- [ ] Execution mode matches user's needs
- [ ] Return chain is logical (each step follows naturally)
- [ ] Context is sufficient but not excessive
- [ ] Not over-engineered (simpler = better)
- [ ] Loop condition is objectively verifiable (if using loop)
- [ ] Parallel tasks are related (if using parallel)

**Confirm with user:** "Before I create this, let me confirm the design: [summarize]. Does this match what you need?"

### Phase 7: Generate Command

After user confirms, create the command file at appropriate location:
- Project: `.opencode/command/<name>.md`
- Global: `~/.config/opencode/command/<name>.md`

## Command Template Structure

```markdown
---
description: Brief what-this-does summary
agent: build|plan|explore        # Optional: routing
model: provider/model-id         # Optional: override
subtask: true                   # Required for subtask2

# Add based on workflow analysis:
return: |
  Step 1: [action]
  /next-command
  Final step: [action]

loop:
  max: 5
  until: "clear success condition"

parallel:
  - /command-a {as:result-a}
  - /command-b {as:result-b}
---

## Summary

Line 1: You MUST [primary objective].
Line 2: You SHOULD [key input/constraint].
Line 3: You MUST [expected outcome].

## Context

<!-- Use if needed -->
<context>
$ARGUMENTS
!`git status`
@src/config.ts
</context>

## Objective

Detailed instructions for the subagent.
</workflow>
```

## Common Patterns Library

Show user relevant patterns from their use case:

### Pattern 1: Research → Plan → Implement → Test
```yaml
subtask: true
return:
  - Research requirements
  - Design architecture
  - /implement code with tests
  - /verify quality check
---
Implement: $ARGUMENTS
```

### Pattern 2: Parallel Model Comparison
```yaml
subtask: true
parallel:
  - /solve {model:claude-sonnet-4 && as:sol-a}
  - /solve {model:gpt-4o && as:sol-b}
return:
  - Compare $RESULT[sol-a] vs $RESULT[sol-b]
  - Select and implement better approach
---
Solve: $ARGUMENTS
```

### Pattern 3: Iterative Refinement
```yaml
subtask: true
loop:
  max: 5
  until: "tests pass and review approved"
return:
  - Report final quality metrics
---
Refine: $ARGUMENTS
```

### Pattern 4: Context-Aware Processing
```yaml
subtask: true
---
Based on discussion:

$TURN[10]

Continue: $ARGUMENTS
```

## Anti-Patterns to Warn Users About

### ❌ Vague Loop Conditions
```yaml
# BAD - Not verifiable
loop:
  until: "DONE"

# GOOD - Objective criteria
loop:
  until: "all tests pass and coverage > 80%"
```

### ❌ Unrelated Parallel Tasks
```yaml
# BAD - No synthesis value
parallel:
  - /update-docs
  - /deploy-prod
  - /order-pizza

# GOOD - Related research tasks
parallel:
  - /research-security {as:sec}
  - /research-performance {as:perf}
return: Synthesize $RESULT[sec] and $RESULT[perf]
```

### ❌ Placeholder Misuse
```yaml
# BAD - Used 3 times
Fix $ARGUMENTS in $ARGUMENTS and verify $ARGUMENTS

# GOOD - Once in dedicated block
<context>
$ARGUMENTS
</context>

Fix the issue in the specified file.
```

### ❌ Excessive Context
```yaml
# BAD - Entire conversation
$TURN[*]

# GOOD - Last 5 relevant messages
$TURN[5]
```

## Resources

- **Pattern Library**: `references/PATTERNS.md` - Reusable workflow patterns
- **Comprehensive Examples**: `references/EXAMPLES.md` - Real-world use cases
- **Syntax Quick Reference**: `references/QUICKREF.md` - Cheat sheet
- **Troubleshooting**: `references/TROUBLESHOOTING.md` - Common issues and solutions

## Installation Check

Verify subtask2 is installed:
```bash
grep "@spoons-and-mirrors/subtask2" ~/.config/opencode/opencode.json
```

If missing, add to `~/.config/opencode/opencode.json`:
```json
{
  "plugins": ["@spoons-and-mirrors/subtask2@latest"]
}
```
