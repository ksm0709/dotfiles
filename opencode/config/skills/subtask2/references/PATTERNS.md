# Subtask Design Patterns

Reusable workflow patterns for creating subtask2 commands.

## Pattern Selection Guide

Use this guide to select the right pattern during command creation:

| If User Needs... | Use Pattern | Key Features |
|------------------|-------------|--------------|
| Multiple perspectives on same task | **Multi-Model Ensemble** | `parallel` with different models |
| Quality gate before proceeding | **Iterative Refinement** | `loop` with clear criteria |
| Gather info from multiple sources | **Parallel Research** | `parallel` with synthesis |
| Multi-step development workflow | **Pipeline Chain** | Sequential `return` array |
| Continue from previous discussion | **Context-Aware** | `$TURN[n]` for context |
| Compare and select best option | **Tournament** | Parallel + synthesis + decision |

---

## Pattern 1: Multi-Model Ensemble

**When to use:** Need multiple perspectives on same task to improve confidence/quality.

**Template:**
```yaml
---
description: Multi-model [task] for [goal]
model: [primary-model]
subtask: true
parallel:
  - /[command] {model:[model-1] && as:[result-1]}
  - /[command] {model:[model-2] && as:[result-2]}
  - /[command] {model:[model-3] && as:[result-3]}
return:
  - Compare $RESULT[result-1], $RESULT[result-2], $RESULT[result-3]
  - Identify best ideas from each approach
  - Synthesize into unified [output]
  - /validate verify the synthesis
---
[Task description]: $ARGUMENTS
```

**Variations:**

### A/B Testing (2 models)
```yaml
parallel:
  - /plan {model:anthropic/claude-sonnet-4 && as:claude}
  - /plan {model:openai/gpt-4o && as:gpt}
return:
  - Compare $RESULT[claude] vs $RESULT[gpt]
  - Select better approach and explain why
```

### Ensemble with Analysis
```yaml
parallel:
  - /solve {model:claude-opus-4.5 && as:opus}
  - /solve {model:claude-sonnet-4 && as:sonnet}
  - /solve {model:gpt-4o && as:gpt}
return:
  - /deep-analysis {as:analysis} compare all three solutions
  - "Select winner based on: correctness, efficiency, maintainability"
  - Implement winning solution
```

**When NOT to use:**
- Simple tasks where one model is sufficient
- When consistency matters more than diversity
- Time-critical tasks (parallel saves time, but adds overhead)

---

## Pattern 2: Iterative Refinement (Loop)

**When to use:** Task needs repeated attempts until quality criteria met.

**Template:**
```yaml
---
description: [Task] with quality gate
subtask: true
loop:
  max: [5-10]
  until: "[clear, verifiable success condition]"
return:
  - Report final quality metrics
  - Proceed to next phase if successful
---
[Task]: $ARGUMENTS

Quality criteria:
- [Criterion 1]
- [Criterion 2]
- [Criterion 3]
```

**Variations:**

### Retry Until Success
```yaml
loop:
  max: 5
  until: "all tests pass with >90% coverage"
```

### Refinement with Feedback
```yaml
loop:
  max: 3
  until: "documentation is clear, complete, and follows style guide"
return:
  - /review-docs critique current version
  - Apply feedback and continue loop if needed
```

**Best practices:**
- Condition must be objectively verifiable (tests pass, coverage > X%)
- Always set `max` to prevent infinite loops
- Include quality criteria in subtask body
- Use return chain for post-loop actions

**When NOT to use:**
- One-shot tasks that don't benefit from iteration
- Tasks where failure should stop immediately (not retry)
- Creative tasks without clear quality metrics

---

## Pattern 3: Parallel Research

**When to use:** Need to gather information from multiple independent sources.

**Template:**
```yaml
---
description: Comprehensive [research topic] from multiple angles
subtask: true
parallel:
  - command: [researcher-1]
    arguments: [specific aspect 1]
  - command: [researcher-2]
    arguments: [specific aspect 2]
  - command: [researcher-3]
    arguments: [specific aspect 3]
return:
  - Synthesize findings into coherent summary
  - Identify conflicts or gaps between sources
  - Create action plan based on research
---
Research: $ARGUMENTS
```

**Variations:**

### Multi-Source Research
```yaml
parallel:
  - /research-docs {as:docs} best practices for $ARGUMENTS
  - /research-github {as:github} popular $ARGUMENTS implementations
  - /analyze-codebase {as:internal} existing $ARGUMENTS patterns
return:
  - Synthesize $RESULT[docs], $RESULT[github], $RESULT[internal]
  - Create gap analysis and recommendations
```

### Security Audit
```yaml
parallel:
  - /scan-dependencies {as:deps} check for known vulnerabilities
  - /review-auth {as:auth} audit authentication flow
  - /check-secrets {as:secrets} scan for exposed secrets
return:
  - Compile all findings from $RESULT[deps], $RESULT[auth], $RESULT[secrets]
  - Prioritize by risk severity
  - Create remediation plan with timelines
```

**Best practices:**
- Parallel tasks should be truly independent
- Each should produce structured output for synthesis
- Use `as:name` to capture results
- Return chain should synthesize, not just list

**When NOT to use:**
- Tasks that depend on each other (use sequential return chain)
- When total task count > 5-7 (coordination overhead)
- When real-time collaboration needed (parallel is async)

---

## Pattern 4: Pipeline Chain (Sequential)

**When to use:** Multi-step workflow where each step depends on previous.

**Template:**
```yaml
---
description: [N]-step [workflow name] pipeline
subtask: true
return:
  - /[step1] {as:step1} [description]
  - /[step2] using $RESULT[step1] {as:step2}
  - /[step3] using $RESULT[step2] {as:step3}
  - /[final] synthesize all results
---
[Workflow trigger]: $ARGUMENTS
```

**Variations:**

### Research → Plan → Implement → Test
```yaml
return:
  - /research {as:research} gather requirements for $ARGUMENTS
  - /plan using $RESULT[research] {as:plan} create detailed design
  - /implement using $RESULT[plan] {as:impl} write code with tests
  - /verify using $RESULT[impl] run full test suite
  - Report success metrics
```

### PR Review Pipeline
```yaml
return:
  - /analyze-pr {as:analysis} fetch and understand PR#$ARGUMENTS
  - /review-code using $RESULT[analysis] {as:review} check for issues
  - /check-tests using $RESULT[review] verify test coverage
  - /decision synthesize $RESULT[analysis], $RESULT[review], $RESULT[tests]
  - Recommend: approve, request changes, or needs discussion
```

**Best practices:**
- Each step should have clear input/output contract
- Use `{as:name}` to capture intermediate results
- Keep chain to 3-5 steps (longer chains get unwieldy)
- Consider parallelizing independent branches

**When NOT to use:**
- When steps can be parallelized (use `parallel`)
- For simple 1-2 step tasks (overhead not worth it)
- When user needs to see intermediate results immediately

---

## Pattern 5: Context-Aware

**When to use:** Need to continue work based on previous conversation.

**Template:**
```yaml
---
description: Context-aware [task] continuing from discussion
subtask: true
---
Based on our previous discussion:

$[TURN[n]]

[Task]: $ARGUMENTS

Consider all requirements and constraints mentioned above.
```

**Variations:**

### Continue Discussion
```yaml
---
$TURN[10]

Continue implementing based on the above context.
Focus on: $ARGUMENTS
```

### Summarize and Extract
```yaml
---
$TURN[*]

Provide:
1. Executive summary of decisions made
2. Key technical approaches agreed upon
3. All action items with owners
4. Open questions needing resolution
```

### Reference Specific Messages
```yaml
---
Requirements (msg 5): $TURN[:5]
Clarifications (msg 8): $TURN[:8]
Edge cases (msg 12): $TURN[:12]

Implement: $ARGUMENTS
```

**Best practices:**
- Use `$TURN[n]` not `$TURN[*]` unless truly needed (token efficiency)
- Include specific message references for clarity
- Summarize context in subtask body for better results

**When NOT to use:**
- For isolated tasks without conversation history
- When context is provided via files/arguments instead

---

## Pattern 6: Tournament Selection

**When to use:** Multiple solutions compete, best one selected.

**Template:**
```yaml
---
description: Multiple solutions compete for best approach
subtask: true
parallel:
  - /[solve] {model:[model-1] && as:sol-a} [approach-1]
  - /[solve] {model:[model-2] && as:sol-b} [approach-2]
  - /[solve] {model:[model-3] && as:sol-c} [approach-3]
return:
  - /benchmark compare $RESULT[sol-a], $RESULT[sol-b], $RESULT[sol-c]
  - Evaluate on: [criteria-1], [criteria-2], [criteria-3]
  - Select winner and explain why it outperforms
  - Deploy winning solution
---
Solve: $ARGUMENTS
```

**Variations:**

### Algorithm Comparison
```yaml
parallel:
  - /implement {as:algo-a} approach A (recursive)
  - /implement {as:algo-b} approach B (iterative)
  - /implement {as:algo-c} approach C (hybrid)
return:
  - /benchmark performance and memory usage
  - Select best algorithm
```

### Architecture Comparison
```yaml
parallel:
  - /design {as:arch-monolith} monolithic approach
  - /design {as:arch-microservices} microservices approach
  - /design {as:arch-serverless} serverless approach
return:
  - /evaluate scalability, cost, complexity
  - Recommend best fit for our constraints
```

**Best practices:**
- Define evaluation criteria upfront
- Use consistent benchmarking/metrics
- Document why winner was selected

---

## Pattern 7: Router Chain

**When to use:** Different request types need different specialized handling.

**Template:**
```yaml
---
description: Route to specialized handler based on input type
subtask: true
return:
  - /classify {as:type} classify input: $ARGUMENTS
  - Route based on $RESULT[type]:
      - if "bug": /handle-bug
      - if "feature": /handle-feature
      - if "docs": /handle-docs
---
Process request: $ARGUMENTS
```

**Best practices:**
- Classification should be reliable
- Handlers should have clear boundaries
- Consider default/fallback handler

---

## Pattern 8: Orchestrator-Workers

**When to use:** Complex task needs coordination of multiple specialists.

**Template:**
```yaml
---
description: Orchestrate specialists for comprehensive [task]
subtask: true
parallel:
  - /specialist-security {as:sec} security review
  - /specialist-performance {as:perf} performance analysis
  - /specialist-maintainability {as:maint} code quality review
return:
  - /orchestrator synthesize $RESULT[sec], $RESULT[perf], $RESULT[maint]
  - Create unified report with prioritized recommendations
  - /implement-high-priority execute critical fixes
---
Comprehensive review of: $ARGUMENTS
```

**Best practices:**
- Workers should be truly specialized
- Orchestrator should resolve conflicts
- Consider using different agents for each specialist

---

## Pattern 9: Checkpoint & Resume

**When to use:** Long workflow needs save points for resumption.

**Template:**
```yaml
---
description: [Workflow] with checkpoints
subtask: true
return:
  - /phase1 {as:checkpoint1} complete phase 1
  - "Checkpoint saved: $RESULT[checkpoint1]"
  - /phase2 {as:checkpoint2} complete phase 2
  - "Checkpoint saved: $RESULT[checkpoint2]"
  - /phase3 complete final phase
---
Execute: $ARGUMENTS
```

**Best practices:**
- Save intermediate state as named results
- Allow resumption from any checkpoint
- Document checkpoint format

---

## Combining Patterns

Most real commands combine multiple patterns:

### Example: Complex Feature Development
```yaml
---
description: Full feature development with quality gates
subtask: true
return:
  # Phase 1: Research (Parallel sources)
  - parallel:
      - /research-docs {as:docs}
      - /research-codebase {as:codebase}
    return: Synthesize $RESULT[docs] and $RESULT[codebase]
  
  # Phase 2: Design (Multi-model ensemble)
  - parallel:
      - /design {model:opus && as:design-a}
      - /design {model:sonnet && as:design-b}
    return: Compare and select best from $RESULT[design-a] and $RESULT[design-b]
  
  # Phase 3: Implement with quality gate (Loop)
  - /implement {loop:5 && until:tests pass && as:impl}
  
  # Phase 4: Verify
  - /deploy-staging test $RESULT[impl]
---
Build feature: $ARGUMENTS
```

**Guidelines for combining:**
- Flatten where possible (avoid deep nesting)
- Use named results to pass between patterns
- Keep overall workflow to < 10 steps
- Document pattern transitions in comments
