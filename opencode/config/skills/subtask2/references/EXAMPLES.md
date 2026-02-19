# Subtask2 Examples: Comprehensive Use Cases

This document provides practical, real-world examples of using Subtask2 for various development workflows.

## Table of Contents

1. [Multi-Model Comparison (A/B/C Testing)](#1-multi-model-comparison)
2. [Iterative Refinement with Loop](#2-iterative-refinement)
3. [Parallel Research & Analysis](#3-parallel-research)
4. [Complex Workflow Chains](#4-complex-workflows)
5. [Context-Aware Commands](#5-context-aware)
6. [Named Results & Orchestration](#6-named-results)
7. [Inline Ad-hoc Subtasks](#7-inline-subtasks)

---

## 1. Multi-Model Comparison (A/B/C Testing)

Run the same task with different models and compare results.

### Example: Plan Comparison

```yaml
---
description: Multi-model ensemble - 3 models plan in parallel, best ideas unified
model: github-copilot/claude-opus-4.5
subtask: true
parallel: 
  - /plan-gemini
  - /plan-gpt
return:
  - Compare all 3 plans and validate each against the codebase. Pick the best ideas from each and create a unified implementation plan.
  - /review-plan focus on simplicity and correctness
---
Plan the implementation for the following feature:
$ARGUMENTS
```

### Example: Code Review with Multiple Perspectives

```yaml
---
description: Multi-model code review for thorough analysis
subtask: true
parallel:
  - /review-security {model:openai/gpt-4o && as:security-review}
  - /review-performance {model:anthropic/claude-sonnet-4 && as:perf-review}
  - /review-style {as:style-review}
return:
  - Synthesize findings from $RESULT[security-review], $RESULT[perf-review], and $RESULT[style-review]
  - Prioritize issues by severity and create a fix plan
  - /implement-fixes
---
Review this PR: $ARGUMENTS
```

---

## 2. Iterative Refinement with Loop

Repeat a task until quality criteria are met.

### Example: Test Generation Loop

```yaml
---
description: Generate comprehensive tests with quality gate
subtask: true
loop:
  max: 5
  until: "all edge cases covered and tests compile successfully"
return:
  - Run the test suite
  - Fix any failing tests
---
Generate unit tests for $ARGUMENTS covering:
- Happy path scenarios
- Edge cases and boundary conditions
- Error handling paths
```

### Example: Documentation Improvement Loop

```
/improve-docs {loop:3 && until:documentation is clear and complete} update README for the auth module
```

### Example: Bug Fix with Verification

```yaml
---
description: Fix bug and verify with tests
subtask: true
return:
  - /fix-bug {loop:5 && until:bug is fixed and all tests pass} $ARGUMENTS
  - /verify-fix run comprehensive test suite
  - Update changelog with fix description
---
Fix the critical bug reported in issue #$ARGUMENTS
```

---

## 3. Parallel Research & Analysis

Gather information from multiple sources simultaneously.

### Example: Security Audit

```yaml
---
description: Comprehensive security assessment
subtask: true
parallel:
  - command: scan-dependencies
    arguments: check for known vulnerabilities
  - command: review-auth
    arguments: audit authentication flow
  - command: check-secrets
    arguments: scan for exposed secrets
return:
  - Compile all security findings into a report
  - Prioritize by risk severity (Critical > High > Medium > Low)
  - Create remediation plan with timelines
---
Perform security audit for $ARGUMENTS
```

### Example: Architecture Research

```yaml
---
description: Research best practices from multiple angles
subtask: true
parallel:
  - /research-docs {as:docs-research} best practices for $ARGUMENTS
  - /research-github {as:github-research} popular implementations of $ARGUMENTS
  - /analyze-codebase {as:codebase-analysis} existing patterns for $ARGUMENTS
return:
  - Synthesize $RESULT[docs-research], $RESULT[github-research], and $RESULT[codebase-analysis]
  - Identify gaps between current implementation and best practices
  - Propose architecture improvements
---
Research microservices patterns for our API gateway
```

---

## 4. Complex Workflow Chains

Orchestrate multi-step development workflows.

### Example: Feature Development Pipeline

```yaml
---
description: Complete feature development workflow
model: github-copilot/claude-opus-4.5
subtask: true
return:
  - Research existing implementation patterns and requirements
  - Design the feature architecture and data models
  - /implement {model:anthropic/claude-sonnet-4 && agent:build} write the code with tests
  - /review {agent:plan} code review and security check
  - /docs update documentation and examples
  - Final verification and sign-off
---
Implement user authentication system with:
- JWT token management
- Refresh token rotation
- Role-based access control
- Session management
```

### Example: PR Review Workflow

```yaml
---
description: Comprehensive PR review process
subtask: true
return:
  - /analyze-pr fetch and understand changes in PR#$ARGUMENTS
  - /review-code check for bugs, security issues, and style violations
  - /check-tests verify test coverage and quality
  - /benchmark measure performance impact
  - "Summarize findings: approve, request changes, or needs discussion"
---
Review pull request #$ARGUMENTS thoroughly
```

---

## 5. Context-Aware Commands

Pass conversation context to maintain continuity.

### Example: Continuing a Discussion

```yaml
---
description: Context-aware code generation
subtask: true
---
Based on our previous discussion:

$TURN[10]

Implement the feature we discussed. Consider all the requirements and constraints mentioned above.
```

### Example: Summarizing Long Conversations

```yaml
---
description: Summarize conversation and extract action items
subtask: true
return:
  - Extract all action items from the summary
  - Create tasks for each action item
  - Prioritize by urgency and dependencies
---
Review this entire conversation and provide:
1. Executive summary of decisions made
2. Key technical approaches agreed upon
3. All action items with owners
4. Open questions needing resolution

$TURN[*]
```

### Example: Referencing Specific Messages

```yaml
---
description: Implement based on specific requirements
subtask: true
---
Implement the feature according to these specific requirements:

Original requirements (message 5):
$TURN[:5]

Clarifications provided (message 8):
$TURN[:8]

Edge cases discussed (message 12):
$TURN[:12]
```

---

## 6. Named Results & Orchestration

Capture and orchestrate multiple subtask outputs.

### Example: Design-Build-Test Pipeline

```yaml
---
description: Design then build with verification
subtask: true
return:
  - /design {as:design-spec} create detailed design for $ARGUMENTS
  - /estimate {as:time-estimate} estimate effort based on $RESULT[design-spec]
  - /implement {as:implementation} build according to $RESULT[design-spec]
  - /test {as:test-results} verify $RESULT[implementation]
  - "Report: Design took X hours, implementation complete, tests: $RESULT[test-results]"
---
Build the payment processing module
```

### Example: Multi-Stage Analysis

```yaml
---
description: Deep analysis with checkpoints
subtask: true
return:
  - /data-collection {as:raw-data} gather metrics for $ARGUMENTS
  - /data-analysis {as:analysis} analyze $RESULT[raw-data]
  - /recommendations {as:recommendations} propose solutions based on $RESULT[analysis]
  - /impact-assessment {as:impact} evaluate impact of $RESULT[recommendations]
  - "Final report incorporating $RESULT[analysis], $RESULT[recommendations], and $RESULT[impact]"
---
Analyze database performance bottlenecks
```

### Example: Parallel Comparison with Decision

```yaml
---
description: Compare approaches and decide
subtask: true
parallel:
  - /approach-a {as:approach-a} implement with ORM
  - /approach-b {as:approach-b} implement with raw SQL
return:
  - /compare {as:comparison} compare $RESULT[approach-a] vs $RESULT[approach-b] on:
    * Performance
    * Maintainability
    * Type safety
    * Migration complexity
  - "Based on $RESULT[comparison], recommend and implement the better approach"
---
Choose implementation strategy for data access layer
```

---

## 7. Inline Ad-hoc Subtasks

Create one-off subtasks without command files.

### Example: Quick Analysis

```
/subtask analyze the performance of this function and suggest optimizations
```

### Example: With Model Override

```
/subtask {model:anthropic/claude-opus-4.5} refactor this complex algorithm for better readability
```

### Example: With Loop

```
/subtask {loop:3 && until:code is clean and follows style guide} refactor this messy code
```

### Example: Inline Chain

```yaml
---
description: Quick workflow
subtask: true
return:
  - /subtask {as:research} research the topic
  - /subtask {as:draft} draft content based on $RESULT[research]
  - /subtask {as:review} review and polish $RESULT[draft]
  - "Final output: $RESULT[review]"
---
Create technical documentation
```

### Example: Complex Inline with All Features

```
/subtask {model:openai/gpt-4o && agent:build && loop:5 && until:all requirements met && return:/test || /validate} implement user dashboard with real-time updates, charts, and filtering
```

---

## Advanced Patterns

### Pattern: Retry with Escalation

```yaml
---
description: Retry with model escalation on failure
subtask: true
return:
  - /attempt-fix {loop:3 && until:tests pass} try simple fix
  - /escalated-fix {model:anthropic/claude-opus-4.5 && loop:3 && until:tests pass} escalate if needed
  - /manual-review if still failing after escalation
---
Fix the flaky integration test
```

### Pattern: Fan-Out/Fan-In

```yaml
---
description: Process components in parallel, then integrate
subtask: true
parallel:
  - /implement-component {as:comp-a} Component A
  - /implement-component {as:comp-b} Component B
  - /implement-component {as:comp-c} Component C
return:
  - /integrate integrate $RESULT[comp-a], $RESULT[comp-b], and $RESULT[comp-c]
  - /test-integration verify the integrated system
  - /optimize optimize the integration points
---
Build the three-tier architecture
```

### Pattern: Tournament Selection

```yaml
---
description: Multiple solutions compete
subtask: true
parallel:
  - /solve {model:anthropic/claude-sonnet-4 && as:solution-a} implement approach A
  - /solve {model:openai/gpt-4o && as:solution-b} implement approach B
  - /solve {model:github-copilot/claude-sonnet-4.5 && as:solution-c} implement approach C
return:
  - /benchmark compare $RESULT[solution-a], $RESULT[solution-b], $RESULT[solution-c]
  - "Select winner and explain why it outperformed others"
  - Deploy the winning solution
---
Optimize the database query
```

---

## Best Practices Summary

1. **Always use descriptive names** for captured results
2. **Set max iterations** for loops to prevent infinite loops
3. **Write clear conditions** for loop termination
4. **Limit parallel tasks** to 3-5 for manageable complexity
5. **Chain logically** - each step should build on previous
6. **Use context sparingly** - only pass what's needed
7. **Test your commands** - verify they work as expected
