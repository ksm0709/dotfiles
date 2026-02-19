---
description: Iterative document/code improvement through expert critique, defense, aggregation, and update loop
type: subtask
loop:
  max: 10
  until: "all critics return grade S AND aggregation produces actionable plan AND update executes successfully"
return:
  # STAGE 1: CRITIC - 5 Expert Critics in Parallel
  - parallel:
      - /critic-loop:architecture {as:critic-arch} evaluate architectural soundness of $ARGUMENTS
      - /critic-loop:security {as:critic-sec} evaluate security posture of $ARGUMENTS
      - /critic-loop:quality {as:critic-quality} evaluate code/documentation quality of $ARGUMENTS
      - /critic-loop:performance {as:critic-perf} evaluate performance characteristics of $ARGUMENTS
      - /critic-loop:maintainability {as:critic-maint} evaluate long-term maintainability of $ARGUMENTS
    return: "Critic stage complete - all 5 experts evaluated"
  
  # STAGE 2: DEFEND - 2 Validators in Parallel
  - parallel:
      - /critic-loop:defender-logic {as:defend-logic} cross-check critic findings for logical consistency using $RESULT[critic-arch] $RESULT[critic-sec] $RESULT[critic-quality] $RESULT[critic-perf] $RESULT[critic-maint]
      - /critic-loop:defender-facts {as:defend-facts} verify critic claims against document $ARGUMENTS using $RESULT[critic-arch] $RESULT[critic-sec] $RESULT[critic-quality] $RESULT[critic-perf] $RESULT[critic-maint]
    return: "Defend stage complete - critic assessments validated"
  
  # STAGE 3: AGGREGATE - Synthesize all opinions
  - /critic-loop:aggregator {as:aggregation-plan} synthesize $RESULT[critic-arch] $RESULT[critic-sec] $RESULT[critic-quality] $RESULT[critic-perf] $RESULT[critic-maint] $RESULT[defend-logic] $RESULT[defend-facts] into actionable update plan
  
  # STAGE 4: UPDATE - Execute improvements
  - /critic-loop:updater {as:update-result} execute $RESULT[aggregation-plan] on target document $ARGUMENTS
  
  # STAGE 5: EXIT CHECK - Verify completion criteria
  - "Check exit conditions: All critics graded S? Aggregation complete? Update successful?"
  - "If all S grades achieved: Exit loop and report success"
  - "If any non-S grades: Continue to next iteration (loop counter +1)"
---

## Summary

Line 1: You MUST iteratively improve documents/code through a 4-stage critique loop until perfection.
Line 2: You SHOULD orchestrate 5 expert critics in parallel, 2 defenders in parallel, then 1 aggregator and 1 updater in sequence.
Line 3: You MUST continue looping until ALL critics return grade S AND aggregation produces valid plan AND update executes successfully.

## Context

<target_document>
$ARGUMENTS
</target_document>

<document_type>
!`file "$ARGUMENTS" 2>/dev/null | head -1 || echo "unknown"`
</document_type>

## Objective

Execute the CRITIC-LOOP workflow to continuously improve the target document until all quality criteria are met.

### Stage 1: CRITIC - Expert Assessment (Parallel)

Spawn 5 specialized critics in parallel within a `parallel:` block to evaluate the document from different perspectives simultaneously:

**Critic Personas:**

1. **Architecture Expert** (`/critic-architecture`)
   - **Scope**: System design, modularity, separation of concerns, design patterns
   - **Persona**: Senior software architect with 15+ years experience
   - **Grading Criteria**:
     - S: Flawless architecture, exemplary patterns, perfect modularity
     - A: Solid architecture, minor improvements possible
     - B: Adequate but has structural issues
     - C: Poor architecture, significant refactoring needed
   - **Output Format**:
     ```
     Grade: [S/A/B/C]
     Strengths: [list]
     Issues: [list with severity]
     Recommendations: [specific actionable items]
     ```

2. **Security Expert** (`/critic-security`)
   - **Scope**: Vulnerabilities, input validation, authentication, data protection
   - **Persona**: Security engineer with penetration testing background
   - **Grading Criteria**:
     - S: No security issues, defense in depth, best practices followed
     - A: Minor security concerns, easily addressable
     - B: Moderate security risks requiring attention
     - C: Critical vulnerabilities present
   - **Output Format**:
     ```
     Grade: [S/A/B/C]
     Risk Assessment: [Low/Medium/High/Critical]
     Vulnerabilities: [list with CVE severity]
     Security Gaps: [list]
     Remediation: [specific fixes]
     ```

3. **Code Quality Expert** (`/critic-quality`)
   - **Scope**: Readability, naming conventions, comments, complexity, test coverage
   - **Persona**: Senior developer focused on clean code practices
   - **Grading Criteria**:
     - S: Exceptional quality, self-documenting, perfect test coverage
     - A: Good quality, minor style issues
     - B: Acceptable but inconsistent, needs cleanup
     - C: Poor quality, hard to maintain
   - **Output Format**:
     ```
     Grade: [S/A/B/C]
     Metrics: [complexity score, coverage %, etc.]
     Style Issues: [list]
     Complexity Concerns: [list]
     Testing Gaps: [list]
     Improvements: [specific recommendations]
     ```

4. **Performance Expert** (`/critic-performance`)
   - **Scope**: Algorithmic efficiency, resource usage, scalability, bottlenecks
   - **Persona**: Performance engineer specializing in optimization
   - **Grading Criteria**:
     - S: Optimal performance, no bottlenecks, excellent scalability
     - A: Good performance, minor optimizations possible
     - B: Acceptable but has noticeable inefficiencies
     - C: Poor performance, major optimization needed
   - **Output Format**:
     ```
     Grade: [S/A/B/C]
     Performance Profile: [CPU/Memory/I/O analysis]
     Bottlenecks: [list with impact assessment]
     Scalability Concerns: [list]
     Optimizations: [specific recommendations]
     ```

5. **Maintainability Expert** (`/critic-maintainability`)
   - **Scope**: Documentation, onboarding ease, dependency management, technical debt
   - **Persona**: Engineering manager focused on long-term project health
   - **Grading Criteria**:
     - S: Perfect documentation, zero tech debt, easy to extend
     - A: Good maintainability, minor documentation gaps
     - B: Moderate tech debt, documentation incomplete
     - C: High tech debt, poor documentation, hard to maintain
   - **Output Format**:
     ```
     Grade: [S/A/B/C]
     Documentation Quality: [assessment]
     Technical Debt: [list with estimated effort]
     Onboarding Complexity: [assessment]
     Dependency Health: [assessment]
     Long-term Recommendations: [list]
     ```

### Stage 2: DEFEND - Validation & Cross-Check (Parallel)

Spawn 2 defenders in parallel within a `parallel:` block to validate critic assessments simultaneously. Both defenders receive all critic results ($RESULT[critic-*]):

**Defender Personas:**

1. **Logic Validator** (`/defender-logic`)
   - **Scope**: Check critic opinions for logical consistency and contradictions
   - **Persona**: Analytical philosopher with software engineering background
   - **Responsibilities**:
     - Identify contradictions between critic opinions
     - Check for logical fallacies in critic reasoning
     - Validate that critic recommendations are internally consistent
     - Flag critics who may be over-critical or under-critical
   - **Output Format**:
     ```
     Validation Report:
     - Contradictions Found: [list]
     - Logical Issues: [list]
     - Over-critical Assessments: [list]
     - Under-critical Assessments: [list]
     - Recommendations: [which critics to trust more/less]
     ```

2. **Fact Checker** (`/defender-facts`)
   - **Scope**: Verify critic claims against actual document content
   - **Persona**: Technical fact-checker with attention to detail
   - **Responsibilities**:
     - Verify that critic claims are supported by document evidence
     - Identify false positives (issues that don't exist)
     - Identify false negatives (missed issues)
     - Check that critic examples and line references are accurate
   - **Output Format**:
     ```
     Fact Check Report:
     - Verified Claims: [list]
     - False Positives: [list with explanation]
     - False Negatives: [list with explanation]
     - Inaccurate References: [list]
     - Trust Score per Critic: [percentage]
     ```

### Stage 3: AGGREGATE - Synthesis & Planning (Sequential)

Execute 1 aggregator sequentially (after defend stage completes) to synthesize all critic and defender outputs into a unified action plan:

**Aggregator Specifications** (`/aggregator`)
- **Scope**: Conflict resolution, priority assignment, action item creation
- **Persona**: Technical project manager with engineering background
- **Responsibilities**:
  1. **Conflict Resolution**: When critics disagree, choose the valid side or find compromise
  2. **Priority Assignment**: Rank issues by severity, impact, and effort
  3. **Action Item Creation**: Convert recommendations into concrete, executable tasks
  4. **Dependency Mapping**: Identify which fixes must happen in sequence
  5. **Effort Estimation**: Provide rough time estimates for each action item
- **Input**: All critic grades and feedback + defender validation reports
- **Output Format**:
  ```
  Aggregation Plan:
  
  ## Executive Summary
  - Overall Grade: [weighted average of critic grades]
  - Critical Issues: [count]
  - High Priority: [count]
  - Medium Priority: [count]
  - Low Priority: [count]
  
  ## Conflicts Resolved
  - [Description of each conflict and resolution]
  
  ## Action Items (Priority Order)
  1. [CRITICAL] [Description] - Effort: [time] - Owner: [critic source]
  2. [HIGH] [Description] - Effort: [time] - Owner: [critic source]
  ...
  
  ## Implementation Order
  Phase 1: [items that can be done in parallel]
  Phase 2: [items dependent on Phase 1]
  ...
  
  ## Success Criteria
  - [List of verifiable conditions that indicate completion]
  ```

### Stage 4: UPDATE - Execution (Sequential)

Execute 1 updater sequentially (after aggregation completes) to implement the aggregation plan:

**Updater Specifications** (`/updater`)
- **Scope**: Document/code modification based on aggregation plan
- **Persona**: Senior developer with broad expertise across all domains
- **Responsibilities**:
  1. Execute all action items from the aggregation plan
  2. Apply fixes in the specified implementation order
  3. Maintain document integrity and consistency
  4. Verify each fix doesn't break existing functionality
  5. Update documentation to reflect changes
- **Input**: Aggregation plan with action items
- **Output Format**:
  ```
  Update Report:
  
  ## Changes Made
  - [List of specific modifications with line references]
  
  ## Action Items Completed
  - [X] [Item 1]
  - [X] [Item 2]
  - [ ] [Item 3 - reason if skipped]
  
  ## Verification Results
  - Syntax Check: [Pass/Fail]
  - Tests Status: [Pass/Fail/Not Applicable]
  - Documentation Updated: [Yes/No]
  
  ## Issues Encountered
  - [Any blockers or unexpected issues]
  
  ## Update Status: [Complete/Partial/Failed]
  ```

### Stage 5: EXIT CHECK - Loop Control

After each iteration, verify exit conditions:

**Exit Criteria** (ALL must be true):
1. ✅ All 5 critics return grade **S** (Superior/Excellent)
2. ✅ Aggregation produces valid, actionable plan
3. ✅ Update executes successfully (status: Complete)

**Continue Criteria** (ANY is true):
1. ❌ Any critic returns grade A, B, or C
2. ❌ Aggregation plan has unresolved critical issues
3. ❌ Update fails or is only partial

**Loop Behavior**:
- If exit criteria met: Report final quality metrics and terminate
- If continue criteria met: Increment iteration counter and restart CRITIC stage
- If max iterations (10) reached: Report best effort results and terminate with warning

## Variable/Result Passing Strategy

### Capture Points (using `{as:name}`)

| Stage | Subtask | Capture Name | Content |
|-------|---------|--------------|---------|
| CRITIC | Architecture Critic | `critic-arch` | Grade + architectural feedback |
| CRITIC | Security Critic | `critic-sec` | Grade + security assessment |
| CRITIC | Quality Critic | `critic-quality` | Grade + quality metrics |
| CRITIC | Performance Critic | `critic-perf` | Grade + performance analysis |
| CRITIC | Maintainability Critic | `critic-maint` | Grade + maintainability report |
| DEFEND | Logic Validator | `defend-logic` | Contradictions + trust assessment |
| DEFEND | Fact Checker | `defend-facts` | False positives/negatives report |
| AGGREGATE | Aggregator | `aggregation-plan` | Unified action plan with priorities |
| UPDATE | Updater | `update-result` | Execution report + status |

### Reference Points (using `$RESULT[name]`)

| Stage | References | Purpose |
|-------|------------|---------|
| DEFEND | `$RESULT[critic-*]` | Validate all critic outputs |
| AGGREGATE | `$RESULT[critic-*]`, `$RESULT[defend-*]` | Synthesize all assessments |
| UPDATE | `$RESULT[aggregation-plan]` | Execute the action plan |
| EXIT CHECK | `$RESULT[critic-*]`, `$RESULT[update-result]` | Verify completion criteria |

### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         CRITIC STAGE                            │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐  │
│  │critic-   │ │critic-   │ │critic-   │ │critic-   │ │critic- │  │
│  │arch      │ │sec       │ │quality   │ │perf      │ │maint   │  │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ └───┬────┘  │
│       │            │            │            │           │       │
│       └────────────┴────────────┴────────────┴───────────┘       │
│                          │                                       │
│                          ▼                                       │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              $RESULT[critic-*] → DEFEND STAGE            │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        DEFEND STAGE                             │
│  ┌──────────────────┐          ┌──────────────────┐              │
│  │defender-logic    │          │defender-facts    │              │
│  │                  │          │                  │              │
│  └────────┬─────────┘          └────────┬─────────┘              │
│           │                            │                       │
│           └────────────┬───────────────┘                          │
│                        │                                         │
│                        ▼                                         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │    $RESULT[defend-*] + $RESULT[critic-*] → AGGREGATE    │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      AGGREGATE STAGE                            │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    aggregator                             │   │
│  │  Synthesizes all inputs → Unified Action Plan           │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                          │
│                       ▼                                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         $RESULT[aggregation-plan] → UPDATE              │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        UPDATE STAGE                             │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                      updater                              │   │
│  │  Executes action plan → Modified Document               │   │
│  └────────────────────┬─────────────────────────────────────┘   │
│                       │                                          │
│                       ▼                                          │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         $RESULT[update-result] → EXIT CHECK               │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       EXIT CHECK                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  IF all critics = S AND update = Complete:              │   │
│  │     → EXIT LOOP (Success)                                │   │
│  │  ELSE IF iteration < max:                                 │   │
│  │     → RESTART CRITIC STAGE (Continue)                    │   │
│  │  ELSE:                                                    │   │
│  │     → EXIT LOOP (Max iterations reached)                 │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Loop Termination Logic

### Iteration Counter
- **Initial**: 1
- **Increment**: After each UPDATE stage
- **Maximum**: 10 (hard limit to prevent infinite loops)

### Exit Conditions (ALL must be satisfied)

```yaml
exit_conditions:
  critic_grades:
    - critic-arch: "S"
    - critic-sec: "S"
    - critic-quality: "S"
    - critic-perf: "S"
    - critic-maint: "S"
  aggregation_status: "valid_plan_produced"
  update_status: "complete"
```

### Termination Scenarios

1. **Success Exit** (All criteria met before max iterations):
   ```
   ✅ CRITIC-LOOP COMPLETED SUCCESSFULLY
   
   Final Quality Metrics:
   - Architecture: S
   - Security: S
   - Quality: S
   - Performance: S
   - Maintainability: S
   
   Iterations: N/10
   Total Improvements: [count]
   Document Status: PERFECTION ACHIEVED
   ```

2. **Max Iterations Exit** (Criteria not met after 10 iterations):
   ```
   ⚠️ CRITIC-LOOP REACHED MAXIMUM ITERATIONS
   
   Best Effort Results:
   - Architecture: [final grade]
   - Security: [final grade]
   - Quality: [final grade]
   - Performance: [final grade]
   - Maintainability: [final grade]
   
   Iterations: 10/10
   Unresolved Issues: [count]
   Recommendation: Manual review required
   ```

3. **Early Termination** (Critical failure):
   ```
   ❌ CRITIC-LOOP TERMINATED EARLY
   
   Reason: [e.g., document deleted, unrecoverable error]
   Last Successful Stage: [stage name]
   Recovery Actions: [recommendations]
   ```

## Example Usage Scenarios

### Scenario 1: Code Review and Refinement

```bash
/critic-loop src/auth/authentication.ts
```

**Expected Flow**:
1. **Iteration 1**:
   - Critics find: Security (B - missing input validation), Quality (A - minor style issues)
   - Defenders validate: Security concern is valid, Quality issues are minor
   - Aggregation: 2 action items (add validation, fix style)
   - Update: Implements fixes
   - Exit check: Security now A, Quality now S → Continue

2. **Iteration 2**:
   - Critics find: Security (S), Quality (S), Architecture (A - could use strategy pattern)
   - Defenders validate: Architecture suggestion is valid but optional
   - Aggregation: 1 action item (refactor to strategy pattern)
   - Update: Implements refactoring
   - Exit check: All S grades → EXIT SUCCESS

### Scenario 2: Documentation Improvement

```bash
/critic-loop docs/api-reference.md
```

**Expected Flow**:
1. **Iteration 1**:
   - Critics find: Maintainability (C - missing examples), Quality (B - inconsistent formatting)
   - Defenders validate: Both concerns are legitimate
   - Aggregation: 5 action items (add examples, standardize format, etc.)
   - Update: Implements improvements
   - Exit check: Grades improved but not all S → Continue

2. **Iteration 2-3**:
   - Continues refining until all documentation criteria met
   - Exit check: All S grades → EXIT SUCCESS

### Scenario 3: Configuration File Audit

```bash
/critic-loop config/production.yaml
```

**Expected Flow**:
1. **Iteration 1**:
   - Critics find: Security (C - hardcoded secrets), Performance (B - inefficient caching)
   - Defenders validate: Security issue is critical, Performance concern is valid
   - Aggregation: 3 action items (move secrets to env vars, optimize cache, add validation)
   - Update: Implements fixes
   - Exit check: Security now A, Performance now A → Continue

2. **Iteration 2**:
   - Critics find: Security (S), Performance (S), all others S
   - Exit check: All S grades → EXIT SUCCESS

### Scenario 4: Architecture Decision Record (ADR) Review

```bash
/critic-loop docs/adr/0001-database-selection.md
```

**Expected Flow**:
1. **Iteration 1**:
   - Critics find: Architecture (A - good but missing alternatives), Maintainability (B - incomplete context)
   - Defenders validate: Architecture concern is minor, Maintainability concern is valid
   - Aggregation: 2 action items (add alternatives section, expand context)
   - Update: Implements improvements
   - Exit check: Architecture now S, Maintainability now A → Continue

2. **Iteration 2**:
   - Critics find: All S grades
   - Exit check: EXIT SUCCESS

## Command Reference

### Subtask Definitions (Namespace: `critic-loop:`)

All subcommands are accessed via the `critic-loop:` namespace:
- Main: `/critic-loop <document>`
- Sub: `/critic-loop:<subcommand> <document>`

#### /critic-loop:architecture
```yaml
---
description: Architecture expert critique
agent: plan
---
You are a Senior Software Architect with 15+ years of experience.

Evaluate the architecture of the provided document/code:

<context>
$ARGUMENTS
</context>

Grade on S/A/B/C scale:
- S: Flawless architecture, exemplary patterns, perfect modularity
- A: Solid architecture, minor improvements possible
- B: Adequate but has structural issues
- C: Poor architecture, significant refactoring needed

Output format:
Grade: [S/A/B/C]
Strengths: [list]
Issues: [list with severity]
Recommendations: [specific actionable items]
```

#### /critic-loop:security
```yaml
---
description: Security expert critique
agent: plan
---
You are a Security Engineer with penetration testing background.

Evaluate the security posture of the provided document/code:

<context>
$ARGUMENTS
</context>

Grade on S/A/B/C scale:
- S: No security issues, defense in depth, best practices followed
- A: Minor security concerns, easily addressable
- B: Moderate security risks requiring attention
- C: Critical vulnerabilities present

Output format:
Grade: [S/A/B/C]
Risk Assessment: [Low/Medium/High/Critical]
Vulnerabilities: [list with severity]
Security Gaps: [list]
Remediation: [specific fixes]
```

#### /critic-loop:quality
```yaml
---
description: Code quality expert critique
agent: plan
---
You are a Senior Developer focused on clean code practices.

Evaluate the code/documentation quality:

<context>
$ARGUMENTS
</context>

Grade on S/A/B/C scale:
- S: Exceptional quality, self-documenting, perfect test coverage
- A: Good quality, minor style issues
- B: Acceptable but inconsistent, needs cleanup
- C: Poor quality, hard to maintain

Output format:
Grade: [S/A/B/C]
Metrics: [complexity score, coverage %, etc.]
Style Issues: [list]
Complexity Concerns: [list]
Testing Gaps: [list]
Improvements: [specific recommendations]
```

#### /critic-loop:performance
```yaml
---
description: Performance expert critique
agent: plan
---
You are a Performance Engineer specializing in optimization.

Evaluate the performance characteristics:

<context>
$ARGUMENTS
</context>

Grade on S/A/B/C scale:
- S: Optimal performance, no bottlenecks, excellent scalability
- A: Good performance, minor optimizations possible
- B: Acceptable but has noticeable inefficiencies
- C: Poor performance, major optimization needed

Output format:
Grade: [S/A/B/C]
Performance Profile: [CPU/Memory/I/O analysis]
Bottlenecks: [list with impact assessment]
Scalability Concerns: [list]
Optimizations: [specific recommendations]
```

#### /critic-loop:maintainability
```yaml
---
description: Maintainability expert critique
agent: plan
---
You are an Engineering Manager focused on long-term project health.

Evaluate the maintainability aspects:

<context>
$ARGUMENTS
</context>

Grade on S/A/B/C scale:
- S: Perfect documentation, zero tech debt, easy to extend
- A: Good maintainability, minor documentation gaps
- B: Moderate tech debt, documentation incomplete
- C: High tech debt, poor documentation, hard to maintain

Output format:
Grade: [S/A/B/C]
Documentation Quality: [assessment]
Technical Debt: [list with estimated effort]
Onboarding Complexity: [assessment]
Dependency Health: [assessment]
Long-term Recommendations: [list]
```

#### /critic-loop:defender-logic
```yaml
---
description: Logic validator for critic assessments
agent: plan
---
You are an Analytical Philosopher with software engineering background.

Validate the logical consistency of critic opinions:

<context>
All critic assessments: $RESULT[critic-arch], $RESULT[critic-sec], $RESULT[critic-quality], $RESULT[critic-perf], $RESULT[critic-maint]
</context>

Check for:
1. Contradictions between critic opinions
2. Logical fallacies in critic reasoning
3. Internal consistency of recommendations
4. Over-critical or under-critical assessments

Output format:
Validation Report:
- Contradictions Found: [list]
- Logical Issues: [list]
- Over-critical Assessments: [list]
- Under-critical Assessments: [list]
- Recommendations: [which critics to trust more/less]
```

#### /critic-loop:defender-facts
```yaml
---
description: Fact checker for critic claims
agent: plan
---
You are a Technical Fact-Checker with attention to detail.

Verify critic claims against the actual document:

<context>
Original document: $ARGUMENTS
Critic claims: $RESULT[critic-arch], $RESULT[critic-sec], $RESULT[critic-quality], $RESULT[critic-perf], $RESULT[critic-maint]
</context>

Check for:
1. Claims supported by document evidence
2. False positives (issues that don't exist)
3. False negatives (missed issues)
4. Accurate line references and examples

Output format:
Fact Check Report:
- Verified Claims: [list]
- False Positives: [list with explanation]
- False Negatives: [list with explanation]
- Inaccurate References: [list]
- Trust Score per Critic: [percentage]
```

#### /critic-loop:aggregator
```yaml
---
description: Synthesize critic and defender outputs into action plan
agent: build
---
You are a Technical Project Manager with engineering background.

Synthesize all assessments into a unified action plan:

<context>
Critic assessments: $RESULT[critic-arch], $RESULT[critic-sec], $RESULT[critic-quality], $RESULT[critic-perf], $RESULT[critic-maint]
Defender validations: $RESULT[defend-logic], $RESULT[defend-facts]
</context>

Responsibilities:
1. Resolve conflicts between critics
2. Assign priorities (Critical > High > Medium > Low)
3. Create concrete, executable action items
4. Map dependencies between fixes
5. Estimate effort for each item

Output format:
Aggregation Plan:

## Executive Summary
- Overall Grade: [weighted average]
- Critical Issues: [count]
- High Priority: [count]
- Medium Priority: [count]
- Low Priority: [count]

## Conflicts Resolved
- [Description and resolution]

## Action Items (Priority Order)
1. [CRITICAL] [Description] - Effort: [time] - Source: [critic]
2. [HIGH] [Description] - Effort: [time] - Source: [critic]
...

## Implementation Order
Phase 1: [parallel items]
Phase 2: [dependent items]
...

## Success Criteria
- [Verifiable conditions]
```

#### /critic-loop:updater
```yaml
---
description: Execute aggregation plan on target document
agent: build
---
You are a Senior Developer with broad domain expertise.

Execute the aggregation plan:

<context>
Target document: $ARGUMENTS
Action plan: $RESULT[aggregation-plan]
</context>

Responsibilities:
1. Execute all action items from the plan
2. Apply fixes in specified order
3. Maintain document integrity
4. Verify fixes don't break functionality
5. Update documentation

Output format:
Update Report:

## Changes Made
- [List with line references]

## Action Items Completed
- [X] [Item 1]
- [X] [Item 2]
- [ ] [Item 3 - reason]

## Verification Results
- Syntax Check: [Pass/Fail]
- Tests Status: [Pass/Fail/N/A]
- Documentation Updated: [Yes/No]

## Issues Encountered
- [Blockers or issues]

## Update Status: [Complete/Partial/Failed]
```

## Best Practices

1. **Start with clear documents**: The loop works best when the initial document is complete enough to critique
2. **Monitor iteration count**: If approaching max iterations, consider manual intervention
3. **Review defender outputs**: Pay attention to false positives to avoid unnecessary work
4. **Trust the aggregation**: The aggregator resolves conflicts; trust its prioritization
5. **Verify updates**: Always check that updates don't introduce new issues

## Troubleshooting

### Issue: Critics consistently disagree
**Solution**: Check defender-logic output for systematic biases. Consider adjusting critic personas or adding a tie-breaker critic.

### Issue: Loop never reaches all S grades
**Solution**: Review aggregation plan for completeness. Some issues may require domain expertise beyond the critics' scope.

### Issue: Updates fail repeatedly
**Solution**: Check that action items are specific and executable. The aggregator may be producing vague recommendations.

### Issue: False positives waste iterations
**Solution**: Strengthen defender-facts validation. Consider adding stricter evidence requirements for critics.
