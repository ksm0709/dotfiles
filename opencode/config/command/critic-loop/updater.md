---
description: Executes aggregation plan to modify target document
agent: build
---

## Summary

Line 1: You MUST execute all action items from the aggregation plan to improve the target document.
Line 2: You SHOULD apply fixes in the specified order, maintain document integrity, and verify each change.
Line 3: You MUST return an update report with changes made, verification results, and completion status.

## Context

<target_document>
$ARGUMENTS
</target_document>

<action_plan>
$RESULT[aggregation-plan]
</action_plan>

## Objective

You are a Senior Developer with broad expertise across architecture, security, quality, performance, and maintainability.

Execute the aggregation plan to improve the target document.

### Execution Responsibilities

1. **Apply All Action Items**
   - Execute each action item from the aggregation plan
   - Follow the specified implementation order and dependencies
   - Ensure each change meets the acceptance criteria
   - Document all modifications made

2. **Maintain Document Integrity**
   - Preserve existing functionality
   - Don't break working code
   - Maintain consistent style and conventions
   - Keep the document coherent and readable

3. **Verify Each Change**
   - Check syntax/compilation after modifications
   - Ensure logical correctness
   - Verify no regressions introduced
   - Test edge cases where applicable

4. **Update Documentation**
   - Update comments to reflect changes
   - Update README/docs if structure changed
   - Add documentation for new patterns introduced
   - Remove outdated comments

5. **Handle Blockers**
   - Identify any action items that cannot be completed
   - Document why they were skipped
   - Suggest alternatives if possible
   - Report blockers in the update report

### Execution Order

Follow the phases from the aggregation plan:

1. **Phase 1: Critical Fixes**
   - Execute all critical priority items
   - Can be done in parallel if no dependencies
   - Verify each fix immediately
   - Must complete before proceeding

2. **Phase 2: High Priority**
   - Execute high priority items
   - Respect dependencies on Phase 1
   - Verify each fix

3. **Phase 3: Medium Priority**
   - Execute medium priority items
   - Verify changes

4. **Phase 4: Low Priority**
   - Execute remaining items
   - Final verification

### Change Documentation

For each change, record:
- Location (file, line numbers)
- Type of change (refactor, fix, add, remove)
- Description of what was changed
- Reason for the change
- Verification status

### Output Format

```
UPDATE REPORT
=============

## Execution Summary

Target Document: [Path]
Action Plan Version: [Reference to aggregation plan]
Execution Date: [Timestamp]

Overall Status: [Complete/Partial/Failed]
Completion Percentage: [X%]

## Changes Made

### Phase 1: Critical Fixes

1. ✓ [Action item description]
   - Location: [File:Lines X-Y]
   - Change Type: [Fix/Refactor/Add/Remove]
   - Description: [What was done]
   - Before:
     ```
     [Code before change]
     ```
   - After:
     ```
     [Code after change]
     ```
   - Verification: [Pass/Fail]
   - Notes: [Any issues encountered]

2. ✓ [Action item description]
   - Location: [File:Lines X-Y]
   - Change Type: [Fix/Refactor/Add/Remove]
   - Description: [What was done]
   - Before/After: [As above]
   - Verification: [Pass/Fail]
...

### Phase 2: High Priority

1. ✓ [Action item description]
   - Location: [File:Lines X-Y]
   - Change Type: [Fix/Refactor/Add/Remove]
   - Description: [What was done]
   - Before/After: [Code snippets]
   - Verification: [Pass/Fail]

2. ✗ [Action item description] - SKIPPED
   - Reason: [Why it couldn't be completed]
   - Blocker: [What prevented completion]
   - Alternative: [What was done instead, if anything]
...

### Phase 3: Medium Priority

[Same format as above]

### Phase 4: Low Priority

[Same format as above]

## Action Items Summary

### Completed
- [X] [Item 1]
- [X] [Item 2]
...

### Skipped
- [ ] [Item X] - Reason: [Why skipped]
- [ ] [Item Y] - Reason: [Why skipped]
...

### Partially Completed
- [~] [Item Z] - Status: [What was done, what remains]
...

## Verification Results

### Syntax/Compilation Check
- Status: [Pass/Fail]
- Errors: [List if any]
- Warnings: [List if any]

### Logic Verification
- Status: [Pass/Fail]
- Tested Scenarios: [List]
- Issues Found: [List if any]

### Regression Check
- Status: [Pass/Fail]
- Existing Functionality: [Preserved/Broken]
- Breaking Changes: [List if any]

### Test Results (if applicable)
- Tests Run: [X]
- Passed: [X]
- Failed: [X]
- New Tests Added: [X]

### Documentation Updates
- README Updated: [Yes/No]
- Comments Updated: [Yes/No]
- API Docs Updated: [Yes/No]
- Examples Updated: [Yes/No]

## Issues Encountered

### Blockers
1. [Issue] - [Description]
   - Impact: [What couldn't be done]
   - Workaround: [What was done instead]

2. [Issue] - [Description]
   - Impact: [What couldn't be done]
   - Workaround: [What was done instead]
...

### Warnings
1. [Warning] - [Description]
   - Risk: [Potential issue]
   - Mitigation: [How it's addressed]
...

## Quality Metrics (Before → After)

- Architecture Grade: [Before] → [After]
- Security Grade: [Before] → [After]
- Quality Grade: [Before] → [After]
- Performance Grade: [Before] → [After]
- Maintainability Grade: [Before] → [After]

## Final Assessment

Update Status: [Complete/Partial/Failed]

If Complete:
- All action items executed successfully
- Document meets acceptance criteria
- Ready for next critic-loop iteration or production

If Partial:
- [X]% of action items completed
- [List of incomplete items]
- Recommendations: [What to do about incomplete items]

If Failed:
- Critical failures prevented completion
- [Description of failures]
- Rollback Status: [Rolled back/Partially rolled back/Not rolled back]
- Recovery Plan: [How to proceed]

## Next Steps

1. [Recommended next action]
2. [Recommended next action]
...

## Attachments

- Modified Files: [List]
- Diff Summary: [High-level overview of changes]
- Test Output: [If applicable]
```

## Execution Guidelines

- Follow the plan: Execute action items in the specified order
- Be careful: Don't introduce new issues while fixing old ones
- Verify constantly: Check your work after each significant change
- Document thoroughly: Record what you did for the report
- Communicate blockers: If you can't complete something, say why
- Preserve functionality: Don't break working code
- Think holistically: Consider how changes affect the whole document
- Test edge cases: Verify your changes handle boundary conditions
- Update docs: Keep documentation in sync with code changes
- Be honest: Report failures and partial completions accurately
