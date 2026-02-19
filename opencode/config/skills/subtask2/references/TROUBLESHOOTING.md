# Troubleshooting Subtask2 Commands

Common issues and solutions when creating subtask2 commands.

## Issue Categories

- [Command Not Working](#command-not-working)
- [Loop Issues](#loop-issues)
- [Parallel Execution Problems](#parallel-execution-problems)
- [Return Chain Failures](#return-chain-failures)
- [Context/Variable Issues](#contextvariable-issues)
- [Result Handling Problems](#result-handling-problems)

---

## Command Not Working

### Problem: Command appears in list but does nothing

**Symptoms:**
- `/mycommand` shows in command list
- When invoked, nothing happens or generic response

**Check:**
1. Is `subtask: true` in frontmatter?
   ```yaml
   ---
   subtask: true  # REQUIRED for subtask2 features
   ---
   ```

2. Is plugin installed?
   ```bash
   grep "@spoons-and-mirrors/subtask2" ~/.config/opencode/opencode.json
   ```
   If not found, add to `opencode.json`:
   ```json
   {
     "plugins": ["@spoons-and-mirrors/subtask2@latest"]
   }
   ```

3. File location correct?
   - Project: `.opencode/command/name.md`
   - Global: `~/.config/opencode/command/name.md`

**Solution:**
- Add `subtask: true` to frontmatter
- Install subtask2 plugin
- Verify file path

---

### Problem: Subtask runs but return chain doesn't trigger

**Symptoms:**
- Subtask completes successfully
- Next action in `return:` doesn't execute
- Session just ends

**Check:**
1. Return syntax correct?
   ```yaml
   # GOOD
   return: Next action here
   
   # GOOD (array)
   return:
     - Step 1
     - Step 2
   
   # BAD (missing colon)
   return Next action
   ```

2. Indentation correct (YAML)?
   ```yaml
   # GOOD
   return:
     - Step 1
     - Step 2
   
   # BAD (inconsistent)
   return:
   - Step 1
     - Step 2
   ```

**Solution:**
- Ensure proper YAML syntax
- Use array format for multiple returns
- Check indentation (2 spaces)

---

## Loop Issues

### Problem: Loop runs forever

**Symptoms:**
- Command keeps running indefinitely
- Never reaches termination condition

**Check:**
1. Is `max` set?
   ```yaml
   # GOOD - Has safety limit
   loop:
     max: 10
     until: "tests pass"
   
   # BAD - No max (can loop forever)
   loop:
     until: "tests pass"
   ```

2. Is condition objectively verifiable?
   ```yaml
   # GOOD - Measurable
   until: "all tests pass and coverage > 80%"
   
   # BAD - Subjective/vague
   until: "code is good"
   until: "DONE"
   ```

**Solution:**
- Always set `max: N` as safety net
- Use measurable, verifiable conditions
- Test condition manually: can YOU verify it?

---

### Problem: Loop exits immediately

**Symptoms:**
- Loop runs only once
- Condition seems to always pass

**Check:**
1. Condition evaluation prompt clear?
   - The main session evaluates the condition
   - It must have enough context to evaluate

2. Is the condition actually being checked?
   ```yaml
   # GOOD - Explicit check in return
   return:
     - Run tests
     - Check if tests pass
   
   # BAD - No verification step
   return: Just continue
   ```

**Solution:**
- Add explicit verification step in return chain
- Ensure evaluation has access to results
- Use clear success/failure criteria

---

## Parallel Execution Problems

### Problem: Parallel commands don't run concurrently

**Symptoms:**
- Commands run sequentially
- Total time = sum of all tasks

**Check:**
1. Is `parallel:` correctly formatted?
   ```yaml
   # GOOD
   parallel:
     - /command1
     - /command2
   
   # BAD (inline syntax wrong)
   parallel: /command1, /command2
   ```

2. Are commands properly defined?
   - Each parallel command must exist
   - They will be forced to `subtask: true`

**Solution:**
- Use YAML array syntax
- Ensure all parallel commands exist
- Check PR #6478 if using latest features

---

### Problem: Parallel results not captured

**Symptoms:**
- Parallel tasks run
- Results not available in return chain
- `$RESULT[name]` shows "not found"

**Check:**
1. Using `{as:name}` correctly?
   ```yaml
   # GOOD
   parallel:
     - /task {as:result1}
     - /task {as:result2}
   
   # BAD (missing as:)
   parallel:
     - /task
     - /task
   ```

2. Referencing correctly?
   ```yaml
   # GOOD
   return: Synthesize $RESULT[result1] and $RESULT[result2]
   
   # BAD (wrong syntax)
   return: Synthesize $result1
   ```

**Solution:**
- Always use `{as:name}` in parallel
- Use `$RESULT[name]` (not `$name`)
- Names are case-sensitive

---

## Return Chain Failures

### Problem: Return chain stops mid-way

**Symptoms:**
- First return executes
- Subsequent returns don't run
- Chain breaks unexpectedly

**Check:**
1. Array syntax used for multiple steps?
   ```yaml
   # GOOD
   return:
     - Step 1
     - Step 2
     - Step 3
   
   # BAD (pipe in return not supported)
   return: Step 1 || Step 2 || Step 3
   ```

2. Are return items valid?
   - Prompts must be strings
   - Commands must start with `/`
   - No empty items

**Solution:**
- Use YAML array format
- Verify each item is valid
- Test one item at a time

---

### Problem: Command in return doesn't execute

**Symptoms:**
- `return: /mycommand` appears in output
- Command doesn't actually run

**Check:**
1. Command syntax correct?
   ```yaml
   # GOOD
   return: /mycommand args
   
   # BAD (missing slash)
   return: mycommand args
   ```

2. Does command exist?
   - Check if `/mycommand` is defined
   - Verify command file exists

**Solution:**
- Ensure command starts with `/`
- Verify command exists
- Check command file syntax

---

## Context/Variable Issues

### Problem: Variables not substituted

**Symptoms:**
- `$ARGUMENTS` appears literally in output
- `$TURN[5]` not replaced with conversation
- Variables not working

**Check:**
1. Using correct syntax?
   ```yaml
   # GOOD
   $ARGUMENTS
   $TURN[5]
   $RESULT[name]
   
   # BAD
   ${ARGUMENTS}
   $ARGUMENT
   $Turn[5]
   ```

2. Is placeholder used only once?
   ```yaml
   # BAD - Used multiple times
   Fix $ARGUMENTS in $ARGUMENTS and test $ARGUMENTS
   
   # GOOD - Once in dedicated block
   <context>
   $ARGUMENTS
   </context>
   ```

**Solution:**
- Use exact syntax: `$ARGUMENTS`, `$TURN[n]`, `$RESULT[name]`
- Insert each placeholder only once
- Use dedicated XML blocks for context

---

### Problem: `$TURN[n]` includes wrong messages

**Symptoms:**
- Wrong conversation context passed
- Missing important messages
- Too much irrelevant context

**Check:**
1. Index understanding:
   - `$TURN[5]` = last 5 messages
   - `$TURN[:3]` = 3rd from end
   - `$TURN[*]` = all messages

2. Is conversation long enough?
   - If only 3 messages exist, `$TURN[10]` still works but only gets 3

**Solution:**
- Use `$TURN[5]` for recent context (usually sufficient)
- Use `$TURN[:n]` for specific message
- Avoid `$TURN[*]` unless truly needed

---

### Problem: File references (`@file`) not working

**Symptoms:**
- `@src/file.ts` appears literally
- File content not injected

**Check:**
1. File exists?
   ```bash
   ls -la src/file.ts
   ```

2. Path correct?
   - Relative to command file location
   - Or absolute path

**Solution:**
- Verify file exists
- Use correct relative path
- Consider using ``!`cat file` `` as alternative

---

## Result Handling Problems

### Problem: `$RESULT[name]` not found

**Symptoms:**
- Shows `[Result 'name' not found]`
- Empty or missing result

**Check:**
1. Was result captured with `{as:name}`?
   ```yaml
   # Must capture first
   parallel:
     - /task {as:myresult}
   
   # Then reference
   return: Use $RESULT[myresult]
   ```

2. Name matches exactly?
   - Case-sensitive: `myResult` ≠ `myresult`
   - No extra spaces: `{as: myresult}` ≠ `{as:myresult}`

3. Is result from same parent session?
   - Results are scoped to parent
   - Can't access results from other command trees

**Solution:**
- Ensure `{as:name}` is set when capturing
- Match case exactly
- Reference from same parent session

---

### Problem: Named result is empty

**Symptoms:**
- `$RESULT[name]` exists but empty
- No content captured

**Check:**
1. Did subtask produce output?
   - Some subtasks return nothing
   - Check subtask body for explicit output

2. Is output format correct?
   ```yaml
   # GOOD - Explicit return value
   subtask: true
   ---
   Analyze and return findings as:
   - Finding 1
   - Finding 2
   ```

**Solution:**
- Ensure subtask has explicit output
- Use structured formats (lists, JSON)
- Add "Return your findings as..." instruction

---

## Debugging Tips

### 1. Start Simple

Test with minimal command first:
```yaml
---
subtask: true
description: Test command
---
Simple test: $ARGUMENTS
```

Once working, add features incrementally.

### 2. Add Logging

Use return chain to inspect:
```yaml
return:
  - "Debug: About to run parallel tasks"
  - /task
  - "Debug: Task completed, result: $RESULT[name]"
```

### 3. Verify Each Component

Test individually:
1. Test subtask without return
2. Test return without loop
3. Test loop with simple condition
4. Add parallel one at a time

### 4. Check YAML Syntax

Use YAML validator:
```bash
# Install yamllint if needed
pip install yamllint

# Validate
yamllint command.md
```

Common YAML errors:
- Tabs instead of spaces
- Missing colons
- Inconsistent indentation
- Unclosed quotes

### 5. Review Generated Prompt

Look at what subagent actually receives:
- Variables should be substituted
- No literal `$VARIABLE` in final prompt
- Context properly formatted

---

## Common Error Messages

| Error | Cause | Solution |
|-------|-------|----------|
| `subtask2 loop="continue"` | Loop condition not met | Check condition is verifiable |
| `Result 'name' not found` | Missing `{as:name}` | Add result capture |
| Command not appearing | Wrong directory | Move to `.opencode/command/` |
| Return not executing | Missing `subtask: true` | Add to frontmatter |
| Parallel not concurrent | Syntax error | Use YAML array `- /cmd` |

---

## Getting Help

If issue persists:

1. **Simplify** to minimal reproducible example
2. **Check** `~/.config/opencode/subtask2.jsonc` config
3. **Verify** plugin is latest version
4. **Review** GitHub issues: https://github.com/spoons-and-mirrors/subtask2

## Prevention Checklist

Before deploying a new command:

- [ ] `subtask: true` in frontmatter
- [ ] Plugin installed and enabled
- [ ] YAML syntax validated
- [ ] All variables used only once
- [ ] Loop has `max` safety limit
- [ ] Parallel commands use `{as:name}`
- [ ] Results referenced with `$RESULT[name]`
- [ ] File paths verified
- [ ] Tested with sample input
