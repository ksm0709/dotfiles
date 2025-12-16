---
description: Create git commit message
agent: build
---
Create a git commit following project standards

User invoked: /git:commit $ARGUMENTS

This command creates a properly formatted git commit that:
1. Runs code formatting before committing
2. Follows the commit template at tmpl.md
3. Includes proper Jira ticket reference

## Workflow:

1. **Run formatting first:**
   - Execute `bazel run //:format` to format all code
   - Wait for formatting to complete

2. **Check git status:**
   - Run `git status` to see what files are staged/modified
   - Run `git diff --staged` to review staged changes
   - If nothing staged, stage appropriate files with `git add`

3. **Determine Jira ticket:**
   - The user provided these arguments to the command: "$ARGUMENTS"
   - If arguments were provided, use them as the Jira ticket (e.g., PS-1234)
   - If no arguments provided, check current branch name for Jira ticket (e.g., PS-1234, FM-5678)
   - If no ticket found in either place, ask user for Jira ticket ID

4. **Check branch and suggest new branch if needed:**
   - Check if current branch is `devel` OR has a different Jira ticket name than the determined ticket
   - If either condition is true, suggest creating a new branch
   - Suggest branch name format: `{JIRA-TICKET}-brief-description` (e.g., PS-1234-add-auth)
   - Ask user if they want to create and checkout the new branch
   - If user agrees, run `git checkout -b {suggested-branch-name}`

5. **Create commit message following tmpl.md structure:**
   - First line: `[PS-####] One line description` (becomes PR title)
   - IMPORTANT: Keep first line under 72 characters (including [PS-####])
   - Include sections: Changelog, Testing, Version Compatibility, Demo, Ops Impact
   - Follow the template format exactly

6. **Create the commit:**
   - Use git commit with the formatted message
   - Follow Git Safety Protocol (no --no-verify, check authorship if amending)

7. **Verify:**
   - Run `git log -1` to show the created commit
   - Run `git status` to confirm clean state

## Example commit message format:

```
[PS-1234] Add user authentication endpoint

Changelog
======
- Add new /api/auth/login endpoint
- Implement JWT token generation
- Add user session management
- Fixes [PS-1234]

Testing
======
- Added unit tests for auth handler
- Manual testing: curl -X POST http://localhost:8080/api/auth/login
- All tests pass: bazel test //...

Version Compatibility
======
- Backwards compatible, no breaking changes

Demo
======
- Feature behind AUTH_ENABLED flag
- Set AUTH_ENABLED=true in environment

Ops Impact
======
- New database migration required for session table
- JWT_SECRET environment variable must be set
```

## Notes:
- NEVER use `NOJIRA` - always require a valid Jira ticket
- First line MUST be under 72 characters (standard git practice)
- Format code BEFORE committing to ensure consistent style
- Do not commit if tests are failing
- Follow project rules: no mock data outside tests, document exported identifiers
