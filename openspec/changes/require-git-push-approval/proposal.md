# Change: Require user approval before git push

## Why
To prevent accidental or unwanted changes from being pushed to the remote repository, AI assistants must explicitly ask for and receive user approval before executing any `git push` command. This ensures the user has a final chance to review the changes that will be shared.

## What Changes
- Update `openspec/AGENTS.md` to include a mandatory "Git Push Approval" rule.
- AI assistants must summarize the changes to be pushed and wait for user confirmation.

## Impact
- Affected specs: `workflow`
- Affected documentation: `openspec/AGENTS.md`
