## MODIFIED Requirements

### Requirement: Mandatory Todo List Management
AI assistants SHALL create and maintain a structured todo list for every task that involves more than two steps.

#### Scenario: Task initialization
- **WHEN** a new task is received
- **THEN** the assistant MUST initialize a todo list using the `todowrite` tool or a clear markdown list

#### Scenario: Real-time updates
- **WHEN** a sub-task is started or completed
- **THEN** the assistant MUST update the status of that item in the todo list immediately

## ADDED Requirements

### Requirement: Git Push Approval
AI assistants SHALL NOT execute `git push` without explicit user approval.

#### Scenario: Requesting push approval
- **WHEN** changes are ready to be pushed to the remote repository
- **THEN** the assistant MUST summarize the changes and ask the user for approval before executing `git push`
