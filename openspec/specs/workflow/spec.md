## ADDED Requirements

### Requirement: Mandatory Todo List Management
AI assistants SHALL create and maintain a structured todo list for every task that involves more than two steps.

#### Scenario: Task initialization
- **WHEN** a new task is received
- **THEN** the assistant MUST initialize a todo list using the `todowrite` tool or a clear markdown list

#### Scenario: Real-time updates
- **WHEN** a sub-task is started or completed
- **THEN** the assistant MUST update the status of that item in the todo list immediately
