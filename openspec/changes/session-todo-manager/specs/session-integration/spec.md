# Session Integration Specification

## ADDED Requirements

### Requirement: Session ID Persistence
The SimpleTodoManager shall maintain the same session ID across multiple calls within the same OpenCode session.

#### Scenario: Subagent Multiple Calls
Given a subagent calls SimpleTodoManager multiple times in the same session
When the first call creates a session with ID "abc123"
Then subsequent calls in the same session shall use ID "abc123"
And the session data shall be stored in the same directory

#### Scenario: Session ID Extraction
Given a TypeScript wrapper is called as an OpenCode Custom Tool
When the context object contains sessionID "xyz789"
Then the wrapper shall extract and pass "xyz789" to the Python core
And the Python core shall use "xyz789" as the session identifier

### Requirement: Project-Relative Path Resolution
The SimpleTodoManager shall resolve all paths relative to the project root without hardcoded absolute paths.

#### Scenario: Project Root Detection
Given SimpleTodoManager is initialized in any project directory
When the tool needs to create session directories
Then it shall detect the project root by looking for ".opencode" directory
And shall create session directories under "{project_root}/.opencode/sessions/"

#### Scenario: Path Security
Given a malicious agent provides "../evil" as session ID
When the tool constructs the session directory path
Then it shall validate and sanitize the path components
And shall restrict directory creation to the project root boundary

### Requirement: Hybrid Architecture Integration
The system shall integrate TypeScript wrapper with Python core through CLI interface.

#### Scenario: TypeScript Wrapper Execution
Given a subagent calls the session-todo tool with action "add"
When the wrapper extracts sessionID from context
Then it shall call the Python core with appropriate CLI arguments
And shall return the result in structured format

#### Scenario: Python Core CLI Interface
Given the Python core is called with CLI arguments
When it receives "--agent senior-sw-engineer --session abc123 --action add"
Then it shall parse the arguments and execute the requested operation
And shall return results in machine-readable format

## MODIFIED Requirements

### Requirement: Backward Compatibility
The SimpleTodoManager shall maintain backward compatibility with existing direct Python usage.

#### Scenario: Direct Python Usage
Given existing code uses `SimpleTodoManager("agent", "session-id")`
When the code is executed after the enhancement
Then it shall continue to work without modification
And shall produce the same results as before

#### Scenario: Tool Location Migration
Given the tool is moved from `~/.config/opencode/tools/` to `opencode/config/tools/`
When existing subagents reference the old location
Then they shall be updated to use the new location
And shall maintain full functionality

## REMOVED Requirements

### Requirement: Temporary Directory Storage
The requirement to store session data in `/tmp/agent_sessions/` is removed.

#### Scenario: Storage Location Migration
Given existing session data is stored in `/tmp/agent_sessions/`
When the new system is implemented
Then session data shall be stored in project-relative locations
And existing data shall be migrated or considered obsolete