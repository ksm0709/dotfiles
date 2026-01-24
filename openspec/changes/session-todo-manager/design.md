# Session Todo Manager Design

## Architecture Overview

### Hybrid TypeScript/Python Approach
```
Subagent Call → TypeScript Wrapper → OpenCode Context → Python Core → File System
```

### Components

#### 1. TypeScript Wrapper (`session-todo.ts`)
- **Purpose**: Extract session ID from OpenCode Context API
- **Location**: `opencode/config/tools/session-todo.ts`
- **Interface**: OpenCode Custom Tool specification
- **Responsibility**: 
  - Extract `sessionID` from context object
  - Call Python SimpleTodoManager with session ID
  - Return structured responses

#### 2. Python Core (`simple-todo.py`)
- **Purpose**: Core Todo management logic
- **Location**: `opencode/config/tools/simple-todo.py`
- **Enhancement**: CLI interface for TypeScript integration
- **Responsibility**:
  - Maintain existing functionality
  - Add CLI argument parsing
  - Session-aware directory management

#### 3. Session Storage Strategy
- **Base Path**: `{PROJECT_ROOT}/.opencode/sessions/`
- **Session Directory**: `{base_path}/{agent_name}_{session_id}/`
- **Todo File**: `todos.json`
- **Metadata**: `session_info.json`

## Data Flow

### Session ID Extraction
```typescript
async execute(args, context) {
  const { sessionID } = context;  // OpenCode Context API
  // Pass sessionID to Python core
}
```

### Python Core Integration
```python
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True)  # From TypeScript
    args = parser.parse_args()
    
    todos = SimpleTodoManager(args.agent, args.session)
```

## Path Resolution Strategy

### Project-Relative Paths
```python
import os
from pathlib import Path

def get_project_root():
    """Find project root by looking for .opencode directory"""
    current = Path.cwd()
    while current.parent != current:
        if (current / ".opencode").exists():
            return current
        current = current.parent
    return Path.cwd()

def get_session_base_path():
    """Get session base path relative to project"""
    project_root = get_project_root()
    return project_root / ".opencode" / "sessions"
```

## Backward Compatibility

### Direct Python Usage
```python
# Existing usage still works
from tools.simple_todo import SimpleTodoManager
todos = SimpleTodoManager("senior-sw-engineer", "custom-session-id")
```

### TypeScript Wrapper Usage
```typescript
// New OpenCode-integrated usage
await sessionTodo({
  agent_name: "senior-sw-engineer",
  action: "add",
  content: "API implementation"
});
```

## Error Handling

### Session ID Validation
- TypeScript: Validate sessionID from context
- Python: Fallback to generated ID if sessionID invalid

### Path Resolution
- Project root detection with fallbacks
- Directory creation with proper permissions
- Graceful degradation if paths unavailable

## Performance Considerations

### Lazy Loading
- Session directory created only when needed
- Python subprocess spawned only on tool calls

### Caching Strategy
- Session info cached in memory during tool session
- File I/O minimized through batch operations

## Security Considerations

### Path Traversal Prevention
- Validate all path components
- Restrict to project-relative paths
- Sanitize session IDs and agent names

### Permission Management
- Ensure proper file permissions
- Handle cross-user session isolation
- Validate directory creation rights