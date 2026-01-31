# Design: Tasks Plugin Unification & Session Isolation

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     OpenCode Agent                          │
│                                                              │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   tasks     │───▶│   Plugin    │───▶│   Storage   │     │
│  │   (统合)     │    │   Core      │    │   (Session  │     │
│  │             │    │             │    │   Isolated) │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ Operations  │    │  Commands   │    │  Session    │     │
│  │  (Batch)    │    │  (Unified)  │    │   Store     │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. Unified Tool Interface (`tasks`)

```typescript
// src/index.ts - Unified tasks tool
tasks: tool({
  description: "Unified task management tool. Supports init, add, update, complete, remove operations. Always shows current session's task status after execution.",
  args: {
    operations: tool.schema.array(
      tool.schema.object({
        type: tool.schema.enum(
          ['init', 'add', 'update', 'complete', 'remove'],
          { description: "Operation type" }
        ),
        // Operation-specific fields
        agent: tool.schema.optional(tool.schema.string()),
        title: tool.schema.optional(tool.schema.string()),
        id: tool.schema.optional(tool.schema.string()),
        parent: tool.schema.optional(tool.schema.string()),
        status: tool.schema.optional(
          tool.schema.enum(['pending', 'in_progress', 'completed'])
        ),
      }),
      { description: "Array of operations (max 50)", maxItems: 50 }
    ),
  },
  async execute(args: { operations: Operation[] }, ctx: Context) {
    const sessionId = ctx.sessionId || 'default-session';
    const result = await unifiedCommand({ sessionId, operations });
    return result.response; // Always includes current session status
  }
})
```

### 2. Session Isolation

```typescript
// src/lib/storage.ts - Session-aware storage
class TaskStorage {
  private getSessionDir(sessionId: string): string {
    return path.join(TASKS_DIR, sessionId);
  }

  async readTasks(sessionId: string): Promise<Task[]> {
    const sessionDir = this.getSessionDir(sessionId);
    // Only read from current session directory
    // Other sessions are completely isolated
  }

  async writeTasks(sessionId: string, tasks: Task[]): Promise<void> {
    const sessionDir = this.getSessionDir(sessionId);
    // Write to session-specific directory
  }
}
```

### 3. Unified Command Handler

```typescript
// src/commands/unified.ts
export async function unifiedCommand(
  params: UnifiedCommandParams
): Promise<UnifiedCommandResult> {
  const { sessionId, operations } = params;
  const results: OperationResult[] = [];

  for (const operation of operations) {
    try {
      const result = await executeOperation(sessionId, operation);
      results.push({ success: true, operation, result });
    } catch (error) {
      results.push({ success: false, operation, error: error.message });
      // Continue with next operation (partial failure allowed)
    }
  }

  // Always return current session status
  const currentStatus = await getSessionStatus(sessionId);
  
  return {
    results,
    status: currentStatus,
    response: formatUnifiedResponse(results, currentStatus)
  };
}
```

### 4. Operation Router

```typescript
// src/commands/operations.ts
async function executeOperation(
  sessionId: string,
  operation: Operation
): Promise<any> {
  switch (operation.type) {
    case 'init':
      return initCommand({ 
        sessionId, 
        agent: operation.agent!, 
        title: operation.title! 
      });
    case 'add':
      return addCommand({
        sessionId,
        title: operation.title!,
        parent: operation.parent
      });
    case 'update':
      return updateCommand({
        sessionId,
        id: operation.id!,
        status: operation.status!
      });
    case 'complete':
      return completeCommand({
        sessionId,
        id: operation.id!
      });
    case 'remove':
      return removeCommand({
        sessionId,
        id: operation.id!
      });
    default:
      throw new Error(`Unknown operation type: ${operation.type}`);
  }
}
```

### 5. Unified Output Formatter

```typescript
// src/lib/formatter.ts - Session-aware formatter
export function formatSessionStatus(
  sessionId: string,
  taskList: TaskList
): ToolResponse {
  const summary = calculateSummary(taskList.tasks);
  
  return {
    title: `Task List: ${taskList.title}`,
    output: formatMarkdownOutput(taskList, summary),
    metadata: {
      sessionId, // Current session only
      agent: taskList.agent,
      tasks: taskList.tasks,
      summary,
      operation: 'unified'
    }
  };
}
```

## Data Flow

### 1. Single Operation Flow

```
Agent → tasks({operations: [{type: 'add', title: 'Task'}]})
  ↓
Plugin Core (unifiedCommand)
  ↓
Operation Router (executeOperation)
  ↓
Command Handler (addCommand)
  ↓
Session Storage (writeTasks - current session only)
  ↓
Status Formatter (formatSessionStatus - current session only)
  ↓
ToolResponse (with current session status)
```

### 2. Batch Operation Flow

```
Agent → tasks({operations: [op1, op2, op3]})
  ↓
Plugin Core (unifiedCommand)
  ↓
Loop through operations
  ├─→ Operation 1 → execute → result
  ├─→ Operation 2 → execute → result (continue even if error)
  └─→ Operation 3 → execute → result
  ↓
Collect all results
  ↓
Get Current Session Status
  ↓
Format Unified Response
  ↓
ToolResponse
```

## Session Isolation Implementation

### Directory Structure

```
~/.local/share/opencode/tasks/
├── {session-id-1}/
│   ├── agent-a-tasks.md
│   └── agent-b-tasks.md
├── {session-id-2}/
│   └── agent-c-tasks.md
└── {session-id-3}/
    └── agent-d-tasks.md
```

### Key Principles

1. **Current Session Only**: All read/write operations use current session ID
2. **No Cross-Session Access**: Cannot read or modify other sessions' data
3. **Automatic Session ID**: Extracted from OpenCode context, not user-provided
4. **Default Session**: Falls back to 'default-session' if not available

## Deprecation Implementation

### Phase 1: Soft Deprecation

```typescript
// Add deprecation warning to existing tools
function addDeprecationWarning(response: string): string {
  return `${response}\n\n⚠️ **DEPRECATED**: This tool will be removed in v4.0.0. Please use the unified \`tasks\` tool instead. See migration guide: https://...`;
}

// Wrap existing tool responses
export async function deprecatedInitCommand(...): Promise<string> {
  const result = await initCommand(...);
  return addDeprecationWarning(result);
}
```

### Phase 2: Hard Removal

- Delete deprecated tool definitions from `index.ts`
- Remove deprecated command handlers
- Update documentation

## Error Handling

### Partial Failure Strategy

```typescript
// Continue processing even if one operation fails
for (const operation of operations) {
  try {
    await executeOperation(sessionId, operation);
    succeeded++;
  } catch (error) {
    failed++;
    errors.push({ operation, error: error.message });
    // Continue with next operation
  }
}

// Return results with both successes and failures
return {
  summary: { total, succeeded, failed },
  errors: errors.length > 0 ? errors : undefined,
  status: await getSessionStatus(sessionId) // Always show current status
};
```

## Testing Strategy

### Unit Tests

1. **Operation Router**: Test each operation type
2. **Session Isolation**: Verify cross-session data isolation
3. **Batch Processing**: Test partial failure scenarios
4. **Output Format**: Verify unified response format

### Integration Tests

1. **End-to-End**: Full workflow from tool call to response
2. **Multi-Session**: Verify session isolation in practice
3. **Migration**: Test deprecated tools still work with warnings

## Migration Guide (for Users)

### Before (Individual Tools)

```typescript
tasks_init(agent="dev", title="Project")
tasks_add(title="Task 1")
tasks_add(title="Task 2", parent="1")
tasks_update(id="1", status="completed")
tasks_complete(id="2")
```

### After (Unified Tool)

```typescript
tasks(operations=[
  { type: 'init', agent: 'dev', title: 'Project' },
  { type: 'add', title: 'Task 1' },
  { type: 'add', title: 'Task 2', parent: '1' },
  { type: 'update', id: '1', status: 'completed' },
  { type: 'complete', id: '2' }
])
```

## Performance Considerations

1. **Batch Operations**: Process up to 50 operations in one call
2. **Lazy Loading**: Only read current session's data
3. **Efficient Storage**: Use file-based storage with minimal I/O
4. **Caching**: Consider in-memory caching for frequent reads
