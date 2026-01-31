# Design: Tasks Plugin Batch Operations & UI Enhancement

## Overview

이 문서는 tasks 플러그인의 배치 작업 및 UI 개선에 대한 상세 설계를 다룹니다.

## Architecture

### 전체 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    Tasks Plugin                             │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  Tool Layer  │  │ Command Layer│  │   Formatter  │      │
│  │              │  │              │  │              │      │
│  │ • tasks_init │  │ • init.ts    │  │ • markdown   │      │
│  │ • tasks_add  │  │ • add-task.ts│  │ • table      │      │
│  │ • tasks_batch│  │ • batch.ts   │  │ • progress   │      │
│  │ • ...        │  │ • ...        │  │ • summary    │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐                        │
│  │   Storage    │  │    Types     │                        │
│  │              │  │              │                        │
│  │ • loadTasks  │  │ • TaskList   │                        │
│  │ • saveTasks  │  │ • TaskDetail │                        │
│  │ • getPath    │  │ • Operation  │                        │
│  └──────────────┘  └──────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

## Data Models

### 1. Batch Operation Types

```typescript
// src/types/index.ts

export type OperationType = 'add' | 'update' | 'complete' | 'remove';

export interface BatchOperation {
  type: OperationType;
  // 공통 필드
  id?: string;           // update, complete, remove에 필요
  title?: string;        // add에 필요
  parent?: string;       // add에 optional
  status?: TaskStatus;   // update에 필요
}

export interface BatchResult {
  success: boolean;
  operation: BatchOperation;
  taskId?: string;       // add 성공 시 생성된 ID
  message: string;
  error?: string;        // 실패 시 에러 메시지
}

export interface BatchCommandResult {
  success: boolean;
  results: BatchResult[];
  summary: {
    total: number;
    succeeded: number;
    failed: number;
  };
  currentStatus: TaskList;  // 작업 후 현재 상태
  formattedOutput: string;  // 마크다운 형식의 출력
}
```

### 2. Enhanced Response Types

```typescript
// 모든 command 결과에 현황 정보 포함

export interface CommandResultWithStatus {
  success: boolean;
  message: string;
  currentStatus: TaskList;      // 현재 전체 task 상태
  statusSummary: StatusSummary; // 요약 정보
  formattedOutput: string;      // 마크다운 형식 출력
}

export interface StatusSummary {
  agent: string;
  title: string;
  total: number;
  completed: number;
  inProgress: number;
  pending: number;
  completionRate: number;  // 0-100
}
```

## API Design

### 1. Batch Tool Schema

```typescript
// src/index.ts - tasks_batch tool definition

tasks_batch: tool({
  description: "Execute multiple task operations in a single call. Supports add, update, complete, and remove operations.",
  args: {
    operations: tool.schema.array(
      tool.schema.object({
        type: tool.schema.enum(['add', 'update', 'complete', 'remove']),
        id: tool.schema.string().optional(),
        title: tool.schema.string().optional(),
        parent: tool.schema.string().optional(),
        status: tool.schema.enum(['pending', 'in_progress', 'completed']).optional(),
      })
    ).describe("Array of operations to execute"),
  },
  async execute(args: { operations: BatchOperation[] }, ctx: any) {
    const sessionId = ctx.sessionId || ctx.session_id || 'default-session';
    const result = await batchCommand({
      sessionId,
      operations: args.operations
    });
    return result.formattedOutput;
  }
})
```

### 2. Batch Command Implementation

```typescript
// src/commands/batch.ts

export async function batchCommand(args: {
  sessionId: string;
  operations: BatchOperation[];
}): Promise<BatchCommandResult> {
  const results: BatchResult[] = [];
  let successCount = 0;
  let failCount = 0;

  // 각 operation 순차 처리
  for (const operation of args.operations) {
    try {
      let result: BatchResult;

      switch (operation.type) {
        case 'add':
          result = await executeAdd(args.sessionId, operation);
          break;
        case 'update':
          result = await executeUpdate(args.sessionId, operation);
          break;
        case 'complete':
          result = await executeComplete(args.sessionId, operation);
          break;
        case 'remove':
          result = await executeRemove(args.sessionId, operation);
          break;
        default:
          throw new Error(`Unknown operation type: ${operation.type}`);
      }

      results.push(result);
      if (result.success) successCount++;
      else failCount++;

    } catch (error) {
      results.push({
        success: false,
        operation,
        message: `Operation failed: ${error}`,
        error: String(error)
      });
      failCount++;
    }
  }

  // 현재 상태 로드
  const currentStatus = await loadTaskList(args.sessionId);
  const statusSummary = calculateStatusSummary(currentStatus);

  // 마크다운 형식으로 출력 생성
  const formattedOutput = formatBatchResult(results, statusSummary, currentStatus);

  return {
    success: failCount === 0,
    results,
    summary: {
      total: args.operations.length,
      succeeded: successCount,
      failed: failCount
    },
    currentStatus,
    formattedOutput
  };
}
```

## Formatter Design

### 1. Markdown Formatter

```typescript
// src/lib/formatter.ts

export function formatTaskListWithStatus(
  taskList: TaskList,
  summary: StatusSummary
): string {
  const lines: string[] = [];

  // 헤더
  lines.push(`## 📋 Task Status: ${taskList.title}`);
  lines.push('');

  // 진행 상황 요약
  lines.push('### 🎯 Progress Summary');
  lines.push(`- ✅ Completed: ${summary.completed} (${Math.round(summary.completed / summary.total * 100)}%)`);
  lines.push(`- 🔄 In Progress: ${summary.inProgress} (${Math.round(summary.inProgress / summary.total * 100)}%)`);
  lines.push(`- ⏳ Pending: ${summary.pending} (${Math.round(summary.pending / summary.total * 100)}%)`);
  lines.push('');

  // 전체 진행률 바
  lines.push('### 📊 Overall Progress');
  lines.push(formatProgressBar(summary.completionRate));
  lines.push('');

  // 작업 목록
  lines.push('### 📝 Task List');
  lines.push('');

  // 완료된 작업
  if (summary.completed > 0) {
    lines.push('#### ✅ Completed');
    taskList.tasks
      .filter(t => t.status === 'completed')
      .forEach(task => {
        lines.push(formatTaskLine(task));
      });
    lines.push('');
  }

  // 진행중인 작업
  if (summary.inProgress > 0) {
    lines.push('#### 🔄 In Progress');
    taskList.tasks
      .filter(t => t.status === 'in_progress')
      .forEach(task => {
        lines.push(formatTaskLine(task));
      });
    lines.push('');
  }

  // 대기중인 작업
  if (summary.pending > 0) {
    lines.push('#### ⏳ Pending');
    taskList.tasks
      .filter(t => t.status === 'pending')
      .forEach(task => {
        lines.push(formatTaskLine(task));
      });
    lines.push('');
  }

  return lines.join('\n');
}

function formatProgressBar(percentage: number): string {
  const filled = Math.round(percentage / 10);
  const empty = 10 - filled;
  const bar = '█'.repeat(filled) + '░'.repeat(empty);
  return `[${bar}] ${percentage}%`;
}

function formatTaskLine(task: TaskDetail): string {
  const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
  const icon = getStatusIcon(task.status);
  return `- ${checkbox} ${icon} **${task.id}**. ${task.title}`;
}

function getStatusIcon(status: TaskStatus): string {
  switch (status) {
    case 'completed': return '✅';
    case 'in_progress': return '🔄';
    case 'pending': return '⏳';
    default: return '⏳';
  }
}
```

### 2. Batch Result Formatter

```typescript
// src/lib/formatter.ts

export function formatBatchResult(
  results: BatchResult[],
  summary: StatusSummary,
  taskList: TaskList
): string {
  const lines: string[] = [];

  // 작업 결과 헤더
  lines.push('## ✅ Batch Operations Complete');
  lines.push('');

  // 요약
  const successCount = results.filter(r => r.success).length;
  const failCount = results.length - successCount;
  
  lines.push(`**Results:** ${successCount} succeeded, ${failCount} failed (Total: ${results.length})`);
  lines.push('');

  // 개별 결과
  if (results.length > 0) {
    lines.push('### 📋 Operation Details');
    lines.push('');
    
    results.forEach((result, index) => {
      const icon = result.success ? '✅' : '❌';
      lines.push(`${icon} **${index + 1}. ${result.operation.type}**`);
      
      if (result.operation.title) {
        lines.push(`   Title: ${result.operation.title}`);
      }
      if (result.operation.id) {
        lines.push(`   ID: ${result.operation.id}`);
      }
      if (result.taskId) {
        lines.push(`   New ID: ${result.taskId}`);
      }
      lines.push(`   Result: ${result.message}`);
      lines.push('');
    });
  }

  // 현재 상태 추가
  lines.push('---');
  lines.push('');
  lines.push(formatTaskListWithStatus(taskList, summary));

  return lines.join('\n');
}
```

## Modified Commands

### 1. Enhanced Add Task Command

```typescript
// src/commands/add-task.ts (수정된 버전)

export async function addTaskCommand(args: {
  sessionId: string;
  title: string;
  parent?: string;
}): Promise<CommandResultWithStatus> {
  // 기존 로직으로 작업 추가
  const taskList = await loadTaskList(args.sessionId);
  const newTask = createTask(args.title, args.parent);
  taskList.tasks.push(newTask);
  await saveTaskList(args.sessionId, taskList);

  // 현재 상태 및 요약 계산
  const statusSummary = calculateStatusSummary(taskList);
  
  // 마크다운 출력 생성
  const formattedOutput = formatAddResult(newTask, taskList, statusSummary);

  return {
    success: true,
    message: `Task "${args.title}" added successfully (ID: ${newTask.id})`,
    currentStatus: taskList,
    statusSummary,
    formattedOutput
  };
}
```

### 2. Enhanced Update Command

```typescript
// src/commands/update.ts (수정된 버전)

export async function updateCommand(args: {
  sessionId: string;
  id: string;
  status: TaskStatus;
}): Promise<CommandResultWithStatus> {
  // 기존 로직으로 작업 업데이트
  const taskList = await loadTaskList(args.sessionId);
  const task = findTask(taskList, args.id);
  
  if (!task) {
    const statusSummary = calculateStatusSummary(taskList);
    return {
      success: false,
      message: `Task ${args.id} not found`,
      currentStatus: taskList,
      statusSummary,
      formattedOutput: formatErrorResult(`Task ${args.id} not found`, taskList, statusSummary)
    };
  }

  task.status = args.status;
  task.updatedAt = new Date().toISOString();
  await saveTaskList(args.sessionId, taskList);

  // 현재 상태 및 요약 계산
  const statusSummary = calculateStatusSummary(taskList);
  
  // 마크다운 출력 생성
  const formattedOutput = formatUpdateResult(task, taskList, statusSummary);

  return {
    success: true,
    message: `Task ${args.id} updated to ${args.status}`,
    currentStatus: taskList,
    statusSummary,
    formattedOutput
  };
}
```

## Error Handling

### 1. Batch Operation Error Strategy

```typescript
// 에러 처리 전략: 부분적 성공 허용

export async function batchCommand(args: {
  sessionId: string;
  operations: BatchOperation[];
}): Promise<BatchCommandResult> {
  const results: BatchResult[] = [];
  
  for (const operation of args.operations) {
    try {
      const result = await executeOperation(args.sessionId, operation);
      results.push(result);
    } catch (error) {
      // 개별 작업 실패해도 계속 진행
      results.push({
        success: false,
        operation,
        message: `Failed: ${error}`,
        error: String(error)
      });
    }
  }

  // 모든 결과 반환 (성공/실패 모두 포함)
  return {
    success: results.every(r => r.success),  // 모두 성공했을 때만 true
    results,
    // ...
  };
}
```

### 2. Error Response Format

```markdown
## ❌ Operation Failed

**Error:** Task 999 not found

**Current Status:**
- Total tasks: 5
- Last successful operation: Task 3 update

**Suggestion:** Please check the task ID and try again.
```

## Testing Strategy

### 1. Unit Tests

```typescript
// tests/commands/batch.test.ts

describe('batch command', () => {
  it('should execute multiple add operations', async () => {
    const result = await batchCommand({
      sessionId: 'test-session',
      operations: [
        { type: 'add', title: 'Task 1' },
        { type: 'add', title: 'Task 2' },
        { type: 'add', title: 'Task 3' }
      ]
    });

    expect(result.success).toBe(true);
    expect(result.summary.succeeded).toBe(3);
    expect(result.summary.failed).toBe(0);
  });

  it('should handle partial failures', async () => {
    const result = await batchCommand({
      sessionId: 'test-session',
      operations: [
        { type: 'add', title: 'Task 1' },
        { type: 'update', id: 'non-existent', status: 'completed' },
        { type: 'add', title: 'Task 2' }
      ]
    });

    expect(result.success).toBe(false);  // 전체 실패로 표시
    expect(result.summary.succeeded).toBe(2);
    expect(result.summary.failed).toBe(1);
  });

  it('should include current status in response', async () => {
    const result = await batchCommand({
      sessionId: 'test-session',
      operations: [{ type: 'add', title: 'New Task' }]
    });

    expect(result.currentStatus).toBeDefined();
    expect(result.statusSummary).toBeDefined();
    expect(result.formattedOutput).toContain('📋 Task Status');
  });
});
```

### 2. Formatter Tests

```typescript
// tests/lib/formatter.test.ts

describe('markdown formatter', () => {
  it('should format progress bar correctly', () => {
    const bar50 = formatProgressBar(50);
    expect(bar50).toContain('█████');
    expect(bar50).toContain('50%');
  });

  it('should include all status icons', () => {
    const output = formatTaskListWithStatus(mockTaskList, mockSummary);
    expect(output).toContain('✅');  // completed
    expect(output).toContain('🔄');  // in_progress
    expect(output).toContain('⏳');  // pending
  });

  it('should format batch results with operation details', () => {
    const output = formatBatchResult(mockResults, mockSummary, mockTaskList);
    expect(output).toContain('✅ Batch Operations Complete');
    expect(output).toContain('📋 Operation Details');
    expect(output).toContain('📋 Task Status');
  });
});
```

## Migration Guide

### For Plugin Users

**Before (기존 방식):**
```typescript
// 여러 번의 개별 호출
await tasks_add({ title: 'Task 1' });
await tasks_add({ title: 'Task 2' });
await tasks_update({ id: '1', status: 'in_progress' });
await tasks_complete({ id: '1' });
```

**After (새로운 방식):**
```typescript
// 단일 배치 호출
await tasks_batch({
  operations: [
    { type: 'add', title: 'Task 1' },
    { type: 'add', title: 'Task 2' },
    { type: 'update', id: '1', status: 'in_progress' },
    { type: 'complete', id: '1' }
  ]
});
```

### Backward Compatibility

- 기존 개별 툴(tasks_add, tasks_update 등)은 그대로 유지
- 응답 형식이 개선되지만 기존 기능은 그대로 동작
- 새로운 `tasks_batch` 툴은 추가 형태로 제공

## Performance Considerations

### 1. Batch Size Limits

```typescript
const MAX_BATCH_SIZE = 50;  // 한 번에 최대 50개 작업

export async function batchCommand(args: {
  sessionId: string;
  operations: BatchOperation[];
}): Promise<BatchCommandResult> {
  if (args.operations.length > MAX_BATCH_SIZE) {
    throw new Error(`Batch size exceeds maximum limit of ${MAX_BATCH_SIZE}`);
  }
  // ...
}
```

### 2. Response Size Optimization

```typescript
// 큰 task list에서 요약 정보만 포함
export function formatTaskListWithStatus(
  taskList: TaskList,
  summary: StatusSummary,
  options?: { includeAllTasks?: boolean }
): string {
  if (taskList.tasks.length > 20 && !options?.includeAllTasks) {
    // 20개 초과 시 요약만 표시
    return formatSummaryOnly(taskList, summary);
  }
  // 전체 목록 표시
  return formatFullTaskList(taskList, summary);
}
```

## Security Considerations

### 1. Input Validation

```typescript
function validateOperation(operation: BatchOperation): void {
  if (!operation.type) {
    throw new Error('Operation type is required');
  }
  
  if (!['add', 'update', 'complete', 'remove'].includes(operation.type)) {
    throw new Error(`Invalid operation type: ${operation.type}`);
  }
  
  if (operation.type === 'add' && !operation.title) {
    throw new Error('Title is required for add operation');
  }
  
  if (['update', 'complete', 'remove'].includes(operation.type) && !operation.id) {
    throw new Error('ID is required for update/complete/remove operations');
  }
}
```

### 2. Rate Limiting (향후 고려)

```typescript
// 향후 배치 작업 빈도 제한을 위한 구조
interface RateLimitConfig {
  maxOperationsPerMinute: number;
  maxBatchSize: number;
  cooldownMs: number;
}
```
