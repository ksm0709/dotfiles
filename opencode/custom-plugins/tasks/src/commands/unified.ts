// src/commands/unified.ts
// Unified task management command - handles all operations

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import {
  TaskList,
  TaskDetail,
  StatusSummary,
  ToolResponse,
  UnifiedOperation,
  UnifiedCommandParams,
  UnifiedCommandResult,
  UnifiedOperationResult
} from '../types';
import { initCommand } from './init';
import { addTaskCommand } from './add-task';
import { updateCommand } from './update';
import { completeCommand } from './complete';
import { removeCommand } from './remove';

/**
 * Unified command handler - processes multiple operations in batch
 * Shows current session status after execution
 */
export async function unifiedCommand(
  params: UnifiedCommandParams
): Promise<UnifiedCommandResult> {
  const { sessionId, operations } = params;
  
  // Validate inputs
  if (!sessionId || typeof sessionId !== 'string') {
    throw new Error('Invalid sessionId: must be a non-empty string');
  }
  
  if (!Array.isArray(operations)) {
    throw new Error('Invalid operations: must be an array');
  }
  
  if (operations.length === 0) {
    throw new Error('No operations provided: operations array cannot be empty');
  }
  
  if (operations.length > 50) {
    throw new Error(`Too many operations: maximum 50 allowed, got ${operations.length}`);
  }
  
  // Validate each operation
  for (let i = 0; i < operations.length; i++) {
    const op = operations[i];
    if (!op || typeof op !== 'object') {
      throw new Error(`Invalid operation at index ${i}: must be an object`);
    }
    if (!op.type || !['init', 'add', 'update', 'complete', 'remove'].includes(op.type)) {
      throw new Error(`Invalid operation type at index ${i}: must be one of init, add, update, complete, remove`);
    }
  }
  
  const storage = new Storage();
  const formatter = new Formatter();

  const results: UnifiedOperationResult[] = [];
  let succeeded = 0;
  let failed = 0;

  // Execute each operation with error handling
  for (const operation of operations) {
    const result = await executeOperationWithErrorHandling(sessionId, operation);
    results.push(result);
    if (result.success) {
      succeeded++;
    } else {
      failed++;
    }
  }

  // Get current session status
  const currentStatus = await getCurrentSessionStatus(sessionId, storage);

  // Format response
  const response = formatUnifiedResponse(
    results,
    { total: operations.length, succeeded, failed },
    currentStatus,
    formatter
  );

  return {
    success: failed === 0,
    results,
    summary: {
      total: operations.length,
      succeeded,
      failed
    },
    currentStatus,
    response
  };
}

/**
 * Execute operation with centralized error handling
 */
async function executeOperationWithErrorHandling(
  sessionId: string,
  operation: UnifiedOperation
): Promise<UnifiedOperationResult> {
  try {
    return await executeOperation(sessionId, operation);
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : String(error);
    console.error(`❌ Operation ${operation.type} failed:`, errorMessage);
    
    return {
      success: false,
      operation,
      message: `Failed to execute ${operation.type}: ${errorMessage}`,
      error: errorMessage
    };
  }
}

/**
 * Execute a single operation
 */
async function executeOperation(
  sessionId: string,
  operation: UnifiedOperation
): Promise<UnifiedOperationResult> {
  switch (operation.type) {
    case 'init':
      if (!operation.agent || !operation.title) {
        return {
          success: false,
          operation,
          message: 'Missing required fields: agent and title'
        };
      }
      const initResult = await initCommand({
        sessionId,
        agent: operation.agent,
        title: operation.title
      });
      return {
        success: initResult.success,
        operation,
        message: initResult.message,
        taskId: initResult.taskIds?.[0]
      };

    case 'add':
      if (!operation.title) {
        return {
          success: false,
          operation,
          message: 'Missing required field: title'
        };
      }
      const addResult = await addTaskCommand({
        sessionId,
        title: operation.title,
        parent: operation.parent
      });
      return {
        success: addResult.success,
        operation,
        message: addResult.message,
        taskId: addResult.taskId,
        taskTitle: addResult.title
      };

    case 'update':
      if (!operation.id || !operation.status) {
        return {
          success: false,
          operation,
          message: 'Missing required fields: id and status'
        };
      }
      const updateResult = await updateCommand({
        sessionId,
        id: operation.id,
        status: operation.status
      });
      return {
        success: updateResult.success,
        operation,
        message: updateResult.message,
        taskId: operation.id
      };

    case 'complete':
      if (!operation.id) {
        return {
          success: false,
          operation,
          message: 'Missing required field: id'
        };
      }
      const completeResult = await completeCommand({
        sessionId,
        id: operation.id
      });
      return {
        success: completeResult.success,
        operation,
        message: completeResult.message,
        taskId: operation.id
      };

    case 'remove':
      if (!operation.id) {
        return {
          success: false,
          operation,
          message: 'Missing required field: id'
        };
      }
      const removeResult = await removeCommand({
        sessionId,
        id: operation.id,
        force: true
      });
      return {
        success: removeResult.success,
        operation,
        message: removeResult.message,
        taskId: operation.id
      };

    default:
      // Exhaustive type checking - TypeScript will error if a new operation type is added
      // but not handled in this switch statement
      const _exhaustiveCheck: never = operation as never;
      return {
        success: false,
        operation: operation,
        message: `Unknown operation type: ${(operation as UnifiedOperation).type}`
      };
  }
}

/**
 * Get current session status
 */
async function getCurrentSessionStatus(
  sessionId: string,
  storage: Storage
): Promise<TaskList | null> {
  try {
    const files = await storage.listTaskFiles(sessionId);
    if (files.length === 0) {
      return null;
    }

    // Read the first (and typically only) task list
    const parser = new Parser();
    const content = await storage.readTaskList(sessionId, files[0].replace('.md', ''));
    if (!content) {
      return null;
    }

    return parser.parseTaskList(content);
  } catch (error) {
    console.error('Failed to get current session status:', error);
    return null;
  }
}

/**
 * Format unified response with current session status
 */
function formatUnifiedResponse(
  results: UnifiedOperationResult[],
  summary: { total: number; succeeded: number; failed: number },
  currentStatus: TaskList | null,
  formatter: Formatter
): ToolResponse {
  const lines: string[] = [];

  // 작업 목록 현황 요약 (맨 처음에 표시)
  if (currentStatus) {
    const statusSummary = formatter.calculateStatusSummary(currentStatus);
    const total = statusSummary.total;
    const completed = statusSummary.completed;
    const inProgress = statusSummary.inProgress;
    const pending = statusSummary.pending;
    
    // 간결한 요약 라인: "📋 total 5, in-progress 2, done 1"
    lines.push(`📋 total ${total}, in-progress ${inProgress}, done ${completed}`);
    lines.push('');
  }

  // Separate results into failed and succeeded (single pass)
  const failedResults: UnifiedOperationResult[] = [];
  const succeededResults: UnifiedOperationResult[] = [];
  
  for (const result of results) {
    if (result.success) {
      succeededResults.push(result);
    } else {
      failedResults.push(result);
    }
  }

  // Failed operations - 최상단에 노출 (실패한 경우에만)
  if (failedResults.length > 0) {
    lines.push('## ❌ 실패한 작업');
    lines.push('');
    for (const result of failedResults) {
      lines.push(`- ❌ **${result.operation.type}**: ${result.message}`);
      if (result.error) {
        lines.push(`  - 상세: ${result.error}`);
      }
    }
    lines.push('');
  }

  // 작업 목록 표시 (한 줄 요약 아래에 체크박스 형태로만)
  if (currentStatus) {
    lines.push('');
    for (const task of currentStatus.tasks) {
      lines.push(...formatTaskCheckbox(task, 0));
    }
    lines.push(''); // tasks list 마지막에 빈 줄 1줄 추가
  } else {
    lines.push('ℹ️ 현재 세션에 작업 목록이 없습니다.');
  }

  const output = lines.join('\n');

  return {
    title: `작업 완료: ${summary.succeeded}/${summary.total} 성공`,
    output,
    metadata: {
      results: results,
      // Operation execution summary (not task list status)
      // completed: operations that succeeded
      // failed: operations that failed
      // total: total operations attempted
      summary: currentStatus ? {
        agent: currentStatus.agent,
        title: currentStatus.title,
        total: summary.total,
        completed: summary.succeeded,
        inProgress: 0,
        pending: summary.failed,
        completionRate: summary.total > 0 ? Math.round((summary.succeeded / summary.total) * 100) : 0
      } : undefined,
      taskList: currentStatus || undefined,
      operation: 'unified'
    }
  };
}

/**
 * 작업을 체크박스 형태로 포맷팅
 */
function formatTaskCheckbox(task: any, indent: number): string[] {
  const lines: string[] = [];
  const prefix = '  '.repeat(indent);
  const checkbox = task.status === 'completed' ? '[x]' : '[ ]';
  const statusEmoji = task.status === 'completed' ? '✅' : 
                     task.status === 'in_progress' ? '🔄' : '⏳';
  
  lines.push(`${prefix}- ${checkbox} ${statusEmoji} **${task.id}**. ${task.title}`);

  if (task.subtasks && task.subtasks.length > 0) {
    for (const subtask of task.subtasks) {
      lines.push(...formatTaskCheckbox(subtask, indent + 1));
    }
  }

  return lines;
}
