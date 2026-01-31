// src/commands/batch.ts
// Batch operations for task management - process multiple operations in one call

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { addTaskCommand } from './add-task';
import { updateCommand } from './update';
import { completeCommand } from './complete';
import { removeCommand } from './remove';
import {
  BatchOperation,
  BatchResult,
  BatchCommandResult,
  TaskList,
  StatusSummary,
  ToolResponse
} from '../types';
import { Formatter } from '../lib/formatter';

export interface BatchArgs {
  sessionId: string;      // 세션 ID (필수)
  operations: BatchOperation[];  // 작업 목록 (필수)
}

const MAX_OPERATIONS = 50;  // 최대 작업 수 제한

/**
 * 여러 작업을 한 번에 처리하는 배치 명령어
 * 최대 50개 작업 제한, 부분적 실패 허용
 */
export async function batchCommand(args: BatchArgs): Promise<BatchCommandResult> {
  const storage = new Storage();
  const formatter = new Formatter();

  try {
    // 입력값 검증
    validateOperations(args.operations);

    const results: BatchResult[] = [];

    // 각 작업 순차 처리
    for (const operation of args.operations) {
      try {
        const result = await executeOperation(args.sessionId, operation);
        results.push(result);
      } catch (error) {
        // 부분적 실패 허용 - 개별 작업 실패해도 계속 진행
        results.push({
          success: false,
          operation,
          message: `Operation failed: ${error instanceof Error ? error.message : String(error)}`,
          error: String(error)
        });
      }
    }

    // 현재 상태 조회
    const currentStatus = await getCurrentStatus(args.sessionId, storage);
    const statusSummary = formatter.calculateStatusSummary(currentStatus);

    // 요약 계산
    const summary = {
      total: results.length,
      succeeded: results.filter(r => r.success).length,
      failed: results.filter(r => !r.success).length
    };

    // 마크다운 출력 생성
    const formattedOutput = formatter.formatBatchResult(results, summary, currentStatus, statusSummary);

    // Native UI Response 생성
    const response: ToolResponse = {
      title: `Batch: ${summary.succeeded} succeeded, ${summary.failed} failed`,
      output: formattedOutput,
      metadata: {
        taskList: currentStatus,
        tasks: currentStatus.tasks,
        summary: statusSummary,
        operation: 'batch',
        results: results,
        batchSummary: summary,
        message: summary.failed === 0 
          ? `All ${summary.total} operations completed successfully` 
          : `${summary.succeeded} succeeded, ${summary.failed} failed out of ${summary.total} operations`
      }
    };

    return {
      success: summary.failed === 0,  // 전체 성공 여부
      results,
      summary,
      currentStatus,
      formattedOutput,
      response
    };

  } catch (error) {
    // 전체 작업 실패 (검증 실패 등)
    throw error;
  }
}

/**
 * 작업 목록 유효성 검사
 */
function validateOperations(operations: BatchOperation[]): void {
  if (!Array.isArray(operations)) {
    throw new Error('Operations must be an array');
  }

  if (operations.length === 0) {
    throw new Error('No operations provided');
  }

  if (operations.length > MAX_OPERATIONS) {
    throw new Error(`Too many operations. Maximum is ${MAX_OPERATIONS}, got ${operations.length}`);
  }

  for (let i = 0; i < operations.length; i++) {
    const op = operations[i];

    // type 필수 검증
    if (!op.type) {
      throw new Error(`Operation ${i + 1}: type is required`);
    }

    // add 작업에는 title 필수
    if (op.type === 'add' && !op.title) {
      throw new Error(`Operation ${i + 1}: title is required for add operation`);
    }

    // update/complete/remove 작업에는 id 필수
    if ((op.type === 'update' || op.type === 'complete' || op.type === 'remove') && !op.id) {
      throw new Error(`Operation ${i + 1}: id is required for ${op.type} operation`);
    }

    // update 작업에는 status 필수
    if (op.type === 'update' && !op.status) {
      throw new Error(`Operation ${i + 1}: status is required for update operation`);
    }
  }
}

/**
 * 개별 작업 실행
 */
async function executeOperation(
  sessionId: string,
  operation: BatchOperation
): Promise<BatchResult> {
  switch (operation.type) {
    case 'add':
      return await executeAdd(sessionId, operation);
    case 'update':
      return await executeUpdate(sessionId, operation);
    case 'complete':
      return await executeComplete(sessionId, operation);
    case 'remove':
      return await executeRemove(sessionId, operation);
    default:
      throw new Error(`Unknown operation type: ${operation.type}`);
  }
}

/**
 * add 작업 실행
 */
async function executeAdd(
  sessionId: string,
  operation: BatchOperation
): Promise<BatchResult> {
  const result = await addTaskCommand({
    sessionId,
    title: operation.title!,
    parent: operation.parent
  });

  return {
    success: result.success,
    operation,
    taskId: undefined,  // add는 새로운 ID를 반환하지 않음 (Parser가 생성)
    message: result.message
  };
}

/**
 * update 작업 실행
 */
async function executeUpdate(
  sessionId: string,
  operation: BatchOperation
): Promise<BatchResult> {
  const result = await updateCommand({
    sessionId,
    id: operation.id!,
    status: operation.status!
  });

  return {
    success: result.success,
    operation,
    taskId: result.taskId,
    message: result.message
  };
}

/**
 * complete 작업 실행
 */
async function executeComplete(
  sessionId: string,
  operation: BatchOperation
): Promise<BatchResult> {
  const result = await completeCommand({
    sessionId,
    id: operation.id!
  });

  return {
    success: result.success,
    operation,
    taskId: result.taskId,
    message: result.message
  };
}

/**
 * remove 작업 실행
 */
async function executeRemove(
  sessionId: string,
  operation: BatchOperation
): Promise<BatchResult> {
  const result = await removeCommand({
    sessionId,
    id: operation.id!,
    force: true
  });

  return {
    success: result.success,
    operation,
    taskId: result.taskId,
    taskTitle: result.taskTitle,
    message: result.message
  };
}

/**
 * 현재 작업 목록 상태 조회
 */
async function getCurrentStatus(
  sessionId: string,
  storage: Storage
): Promise<TaskList> {
  const files = await storage.listTaskFiles(sessionId);

  if (files.length === 0) {
    // 작업 목록이 없으면 빈 목록 반환
    return {
      title: 'Empty',
      agent: 'unknown',
      createdAt: new Date().toISOString(),
      sessionId,
      tasks: []
    };
  }

  // 첫 번째 작업 목록을 현재 상태로 사용
  const file = files[0];
  const title = file.replace('.md', '');
  const content = await storage.readTaskList(sessionId, title);

  if (!content) {
    throw new Error('Failed to read task list');
  }

  const parser = new Parser();
  return parser.parseTaskList(content);
}
