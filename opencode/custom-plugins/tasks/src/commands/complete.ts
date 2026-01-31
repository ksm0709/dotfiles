// src/commands/complete.ts

import { updateCommand } from './update';
import { CommandResultWithStatus, TaskList, StatusSummary, ToolResponse } from '../types';

export interface CompleteArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
}

export interface CompleteResult extends CommandResultWithStatus {
  taskId: string;
  response: ToolResponse;
}

export async function completeCommand(args: CompleteArgs): Promise<CompleteResult> {
  // Complete is just a special case of update with status 'completed'
  const result = await updateCommand({
    sessionId: args.sessionId,
    id: args.id,
    status: 'completed'
  });
  
  // Update command의 response를 기반으로 Complete용 response 생성
  const completeResponse: ToolResponse = {
    title: result.success 
      ? `Completed: Task ${args.id}` 
      : `Failed to complete: Task ${args.id}`,
    output: result.formattedOutput,
    metadata: {
      taskList: result.currentStatus,
      tasks: result.currentStatus.tasks,
      summary: result.statusSummary,
      operation: 'complete',
      taskId: args.id,
      status: 'completed',
      message: result.success 
        ? `Task ${args.id} marked as completed` 
        : result.message
    }
  };
  
  if (result.success) {
    return {
      success: true,
      taskId: args.id,
      message: `Task ${args.id} marked as completed`,
      currentStatus: result.currentStatus,
      statusSummary: result.statusSummary,
      formattedOutput: result.formattedOutput,
      response: completeResponse
    };
  } else {
    // 실패 시에도 현황 정보 포함하여 반환
    return {
      success: false,
      taskId: args.id,
      message: result.message,
      currentStatus: result.currentStatus,
      statusSummary: result.statusSummary,
      formattedOutput: result.formattedOutput,
      response: completeResponse
    };
  }
}
