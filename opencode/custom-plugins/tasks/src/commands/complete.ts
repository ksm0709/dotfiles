// src/commands/complete.ts

import { updateCommand } from './update';

export interface CompleteArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
}

export interface CompleteResult {
  success: boolean;
  taskId: string;
  message: string;
}

export async function completeCommand(args: CompleteArgs): Promise<CompleteResult> {
  // Complete is just a special case of update with status 'completed'
  const result = await updateCommand({
    sessionId: args.sessionId,
    id: args.id,
    status: 'completed'
  });
  
  if (result.success) {
    return {
      success: true,
      taskId: args.id,
      message: `Task ${args.id} marked as completed`
    };
  } else {
    throw new Error(result.message);
  }
}
