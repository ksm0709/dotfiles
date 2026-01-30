// src/commands/update.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { TaskStatus } from '../types';

export interface UpdateArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
  status: TaskStatus;   // 새 상태 (필수)
}

export interface UpdateResult {
  success: boolean;
  taskId: string;
  status: TaskStatus;
  message: string;
}

export async function updateCommand(args: UpdateArgs): Promise<UpdateResult> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      return {
        success: false,
        taskId: args.id,
        status: args.status,
        message: `No task lists found for session: ${args.sessionId}`
      };
    }

    // Try to find and update the task in any of the task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      const updated = parser.updateTaskStatus(taskList, args.id, args.status);

      if (updated) {
        // Save updated content
        const updatedContent = parser.generateTaskList(taskList);
        await storage.saveTaskList(args.sessionId, title, updatedContent);
        
        return {
          success: true,
          taskId: args.id,
          status: args.status,
          message: `Task ${args.id} status updated to: ${args.status}`
        };
      }
    }

    return {
      success: false,
      taskId: args.id,
      status: args.status,
      message: `Task ${args.id} not found`
    };
  } catch (error) {
    throw error;
  }
}
