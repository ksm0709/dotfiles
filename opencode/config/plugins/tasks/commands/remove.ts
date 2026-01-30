// src/commands/remove.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';

export interface RemoveArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
  force?: boolean;      // 강제 삭제 (선택)
}

export interface RemoveResult {
  success: boolean;
  taskId: string;
  taskTitle?: string;
  message: string;
}

export async function removeCommand(args: RemoveArgs): Promise<RemoveResult> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      return {
        success: false,
        taskId: args.id,
        message: `No task lists found for session: ${args.sessionId}`
      };
    }

    // Try to find the task in any of the task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      
      // Find the task to get its title
      const findTask = (tasks: any[]): any | null => {
        for (const task of tasks) {
          if (task.id === args.id) return task;
          if (task.subtasks) {
            const found = findTask(task.subtasks);
            if (found) return found;
          }
        }
        return null;
      };

      const taskToRemove = findTask(taskList.tasks);

      if (taskToRemove) {
        const removed = parser.removeTask(taskList, args.id);

        if (removed) {
          // Save updated content
          const updatedContent = parser.generateTaskList(taskList);
          await storage.saveTaskList(args.sessionId, title, updatedContent);
          
          return {
            success: true,
            taskId: args.id,
            taskTitle: taskToRemove.title,
            message: `Task ${args.id} "${taskToRemove.title}" removed`
          };
        }
      }
    }

    return {
      success: false,
      taskId: args.id,
      message: `Task ${args.id} not found`
    };
  } catch (error) {
    throw error;
  }
}
