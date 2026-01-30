// src/commands/add-task.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';

export interface AddTaskArgs {
  sessionId: string;    // 세션 ID (필수)
  parent?: string;      // 부모 작업 ID (선택)
  title: string;        // 작업 제목 (필수)
  details?: string;     // 세부사항 (쉼표로 구분, 선택)
}

export interface AddTaskResult {
  success: boolean;
  title: string;
  parent?: string;
  details: string[];
  message: string;
}

export async function addTaskCommand(args: AddTaskArgs): Promise<AddTaskResult> {
  const storage = new Storage();
  const parser = new Parser();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      return {
        success: false,
        title: args.title,
        parent: args.parent,
        details: [],
        message: `No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`
      };
    }

    // For simplicity, add to the first task list found
    const file = files[0];
    const listTitle = file.replace('.md', '');
    const content = await storage.readTaskList(args.sessionId, listTitle);
    
    if (!content) {
      throw new Error('Failed to read task list');
    }

    const taskList = parser.parseTaskList(content);
    
    // Parse details from comma-separated string
    const details = args.details ? args.details.split(',').map(d => d.trim()).filter(d => d) : [];

    // Handle empty or invalid parent values
    const parent = args.parent && args.parent.trim() !== '' && args.parent !== 'root' 
      ? args.parent 
      : undefined;

    const added = parser.addTask(taskList, parent, args.title, details);

    if (added) {
      // Save updated content
      const updatedContent = parser.generateTaskList(taskList);
      await storage.saveTaskList(args.sessionId, listTitle, updatedContent);
      
      return {
        success: true,
        title: args.title,
        parent,
        details,
        message: details.length > 0 
          ? `Task added: ${args.title} with ${details.length} detail(s)`
          : `Task added: ${args.title}`
      };
    } else {
      throw new Error(`Failed to add task. Parent task ${args.parent} not found.`);
    }
  } catch (error) {
    throw error;
  }
}
