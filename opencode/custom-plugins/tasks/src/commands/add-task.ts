// src/commands/add-task.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { CommandResultWithStatus, TaskList, TaskDetail, StatusSummary, ToolResponse } from '../types';

export interface AddTaskArgs {
  sessionId: string;    // 세션 ID (필수)
  title: string;        // 작업 제목 (필수)
  details?: string;     // 세부사항 (쉼표로 구분, 선택)
}

export interface AddTaskResult extends CommandResultWithStatus {
  title: string;
  details: string[];
  taskId?: string;
  response: ToolResponse;
}

export async function addTaskCommand(args: AddTaskArgs): Promise<AddTaskResult> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    const files = await storage.listTaskFiles(args.sessionId);
    
    if (files.length === 0) {
      const emptyStatus: StatusSummary = {
        agent: 'unknown',
        title: 'Empty',
        total: 0,
        completed: 0,
        inProgress: 0,
        pending: 0,
        completionRate: 0
      };

      const emptyTaskList: TaskList = {
        title: 'Empty',
        agent: 'unknown',
        createdAt: new Date().toISOString(),
        sessionId: args.sessionId,
        tasks: []
      };

      const errorResponse: ToolResponse = {
        title: `Failed to add: ${args.title}`,
        output: `❌ No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: 'add',
          message: `No task lists found for session: ${args.sessionId}`
        }
      };

      return {
        success: false,
        title: args.title,
        details: [],
        message: `No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `❌ No task lists found for session: ${args.sessionId}. Create a task list first with: tasks init`,
        response: errorResponse
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

    const added = parser.addTask(taskList, args.title, details);

    if (added) {
      // Save updated content
      const updatedContent = parser.generateTaskList(taskList);
      await storage.saveTaskList(args.sessionId, listTitle, updatedContent);

      // 상태 요약 계산
      const statusSummary = formatter.calculateStatusSummary(taskList);

      // 새로 추가된 작업 찾기
      const newTask = findNewlyAddedTask(taskList, args.title);

      // 포맷팅된 출력 생성
      const formattedOutput = formatter.formatAddResult(newTask, taskList, statusSummary);

      // Native UI Response 생성
      const response: ToolResponse = {
        title: `Added: ${args.title} (ID: ${newTask.id})`,
        output: formattedOutput,
        metadata: {
          taskList: taskList,
          tasks: taskList.tasks,
          summary: statusSummary,
          operation: 'add',
          taskId: newTask.id,
          message: details.length > 0 
            ? `Task added: ${args.title} with ${details.length} detail(s)`
            : `Task added: ${args.title}`
        }
      };
      
      return {
        success: true,
        title: args.title,
        details,
        taskId: newTask.id,
        message: details.length > 0 
          ? `Task added: ${args.title} with ${details.length} detail(s)`
          : `Task added: ${args.title}`,
        currentStatus: taskList,
        statusSummary,
        formattedOutput,
        response
      };
    } else {
      throw new Error(`Failed to add task: ${args.title}`);
    }
  } catch (error) {
    throw error;
  }
}

/**
 * 새로 추가된 작업 찾기 (제목으로 식별)
 */
function findNewlyAddedTask(taskList: TaskList, title: string): TaskDetail {
  // Flat structure: search only in top-level tasks
  const found = taskList.tasks.find(task => task.title === title);
  
  if (found) {
    return found;
  }

  // 못 찾으면 기본값 반환
  return {
    id: 'unknown',
    title,
    status: 'pending',
    details: [],
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString()
  };
}
