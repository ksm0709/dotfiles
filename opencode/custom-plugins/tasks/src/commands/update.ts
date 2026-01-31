// src/commands/update.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskStatus, CommandResultWithStatus, TaskList, StatusSummary, ToolResponse } from '../types';

export interface UpdateArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
  status: TaskStatus;   // 새 상태 (필수)
}

export interface UpdateResult extends CommandResultWithStatus {
  taskId: string;
  status: TaskStatus;
  response: ToolResponse;
}

export async function updateCommand(args: UpdateArgs): Promise<UpdateResult> {
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
        title: `Failed to update: Task ${args.id}`,
        output: `❌ No task lists found for session: ${args.sessionId}`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: 'update',
          taskId: args.id,
          status: args.status,
          message: `No task lists found for session: ${args.sessionId}`
        }
      };

      return {
        success: false,
        taskId: args.id,
        status: args.status,
        message: `No task lists found for session: ${args.sessionId}`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `❌ No task lists found for session: ${args.sessionId}`,
        response: errorResponse
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

        // 상태 요약 계산
        const statusSummary = formatter.calculateStatusSummary(taskList);

        // 포맷팅된 출력 생성
        const formattedOutput = formatter.formatUpdateResult(args.id, args.status, taskList, statusSummary);

        // Native UI Response 생성
        const response: ToolResponse = {
          title: `Updated: Task ${args.id} → ${args.status}`,
          output: formattedOutput,
          metadata: {
            taskList: taskList,
            tasks: taskList.tasks,
            summary: statusSummary,
            operation: 'update',
            taskId: args.id,
            status: args.status,
            message: `Task ${args.id} status updated to: ${args.status}`
          }
        };
        
        return {
          success: true,
          taskId: args.id,
          status: args.status,
          message: `Task ${args.id} status updated to: ${args.status}`,
          currentStatus: taskList,
          statusSummary,
          formattedOutput,
          response
        };
      }
    }

    // 작업을 찾지 못한 경우 - 마지막으로 확인한 taskList 사용 (있는 경우)
    const lastTaskList = files.length > 0 ? await (async () => {
      const file = files[0];
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      if (content) {
        return parser.parseTaskList(content);
      }
      return null;
    })() : null;

    const statusSummary = lastTaskList 
      ? formatter.calculateStatusSummary(lastTaskList)
      : {
          agent: 'unknown',
          title: 'Empty',
          total: 0,
          completed: 0,
          inProgress: 0,
          pending: 0,
          completionRate: 0
        };

    const notFoundTaskList = lastTaskList || {
      title: 'Empty',
      agent: 'unknown',
      createdAt: new Date().toISOString(),
      sessionId: args.sessionId,
      tasks: []
    };

    const notFoundResponse: ToolResponse = {
      title: `Failed to update: Task ${args.id} not found`,
      output: `❌ Task ${args.id} not found`,
      metadata: {
        taskList: notFoundTaskList,
        tasks: notFoundTaskList.tasks,
        summary: statusSummary,
        operation: 'update',
        taskId: args.id,
        status: args.status,
        message: `Task ${args.id} not found`
      }
    };

    return {
      success: false,
      taskId: args.id,
      status: args.status,
      message: `Task ${args.id} not found`,
      currentStatus: notFoundTaskList,
      statusSummary,
      formattedOutput: `❌ Task ${args.id} not found`,
      response: notFoundResponse
    };
  } catch (error) {
    throw error;
  }
}
