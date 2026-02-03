// src/commands/remove.ts

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { CommandResultWithStatus, TaskList, StatusSummary, TaskDetail, ToolResponse } from '../types';

export interface RemoveArgs {
  sessionId: string;    // 세션 ID (필수)
  id: string;           // 작업 ID (필수)
  force?: boolean;      // 강제 삭제 (선택)
}

export interface RemoveResult extends CommandResultWithStatus {
  taskId: string;
  taskTitle?: string;
  response: ToolResponse;
}

export async function removeCommand(args: RemoveArgs): Promise<RemoveResult> {
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
        title: `Failed to remove: Task ${args.id}`,
        output: `❌ No task lists found for session: ${args.sessionId}`,
        metadata: {
          taskList: emptyTaskList,
          tasks: [],
          summary: emptyStatus,
          operation: 'remove',
          taskId: args.id,
          message: `No task lists found for session: ${args.sessionId}`
        }
      };

      return {
        success: false,
        taskId: args.id,
        message: `No task lists found for session: ${args.sessionId}`,
        currentStatus: emptyTaskList,
        statusSummary: emptyStatus,
        formattedOutput: `❌ No task lists found for session: ${args.sessionId}`,
        response: errorResponse
      };
    }

    // Try to find the task in any of the task lists
    for (const file of files) {
      const title = file.replace('.md', '');
      const content = await storage.readTaskList(args.sessionId, title);
      
      if (!content) continue;

      const taskList = parser.parseTaskList(content);
      
      // Find the task to get its title (flat structure)
      const taskToRemove = taskList.tasks.find(task => task.id === args.id) || null;

      if (taskToRemove) {
        const removed = parser.removeTask(taskList, args.id);

        if (removed) {
          // Save updated content
          const updatedContent = parser.generateTaskList(taskList);
          await storage.saveTaskList(args.sessionId, title, updatedContent);

          // 상태 요약 계산
          const statusSummary = formatter.calculateStatusSummary(taskList);

          // 포맷팅된 출력 생성
          const formattedOutput = formatRemoveResult(args.id, taskToRemove.title, taskList, statusSummary, formatter);

          // Native UI Response 생성
          const response: ToolResponse = {
            title: `Removed: Task ${args.id}`,
            output: formattedOutput,
            metadata: {
              taskList: taskList,
              tasks: taskList.tasks,
              summary: statusSummary,
              operation: 'remove',
              taskId: args.id,
              message: `Task ${args.id} "${taskToRemove.title}" removed`
            }
          };
          
          return {
            success: true,
            taskId: args.id,
            taskTitle: taskToRemove.title,
            message: `Task ${args.id} "${taskToRemove.title}" removed`,
            currentStatus: taskList,
            statusSummary,
            formattedOutput,
            response
          };
        }
      }
    }

    // 작업을 찾지 못한 경우 - 마지막으로 확인한 taskList 사용
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
      title: `Failed to remove: Task ${args.id} not found`,
      output: `❌ Task ${args.id} not found`,
      metadata: {
        taskList: notFoundTaskList,
        tasks: notFoundTaskList.tasks,
        summary: statusSummary,
        operation: 'remove',
        taskId: args.id,
        message: `Task ${args.id} not found`
      }
    };

    return {
      success: false,
      taskId: args.id,
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

/**
 * remove 작업 결과 포맷팅
 */
function formatRemoveResult(
  taskId: string,
  taskTitle: string,
  taskList: TaskList,
  summary: StatusSummary,
  formatter: Formatter
): string {
  const lines: string[] = [];

  lines.push(`✅ 작업 삭제 완료`);
  lines.push('');
  lines.push(`**ID**: ${taskId}`);
  lines.push(`**제목**: ${taskTitle}`);
  lines.push('');
  lines.push('---');
  lines.push('');
  lines.push(formatter.formatTaskListWithStatus(taskList, summary));

  return lines.join('\n');
}
