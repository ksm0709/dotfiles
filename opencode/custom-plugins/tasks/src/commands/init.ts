// src/commands/init.ts
// Session-based task list initialization

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { Formatter } from '../lib/formatter';
import { TaskList, TaskDetail, CommandResultWithStatus, StatusSummary, ToolResponse } from '../types';

export interface InitArgs {
  sessionId: string;    // 세션 ID (필수)
  agent: string;        // 에이전트 이름 (필수)
  title: string;        // 작업 목록 제목 (필수)
  tasks?: TaskInput[];  // 초기 작업 목록 (선택)
}

export interface TaskInput {
  id: string;
  title: string;
  status?: 'pending' | 'in_progress' | 'completed';
  details?: string[];
}

export interface InitResult extends CommandResultWithStatus {
  title: string;
  agent: string;
  fileName: string;
  taskIds: string[];
  totalTasks: number;
  response: ToolResponse;
}

/**
 * 작업 목록 초기화
 * ~/.local/share/opencode/tasks/{sessionId}/{agent}-{title}.md 생성
 * @returns 생성된 작업 목록 정보와 작업 ID 목록, 현황 정보
 */
export async function initCommand(args: InitArgs): Promise<InitResult> {
  const storage = new Storage();
  const parser = new Parser();
  const formatter = new Formatter();

  try {
    // 작업 목록 생성
    const taskList = createTaskList(args);

    // Generate markdown content
    const markdownContent = parser.generateTaskList(taskList);

    // 파일명 생성: {agent}-{title}.md
    const fileName = generateFileName(args.agent, args.title);

    // Save to storage (sessionId 기반)
    await storage.saveTaskList(args.sessionId, fileName, markdownContent);

    // 작업 ID 목록 추출
    const taskIds = extractTaskIds(taskList.tasks);
    const totalTasks = countTasks(taskList.tasks);

    // 상태 요약 계산
    const statusSummary = formatter.calculateStatusSummary(taskList);

    // 포맷팅된 출력 생성
    const formattedOutput = formatInitResult(args, taskList, statusSummary, fileName, taskIds, totalTasks, formatter);

    // Native UI Response 생성
    const response: ToolResponse = {
      title: `Initialized: ${args.title} (${totalTasks} tasks)`,
      output: formattedOutput,
      metadata: {
        taskList: taskList,
        tasks: taskList.tasks,
        summary: statusSummary,
        operation: 'init',
        message: `Task list "${args.title}" initialized successfully for agent "${args.agent}"`
      }
    };

    return {
      success: true,
      title: args.title,
      agent: args.agent,
      fileName,
      taskIds,
      totalTasks,
      message: `Task list "${args.title}" initialized successfully for agent "${args.agent}"`,
      currentStatus: taskList,
      statusSummary,
      formattedOutput,
      response
    };
  } catch (error) {
    console.error('❌ Failed to initialize task list:', error);
    throw error;
  }
}

/**
 * init 작업 결과 포맷팅
 */
function formatInitResult(
  args: InitArgs,
  taskList: TaskList,
  summary: StatusSummary,
  fileName: string,
  taskIds: string[],
  totalTasks: number,
  formatter: Formatter
): string {
  const lines: string[] = [];

  lines.push(`# ✅ Task List Initialized`);
  lines.push('');
  lines.push(`**작업 목록**: ${args.title}`);
  lines.push(`**에이전트**: ${args.agent}`);
  lines.push(`**파일명**: ${fileName}.md`);
  lines.push(`**총 작업**: ${totalTasks}`);
  if (taskIds.length > 0) {
    lines.push(`**작업 ID**: ${taskIds.join(', ')}`);
  }
  lines.push('');
  lines.push('---');
  lines.push('');
  lines.push(formatter.formatTaskListWithStatus(taskList, summary));

  return lines.join('\n');
}

/**
 * 작업 ID 목록 추출 (flat structure)
 */
function extractTaskIds(tasks: TaskDetail[]): string[] {
  return tasks.map(task => task.id);
}

/**
 * 파일명 생성: agent-title 조합
 */
function generateFileName(agent: string, title: string): string {
  return `${agent}-${title}`
    .toLowerCase()
    .replace(/[^a-z0-9가-힣\s-]/g, '')
    .replace(/\s+/g, '-')
    .substring(0, 50);
}

/**
 * TaskInput 배열을 TaskDetail 배열로 변환
 */
function convertTasks(inputs: TaskInput[] | undefined): TaskDetail[] {
  if (!inputs || inputs.length === 0) {
    return [];
  }

  return inputs.map(input => ({
    id: input.id,
    title: input.title,
    status: input.status || 'pending',
    details: input.details || [],
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString()
  }));
}

/**
 * TaskList 객체 생성
 */
function createTaskList(args: InitArgs): TaskList {
  const tasks = convertTasks(args.tasks);

  return {
    title: args.title,
    agent: args.agent,
    createdAt: new Date().toISOString(),
    sessionId: args.sessionId,
    tasks,
    currentPhase: tasks.length > 0 ? tasks[0].title : undefined
  };
}

/**
 * 전체 작업 수 계산 (flat structure)
 */
function countTasks(tasks: TaskDetail[]): number {
  return tasks.length;
}
