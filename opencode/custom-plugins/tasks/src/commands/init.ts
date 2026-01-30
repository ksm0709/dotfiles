// src/commands/init.ts
// Session-based task list initialization

import { Storage } from '../lib/storage';
import { Parser } from '../lib/parser';
import { TaskList, TaskDetail } from '../types';

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
  subtasks?: TaskInput[];
}

export interface InitResult {
  title: string;
  agent: string;
  fileName: string;
  taskIds: string[];
  totalTasks: number;
}

/**
 * 작업 목록 초기화
 * ~/.local/share/opencode/tasks/{sessionId}/{agent}-{title}.md 생성
 * @returns 생성된 작업 목록 정보와 작업 ID 목록
 */
export async function initCommand(args: InitArgs): Promise<InitResult> {
  const storage = new Storage();
  const parser = new Parser();

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

    return {
      title: args.title,
      agent: args.agent,
      fileName,
      taskIds,
      totalTasks: countTasks(taskList.tasks)
    };
  } catch (error) {
    console.error('❌ Failed to initialize task list:', error);
    throw error;
  }
}

/**
 * 작업 ID 목록 추출 (하위 작업 포함)
 */
function extractTaskIds(tasks: TaskDetail[]): string[] {
  const ids: string[] = [];
  
  for (const task of tasks) {
    ids.push(task.id);
    if (task.subtasks && task.subtasks.length > 0) {
      ids.push(...extractTaskIds(task.subtasks));
    }
  }
  
  return ids;
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
    subtasks: input.subtasks ? convertTasks(input.subtasks) : [],
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
 * 전체 작업 수 계산 (하위 작업 포함)
 */
function countTasks(tasks: TaskDetail[]): number {
  let count = 0;
  for (const task of tasks) {
    count++;
    if (task.subtasks && task.subtasks.length > 0) {
      count += countTasks(task.subtasks);
    }
  }
  return count;
}
