// src/types/index.ts

export type TaskStatus = 'pending' | 'in_progress' | 'completed';

export interface TaskDetail {
  id: string;
  title: string;
  status: TaskStatus;
  details: string[];  // 작업 세부사항 (짧은 문장 리스트)
  subtasks?: TaskDetail[];
  createdAt: string;
  updatedAt: string;
}

export interface TaskList {
  title: string;
  agent: string;
  createdAt: string;
  sessionId: string;
  tasks: TaskDetail[];
  currentPhase?: string;
  memo?: string;
}

export interface TaskStatusSummary {
  agent: string;
  title: string;
  status: TaskStatus;
  completionRate: number;  // 0~100
  completedCount: number;
  totalCount: number;
  currentPhase?: string;
}

export interface CommandArgs {
  [key: string]: string | boolean | undefined;
}
