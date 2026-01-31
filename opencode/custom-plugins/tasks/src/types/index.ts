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

// ===========================================
// Unified Operations Types (New - v3.0)
// ===========================================

export type UnifiedOperationType = 'init' | 'add' | 'update' | 'complete' | 'remove';

export interface UnifiedOperation {
  type: UnifiedOperationType;
  // For 'init'
  agent?: string;
  // For 'init' and 'add'
  title?: string;
  // For 'update', 'complete', 'remove'
  id?: string;
  // For 'add' (optional)
  parent?: string;
  // For 'update'
  status?: TaskStatus;
}

export interface UnifiedCommandParams {
  sessionId: string;
  operations: UnifiedOperation[];
}

export interface UnifiedCommandResult {
  success: boolean;
  results: UnifiedOperationResult[];
  summary: {
    total: number;
    succeeded: number;
    failed: number;
  };
  currentStatus: TaskList | null;
  response: ToolResponse;
}

export interface UnifiedOperationResult {
  success: boolean;
  operation: UnifiedOperation;
  message: string;
  taskId?: string;
  taskTitle?: string;
  error?: string;
}

// ===========================================
// Batch Operations Types (Deprecated - use UnifiedOperation instead)
// ===========================================

export type OperationType = 'add' | 'update' | 'complete' | 'remove';

export interface BatchOperation {
  type: OperationType;
  id?: string;
  title?: string;
  parent?: string;
  status?: TaskStatus;
}

export interface BatchResult {
  success: boolean;
  operation: BatchOperation;
  taskId?: string;
  taskTitle?: string;
  message: string;
  error?: string;
}

export interface BatchCommandResult {
  success: boolean;
  results: BatchResult[];
  summary: {
    total: number;
    succeeded: number;
    failed: number;
  };
  currentStatus: TaskList;
  formattedOutput: string;
  response: ToolResponse;
}

export interface CommandResultWithStatus {
  success: boolean;
  message: string;
  currentStatus: TaskList;
  statusSummary: StatusSummary;
  formattedOutput: string;
}

export interface StatusSummary {
  agent: string;
  title: string;
  total: number;
  completed: number;
  inProgress: number;
  pending: number;
  completionRate: number;
}

// ===========================================
// Native UI Response Types (New)
// ===========================================

/**
 * OpenCode Native UI용 Tool 응답 타입
 * title: UI 헤더에 표시될 간결한 제목
 * output: 마크다운 형식의 상세 출력 (하위 호환성)
 * metadata: OpenCode가 UI로 렌더링할 구조화된 데이터
 */
export interface ToolResponse {
  title: string;
  output: string;
  metadata: {
    tasks?: TaskDetail[];
    taskList?: TaskList;
    summary?: StatusSummary;
    results?: UnifiedOperationResult[];
    operation?: 'add' | 'update' | 'complete' | 'remove' | 'batch' | 'init' | 'list' | 'status' | 'unified';
    taskId?: string;
    status?: TaskStatus;
    parent?: string;
    message?: string;
    batchSummary?: {
      total: number;
      succeeded: number;
      failed: number;
    };
  };
}

/**
 * Command 결과에 ToolResponse를 포함하는 인터페이스
 * 기존 CommandResultWithStatus를 확장하여 response 필드 추가
 */
export interface CommandResultWithResponse extends CommandResultWithStatus {
  response: ToolResponse;
}
