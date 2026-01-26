/**
 * OpenCode Flows Plugin - Type Definitions
 * 
 * 플로우, 노드, Blackboard 관련 타입 정의
 */

import { z } from "zod";

// ==================== 기본 타입 ====================

/** 노드 결과 상태 */
export type NodeResultName = "success" | "failed" | "running" | string;

/** 노드 타입 */
export type NodeType = 
  | "agent" 
  | "tool" 
  | "command" 
  | "skill"
  | "conditional"
  | "parallel"
  | "delay"
  | "loop"
  | "retry"
  | "sub-flow"
  | "end";

/** 플로우 인스턴스 상태 */
export type FlowStatus = 
  | "initializing" 
  | "running" 
  | "paused" 
  | "completed" 
  | "failed";

// ==================== Zod 스키마 (Canonical) ====================

/** 노드 결과 스키마 */
export const NodeResultSchema = z.object({
  name: z.string(),
  message: z.string(),
  data: z.record(z.any()).optional(),
  duration: z.number().optional(),
});

export type NodeResult = z.infer<typeof NodeResultSchema>;

/** 에이전트 결과 정의 스키마 */
export const AgentResultDefSchema = z.object({
  name: z.string(),
  description: z.string(),
});

export type AgentResultDef = z.infer<typeof AgentResultDefSchema>;

/** 조건 스키마 */
export const ConditionSchema = z.object({
  field: z.string(),
  operator: z.enum(["eq", "ne", "gt", "lt", "contains", "exists"]),
  value: z.any(),
  result: z.string(),
});

export type Condition = z.infer<typeof ConditionSchema>;

// ==================== 노드 설정 스키마 (Canonical) ====================

/** Agent 노드 설정 */
export const AgentNodeConfigSchema = z.object({
  prompt: z.string(),
  agent_type: z.enum([
    "general",
    "senior-sw-engineer",
    "py-code-reviewer",
    "cpp-review",
    "research-analyst",
    "qa",
    "plan",
    "build",
  ]).default("general"),
  results: z.record(z.string()).optional(), // Array -> Record로 변경 (name: desc)
});

export type AgentNodeConfig = z.infer<typeof AgentNodeConfigSchema>;

/** Tool 노드 설정 */
export const ToolNodeConfigSchema = z.object({
  tool: z.enum([
    "read", "write", "edit", "glob", "grep", "bash", "webfetch", "task"
  ]),
  args: z.record(z.any()).optional(),
});

export type ToolNodeConfig = z.infer<typeof ToolNodeConfigSchema>;

/** Command 노드 설정 */
export const CommandNodeConfigSchema = z.object({
  command: z.string(),
  workdir: z.string().optional(),
  expect_exit_code: z.number().optional(),
});

export type CommandNodeConfig = z.infer<typeof CommandNodeConfigSchema>;

/** Skill 노드 설정 */
export const SkillNodeConfigSchema = z.object({
  skill: z.string(),
  args: z.record(z.any()).optional(),
});

export type SkillNodeConfig = z.infer<typeof SkillNodeConfigSchema>;

/** Conditional 노드 설정 */
export const ConditionalNodeConfigSchema = z.object({
  conditions: z.array(ConditionSchema),
  default: z.string().optional(),
});

export type ConditionalNodeConfig = z.infer<typeof ConditionalNodeConfigSchema>;

/** Parallel 노드 설정 */
export const ParallelNodeConfigSchema = z.object({
  nodes: z.array(z.string()),
  nodeConfigs: z.record(z.any()).optional(),
});

export type ParallelNodeConfig = z.infer<typeof ParallelNodeConfigSchema>;

/** Delay 노드 설정 */
export const DelayNodeConfigSchema = z.object({
  duration: z.number(), // ms
});

export type DelayNodeConfig = z.infer<typeof DelayNodeConfigSchema>;

/** Loop 노드 설정 */
export const LoopNodeConfigSchema = z.object({
  max_iterations: z.number().default(10),
  while_condition: ConditionSchema.optional(),
  body_node: z.string(),
});

export type LoopNodeConfig = z.infer<typeof LoopNodeConfigSchema>;

/** Retry 노드 설정 */
export const RetryNodeConfigSchema = z.object({
  target_node: z.string(),
  max_retries: z.number().default(3),
  retry_delay: z.number().default(1000),
});

export type RetryNodeConfig = z.infer<typeof RetryNodeConfigSchema>;

/** Sub-flow 노드 설정 */
export const SubFlowNodeConfigSchema = z.object({
  flow_name: z.string(),
  input_mapping: z.record(z.string()).optional(),
  output_mapping: z.record(z.string()).optional(),
});

export type SubFlowNodeConfig = z.infer<typeof SubFlowNodeConfigSchema>;

/** End 노드 설정 */
export const EndNodeConfigSchema = z.object({
  status: z.enum(["success", "failed"]).default("success"),
  message: z.string().optional(),
});

export type EndNodeConfig = z.infer<typeof EndNodeConfigSchema>;

// ==================== 노드 정의 스키마 (Canonical) ====================

/** 노드 정의 */
export const NodeDefinitionSchema = z.object({
  type: z.enum([
    "agent", "tool", "command", "skill",
    "conditional", "parallel", "delay",
    "loop", "retry", "sub-flow", "end"
  ]),
  config: z.any(), // 노드 타입별로 다른 스키마 적용
  routes: z.record(z.string().nullable()).optional(),
});

export type NodeDefinition = z.infer<typeof NodeDefinitionSchema>;

// ==================== 플로우 정의 스키마 (Canonical) ====================

/** 플로우 설정 */
export const FlowConfigSchema = z.object({
  timeout: z.number().default(300000), // 5분
  max_retries: z.number().default(3),
  retry_delay: z.number().default(1000),
  tick_interval: z.number().default(500),
  history_depth: z.number().default(5),
  max_steps: z.number().default(100), // 최대 실행 단계 수
});

export type FlowConfig = z.infer<typeof FlowConfigSchema>;

/** 플로우 정의 (Canonical) */
export const FlowDefinitionSchema = z.object({
  name: z.string().regex(/^[a-z][a-z0-9-]*$/),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  description: z.string().optional(),
  config: FlowConfigSchema.optional(),
  initial_state: z.string(),
  nodes: z.record(NodeDefinitionSchema),
});

export type FlowDefinition = z.infer<typeof FlowDefinitionSchema>;

// ==================== 단축 문법 스키마 (Concise) ====================

// 공통 필드: on (routes 대체)
const ConciseBase = z.object({
  on: z.record(z.string().nullable()).optional(),
});

// 1. Agent 단축
export const ConciseAgentSchema = ConciseBase.extend({
  agent: z.string(), // agent_type
  prompt: z.string(),
  results: z.record(z.string()).optional(),
});

// 2. Command 단축
export const ConciseCommandSchema = ConciseBase.extend({
  run: z.string(), // command
  workdir: z.string().optional(),
  expect_exit_code: z.number().optional(),
});

// 3. Tool 단축
export const ConciseToolSchema = ConciseBase.extend({
  tool: z.string(),
  args: z.record(z.any()).optional(),
});

// 4. Delay 단축
export const ConciseDelaySchema = ConciseBase.extend({
  wait: z.number(), // duration
});

// 5. End 단축
export const ConciseEndSchema = z.object({
  end: z.boolean().optional(), // true면 end 노드
  status: z.enum(["success", "failed"]).optional(),
  message: z.string().optional(),
});

// 6. Loop 단축
export const ConciseLoopSchema = ConciseBase.extend({
  loop: z.string(), // body_node
  max: z.number().optional(),
  while: ConditionSchema.optional(),
});

// 7. Conditional 단축 (if/elif/else 구조는 복잡하므로 일단 canonical과 유사하게 가되 필드명만 단축 가능)
export const ConciseConditionalSchema = ConciseBase.extend({
  conditions: z.array(ConditionSchema),
  default: z.string().optional(),
});

// Union
export const ConciseNodeSchema = z.union([
  ConciseAgentSchema,
  ConciseCommandSchema,
  ConciseToolSchema,
  ConciseDelaySchema,
  ConciseEndSchema,
  ConciseLoopSchema,
  ConciseConditionalSchema,
  // Fallback to Canonical-like structure but with 'on'
  z.object({
    type: z.string(),
    config: z.any(),
    on: z.record(z.string().nullable()).optional(),
  }),
]);

export type ConciseNode = z.infer<typeof ConciseNodeSchema>;

/** 플로우 정의 (Concise) */
export const ConciseFlowDefinitionSchema = z.object({
  name: z.string().regex(/^[a-z][a-z0-9-]*$/),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  description: z.string().optional(),
  config: FlowConfigSchema.optional(),
  start: z.string(), // initial_state 대체
  nodes: z.record(ConciseNodeSchema),
});

export type ConciseFlowDefinition = z.infer<typeof ConciseFlowDefinitionSchema>;


// ==================== Blackboard 타입 ====================

/** 히스토리 엔트리 */
export interface HistoryEntry {
  node: string;
  result: NodeResult;
  timestamp: string;
}

/** Blackboard 데이터 */
export interface BlackboardData {
  // 초기 입력
  prompt?: string;
  
  // 자동 관리 필드 (언더스코어 접두사)
  _instance_id: string;
  _flow_name: string;
  _current_state: string;
  _started_at: string;
  _session_id?: string;
  
  // 개선된 히스토리 관리: 노드별 결과 저장
  _results: Record<string, {
    result: NodeResult;
    timestamp: string;
    executionCount: number;
  }>;
  
  _execution_order: string[]; // 실행된 노드 순서
  
  _final_status?: string;
  _final_message?: string;
  
  // 사용자 정의 변수
  [key: string]: any;
}

// ==================== Flow 상태 타입 ====================

/** 플로우 인스턴스 상태 정보 */
export interface FlowInstanceStatus {
  instanceId: string;
  flowName: string;
  currentNode: string;
  status: FlowStatus;
  startedAt: string;
  elapsedMs?: number;
  retryCount?: number;
}

// ==================== 노드 팩토리 타입 ====================

/** 노드 생성자 인터페이스 */
export interface NodeConstructor {
  new (
    nodeName: string,
    config: any,
    blackboard: BlackboardData
  ): BaseNodeInterface;
}

/** 기본 노드 인터페이스 */
export interface BaseNodeInterface {
  execute(sessionId: string): Promise<NodeResult>;
}

// ==================== 플러그인 설정 ====================

/** 플러그인 전역 설정 */
export const PluginConfigSchema = z.object({
  flowsDir: z.string().default(".opencode/flows"),
  globalNodesDir: z.string().default("~/.config/opencode/shared/flows/nodes"),
  dataDir: z.string().default("~/.config/opencode/data/flows"),
  enableToasts: z.boolean().default(true),
  debugMode: z.boolean().default(false),
  tickInterval: z.number().default(500),
  keepCompletedInstances: z.boolean().default(false),
});

export type PluginConfig = z.infer<typeof PluginConfigSchema>;
