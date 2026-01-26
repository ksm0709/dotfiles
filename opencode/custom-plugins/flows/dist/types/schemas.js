"use strict";
/**
 * OpenCode Flows Plugin - Type Definitions
 *
 * 플로우, 노드, Blackboard 관련 타입 정의
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.PluginConfigSchema = exports.ConciseFlowDefinitionSchema = exports.ConciseNodeSchema = exports.ConciseConditionalSchema = exports.ConciseLoopSchema = exports.ConciseEndSchema = exports.ConciseDelaySchema = exports.ConciseToolSchema = exports.ConciseCommandSchema = exports.ConciseAgentSchema = exports.FlowDefinitionSchema = exports.FlowConfigSchema = exports.NodeDefinitionSchema = exports.EndNodeConfigSchema = exports.SubFlowNodeConfigSchema = exports.RetryNodeConfigSchema = exports.LoopNodeConfigSchema = exports.DelayNodeConfigSchema = exports.ParallelNodeConfigSchema = exports.ConditionalNodeConfigSchema = exports.SkillNodeConfigSchema = exports.CommandNodeConfigSchema = exports.ToolNodeConfigSchema = exports.AgentNodeConfigSchema = exports.ConditionSchema = exports.AgentResultDefSchema = exports.NodeResultSchema = void 0;
const zod_1 = require("zod");
// ==================== Zod 스키마 (Canonical) ====================
/** 노드 결과 스키마 */
exports.NodeResultSchema = zod_1.z.object({
    name: zod_1.z.string(),
    message: zod_1.z.string(),
    data: zod_1.z.record(zod_1.z.any()).optional(),
    duration: zod_1.z.number().optional(),
});
/** 에이전트 결과 정의 스키마 */
exports.AgentResultDefSchema = zod_1.z.object({
    name: zod_1.z.string(),
    description: zod_1.z.string(),
});
/** 조건 스키마 */
exports.ConditionSchema = zod_1.z.object({
    field: zod_1.z.string(),
    operator: zod_1.z.enum(["eq", "ne", "gt", "lt", "contains", "exists"]),
    value: zod_1.z.any(),
    result: zod_1.z.string(),
});
// ==================== 노드 설정 스키마 (Canonical) ====================
/** Agent 노드 설정 */
exports.AgentNodeConfigSchema = zod_1.z.object({
    prompt: zod_1.z.string(),
    agent_type: zod_1.z.enum([
        "general",
        "senior-sw-engineer",
        "py-code-reviewer",
        "cpp-review",
        "research-analyst",
        "qa",
        "plan",
        "build",
    ]).default("general"),
    results: zod_1.z.record(zod_1.z.string()).optional(), // Array -> Record로 변경 (name: desc)
});
/** Tool 노드 설정 */
exports.ToolNodeConfigSchema = zod_1.z.object({
    tool: zod_1.z.enum([
        "read", "write", "edit", "glob", "grep", "bash", "webfetch", "task"
    ]),
    args: zod_1.z.record(zod_1.z.any()).optional(),
});
/** Command 노드 설정 */
exports.CommandNodeConfigSchema = zod_1.z.object({
    command: zod_1.z.string(),
    workdir: zod_1.z.string().optional(),
    expect_exit_code: zod_1.z.number().optional(),
});
/** Skill 노드 설정 */
exports.SkillNodeConfigSchema = zod_1.z.object({
    skill: zod_1.z.string(),
    args: zod_1.z.record(zod_1.z.any()).optional(),
});
/** Conditional 노드 설정 */
exports.ConditionalNodeConfigSchema = zod_1.z.object({
    conditions: zod_1.z.array(exports.ConditionSchema),
    default: zod_1.z.string().optional(),
});
/** Parallel 노드 설정 */
exports.ParallelNodeConfigSchema = zod_1.z.object({
    nodes: zod_1.z.array(zod_1.z.string()),
    nodeConfigs: zod_1.z.record(zod_1.z.any()).optional(),
});
/** Delay 노드 설정 */
exports.DelayNodeConfigSchema = zod_1.z.object({
    duration: zod_1.z.number(), // ms
});
/** Loop 노드 설정 */
exports.LoopNodeConfigSchema = zod_1.z.object({
    max_iterations: zod_1.z.number().default(10),
    while_condition: exports.ConditionSchema.optional(),
    body_node: zod_1.z.string(),
});
/** Retry 노드 설정 */
exports.RetryNodeConfigSchema = zod_1.z.object({
    target_node: zod_1.z.string(),
    max_retries: zod_1.z.number().default(3),
    retry_delay: zod_1.z.number().default(1000),
});
/** Sub-flow 노드 설정 */
exports.SubFlowNodeConfigSchema = zod_1.z.object({
    flow_name: zod_1.z.string(),
    input_mapping: zod_1.z.record(zod_1.z.string()).optional(),
    output_mapping: zod_1.z.record(zod_1.z.string()).optional(),
});
/** End 노드 설정 */
exports.EndNodeConfigSchema = zod_1.z.object({
    status: zod_1.z.enum(["success", "failed"]).default("success"),
    message: zod_1.z.string().optional(),
});
// ==================== 노드 정의 스키마 (Canonical) ====================
/** 노드 정의 */
exports.NodeDefinitionSchema = zod_1.z.object({
    type: zod_1.z.enum([
        "agent", "tool", "command", "skill",
        "conditional", "parallel", "delay",
        "loop", "retry", "sub-flow", "end"
    ]),
    config: zod_1.z.any(), // 노드 타입별로 다른 스키마 적용
    routes: zod_1.z.record(zod_1.z.string().nullable()).optional(),
});
// ==================== 플로우 정의 스키마 (Canonical) ====================
/** 플로우 설정 */
exports.FlowConfigSchema = zod_1.z.object({
    timeout: zod_1.z.number().default(300000), // 5분
    max_retries: zod_1.z.number().default(3),
    retry_delay: zod_1.z.number().default(1000),
    tick_interval: zod_1.z.number().default(500),
    history_depth: zod_1.z.number().default(5),
    max_steps: zod_1.z.number().default(100), // 최대 실행 단계 수
});
/** 플로우 정의 (Canonical) */
exports.FlowDefinitionSchema = zod_1.z.object({
    name: zod_1.z.string().regex(/^[a-z][a-z0-9-]*$/),
    version: zod_1.z.string().regex(/^\d+\.\d+\.\d+$/),
    description: zod_1.z.string().optional(),
    config: exports.FlowConfigSchema.optional(),
    initial_state: zod_1.z.string(),
    nodes: zod_1.z.record(exports.NodeDefinitionSchema),
});
// ==================== 단축 문법 스키마 (Concise) ====================
// 공통 필드: on (routes 대체)
const ConciseBase = zod_1.z.object({
    on: zod_1.z.record(zod_1.z.string().nullable()).optional(),
});
// 1. Agent 단축
exports.ConciseAgentSchema = ConciseBase.extend({
    agent: zod_1.z.string(), // agent_type
    prompt: zod_1.z.string(),
    results: zod_1.z.record(zod_1.z.string()).optional(),
});
// 2. Command 단축
exports.ConciseCommandSchema = ConciseBase.extend({
    run: zod_1.z.string(), // command
    workdir: zod_1.z.string().optional(),
    expect_exit_code: zod_1.z.number().optional(),
});
// 3. Tool 단축
exports.ConciseToolSchema = ConciseBase.extend({
    tool: zod_1.z.string(),
    args: zod_1.z.record(zod_1.z.any()).optional(),
});
// 4. Delay 단축
exports.ConciseDelaySchema = ConciseBase.extend({
    wait: zod_1.z.number(), // duration
});
// 5. End 단축
exports.ConciseEndSchema = zod_1.z.object({
    end: zod_1.z.boolean().optional(), // true면 end 노드
    status: zod_1.z.enum(["success", "failed"]).optional(),
    message: zod_1.z.string().optional(),
});
// 6. Loop 단축
exports.ConciseLoopSchema = ConciseBase.extend({
    loop: zod_1.z.string(), // body_node
    max: zod_1.z.number().optional(),
    while: exports.ConditionSchema.optional(),
});
// 7. Conditional 단축 (if/elif/else 구조는 복잡하므로 일단 canonical과 유사하게 가되 필드명만 단축 가능)
exports.ConciseConditionalSchema = ConciseBase.extend({
    conditions: zod_1.z.array(exports.ConditionSchema),
    default: zod_1.z.string().optional(),
});
// Union
exports.ConciseNodeSchema = zod_1.z.union([
    exports.ConciseAgentSchema,
    exports.ConciseCommandSchema,
    exports.ConciseToolSchema,
    exports.ConciseDelaySchema,
    exports.ConciseEndSchema,
    exports.ConciseLoopSchema,
    exports.ConciseConditionalSchema,
    // Fallback to Canonical-like structure but with 'on'
    zod_1.z.object({
        type: zod_1.z.string(),
        config: zod_1.z.any(),
        on: zod_1.z.record(zod_1.z.string().nullable()).optional(),
    }),
]);
/** 플로우 정의 (Concise) */
exports.ConciseFlowDefinitionSchema = zod_1.z.object({
    name: zod_1.z.string().regex(/^[a-z][a-z0-9-]*$/),
    version: zod_1.z.string().regex(/^\d+\.\d+\.\d+$/),
    description: zod_1.z.string().optional(),
    config: exports.FlowConfigSchema.optional(),
    start: zod_1.z.string(), // initial_state 대체
    nodes: zod_1.z.record(exports.ConciseNodeSchema),
});
// ==================== 플러그인 설정 ====================
/** 플러그인 전역 설정 */
exports.PluginConfigSchema = zod_1.z.object({
    flowsDir: zod_1.z.string().default(".opencode/flows"),
    globalNodesDir: zod_1.z.string().default("~/.config/opencode/shared/flows/nodes"),
    dataDir: zod_1.z.string().default("~/.config/opencode/data/flows"),
    enableToasts: zod_1.z.boolean().default(true),
    debugMode: zod_1.z.boolean().default(false),
    tickInterval: zod_1.z.number().default(500),
    keepCompletedInstances: zod_1.z.boolean().default(false),
});
//# sourceMappingURL=schemas.js.map