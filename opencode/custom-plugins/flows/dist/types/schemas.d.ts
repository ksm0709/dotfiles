/**
 * OpenCode Flows Plugin - Type Definitions
 *
 * 플로우, 노드, Blackboard 관련 타입 정의
 */
import { z } from "zod";
/** 노드 결과 상태 */
export type NodeResultName = "success" | "failed" | "running" | string;
/** 노드 타입 */
export type NodeType = "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
/** 플로우 인스턴스 상태 */
export type FlowStatus = "initializing" | "running" | "paused" | "completed" | "failed";
/** 노드 결과 스키마 */
export declare const NodeResultSchema: z.ZodObject<{
    name: z.ZodString;
    message: z.ZodString;
    data: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
    duration: z.ZodOptional<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    name: string;
    message: string;
    data?: Record<string, any> | undefined;
    duration?: number | undefined;
}, {
    name: string;
    message: string;
    data?: Record<string, any> | undefined;
    duration?: number | undefined;
}>;
export type NodeResult = z.infer<typeof NodeResultSchema>;
/** 에이전트 결과 정의 스키마 */
export declare const AgentResultDefSchema: z.ZodObject<{
    name: z.ZodString;
    description: z.ZodString;
}, "strip", z.ZodTypeAny, {
    name: string;
    description: string;
}, {
    name: string;
    description: string;
}>;
export type AgentResultDef = z.infer<typeof AgentResultDefSchema>;
/** 조건 스키마 */
export declare const ConditionSchema: z.ZodObject<{
    field: z.ZodString;
    operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
    value: z.ZodAny;
    result: z.ZodString;
}, "strip", z.ZodTypeAny, {
    field: string;
    operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
    result: string;
    value?: any;
}, {
    field: string;
    operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
    result: string;
    value?: any;
}>;
export type Condition = z.infer<typeof ConditionSchema>;
/** Agent 노드 설정 */
export declare const AgentNodeConfigSchema: z.ZodObject<{
    prompt: z.ZodString;
    agent_type: z.ZodDefault<z.ZodEnum<["general", "senior-sw-engineer", "py-code-reviewer", "cpp-review", "research-analyst", "qa", "plan", "build"]>>;
    results: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
}, "strip", z.ZodTypeAny, {
    prompt: string;
    agent_type: "general" | "senior-sw-engineer" | "py-code-reviewer" | "cpp-review" | "research-analyst" | "qa" | "plan" | "build";
    results?: Record<string, string> | undefined;
}, {
    prompt: string;
    agent_type?: "general" | "senior-sw-engineer" | "py-code-reviewer" | "cpp-review" | "research-analyst" | "qa" | "plan" | "build" | undefined;
    results?: Record<string, string> | undefined;
}>;
export type AgentNodeConfig = z.infer<typeof AgentNodeConfigSchema>;
/** Tool 노드 설정 */
export declare const ToolNodeConfigSchema: z.ZodObject<{
    tool: z.ZodEnum<["read", "write", "edit", "glob", "grep", "bash", "webfetch", "task"]>;
    args: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
}, "strip", z.ZodTypeAny, {
    tool: "read" | "write" | "edit" | "glob" | "grep" | "bash" | "webfetch" | "task";
    args?: Record<string, any> | undefined;
}, {
    tool: "read" | "write" | "edit" | "glob" | "grep" | "bash" | "webfetch" | "task";
    args?: Record<string, any> | undefined;
}>;
export type ToolNodeConfig = z.infer<typeof ToolNodeConfigSchema>;
/** Command 노드 설정 */
export declare const CommandNodeConfigSchema: z.ZodObject<{
    command: z.ZodString;
    workdir: z.ZodOptional<z.ZodString>;
    expect_exit_code: z.ZodOptional<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    command: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
}, {
    command: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
}>;
export type CommandNodeConfig = z.infer<typeof CommandNodeConfigSchema>;
/** Skill 노드 설정 */
export declare const SkillNodeConfigSchema: z.ZodObject<{
    skill: z.ZodString;
    args: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
}, "strip", z.ZodTypeAny, {
    skill: string;
    args?: Record<string, any> | undefined;
}, {
    skill: string;
    args?: Record<string, any> | undefined;
}>;
export type SkillNodeConfig = z.infer<typeof SkillNodeConfigSchema>;
/** Conditional 노드 설정 */
export declare const ConditionalNodeConfigSchema: z.ZodObject<{
    conditions: z.ZodArray<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>, "many">;
    default: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
}, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
}>;
export type ConditionalNodeConfig = z.infer<typeof ConditionalNodeConfigSchema>;
/** Parallel 노드 설정 */
export declare const ParallelNodeConfigSchema: z.ZodObject<{
    nodes: z.ZodArray<z.ZodString, "many">;
    nodeConfigs: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
}, "strip", z.ZodTypeAny, {
    nodes: string[];
    nodeConfigs?: Record<string, any> | undefined;
}, {
    nodes: string[];
    nodeConfigs?: Record<string, any> | undefined;
}>;
export type ParallelNodeConfig = z.infer<typeof ParallelNodeConfigSchema>;
/** Delay 노드 설정 */
export declare const DelayNodeConfigSchema: z.ZodObject<{
    duration: z.ZodNumber;
}, "strip", z.ZodTypeAny, {
    duration: number;
}, {
    duration: number;
}>;
export type DelayNodeConfig = z.infer<typeof DelayNodeConfigSchema>;
/** Loop 노드 설정 */
export declare const LoopNodeConfigSchema: z.ZodObject<{
    max_iterations: z.ZodDefault<z.ZodNumber>;
    while_condition: z.ZodOptional<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>>;
    body_node: z.ZodString;
}, "strip", z.ZodTypeAny, {
    max_iterations: number;
    body_node: string;
    while_condition?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}, {
    body_node: string;
    max_iterations?: number | undefined;
    while_condition?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}>;
export type LoopNodeConfig = z.infer<typeof LoopNodeConfigSchema>;
/** Retry 노드 설정 */
export declare const RetryNodeConfigSchema: z.ZodObject<{
    target_node: z.ZodString;
    max_retries: z.ZodDefault<z.ZodNumber>;
    retry_delay: z.ZodDefault<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    target_node: string;
    max_retries: number;
    retry_delay: number;
}, {
    target_node: string;
    max_retries?: number | undefined;
    retry_delay?: number | undefined;
}>;
export type RetryNodeConfig = z.infer<typeof RetryNodeConfigSchema>;
/** Sub-flow 노드 설정 */
export declare const SubFlowNodeConfigSchema: z.ZodObject<{
    flow_name: z.ZodString;
    input_mapping: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
    output_mapping: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
}, "strip", z.ZodTypeAny, {
    flow_name: string;
    input_mapping?: Record<string, string> | undefined;
    output_mapping?: Record<string, string> | undefined;
}, {
    flow_name: string;
    input_mapping?: Record<string, string> | undefined;
    output_mapping?: Record<string, string> | undefined;
}>;
export type SubFlowNodeConfig = z.infer<typeof SubFlowNodeConfigSchema>;
/** End 노드 설정 */
export declare const EndNodeConfigSchema: z.ZodObject<{
    status: z.ZodDefault<z.ZodEnum<["success", "failed"]>>;
    message: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    status: "success" | "failed";
    message?: string | undefined;
}, {
    message?: string | undefined;
    status?: "success" | "failed" | undefined;
}>;
export type EndNodeConfig = z.infer<typeof EndNodeConfigSchema>;
/** 노드 정의 */
export declare const NodeDefinitionSchema: z.ZodObject<{
    type: z.ZodEnum<["agent", "tool", "command", "skill", "conditional", "parallel", "delay", "loop", "retry", "sub-flow", "end"]>;
    config: z.ZodAny;
    routes: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
}, "strip", z.ZodTypeAny, {
    type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
    config?: any;
    routes?: Record<string, string | null> | undefined;
}, {
    type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
    config?: any;
    routes?: Record<string, string | null> | undefined;
}>;
export type NodeDefinition = z.infer<typeof NodeDefinitionSchema>;
/** 플로우 설정 */
export declare const FlowConfigSchema: z.ZodObject<{
    timeout: z.ZodDefault<z.ZodNumber>;
    max_retries: z.ZodDefault<z.ZodNumber>;
    retry_delay: z.ZodDefault<z.ZodNumber>;
    tick_interval: z.ZodDefault<z.ZodNumber>;
    history_depth: z.ZodDefault<z.ZodNumber>;
    max_steps: z.ZodDefault<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    max_retries: number;
    retry_delay: number;
    timeout: number;
    tick_interval: number;
    history_depth: number;
    max_steps: number;
}, {
    max_retries?: number | undefined;
    retry_delay?: number | undefined;
    timeout?: number | undefined;
    tick_interval?: number | undefined;
    history_depth?: number | undefined;
    max_steps?: number | undefined;
}>;
export type FlowConfig = z.infer<typeof FlowConfigSchema>;
/** 플로우 정의 (Canonical) */
export declare const FlowDefinitionSchema: z.ZodObject<{
    name: z.ZodString;
    version: z.ZodString;
    description: z.ZodOptional<z.ZodString>;
    config: z.ZodOptional<z.ZodObject<{
        timeout: z.ZodDefault<z.ZodNumber>;
        max_retries: z.ZodDefault<z.ZodNumber>;
        retry_delay: z.ZodDefault<z.ZodNumber>;
        tick_interval: z.ZodDefault<z.ZodNumber>;
        history_depth: z.ZodDefault<z.ZodNumber>;
        max_steps: z.ZodDefault<z.ZodNumber>;
    }, "strip", z.ZodTypeAny, {
        max_retries: number;
        retry_delay: number;
        timeout: number;
        tick_interval: number;
        history_depth: number;
        max_steps: number;
    }, {
        max_retries?: number | undefined;
        retry_delay?: number | undefined;
        timeout?: number | undefined;
        tick_interval?: number | undefined;
        history_depth?: number | undefined;
        max_steps?: number | undefined;
    }>>;
    initial_state: z.ZodString;
    nodes: z.ZodRecord<z.ZodString, z.ZodObject<{
        type: z.ZodEnum<["agent", "tool", "command", "skill", "conditional", "parallel", "delay", "loop", "retry", "sub-flow", "end"]>;
        config: z.ZodAny;
        routes: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    }, "strip", z.ZodTypeAny, {
        type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
        config?: any;
        routes?: Record<string, string | null> | undefined;
    }, {
        type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
        config?: any;
        routes?: Record<string, string | null> | undefined;
    }>>;
}, "strip", z.ZodTypeAny, {
    name: string;
    nodes: Record<string, {
        type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
        config?: any;
        routes?: Record<string, string | null> | undefined;
    }>;
    version: string;
    initial_state: string;
    description?: string | undefined;
    config?: {
        max_retries: number;
        retry_delay: number;
        timeout: number;
        tick_interval: number;
        history_depth: number;
        max_steps: number;
    } | undefined;
}, {
    name: string;
    nodes: Record<string, {
        type: "agent" | "tool" | "command" | "skill" | "conditional" | "parallel" | "delay" | "loop" | "retry" | "sub-flow" | "end";
        config?: any;
        routes?: Record<string, string | null> | undefined;
    }>;
    version: string;
    initial_state: string;
    description?: string | undefined;
    config?: {
        max_retries?: number | undefined;
        retry_delay?: number | undefined;
        timeout?: number | undefined;
        tick_interval?: number | undefined;
        history_depth?: number | undefined;
        max_steps?: number | undefined;
    } | undefined;
}>;
export type FlowDefinition = z.infer<typeof FlowDefinitionSchema>;
export declare const ConciseAgentSchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    agent: z.ZodString;
    prompt: z.ZodString;
    results: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
}, "strip", z.ZodTypeAny, {
    agent: string;
    prompt: string;
    results?: Record<string, string> | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    agent: string;
    prompt: string;
    results?: Record<string, string> | undefined;
    on?: Record<string, string | null> | undefined;
}>;
export declare const ConciseCommandSchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    run: z.ZodString;
    workdir: z.ZodOptional<z.ZodString>;
    expect_exit_code: z.ZodOptional<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    run: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    run: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
    on?: Record<string, string | null> | undefined;
}>;
export declare const ConciseToolSchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    tool: z.ZodString;
    args: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
}, "strip", z.ZodTypeAny, {
    tool: string;
    args?: Record<string, any> | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    tool: string;
    args?: Record<string, any> | undefined;
    on?: Record<string, string | null> | undefined;
}>;
export declare const ConciseDelaySchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    wait: z.ZodNumber;
}, "strip", z.ZodTypeAny, {
    wait: number;
    on?: Record<string, string | null> | undefined;
}, {
    wait: number;
    on?: Record<string, string | null> | undefined;
}>;
export declare const ConciseEndSchema: z.ZodObject<{
    end: z.ZodOptional<z.ZodBoolean>;
    status: z.ZodOptional<z.ZodEnum<["success", "failed"]>>;
    message: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    end?: boolean | undefined;
    message?: string | undefined;
    status?: "success" | "failed" | undefined;
}, {
    end?: boolean | undefined;
    message?: string | undefined;
    status?: "success" | "failed" | undefined;
}>;
export declare const ConciseLoopSchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    loop: z.ZodString;
    max: z.ZodOptional<z.ZodNumber>;
    while: z.ZodOptional<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>>;
}, "strip", z.ZodTypeAny, {
    loop: string;
    on?: Record<string, string | null> | undefined;
    max?: number | undefined;
    while?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}, {
    loop: string;
    on?: Record<string, string | null> | undefined;
    max?: number | undefined;
    while?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}>;
export declare const ConciseConditionalSchema: z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    conditions: z.ZodArray<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>, "many">;
    default: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
    on?: Record<string, string | null> | undefined;
}>;
export declare const ConciseNodeSchema: z.ZodUnion<[z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    agent: z.ZodString;
    prompt: z.ZodString;
    results: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
}, "strip", z.ZodTypeAny, {
    agent: string;
    prompt: string;
    results?: Record<string, string> | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    agent: string;
    prompt: string;
    results?: Record<string, string> | undefined;
    on?: Record<string, string | null> | undefined;
}>, z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    run: z.ZodString;
    workdir: z.ZodOptional<z.ZodString>;
    expect_exit_code: z.ZodOptional<z.ZodNumber>;
}, "strip", z.ZodTypeAny, {
    run: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    run: string;
    workdir?: string | undefined;
    expect_exit_code?: number | undefined;
    on?: Record<string, string | null> | undefined;
}>, z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    tool: z.ZodString;
    args: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
}, "strip", z.ZodTypeAny, {
    tool: string;
    args?: Record<string, any> | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    tool: string;
    args?: Record<string, any> | undefined;
    on?: Record<string, string | null> | undefined;
}>, z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    wait: z.ZodNumber;
}, "strip", z.ZodTypeAny, {
    wait: number;
    on?: Record<string, string | null> | undefined;
}, {
    wait: number;
    on?: Record<string, string | null> | undefined;
}>, z.ZodObject<{
    end: z.ZodOptional<z.ZodBoolean>;
    status: z.ZodOptional<z.ZodEnum<["success", "failed"]>>;
    message: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    end?: boolean | undefined;
    message?: string | undefined;
    status?: "success" | "failed" | undefined;
}, {
    end?: boolean | undefined;
    message?: string | undefined;
    status?: "success" | "failed" | undefined;
}>, z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    loop: z.ZodString;
    max: z.ZodOptional<z.ZodNumber>;
    while: z.ZodOptional<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>>;
}, "strip", z.ZodTypeAny, {
    loop: string;
    on?: Record<string, string | null> | undefined;
    max?: number | undefined;
    while?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}, {
    loop: string;
    on?: Record<string, string | null> | undefined;
    max?: number | undefined;
    while?: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    } | undefined;
}>, z.ZodObject<{
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
} & {
    conditions: z.ZodArray<z.ZodObject<{
        field: z.ZodString;
        operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
        value: z.ZodAny;
        result: z.ZodString;
    }, "strip", z.ZodTypeAny, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }, {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }>, "many">;
    default: z.ZodOptional<z.ZodString>;
}, "strip", z.ZodTypeAny, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
    on?: Record<string, string | null> | undefined;
}, {
    conditions: {
        field: string;
        operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
        result: string;
        value?: any;
    }[];
    default?: string | undefined;
    on?: Record<string, string | null> | undefined;
}>, z.ZodObject<{
    type: z.ZodString;
    config: z.ZodAny;
    on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
}, "strip", z.ZodTypeAny, {
    type: string;
    config?: any;
    on?: Record<string, string | null> | undefined;
}, {
    type: string;
    config?: any;
    on?: Record<string, string | null> | undefined;
}>]>;
export type ConciseNode = z.infer<typeof ConciseNodeSchema>;
/** 플로우 정의 (Concise) */
export declare const ConciseFlowDefinitionSchema: z.ZodObject<{
    name: z.ZodString;
    version: z.ZodString;
    description: z.ZodOptional<z.ZodString>;
    config: z.ZodOptional<z.ZodObject<{
        timeout: z.ZodDefault<z.ZodNumber>;
        max_retries: z.ZodDefault<z.ZodNumber>;
        retry_delay: z.ZodDefault<z.ZodNumber>;
        tick_interval: z.ZodDefault<z.ZodNumber>;
        history_depth: z.ZodDefault<z.ZodNumber>;
        max_steps: z.ZodDefault<z.ZodNumber>;
    }, "strip", z.ZodTypeAny, {
        max_retries: number;
        retry_delay: number;
        timeout: number;
        tick_interval: number;
        history_depth: number;
        max_steps: number;
    }, {
        max_retries?: number | undefined;
        retry_delay?: number | undefined;
        timeout?: number | undefined;
        tick_interval?: number | undefined;
        history_depth?: number | undefined;
        max_steps?: number | undefined;
    }>>;
    start: z.ZodString;
    nodes: z.ZodRecord<z.ZodString, z.ZodUnion<[z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        agent: z.ZodString;
        prompt: z.ZodString;
        results: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodString>>;
    }, "strip", z.ZodTypeAny, {
        agent: string;
        prompt: string;
        results?: Record<string, string> | undefined;
        on?: Record<string, string | null> | undefined;
    }, {
        agent: string;
        prompt: string;
        results?: Record<string, string> | undefined;
        on?: Record<string, string | null> | undefined;
    }>, z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        run: z.ZodString;
        workdir: z.ZodOptional<z.ZodString>;
        expect_exit_code: z.ZodOptional<z.ZodNumber>;
    }, "strip", z.ZodTypeAny, {
        run: string;
        workdir?: string | undefined;
        expect_exit_code?: number | undefined;
        on?: Record<string, string | null> | undefined;
    }, {
        run: string;
        workdir?: string | undefined;
        expect_exit_code?: number | undefined;
        on?: Record<string, string | null> | undefined;
    }>, z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        tool: z.ZodString;
        args: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodAny>>;
    }, "strip", z.ZodTypeAny, {
        tool: string;
        args?: Record<string, any> | undefined;
        on?: Record<string, string | null> | undefined;
    }, {
        tool: string;
        args?: Record<string, any> | undefined;
        on?: Record<string, string | null> | undefined;
    }>, z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        wait: z.ZodNumber;
    }, "strip", z.ZodTypeAny, {
        wait: number;
        on?: Record<string, string | null> | undefined;
    }, {
        wait: number;
        on?: Record<string, string | null> | undefined;
    }>, z.ZodObject<{
        end: z.ZodOptional<z.ZodBoolean>;
        status: z.ZodOptional<z.ZodEnum<["success", "failed"]>>;
        message: z.ZodOptional<z.ZodString>;
    }, "strip", z.ZodTypeAny, {
        end?: boolean | undefined;
        message?: string | undefined;
        status?: "success" | "failed" | undefined;
    }, {
        end?: boolean | undefined;
        message?: string | undefined;
        status?: "success" | "failed" | undefined;
    }>, z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        loop: z.ZodString;
        max: z.ZodOptional<z.ZodNumber>;
        while: z.ZodOptional<z.ZodObject<{
            field: z.ZodString;
            operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
            value: z.ZodAny;
            result: z.ZodString;
        }, "strip", z.ZodTypeAny, {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }, {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }>>;
    }, "strip", z.ZodTypeAny, {
        loop: string;
        on?: Record<string, string | null> | undefined;
        max?: number | undefined;
        while?: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        } | undefined;
    }, {
        loop: string;
        on?: Record<string, string | null> | undefined;
        max?: number | undefined;
        while?: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        } | undefined;
    }>, z.ZodObject<{
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    } & {
        conditions: z.ZodArray<z.ZodObject<{
            field: z.ZodString;
            operator: z.ZodEnum<["eq", "ne", "gt", "lt", "contains", "exists"]>;
            value: z.ZodAny;
            result: z.ZodString;
        }, "strip", z.ZodTypeAny, {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }, {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }>, "many">;
        default: z.ZodOptional<z.ZodString>;
    }, "strip", z.ZodTypeAny, {
        conditions: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }[];
        default?: string | undefined;
        on?: Record<string, string | null> | undefined;
    }, {
        conditions: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }[];
        default?: string | undefined;
        on?: Record<string, string | null> | undefined;
    }>, z.ZodObject<{
        type: z.ZodString;
        config: z.ZodAny;
        on: z.ZodOptional<z.ZodRecord<z.ZodString, z.ZodNullable<z.ZodString>>>;
    }, "strip", z.ZodTypeAny, {
        type: string;
        config?: any;
        on?: Record<string, string | null> | undefined;
    }, {
        type: string;
        config?: any;
        on?: Record<string, string | null> | undefined;
    }>]>>;
}, "strip", z.ZodTypeAny, {
    name: string;
    nodes: Record<string, {
        agent: string;
        prompt: string;
        results?: Record<string, string> | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        run: string;
        workdir?: string | undefined;
        expect_exit_code?: number | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        tool: string;
        args?: Record<string, any> | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        wait: number;
        on?: Record<string, string | null> | undefined;
    } | {
        end?: boolean | undefined;
        message?: string | undefined;
        status?: "success" | "failed" | undefined;
    } | {
        loop: string;
        on?: Record<string, string | null> | undefined;
        max?: number | undefined;
        while?: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        } | undefined;
    } | {
        conditions: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }[];
        default?: string | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        type: string;
        config?: any;
        on?: Record<string, string | null> | undefined;
    }>;
    version: string;
    start: string;
    description?: string | undefined;
    config?: {
        max_retries: number;
        retry_delay: number;
        timeout: number;
        tick_interval: number;
        history_depth: number;
        max_steps: number;
    } | undefined;
}, {
    name: string;
    nodes: Record<string, {
        agent: string;
        prompt: string;
        results?: Record<string, string> | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        run: string;
        workdir?: string | undefined;
        expect_exit_code?: number | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        tool: string;
        args?: Record<string, any> | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        wait: number;
        on?: Record<string, string | null> | undefined;
    } | {
        end?: boolean | undefined;
        message?: string | undefined;
        status?: "success" | "failed" | undefined;
    } | {
        loop: string;
        on?: Record<string, string | null> | undefined;
        max?: number | undefined;
        while?: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        } | undefined;
    } | {
        conditions: {
            field: string;
            operator: "eq" | "ne" | "gt" | "lt" | "contains" | "exists";
            result: string;
            value?: any;
        }[];
        default?: string | undefined;
        on?: Record<string, string | null> | undefined;
    } | {
        type: string;
        config?: any;
        on?: Record<string, string | null> | undefined;
    }>;
    version: string;
    start: string;
    description?: string | undefined;
    config?: {
        max_retries?: number | undefined;
        retry_delay?: number | undefined;
        timeout?: number | undefined;
        tick_interval?: number | undefined;
        history_depth?: number | undefined;
        max_steps?: number | undefined;
    } | undefined;
}>;
export type ConciseFlowDefinition = z.infer<typeof ConciseFlowDefinitionSchema>;
/** 히스토리 엔트리 */
export interface HistoryEntry {
    node: string;
    result: NodeResult;
    timestamp: string;
}
/** Blackboard 데이터 */
export interface BlackboardData {
    prompt?: string;
    _instance_id: string;
    _flow_name: string;
    _current_state: string;
    _started_at: string;
    _session_id?: string;
    _results: Record<string, {
        result: NodeResult;
        timestamp: string;
        executionCount: number;
    }>;
    _execution_order: string[];
    _final_status?: string;
    _final_message?: string;
    [key: string]: any;
}
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
/** 노드 생성자 인터페이스 */
export interface NodeConstructor {
    new (nodeName: string, config: any, blackboard: BlackboardData): BaseNodeInterface;
}
/** 기본 노드 인터페이스 */
export interface BaseNodeInterface {
    execute(sessionId: string): Promise<NodeResult>;
}
/** 플러그인 전역 설정 */
export declare const PluginConfigSchema: z.ZodObject<{
    flowsDir: z.ZodDefault<z.ZodString>;
    globalNodesDir: z.ZodDefault<z.ZodString>;
    dataDir: z.ZodDefault<z.ZodString>;
    enableToasts: z.ZodDefault<z.ZodBoolean>;
    debugMode: z.ZodDefault<z.ZodBoolean>;
    tickInterval: z.ZodDefault<z.ZodNumber>;
    keepCompletedInstances: z.ZodDefault<z.ZodBoolean>;
}, "strip", z.ZodTypeAny, {
    flowsDir: string;
    globalNodesDir: string;
    dataDir: string;
    enableToasts: boolean;
    debugMode: boolean;
    tickInterval: number;
    keepCompletedInstances: boolean;
}, {
    flowsDir?: string | undefined;
    globalNodesDir?: string | undefined;
    dataDir?: string | undefined;
    enableToasts?: boolean | undefined;
    debugMode?: boolean | undefined;
    tickInterval?: number | undefined;
    keepCompletedInstances?: boolean | undefined;
}>;
export type PluginConfig = z.infer<typeof PluginConfigSchema>;
//# sourceMappingURL=schemas.d.ts.map