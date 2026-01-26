/**
 * OpenCode Flows Plugin
 *
 * 멀티 에이전트 워크플로우 관리 플러그인
 * JSON 기반 FSM을 정의하여 복잡한 멀티스텝 작업을 자동화
 */
import type { Plugin } from "@opencode-ai/plugin";
export declare const FlowsPlugin: Plugin;
export default FlowsPlugin;
export * from "./types/schemas";
export { FlowManager } from "./manager/FlowManager";
export { FlowInstance } from "./manager/FlowInstance";
export { Blackboard } from "./manager/Blackboard";
export { BaseNode } from "./nodes/BaseNode";
//# sourceMappingURL=index.d.ts.map