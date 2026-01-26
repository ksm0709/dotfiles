/**
 * OpenCode Flows Plugin - Tool Node
 *
 * OpenCode 도구(tool)를 호출하는 노드
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult, ToolNodeConfig } from "../types/schemas";
export declare class ToolNode extends BaseNode {
    private client;
    constructor(nodeName: string, config: ToolNodeConfig, blackboard: any, client: any);
    execute(_sessionId: string): Promise<NodeResult>;
    /**
     * 인자 내 변수 해석
     */
    private resolveArgs;
    private executeRead;
    private executeWrite;
    private executeGlob;
    private executeGrep;
    private executeBash;
}
//# sourceMappingURL=ToolNode.d.ts.map