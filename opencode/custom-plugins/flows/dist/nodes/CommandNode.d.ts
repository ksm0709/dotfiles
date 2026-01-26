/**
 * OpenCode Flows Plugin - Command Node
 *
 * 쉘 명령어를 실행하는 노드
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult, CommandNodeConfig } from "../types/schemas";
export declare class CommandNode extends BaseNode {
    private client;
    private pendingPromise?;
    private storedResult?;
    constructor(nodeName: string, config: CommandNodeConfig, blackboard: any, client: any);
    execute(_sessionId: string): Promise<NodeResult>;
    private executeCommand;
}
//# sourceMappingURL=CommandNode.d.ts.map