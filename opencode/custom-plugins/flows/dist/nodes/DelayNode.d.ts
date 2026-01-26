/**
 * OpenCode Flows Plugin - Delay Node
 *
 * 지정된 시간만큼 대기하는 노드
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult } from "../types/schemas";
export declare class DelayNode extends BaseNode {
    private duration;
    execute(_sessionId: string): Promise<NodeResult>;
}
//# sourceMappingURL=DelayNode.d.ts.map