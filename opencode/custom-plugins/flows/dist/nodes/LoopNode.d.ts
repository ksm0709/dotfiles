/**
 * OpenCode Flows Plugin - Loop Node
 *
 * 반복 실행 노드
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult } from "../types/schemas";
export declare class LoopNode extends BaseNode {
    execute(_sessionId: string): Promise<NodeResult>;
    /**
     * 조건 평가
     */
    private evaluateCondition;
}
//# sourceMappingURL=LoopNode.d.ts.map