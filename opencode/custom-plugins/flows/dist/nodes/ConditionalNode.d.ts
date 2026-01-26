/**
 * OpenCode Flows Plugin - Conditional Node
 *
 * 조건 분기 노드
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult } from "../types/schemas";
export declare class ConditionalNode extends BaseNode {
    execute(_sessionId: string): Promise<NodeResult>;
    /**
     * 조건 평가
     */
    private evaluateCondition;
}
//# sourceMappingURL=ConditionalNode.d.ts.map