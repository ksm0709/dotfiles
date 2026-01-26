"use strict";
/**
 * OpenCode Flows Plugin - Conditional Node
 *
 * 조건 분기 노드
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.ConditionalNode = void 0;
const BaseNode_1 = require("./BaseNode");
class ConditionalNode extends BaseNode_1.BaseNode {
    async execute(_sessionId) {
        this.startTimer();
        const conditions = this.config.conditions || [];
        // 조건 순차 평가
        for (const condition of conditions) {
            const value = this.blackboard.getValue(condition.field);
            if (this.evaluateCondition(value, condition.operator, condition.value)) {
                return this.result(condition.result, `Condition matched: ${condition.field} ${condition.operator} ${condition.value}`);
            }
        }
        // 기본 분기
        const defaultResult = this.config.default || "failed";
        return this.result(defaultResult, "No condition matched, using default");
    }
    /**
     * 조건 평가
     */
    evaluateCondition(value, operator, expected) {
        switch (operator) {
            case "eq":
                return value === expected;
            case "ne":
                return value !== expected;
            case "gt":
                return typeof value === "number" && value > expected;
            case "lt":
                return typeof value === "number" && value < expected;
            case "contains":
                if (typeof value === "string") {
                    return value.includes(expected);
                }
                if (Array.isArray(value)) {
                    return value.includes(expected);
                }
                return false;
            case "exists":
                return value !== undefined && value !== null;
            default:
                return false;
        }
    }
}
exports.ConditionalNode = ConditionalNode;
//# sourceMappingURL=ConditionalNode.js.map