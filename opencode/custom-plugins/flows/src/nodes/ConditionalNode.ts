/**
 * OpenCode Flows Plugin - Conditional Node
 * 
 * 조건 분기 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, ConditionalNodeConfig, Condition } from "../types/schemas";

export class ConditionalNode extends BaseNode {
  async execute(_sessionId: string): Promise<NodeResult> {
    this.startTimer();
    
    const conditions: Condition[] = this.config.conditions || [];
    
    // 조건 순차 평가
    for (const condition of conditions) {
      const value = this.blackboard.getValue(condition.field);
      
      if (this.evaluateCondition(value, condition.operator, condition.value)) {
        return this.result(
          condition.result,
          `Condition matched: ${condition.field} ${condition.operator} ${condition.value}`
        );
      }
    }
    
    // 기본 분기
    const defaultResult = this.config.default || "failed";
    return this.result(
      defaultResult,
      "No condition matched, using default"
    );
  }

  /**
   * 조건 평가
   */
  private evaluateCondition(value: any, operator: string, expected: any): boolean {
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
