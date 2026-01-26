/**
 * OpenCode Flows Plugin - Loop Node
 * 
 * 반복 실행 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, LoopNodeConfig, Condition } from "../types/schemas";

export class LoopNode extends BaseNode {
  async execute(_sessionId: string): Promise<NodeResult> {
    this.startTimer();
    
    const maxIterations = this.config.max_iterations || 10;
    const iteration = this.blackboard.get(`${this.nodeName}_iteration`) || 0;
    
    // 조건 체크
    if (this.config.while_condition) {
      const shouldContinue = this.evaluateCondition(this.config.while_condition);
      if (!shouldContinue) {
        // 조건 미충족 - 루프 종료
        await this.blackboard.set(`${this.nodeName}_iteration`, 0);
        return this.success(`Loop completed after ${iteration} iterations`);
      }
    }

    // 반복 횟수 체크
    if (iteration >= maxIterations) {
      await this.blackboard.set(`${this.nodeName}_iteration`, 0);
      return this.result(
        "max_reached",
        `Max iterations (${maxIterations}) reached`
      );
    }

    // 반복 카운터 증가
    await this.blackboard.set(`${this.nodeName}_iteration`, iteration + 1);

    // 'continue' 결과 반환 - routes에서 body_node로 연결
    return this.result(
      "continue",
      `Iteration ${iteration + 1} of max ${maxIterations}`
    );
  }

  /**
   * 조건 평가
   */
  private evaluateCondition(condition: Condition): boolean {
    const value = this.blackboard.getValue(condition.field);
    
    switch (condition.operator) {
      case "eq":
        return value === condition.value;
      case "ne":
        return value !== condition.value;
      case "gt":
        return typeof value === "number" && value > condition.value;
      case "lt":
        return typeof value === "number" && value < condition.value;
      case "contains":
        return typeof value === "string" && value.includes(condition.value);
      case "exists":
        return value !== undefined && value !== null;
      default:
        return false;
    }
  }
}
