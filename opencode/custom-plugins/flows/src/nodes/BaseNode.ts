/**
 * OpenCode Flows Plugin - Base Node
 * 
 * 모든 노드가 상속받는 추상 기본 클래스
 */

import type { NodeResult, NodeResultName } from "../types/schemas";
import type { Blackboard } from "../manager/Blackboard";

export abstract class BaseNode {
  constructor(
    protected nodeName: string,
    protected config: any,
    protected blackboard: Blackboard
  ) {}

  /**
   * 노드 실행 - 하위 클래스에서 구현
   * @param sessionId OpenCode 세션 ID
   * @returns 노드 실행 결과
   */
  abstract execute(sessionId: string): Promise<NodeResult>;

  /**
   * 성공 결과 생성 헬퍼
   */
  protected success(message: string, data?: Record<string, any>): NodeResult {
    return {
      name: "success",
      message,
      data,
      duration: this.getDuration(),
    };
  }

  /**
   * 실패 결과 생성 헬퍼
   */
  protected failed(message: string, data?: Record<string, any>): NodeResult {
    return {
      name: "failed",
      message,
      data,
      duration: this.getDuration(),
    };
  }

  /**
   * 실행 중 결과 생성 헬퍼
   */
  protected running(): NodeResult {
    return {
      name: "running",
      message: "Node is still executing...",
    };
  }

  /**
   * 커스텀 결과 생성 헬퍼
   */
  protected result(name: NodeResultName, message: string, data?: Record<string, any>): NodeResult {
    return {
      name,
      message,
      data,
      duration: this.getDuration(),
    };
  }

  /**
   * 변수 해석 헬퍼
   */
  protected resolveVariables(template: string): string {
    return this.blackboard.resolveVariables(template);
  }

  /**
   * 설정값 변수 해석 헬퍼
   */
  protected resolveConfig<T extends Record<string, any>>(config: T): T {
    return this.blackboard.resolveObjectVariables(config);
  }

  // 실행 시간 측정용
  protected startTime?: number;
  
  protected startTimer(): void {
    this.startTime = Date.now();
  }

  protected getDuration(): number | undefined {
    if (this.startTime) {
      return Date.now() - this.startTime;
    }
    return undefined;
  }
}
