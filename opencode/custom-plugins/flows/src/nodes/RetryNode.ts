/**
 * OpenCode Flows Plugin - Retry Node
 * 
 * 재시도 로직을 관리하는 노드
 * 타겟 노드가 실패했을 때 이 노드로 전이되면, 재시도 횟수를 체크하고
 * 재시도(retry) 또는 실패(max_retries_exceeded)를 결정함
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, RetryNodeConfig } from "../types/schemas";

export class RetryNode extends BaseNode {
  private startTime?: number;
  private isWaiting: boolean = false;

  async execute(_sessionId: string): Promise<NodeResult> {
    const targetNode = this.config.target_node;
    const maxRetries = this.config.max_retries || 3;
    const retryDelay = this.config.retry_delay || 1000;
    
    // 현재 재시도 횟수 조회
    const retryKey = `${this.nodeName}_retry_count`;
    const currentRetries = this.blackboard.get<number>(retryKey) || 0;

    // 대기 중 처리
    if (this.isWaiting) {
      const elapsed = Date.now() - (this.startTime || 0);
      if (elapsed >= retryDelay) {
        this.isWaiting = false;
        this.startTime = undefined;
        
        // 재시도 카운트 증가 및 저장
        await this.blackboard.set(retryKey, currentRetries + 1);
        
        return this.result("retry", `Retrying ${targetNode} (${currentRetries + 1}/${maxRetries})`);
      }
      return this.running();
    }

    // 최대 횟수 초과 체크
    if (currentRetries >= maxRetries) {
      // 카운트 리셋 (다음 실행을 위해)
      await this.blackboard.set(retryKey, 0);
      return this.result("max_retries_exceeded", `Max retries (${maxRetries}) exceeded for ${targetNode}`);
    }

    // 대기 시작
    this.isWaiting = true;
    this.startTimer(); // BaseNode의 startTimer 사용 (this.startTime 설정됨)
    
    return this.running();
  }
}
