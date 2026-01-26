/**
 * OpenCode Flows Plugin - End Node
 * 
 * 플로우 종료 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, EndNodeConfig } from "../types/schemas";

export class EndNode extends BaseNode {
  async execute(_sessionId: string): Promise<NodeResult> {
    const status = this.config.status || "success";
    const message = this.config.message 
      ? this.resolveVariables(this.config.message)
      : `Flow completed with status: ${status}`;
    
    // 최종 결과를 blackboard에 저장
    await this.blackboard.set("_final_status", status);
    await this.blackboard.set("_final_message", message);
    await this.blackboard.set("_completed_at", new Date().toISOString());
    
    return this.result(status, message);
  }
}
