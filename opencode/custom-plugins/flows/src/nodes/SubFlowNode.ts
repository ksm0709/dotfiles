/**
 * OpenCode Flows Plugin - SubFlow Node
 * 
 * 다른 플로우를 서브 루틴으로 실행하는 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, SubFlowNodeConfig } from "../types/schemas";
import { FlowManager } from "../manager/FlowManager";

export class SubFlowNode extends BaseNode {
  private childInstanceId?: string;

  async execute(_sessionId: string): Promise<NodeResult> {
    const flowName = this.resolveVariables(this.config.flow_name);
    const manager = FlowManager.getInstance();

    // 1. 서브 플로우 시작 (아직 시작 안 했으면)
    if (!this.childInstanceId) {
      this.startTimer();
      
      // 입력 매핑: 현재 Blackboard -> 서브 플로우 Prompt 또는 초기 변수
      // 여기서는 간단히 input_mapping을 통해 프롬프트를 구성한다고 가정
      // 실제로는 서브 플로우의 Blackboard를 초기화해주는 기능이 필요할 수 있음
      
      let prompt = "";
      if (this.config.input_mapping && this.config.input_mapping.prompt) {
        prompt = this.resolveVariables(this.config.input_mapping.prompt);
      }

      try {
        this.childInstanceId = await manager.start(flowName, prompt);
        await this.blackboard.set(`${this.nodeName}_child_id`, this.childInstanceId);
      } catch (error: any) {
        return this.failed(`Failed to start sub-flow '${flowName}': ${error.message}`);
      }
      
      return this.running();
    }

    // 2. 서브 플로우 상태 확인
    const status = manager.getInstanceStatus(this.childInstanceId);
    
    if (!status) {
      return this.failed(`Sub-flow instance ${this.childInstanceId} not found`);
    }

    if (status.status === "completed") {
      // 출력 매핑 (필요시 구현)
      // 서브 플로우의 Blackboard에 접근하려면 FlowManager를 통해 인스턴스를 가져와야 함
      // 현재는 단순 성공 처리
      return this.success(`Sub-flow '${flowName}' completed`);
    }

    if (status.status === "failed") {
      return this.failed(`Sub-flow '${flowName}' failed`);
    }

    // 아직 실행 중
    return this.running();
  }
}
