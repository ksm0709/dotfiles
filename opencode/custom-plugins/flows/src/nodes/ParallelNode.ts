/**
 * OpenCode Flows Plugin - Parallel Node
 * 
 * 여러 노드를 병렬로 실행하고 모든 결과를 취합하는 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, ParallelNodeConfig } from "../types/schemas";
import { createNode } from "./NodeFactory";

export class ParallelNode extends BaseNode {
  private client: any;
  private childNodes: Map<string, BaseNode> = new Map();
  private childResults: Map<string, NodeResult> = new Map();
  private initialized: boolean = false;

  constructor(
    nodeName: string,
    config: ParallelNodeConfig,
    blackboard: any,
    client: any
  ) {
    super(nodeName, config, blackboard);
    this.client = client;
  }

  async execute(sessionId: string): Promise<NodeResult> {
    if (!this.initialized) {
      this.startTimer();
      this.initializeChildren();
      this.initialized = true;
    }

    const targetNodes: string[] = this.config.nodes || [];
    let allCompleted = true;
    let hasFailure = false;

    // 모든 자식 노드 실행
    for (const childName of targetNodes) {
      // 이미 완료된 노드는 건너뜀
      if (this.childResults.has(childName)) {
        continue;
      }

      const childNode = this.childNodes.get(childName);
      if (!childNode) {
        this.childResults.set(childName, {
          name: "failed",
          message: `Child node instance not found: ${childName}`
        });
        hasFailure = true;
        continue;
      }

      try {
        const result = await childNode.execute(sessionId);

        if (result.name === "running") {
          allCompleted = false;
        } else {
          // 완료된 결과 저장
          this.childResults.set(childName, result);
          
          // Blackboard에 결과 저장 (개별 노드처럼 접근 가능하게)
          await this.blackboard.saveNodeResult(childName, result);
          
          if (result.name === "failed") {
            hasFailure = true;
          }
        }
      } catch (error: any) {
        this.childResults.set(childName, {
          name: "failed",
          message: `Child node execution error: ${error.message}`
        });
        hasFailure = true;
      }
    }

    if (!allCompleted) {
      return this.running();
    }

    // 모든 노드 완료
    const resultsObj = Object.fromEntries(this.childResults);
    
    if (hasFailure) {
      return this.failed("One or more parallel nodes failed", resultsObj);
    }

    return this.success("All parallel nodes completed successfully", resultsObj);
  }

  private initializeChildren() {
    const targetNodes: string[] = this.config.nodes || [];
    const nodeConfigs = this.config.nodeConfigs || {};

    for (const childName of targetNodes) {
      // 자식 노드의 설정이 nodeConfigs에 정의되어 있어야 함
      // 또는 글로벌 노드 정의를 참조해야 하는데, 여기서는 nodeConfigs에 인라인으로 정의된 것을 가정
      // 실제로는 FlowDefinition의 nodes에서 가져오는 것이 더 유연할 수 있음
      
      // 간소화를 위해 ParallelNode는 인라인 config 또는 빈 config를 사용한다고 가정
      // 더 복잡한 구현을 위해서는 FlowManager로부터 전체 FlowDefinition을 받아야 함
      
      // 여기서는 nodeConfigs에 { type: "...", config: ... } 형태가 있다고 가정
      const childDef = nodeConfigs[childName];
      
      if (!childDef) {
        console.warn(`[ParallelNode] No config found for child node: ${childName}`);
        continue;
      }

      const nodeInstance = createNode(
        childName,
        childDef, // NodeDefinition
        this.blackboard,
        this.client
      );
      
      this.childNodes.set(childName, nodeInstance);
    }
  }
}
