/**
 * OpenCode Flows Plugin - Skill Node
 * 
 * OpenCode 스킬을 실행하는 노드
 * 'skill' 도구를 사용하여 특정 스킬을 호출함
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, SkillNodeConfig } from "../types/schemas";

export class SkillNode extends BaseNode {
  private client: any;

  constructor(
    nodeName: string,
    config: SkillNodeConfig,
    blackboard: any,
    client: any
  ) {
    super(nodeName, config, blackboard);
    this.client = client;
  }

  async execute(_sessionId: string): Promise<NodeResult> {
    this.startTimer();

    const skillName = this.resolveVariables(this.config.skill);
    const args = this.config.args ? this.resolveConfig(this.config.args) : {};

    try {
      // skill 도구 호출
      // client.tools.call(toolName, args) 형태라고 가정
      // 실제 OpenCode SDK 스펙에 따라 조정 필요. 여기서는 ToolNode와 유사하게 구현
      
      // 스킬 실행은 보통 에이전트를 통해 이루어지거나, 특정 도구 호출로 이루어짐
      // 여기서는 'skill'이라는 도구가 있다고 가정하고 호출
      const result = await this.client.tools.call("skill", {
        name: skillName,
        ...args
      });

      return this.success(`Skill '${skillName}' executed successfully`, { result });
    } catch (error: any) {
      return this.failed(`Skill '${skillName}' execution failed: ${error.message}`);
    }
  }
}
