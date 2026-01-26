/**
 * OpenCode Flows Plugin - Node Factory
 * 
 * 노드 타입에 따라 적절한 노드 인스턴스 생성
 */

import type { NodeDefinition } from "../types/schemas";
import type { Blackboard } from "../manager/Blackboard";
import { BaseNode } from "./BaseNode";
import { AgentNode } from "./AgentNode";
import { ToolNode } from "./ToolNode";
import { CommandNode } from "./CommandNode";
import { ConditionalNode } from "./ConditionalNode";
import { EndNode } from "./EndNode";
import { DelayNode } from "./DelayNode";
import { LoopNode } from "./LoopNode";
import { RetryNode } from "./RetryNode";
import { SkillNode } from "./SkillNode";
import { ParallelNode } from "./ParallelNode";
import { SubFlowNode } from "./SubFlowNode";

/**
 * 노드 타입에 맞는 노드 인스턴스 생성
 */
export function createNode(
  nodeName: string,
  nodeDef: NodeDefinition,
  blackboard: Blackboard,
  client: any
): BaseNode {
  const config = nodeDef.config || {};
  
  switch (nodeDef.type) {
    case "agent":
      return new AgentNode(nodeName, config, blackboard, client);
      
    case "tool":
      return new ToolNode(nodeName, config, blackboard, client);
      
    case "command":
      return new CommandNode(nodeName, config, blackboard, client);
      
    case "conditional":
      return new ConditionalNode(nodeName, config, blackboard);
      
    case "delay":
      return new DelayNode(nodeName, config, blackboard);
      
    case "loop":
      return new LoopNode(nodeName, config, blackboard);
      
    case "end":
      return new EndNode(nodeName, config, blackboard);
      
    case "retry":
      return new RetryNode(nodeName, config, blackboard);

    case "skill":
      return new SkillNode(nodeName, config, blackboard, client);

    case "parallel":
      return new ParallelNode(nodeName, config, blackboard, client);

    case "sub-flow":
      return new SubFlowNode(nodeName, config, blackboard);
      
    default:
      throw new Error(`Unknown node type: ${nodeDef.type}`);
  }
}
