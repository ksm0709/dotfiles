/**
 * OpenCode Flows Plugin - Node Factory
 *
 * 노드 타입에 따라 적절한 노드 인스턴스 생성
 */
import type { NodeDefinition } from "../types/schemas";
import type { Blackboard } from "../manager/Blackboard";
import { BaseNode } from "./BaseNode";
/**
 * 노드 타입에 맞는 노드 인스턴스 생성
 */
export declare function createNode(nodeName: string, nodeDef: NodeDefinition, blackboard: Blackboard, client: any): BaseNode;
//# sourceMappingURL=NodeFactory.d.ts.map