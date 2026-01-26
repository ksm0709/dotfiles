"use strict";
/**
 * OpenCode Flows Plugin - Node Factory
 *
 * 노드 타입에 따라 적절한 노드 인스턴스 생성
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.createNode = createNode;
const AgentNode_1 = require("./AgentNode");
const ToolNode_1 = require("./ToolNode");
const CommandNode_1 = require("./CommandNode");
const ConditionalNode_1 = require("./ConditionalNode");
const EndNode_1 = require("./EndNode");
const DelayNode_1 = require("./DelayNode");
const LoopNode_1 = require("./LoopNode");
/**
 * 노드 타입에 맞는 노드 인스턴스 생성
 */
function createNode(nodeName, nodeDef, blackboard, client) {
    const config = nodeDef.config || {};
    switch (nodeDef.type) {
        case "agent":
            return new AgentNode_1.AgentNode(nodeName, config, blackboard, client);
        case "tool":
            return new ToolNode_1.ToolNode(nodeName, config, blackboard, client);
        case "command":
            return new CommandNode_1.CommandNode(nodeName, config, blackboard, client);
        case "conditional":
            return new ConditionalNode_1.ConditionalNode(nodeName, config, blackboard);
        case "delay":
            return new DelayNode_1.DelayNode(nodeName, config, blackboard);
        case "loop":
            return new LoopNode_1.LoopNode(nodeName, config, blackboard);
        case "end":
            return new EndNode_1.EndNode(nodeName, config, blackboard);
        // TODO: 추가 노드 타입 구현
        case "skill":
            throw new Error(`Node type not yet implemented: ${nodeDef.type}`);
        case "parallel":
            throw new Error(`Node type not yet implemented: ${nodeDef.type}`);
        case "retry":
            throw new Error(`Node type not yet implemented: ${nodeDef.type}`);
        case "sub-flow":
            throw new Error(`Node type not yet implemented: ${nodeDef.type}`);
        default:
            throw new Error(`Unknown node type: ${nodeDef.type}`);
    }
}
//# sourceMappingURL=NodeFactory.js.map