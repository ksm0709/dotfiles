"use strict";
/**
 * OpenCode Flows Plugin - End Node
 *
 * 플로우 종료 노드
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.EndNode = void 0;
const BaseNode_1 = require("./BaseNode");
class EndNode extends BaseNode_1.BaseNode {
    async execute(_sessionId) {
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
exports.EndNode = EndNode;
//# sourceMappingURL=EndNode.js.map