"use strict";
/**
 * OpenCode Flows Plugin - Delay Node
 *
 * 지정된 시간만큼 대기하는 노드
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.DelayNode = void 0;
const BaseNode_1 = require("./BaseNode");
class DelayNode extends BaseNode_1.BaseNode {
    duration = 0;
    async execute(_sessionId) {
        // 처음 실행 시 타이머 시작
        if (!this.startTime) {
            this.startTimer();
            this.duration = this.config.duration || 1000;
        }
        const elapsed = Date.now() - (this.startTime || 0);
        if (elapsed >= this.duration) {
            // 대기 완료
            this.startTime = undefined;
            return this.success(`Delayed for ${this.duration}ms`);
        }
        // 아직 대기 중
        return this.running();
    }
}
exports.DelayNode = DelayNode;
//# sourceMappingURL=DelayNode.js.map