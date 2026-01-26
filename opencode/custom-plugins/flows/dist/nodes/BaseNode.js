"use strict";
/**
 * OpenCode Flows Plugin - Base Node
 *
 * 모든 노드가 상속받는 추상 기본 클래스
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.BaseNode = void 0;
class BaseNode {
    nodeName;
    config;
    blackboard;
    constructor(nodeName, config, blackboard) {
        this.nodeName = nodeName;
        this.config = config;
        this.blackboard = blackboard;
    }
    /**
     * 성공 결과 생성 헬퍼
     */
    success(message, data) {
        return {
            name: "success",
            message,
            data,
            duration: this.getDuration(),
        };
    }
    /**
     * 실패 결과 생성 헬퍼
     */
    failed(message, data) {
        return {
            name: "failed",
            message,
            data,
            duration: this.getDuration(),
        };
    }
    /**
     * 실행 중 결과 생성 헬퍼
     */
    running() {
        return {
            name: "running",
            message: "Node is still executing...",
        };
    }
    /**
     * 커스텀 결과 생성 헬퍼
     */
    result(name, message, data) {
        return {
            name,
            message,
            data,
            duration: this.getDuration(),
        };
    }
    /**
     * 변수 해석 헬퍼
     */
    resolveVariables(template) {
        return this.blackboard.resolveVariables(template);
    }
    /**
     * 설정값 변수 해석 헬퍼
     */
    resolveConfig(config) {
        return this.blackboard.resolveObjectVariables(config);
    }
    // 실행 시간 측정용
    startTime;
    startTimer() {
        this.startTime = Date.now();
    }
    getDuration() {
        if (this.startTime) {
            return Date.now() - this.startTime;
        }
        return undefined;
    }
}
exports.BaseNode = BaseNode;
//# sourceMappingURL=BaseNode.js.map