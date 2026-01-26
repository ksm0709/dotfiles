"use strict";
/**
 * OpenCode Flows Plugin - Flow Instance
 *
 * 개별 플로우 실행 인스턴스
 * FSM 상태 관리 및 노드 실행을 담당
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.FlowInstance = void 0;
const Blackboard_1 = require("./Blackboard");
const NodeFactory_1 = require("../nodes/NodeFactory");
class FlowInstance {
    instanceId;
    flowDef;
    initialPrompt;
    client;
    status = "initializing";
    blackboard;
    currentNode;
    retryCount = 0;
    stepCount = 0;
    currentNodeInstance = null;
    sessionId;
    startTime;
    constructor(instanceId, flowDef, initialPrompt, client) {
        this.instanceId = instanceId;
        this.flowDef = flowDef;
        this.initialPrompt = initialPrompt;
        this.client = client;
        this.blackboard = new Blackboard_1.Blackboard(instanceId);
        this.currentNode = flowDef.initial_state;
        this.startTime = Date.now();
    }
    /**
     * 인스턴스 초기화
     */
    async initialize() {
        // 기존 상태 로드 시도 (crash recovery)
        await this.blackboard.load();
        // 새 인스턴스인 경우 초기화
        if (!this.blackboard.has("_instance_id")) {
            await this.blackboard.set("prompt", this.initialPrompt || "");
            await this.blackboard.set("_instance_id", this.instanceId);
            await this.blackboard.set("_flow_name", this.flowDef.name);
            await this.blackboard.set("_current_state", this.currentNode);
            await this.blackboard.set("_started_at", new Date().toISOString());
            await this.blackboard.set("_history", []);
            // 플로우 설정 저장
            const config = this.flowDef.config;
            await this.blackboard.set("_config_history_depth", config?.history_depth ?? 5);
            await this.blackboard.set("_config_timeout", config?.timeout ?? 300000);
            await this.blackboard.set("_config_max_retries", config?.max_retries ?? 3);
            await this.blackboard.set("_config_max_steps", config?.max_steps ?? 100);
        }
        else {
            // 복구된 인스턴스
            this.currentNode = this.blackboard.get("_current_state") || this.currentNode;
        }
        // OpenCode 세션 생성 (플로우당 단일 세션)
        if (!this.blackboard.has("_session_id")) {
            const session = await this.client.session.create();
            this.sessionId = session.id;
            await this.blackboard.set("_session_id", session.id);
        }
        else {
            this.sessionId = this.blackboard.get("_session_id");
        }
        this.status = "running";
    }
    /**
     * Tick - 매니저에 의해 주기적으로 호출됨
     */
    async tick() {
        if (this.status !== "running")
            return;
        console.log(`[FlowInstance] Tick: ${this.currentNode}`);
        // 1. 타임아웃 체크
        const timeout = this.blackboard.get("_config_timeout") || 300000;
        if (Date.now() - this.startTime > timeout) {
            this.status = "failed";
            await this.blackboard.set("error", `Flow timed out after ${timeout}ms`);
            return;
        }
        // 2. 최대 스텝 체크
        const maxSteps = this.blackboard.get("_config_max_steps") || 100;
        if (this.stepCount >= maxSteps) {
            this.status = "failed";
            await this.blackboard.set("error", `Max steps (${maxSteps}) exceeded`);
            return;
        }
        const nodeDef = this.flowDef.nodes[this.currentNode];
        if (!nodeDef) {
            this.status = "failed";
            await this.blackboard.set("error", `Node not found: ${this.currentNode}`);
            return;
        }
        try {
            // 노드 인스턴스 생성 (없는 경우)
            if (!this.currentNodeInstance) {
                this.currentNodeInstance = (0, NodeFactory_1.createNode)(this.currentNode, nodeDef, this.blackboard, this.client);
            }
            // 노드 실행
            const result = await this.currentNodeInstance.execute(this.sessionId);
            // running 상태면 다음 tick 대기
            if (result.name === "running") {
                return;
            }
            // 히스토리 기록 (노드별 전체 저장)
            await this.blackboard.saveNodeResult(this.currentNode, result);
            // 결과 데이터를 blackboard에 저장
            if (result.data) {
                for (const [key, value] of Object.entries(result.data)) {
                    await this.blackboard.set(`${this.currentNode}_${key}`, value);
                }
            }
            // 라우팅
            const nextNode = this.getNextNode(nodeDef, result.name);
            if (!nextNode) {
                // 다음 노드가 없음 - 현재 노드 타입 확인
                if (nodeDef.type === "end") {
                    const endStatus = nodeDef.config?.status || "success";
                    this.status = endStatus === "success" ? "completed" : "failed";
                    await this.blackboard.set("_final_status", endStatus);
                    await this.blackboard.set("_final_message", result.message);
                }
                else {
                    // 라우트가 정의되지 않은 결과
                    this.status = "failed";
                    await this.blackboard.set("error", `No route for result: ${result.name}`);
                }
                return;
            }
            // 상태 전이
            this.currentNode = nextNode;
            this.currentNodeInstance = null; // 노드 인스턴스 리셋
            this.retryCount = 0;
            this.stepCount++;
            await this.blackboard.set("_current_state", nextNode);
        }
        catch (error) {
            // 에러 핸들링 - 재시도
            const maxRetries = this.blackboard.get("_config_max_retries") || 3;
            this.retryCount++;
            if (this.retryCount >= maxRetries) {
                this.status = "failed";
                await this.blackboard.set("error", `Max retries exceeded: ${error}`);
            }
            else {
                // 재시도 대기
                await this.blackboard.set("_retry_count", this.retryCount);
            }
        }
    }
    /**
     * 다음 노드 결정
     */
    getNextNode(nodeDef, resultName) {
        const routes = nodeDef.routes || {};
        // 정확히 일치하는 라우트
        if (resultName in routes) {
            return routes[resultName];
        }
        // 기본 라우트 (success/failed fallback)
        if (resultName !== "success" && resultName !== "failed") {
            if ("success" in routes && !resultName.includes("fail")) {
                return routes["success"];
            }
            if ("failed" in routes) {
                return routes["failed"];
            }
        }
        return null;
    }
    /**
     * 인스턴스 상태 조회
     */
    getStatus() {
        return {
            instanceId: this.instanceId,
            flowName: this.flowDef.name,
            currentNode: this.currentNode,
            status: this.status,
            startedAt: this.blackboard.get("_started_at") || new Date().toISOString(),
            elapsedMs: Date.now() - this.startTime,
            retryCount: this.retryCount,
        };
    }
    /**
     * 인스턴스 중단
     */
    async stop() {
        this.status = "failed";
        await this.blackboard.set("_stopped_at", new Date().toISOString());
        await this.blackboard.set("_stopped_reason", "Manual stop requested");
        // 세션 정리 (선택적)
        if (this.sessionId) {
            try {
                await this.client.session.delete(this.sessionId);
            }
            catch {
                // 무시
            }
        }
    }
    /**
     * Blackboard 직접 접근 (디버깅용)
     */
    getBlackboard() {
        return this.blackboard;
    }
}
exports.FlowInstance = FlowInstance;
//# sourceMappingURL=FlowInstance.js.map