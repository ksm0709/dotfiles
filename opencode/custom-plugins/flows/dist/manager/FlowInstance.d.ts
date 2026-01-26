/**
 * OpenCode Flows Plugin - Flow Instance
 *
 * 개별 플로우 실행 인스턴스
 * FSM 상태 관리 및 노드 실행을 담당
 */
import type { FlowDefinition, FlowStatus, FlowInstanceStatus } from "../types/schemas";
import { Blackboard } from "./Blackboard";
export declare class FlowInstance {
    private instanceId;
    private flowDef;
    private initialPrompt;
    private client;
    status: FlowStatus;
    private blackboard;
    private currentNode;
    private retryCount;
    private stepCount;
    private currentNodeInstance;
    private sessionId?;
    private startTime;
    constructor(instanceId: string, flowDef: FlowDefinition, initialPrompt: string | undefined, client: any);
    /**
     * 인스턴스 초기화
     */
    initialize(): Promise<void>;
    /**
     * Tick - 매니저에 의해 주기적으로 호출됨
     */
    tick(): Promise<void>;
    /**
     * 다음 노드 결정
     */
    private getNextNode;
    /**
     * 인스턴스 상태 조회
     */
    getStatus(): FlowInstanceStatus;
    /**
     * 인스턴스 중단
     */
    stop(): Promise<void>;
    /**
     * Blackboard 직접 접근 (디버깅용)
     */
    getBlackboard(): Blackboard;
}
//# sourceMappingURL=FlowInstance.d.ts.map