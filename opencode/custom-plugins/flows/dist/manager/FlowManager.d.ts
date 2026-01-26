/**
 * OpenCode Flows Plugin - Flow Manager
 *
 * 싱글톤 매니저: 모든 플로우 인스턴스의 라이프사이클 관리
 * 500ms 간격으로 tick loop 실행
 */
import type { FlowInstanceStatus, PluginConfig } from "../types/schemas";
export declare class FlowManager {
    private static instance;
    private instances;
    private tickInterval;
    private config;
    private client;
    private projectDir;
    private constructor();
    /**
     * 싱글톤 인스턴스 획득
     */
    static getInstance(): FlowManager;
    /**
     * 매니저 초기화
     */
    initialize(client: any, projectDir: string, config?: Partial<PluginConfig>): void;
    /**
     * 새 플로우 인스턴스 시작
     */
    start(flowName: string, prompt?: string): Promise<string>;
    /**
     * 플로우 정의 로드
     */
    private loadFlowDefinition;
    /**
     * 사용 가능한 플로우 목록 조회
     */
    listAvailableFlows(): Promise<Array<{
        name: string;
        description?: string;
        path: string;
    }>>;
    /**
     * Tick 루프 시작/유지
     */
    private ensureTickLoop;
    /**
     * Tick 루프 중지
     */
    private stopTickLoop;
    /**
     * 인스턴스 ID 생성
     */
    private generateInstanceId;
    /**
     * 모든 인스턴스 상태 조회
     */
    getStatus(): FlowInstanceStatus[];
    /**
     * 특정 인스턴스 상태 조회
     */
    getInstanceStatus(instanceId: string): FlowInstanceStatus | undefined;
    /**
     * 인스턴스 중단
     */
    stop(instanceId: string): Promise<boolean>;
    /**
     * 모든 인스턴스 중단
     */
    stopAll(): Promise<number>;
    /**
     * 실행 중인 인스턴스 수
     */
    get runningCount(): number;
    /**
     * 전체 인스턴스 수
     */
    get totalCount(): number;
}
//# sourceMappingURL=FlowManager.d.ts.map