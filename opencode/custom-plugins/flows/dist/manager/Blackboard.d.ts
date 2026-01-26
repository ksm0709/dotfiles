/**
 * OpenCode Flows Plugin - Blackboard
 *
 * 플로우 인스턴스의 상태를 관리하는 Key-Value 저장소
 * 파일 기반 영속화를 지원하여 crash recovery 가능
 *
 * ## 히스토리 관리 정책 (개선됨)
 * - 각 노드의 결과는 노드 이름을 키로 하여 `_results`에 저장
 * - 노드 이름은 플로우 내에서 유일하므로 충돌 없음
 * - 같은 노드가 여러 번 실행되면 최신 결과로 덮어쓰고 executionCount 증가
 * - 실행 순서는 `_execution_order` 배열에 별도 기록
 */
import type { BlackboardData, NodeResult } from "../types/schemas";
/** 노드 결과 저장 구조 */
interface NodeResultEntry {
    result: NodeResult;
    timestamp: string;
    executionCount: number;
}
/** 노드 결과 맵 */
interface ResultsMap {
    [nodeName: string]: NodeResultEntry;
}
export declare class Blackboard {
    private cache;
    private dirty;
    private readonly filePath;
    private saveDebounceTimer;
    constructor(instanceId: string);
    /**
     * 저장된 상태 로드 (있는 경우)
     */
    load(): Promise<void>;
    /**
     * 현재 상태를 파일로 저장
     */
    save(): Promise<void>;
    /**
     * 디바운스된 저장 (연속 호출 최적화)
     */
    private debouncedSave;
    /**
     * 값 조회
     */
    get<T = any>(key: string): T | undefined;
    /**
     * 값 조회 (중첩 키 지원)
     * 예: "history.analyze.data.score"
     */
    getValue(path: string): any;
    /**
     * 값 설정
     */
    set(key: string, value: any): Promise<void>;
    /**
     * 값 존재 여부 확인
     */
    has(key: string): boolean;
    /**
     * 값 삭제
     */
    delete(key: string): Promise<boolean>;
    /**
     * 전체 데이터 반환
     */
    getAll(): BlackboardData;
    /**
     * 노드 결과 저장 (개선된 방식)
     *
     * - 노드 이름을 키로 하여 _results에 저장
     * - 같은 노드가 여러 번 실행되면 덮어쓰고 executionCount 증가
     * - 실행 순서는 _execution_order에 기록
     */
    saveNodeResult(nodeName: string, result: NodeResult): Promise<void>;
    /**
     * 특정 노드의 결과 조회
     */
    getNodeResult(nodeName: string): NodeResult | undefined;
    /**
     * 특정 노드의 결과 엔트리 전체 조회 (메타데이터 포함)
     */
    getNodeResultEntry(nodeName: string): NodeResultEntry | undefined;
    /**
     * 모든 노드 결과 조회
     */
    getAllNodeResults(): ResultsMap;
    /**
     * 실행 순서 조회
     */
    getExecutionOrder(): string[];
    /**
     * 변수 해석: ${key} 형태를 실제 값으로 치환
     *
     * 지원 패턴:
     * - ${key}: blackboard의 직접 키
     * - ${prompt}: 초기 프롬프트
     * - ${history.nodeName}: 특정 노드의 결과 메시지 (단축)
     * - ${history.nodeName.message}: 특정 노드의 결과 메시지 (명시적)
     * - ${history.nodeName.data.field}: 특정 노드 결과의 데이터 필드
     * - ${_results.nodeName.result.message}: 직접 접근
     */
    resolveVariables(template: string): string;
    /**
     * 객체 내 모든 문자열 값의 변수 해석
     */
    resolveObjectVariables<T extends Record<string, any>>(obj: T): T;
    /**
     * 히스토리 컨텍스트 빌드 (에이전트 프롬프트용)
     * 실행 순서대로 모든 노드 결과를 포맷팅
     */
    buildHistoryContext(maxLength?: number): string;
    /**
     * 인스턴스 데이터 삭제 (종료 시)
     */
    cleanup(): Promise<void>;
}
export {};
//# sourceMappingURL=Blackboard.d.ts.map