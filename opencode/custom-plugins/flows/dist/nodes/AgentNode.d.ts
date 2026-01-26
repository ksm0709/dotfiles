/**
 * OpenCode Flows Plugin - Agent Node
 *
 * OpenCode 에이전트를 호출하는 노드
 * arbitrary results 지원을 위해 프롬프트에 결과 가이드 주입
 *
 * ## 데이터 추출 기능
 * 에이전트 응답에서 `[DATA:key=value]` 패턴을 파싱하여 blackboard에 저장
 * 이를 통해 후속 노드가 에이전트가 생성한 값을 참조할 수 있음
 */
import { BaseNode } from "./BaseNode";
import type { NodeResult, AgentNodeConfig } from "../types/schemas";
export declare class AgentNode extends BaseNode {
    private pendingPromise?;
    private storedResult?;
    private client;
    constructor(nodeName: string, config: AgentNodeConfig, blackboard: any, client: any);
    execute(sessionId: string): Promise<NodeResult>;
    /**
     * 히스토리 기반 컨텍스트 프롬프트 구성
     */
    private buildContextPrompt;
    /**
     * 에이전트 실행
     */
    private executeAgent;
    /**
     * 결과 가이드 문자열 생성
     * 에이전트가 어떤 결과를 반환할지 선택하도록 안내
     * + 데이터 추출 문법 안내
     */
    private buildResultGuide;
    /**
     * 응답에서 결과 및 데이터 파싱
     */
    private parseResult;
    /**
     * 응답에서 [DATA:key=value] 패턴 추출
     */
    private extractData;
    /**
     * 유효한 결과 이름 목록 조회
     */
    private getValidResultNames;
}
//# sourceMappingURL=AgentNode.d.ts.map