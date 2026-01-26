"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
exports.AgentNode = void 0;
const BaseNode_1 = require("./BaseNode");
class AgentNode extends BaseNode_1.BaseNode {
    pendingPromise;
    storedResult;
    client; // OpenCode client
    constructor(nodeName, config, blackboard, client) {
        super(nodeName, config, blackboard);
        this.client = client;
    }
    async execute(sessionId) {
        console.log(`[AgentNode ${this.nodeName}] Execute`);
        // 이미 실행 중이면 결과 확인
        if (this.pendingPromise) {
            if (this.storedResult) {
                const r = this.storedResult;
                this.storedResult = undefined;
                this.pendingPromise = undefined;
                return r;
            }
            return this.running();
        }
        this.startTimer();
        // 프롬프트 변수 해석
        const resolvedPrompt = this.resolveVariables(this.config.prompt);
        // 히스토리 컨텍스트 빌드
        const contextPrompt = this.buildContextPrompt(resolvedPrompt);
        // 비동기 실행 시작
        this.pendingPromise = this.executeAgent(sessionId, contextPrompt);
        return this.running();
    }
    /**
     * 히스토리 기반 컨텍스트 프롬프트 구성
     */
    buildContextPrompt(basePrompt) {
        // 개선된 히스토리 빌드 메서드 사용
        const historyContext = this.blackboard.buildHistoryContext?.(5000) || "";
        if (!historyContext) {
            return basePrompt;
        }
        return `${historyContext}\n\n---\n\n${basePrompt}`;
    }
    /**
     * 에이전트 실행
     */
    async executeAgent(sessionId, prompt) {
        try {
            // 결과 선택 가이드 주입 (arbitrary results + data extraction)
            const resultGuide = this.buildResultGuide();
            const fullPrompt = `${prompt}\n\n${resultGuide}`;
            // OpenCode 세션에 프롬프트 전송
            await this.client.session.prompt(sessionId, fullPrompt);
            // 응답에서 결과 추출
            const messages = await this.client.session.messages(sessionId);
            const lastMessage = messages[messages.length - 1];
            const response = lastMessage?.parts.find((p) => p.type === "text")?.text || "";
            this.storedResult = await this.parseResult(response);
        }
        catch (error) {
            this.storedResult = this.failed(`Agent execution failed: ${error}`);
        }
    }
    /**
     * 결과 가이드 문자열 생성
     * 에이전트가 어떤 결과를 반환할지 선택하도록 안내
     * + 데이터 추출 문법 안내
     */
    buildResultGuide() {
        const results = this.config.results || {};
        // 개선된 스키마: results가 객체 형태일 수 있음 { name: description }
        let resultOptions;
        if (Array.isArray(results)) {
            // 기존 배열 형식
            resultOptions = results
                .map((r) => `- [RESULT:${r.name}] ${r.description}`)
                .join("\n");
        }
        else {
            // 개선된 객체 형식 { name: description }
            resultOptions = Object.entries(results)
                .map(([name, desc]) => `- [RESULT:${name}] ${desc}`)
                .join("\n");
        }
        // 기본 결과 추가 (정의되지 않은 경우)
        if (!resultOptions) {
            resultOptions = "- [RESULT:success] 작업 성공\n- [RESULT:failed] 작업 실패";
        }
        return `
---
[FLOW RESULT INSTRUCTION]
작업 완료 후, 아래 결과 중 가장 적합한 것을 마지막에 반드시 출력하세요:
${resultOptions}

형식 예시: [RESULT:success] 작업이 성공적으로 완료되었습니다.

[DATA EXTRACTION - 선택사항]
후속 노드에 데이터를 전달하려면 다음 형식을 사용하세요:
[DATA:key=value]

예시:
[DATA:file_path=/src/utils/helper.ts]
[DATA:error_count=3]
---`;
    }
    /**
     * 응답에서 결과 및 데이터 파싱
     */
    async parseResult(response) {
        // 1. [DATA:key=value] 패턴 추출 및 blackboard 저장
        const extractedData = await this.extractData(response);
        // 2. [RESULT:name] 패턴 찾기
        const match = response.match(/\[RESULT:(\w+)\]/);
        if (match) {
            const resultName = match[1];
            const validResults = this.getValidResultNames();
            // 정의된 결과 또는 기본 결과인지 확인
            if (validResults.includes(resultName) || ["success", "failed"].includes(resultName)) {
                return this.result(resultName, response, extractedData);
            }
        }
        // 패턴을 찾지 못한 경우 기본값
        // 에러 키워드가 있으면 failed, 그렇지 않으면 success
        if (response.toLowerCase().includes("error") ||
            response.toLowerCase().includes("failed") ||
            response.toLowerCase().includes("실패")) {
            return this.failed(response, extractedData);
        }
        return this.success(response, extractedData);
    }
    /**
     * 응답에서 [DATA:key=value] 패턴 추출
     */
    async extractData(response) {
        const data = {};
        // [DATA:key=value] 패턴 매칭 (여러 개 가능)
        const dataPattern = /\[DATA:(\w+)=([^\]]+)\]/g;
        let match;
        while ((match = dataPattern.exec(response)) !== null) {
            const key = match[1];
            let value = match[2];
            // 숫자 변환 시도
            if (/^\d+$/.test(value)) {
                value = parseInt(value, 10);
            }
            else if (/^\d+\.\d+$/.test(value)) {
                value = parseFloat(value);
            }
            else if (value === "true") {
                value = true;
            }
            else if (value === "false") {
                value = false;
            }
            data[key] = value;
        }
        return data;
    }
    /**
     * 유효한 결과 이름 목록 조회
     */
    getValidResultNames() {
        const results = this.config.results || {};
        if (Array.isArray(results)) {
            return results.map(r => r.name);
        }
        else {
            return Object.keys(results);
        }
    }
}
exports.AgentNode = AgentNode;
//# sourceMappingURL=AgentNode.js.map