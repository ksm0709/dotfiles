export declare class MockClient {
    session: any;
    toast: any;
    private responseQueue;
    private defaultResponse;
    constructor();
    /**
     * 다음 에이전트 응답 설정
     */
    enqueueResponse(response: string): void;
    /**
     * 기본 응답 설정
     */
    setDefaultResponse(response: string): void;
    /**
     * 큐 초기화
     */
    clearResponses(): void;
}
//# sourceMappingURL=MockClient.d.ts.map