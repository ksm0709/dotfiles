"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.MockClient = void 0;
const vitest_1 = require("vitest");
class MockClient {
    session;
    toast;
    responseQueue = [];
    defaultResponse = "[RESULT:success] Default mock response";
    constructor() {
        this.toast = vitest_1.vi.fn();
        this.session = {
            create: vitest_1.vi.fn().mockResolvedValue({ id: "mock-session-id" }),
            delete: vitest_1.vi.fn().mockResolvedValue(undefined),
            prompt: vitest_1.vi.fn().mockResolvedValue(undefined),
            messages: vitest_1.vi.fn().mockImplementation(async () => {
                const response = this.responseQueue.shift() || this.defaultResponse;
                return [
                    {
                        role: "assistant",
                        parts: [{ type: "text", text: response }]
                    }
                ];
            }),
        };
    }
    /**
     * 다음 에이전트 응답 설정
     */
    enqueueResponse(response) {
        this.responseQueue.push(response);
    }
    /**
     * 기본 응답 설정
     */
    setDefaultResponse(response) {
        this.defaultResponse = response;
    }
    /**
     * 큐 초기화
     */
    clearResponses() {
        this.responseQueue = [];
    }
}
exports.MockClient = MockClient;
//# sourceMappingURL=MockClient.js.map