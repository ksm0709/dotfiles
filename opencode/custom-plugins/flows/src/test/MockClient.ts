import { vi } from "vitest";

export class MockClient {
  public session: any;
  public toast: any;
  public tools: any;
  
  private responseQueue: string[] = [];
  private defaultResponse: string = "[RESULT:success] Default mock response";

  constructor() {
    this.toast = vi.fn();
    this.tools = {
      call: vi.fn().mockResolvedValue("mock tool result")
    };
    
    this.session = {
      create: vi.fn().mockResolvedValue({ id: "mock-session-id" }),
      delete: vi.fn().mockResolvedValue(undefined),
      prompt: vi.fn().mockResolvedValue(undefined),
      messages: vi.fn().mockImplementation(async () => {
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
  enqueueResponse(response: string) {
    this.responseQueue.push(response);
  }

  /**
   * 기본 응답 설정
   */
  setDefaultResponse(response: string) {
    this.defaultResponse = response;
  }

  /**
   * 큐 초기화
   */
  clearResponses() {
    this.responseQueue = [];
  }
}
