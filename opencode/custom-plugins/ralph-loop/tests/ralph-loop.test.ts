import { describe, it, expect, vi, beforeEach } from "vitest";
import { RalphLoopPlugin } from "../src/index";
import * as fs from "node:fs/promises";

// node:fs/promises 모킹
vi.mock("node:fs/promises", () => ({
  mkdir: vi.fn(),
  writeFile: vi.fn(),
}));

// Mock SDK 및 Client
const mockClient = {
  session: {
    summarize: vi.fn(),
    delete: vi.fn(),
    create: vi.fn(),
    prompt: vi.fn(),
    messages: vi.fn(),
  },
  toast: vi.fn(),
};

const mockDirectory = "/tmp/project";
const mock$ = vi.fn();

describe("Ralph Loop Plugin Scenarios", () => {
  let pluginInstance: {
    config: (newConfig: any) => Promise<void>;
    event: (input: { event: any }) => Promise<void>;
    "chat.message": (input: string, output: any) => Promise<void>;
  };

  beforeEach(async () => {
    vi.clearAllMocks();
    pluginInstance = (await RalphLoopPlugin({
      $: mock$,
      directory: mockDirectory,
      client: mockClient as any,
    })) as any;
  });

  it("최초 프롬프트 인젝션 (chat.message 훅)", async () => {
    const input = "프로젝트 구조 분석해줘";
    const output = {
      sessionId: "sess_123",
      parts: [] as { type: string; text: string }[],
    };

    await pluginInstance["chat.message"](input, output);

    expect(output.parts.length).toBe(1);
    expect(output.parts[0].text).toContain(
      "(중요: 모든 작업이 완료되면 반드시 'DONE'을 출력하세요.)",
    );

    // 두 번째 메시지에서는 추가되지 않아야 함
    await pluginInstance["chat.message"](input, output);
    expect(output.parts.length).toBe(1);
  });

  it("Promise Word 감지 시 루프 종료", async () => {
    // 마지막 메시지에 DONE 포함
    mockClient.session.messages.mockResolvedValue([
      {
        role: "assistant",
        parts: [{ type: "text", text: "분석을 마쳤습니다. DONE" }],
      },
    ]);

    await pluginInstance.event({
      event: {
        type: "session.idle",
        data: { sessionId: "sess_123" },
      },
    });

    // 추가 작업이 실행되지 않아야 함
    expect(mockClient.session.summarize).not.toHaveBeenCalled();
    expect(mockClient.session.create).not.toHaveBeenCalled();
  });

  it("Promise Word 부재 시 루프 실행 (summarize, delete, create, prompt 호출)", async () => {
    // 마지막 메시지에 DONE 없음
    mockClient.session.messages.mockResolvedValue([
      {
        role: "assistant",
        parts: [{ type: "text", text: "아직 작업 중입니다..." }],
      },
    ]);
    mockClient.session.summarize.mockResolvedValue("요약된 내용입니다.");
    mockClient.session.create.mockResolvedValue({ id: "sess_456" });

    await pluginInstance.event({
      event: {
        type: "session.idle",
        data: { sessionId: "sess_123" },
      },
    });

    // 요약, 저장, 삭제, 생성, 프롬프트 순서대로 호출되는지 확인
    expect(mockClient.session.summarize).toHaveBeenCalledWith("sess_123");

    // fs.writeFile 호출 확인
    expect(fs.writeFile).toHaveBeenCalledWith(
      expect.stringContaining("ralph_summary.md"),
      "요약된 내용입니다.",
      "utf8",
    );

    expect(mockClient.session.delete).toHaveBeenCalledWith("sess_123");
    expect(mockClient.session.create).toHaveBeenCalled();
    expect(mockClient.session.prompt).toHaveBeenCalled();
  });

  it("최대 반복 횟수 도달 시 중단 및 Toast 표시", async () => {
    mockClient.session.messages.mockResolvedValue([
      {
        role: "assistant",
        parts: [{ type: "text", text: "루프 도는 중..." }],
      },
    ]);

    await pluginInstance.config({ maxRetries: 1 });

    // 1회차 실행: sess_retry -> sess_new_1 생성
    mockClient.session.create.mockResolvedValueOnce({ id: "sess_new_1" });
    await pluginInstance.event({
      event: { type: "session.idle", data: { sessionId: "sess_retry" } },
    });

    // 2회차: sess_new_1에서 최대 횟수 도달로 중단
    await pluginInstance.event({
      event: { type: "session.idle", data: { sessionId: "sess_new_1" } },
    });

    expect(mockClient.toast).toHaveBeenCalledWith(
      expect.stringContaining("최대 재시도 횟수에 도달했습니다"),
    );
  });

  it("세션 ID가 변경되어도 retryCount가 유지되어야 함 (QA 결함 1 재현)", async () => {
    mockClient.session.messages.mockResolvedValue([
      { role: "assistant", parts: [{ type: "text", text: "작업 중..." }] },
    ]);
    mockClient.session.summarize.mockResolvedValue("Summary");

    // 첫 번째 호출: sess_1 -> sess_2 생성
    mockClient.session.create.mockResolvedValueOnce({ id: "sess_2" });
    await pluginInstance.event({ event: { type: "session.idle", data: { sessionId: "sess_1" } } });

    // 두 번째 호출: sess_2 -> sess_3 생성
    mockClient.session.create.mockResolvedValueOnce({ id: "sess_3" });
    await pluginInstance.event({ event: { type: "session.idle", data: { sessionId: "sess_2" } } });

    // maxRetries가 5(기본값)이므로 2회 호출로는 중단되지 않아야 함.
    // 하지만 retryCount가 유지되는지 확인하기 위해 maxRetries를 2로 설정해봄
    await pluginInstance.config({ maxRetries: 2 });

    // 세 번째 호출: sess_3에서 중단되어야 함 (retryCount가 1, 2 쌓여서 2에 도달)
    await pluginInstance.event({ event: { type: "session.idle", data: { sessionId: "sess_3" } } });

    expect(mockClient.toast).toHaveBeenCalledWith(
      expect.stringContaining("최대 재시도 횟수에 도달했습니다"),
    );
  });

  it("새 세션의 첫 메시지에서 지시사항 중복 주입 방지 (QA 결함 2 재현)", async () => {
    // 세션 재생성 시나리오
    mockClient.session.messages.mockResolvedValue([
      { role: "assistant", parts: [{ type: "text", text: "계속..." }] },
    ]);
    mockClient.session.create.mockResolvedValue({ id: "sess_new" });

    // 1. 루프 발생 (sess_old -> sess_new)
    await pluginInstance.event({
      event: { type: "session.idle", data: { sessionId: "sess_old" } },
    });

    // 2. sess_new에 대한 첫 메시지(사용자 또는 시스템 프롬프트) 발생 시 chat.message 훅 동작
    const output = {
      sessionId: "sess_new",
      parts: [{ type: "text", text: "이전 세션 요약..." }], // 이미 프롬프트에 지시사항이 포함되어 있을 수 있음
    };

    await pluginInstance["chat.message"]("입력", output);

    // 새 세션의 첫 프롬프트에 이미 지시사항이 포함되어 있다면, chat.message에서 또 추가하면 안 됨.
    // 현재 코드(src/index.ts)는 restartPrompt에 지시사항을 포함하고(라인 79),
    // chat.message에서도 messageCount === 1이면 추가함(라인 90).
    // 만약 client.session.prompt 호출이 chat.message 훅을 트리거한다면 중복 발생.
    // (이 테스트는 chat.message 훅 단독으로도 이미 이관된 상태를 인지해야 함을 검증)

    // 중복 주입 방지 로직이 없다면 output.parts에 지시사항이 추가되어 길이가 2가 됨.
    // 결함 수정 후에는 추가되지 않아야 함.
    expect(output.parts.length).toBe(1);
  });
});
