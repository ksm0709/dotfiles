import { describe, it, expect, vi, beforeEach } from "vitest";
import { RalphLoopPlugin } from "../src/index";
import * as fs from "node:fs/promises";

// node:fs/promises 모킹
vi.mock("node:fs/promises", () => ({
  mkdir: vi.fn(),
  writeFile: vi.fn(),
}));

// console.warn 모킹
const consoleWarnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {});

// Mock SDK 및 Client
const mockClient = {
  session: {
    summarize: vi.fn(),
    delete: vi.fn(),
    create: vi.fn(),
    prompt: vi.fn(),
    messages: vi.fn(),
  },
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
      "[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 'DONE'을 출력하세요.",
    );

    // 두 번째 메시지에서는 추가되지 않아야 함
    await pluginInstance["chat.message"](input, output);
    expect(output.parts.length).toBe(1);
  });

  it("Promise Word 감지 시 루프 종료", async () => {
    // 마지막 메시지에 DONE 포함
    mockClient.session.messages.mockResolvedValue({
      data: [
        {
          role: "assistant",
          parts: [{ type: "text", text: "분석을 마쳤습니다. DONE" }],
        },
      ],
      error: undefined,
    });

    await pluginInstance.event({
      event: {
        type: "session.idle",
        sessionId: "sess_123",
      },
    });

    // 추가 작업이 실행되지 않아야 함
    expect(mockClient.session.summarize).not.toHaveBeenCalled();
    expect(mockClient.session.create).not.toHaveBeenCalled();
  });

  it("Promise Word 부재 시 루프 실행 (summarize, delete, create, prompt 호출)", async () => {
    // 마지막 메시지에 DONE 없음
    mockClient.session.messages.mockResolvedValue({
      data: [
        {
          role: "assistant",
          parts: [{ type: "text", text: "아직 작업 중입니다..." }],
        },
      ],
      error: undefined,
    });
    mockClient.session.summarize.mockResolvedValue({
      data: "요약된 내용입니다.",
      error: undefined,
    });
    mockClient.session.create.mockResolvedValue({
      data: { id: "sess_456" },
      error: undefined,
    });

    await pluginInstance.event({
      event: {
        type: "session.idle",
        sessionId: "sess_123",
      },
    });

    // 요약, 저장, 삭제, 생성, 프롬프트 순서대로 호출되는지 확인
    expect(mockClient.session.summarize).toHaveBeenCalledWith({ path: { id: "sess_123" } });

    // fs.writeFile 호출 확인
    expect(fs.writeFile).toHaveBeenCalledWith(
      expect.stringContaining("ralph_summary.md"),
      "요약된 내용입니다.",
      "utf8",
    );

    expect(mockClient.session.delete).toHaveBeenCalledWith({ path: { id: "sess_123" } });
    expect(mockClient.session.create).toHaveBeenCalledWith({});
    expect(mockClient.session.prompt).toHaveBeenCalled();
  });

  it("최대 반복 횟수 도달 시 중단 및 로그 출력", async () => {
    mockClient.session.messages.mockResolvedValue({
      data: [
        {
          role: "assistant",
          parts: [{ type: "text", text: "루프 도는 중..." }],
        },
      ],
      error: undefined,
    });

    await pluginInstance.config({ maxRetries: 1 });

    // 1회차 실행: sess_retry -> sess_new_1 생성
    mockClient.session.create.mockResolvedValueOnce({
      data: { id: "sess_new_1" },
      error: undefined,
    });
    await pluginInstance.event({
      event: { type: "session.idle", sessionId: "sess_retry" },
    });

    // 2회차: sess_new_1에서 최대 횟수 도달로 중단
    await pluginInstance.event({
      event: { type: "session.idle", sessionId: "sess_new_1" },
    });

    // console.warn가 호출되었는지 확인 (toast 대신)
    expect(console.warn).toHaveBeenCalledWith(
      expect.stringContaining("최대 재시도 횟수에 도달했습니다"),
    );
  });

  it("세션 ID가 변경되어도 retryCount가 유지되어야 함 (QA 결함 1 재현)", async () => {
    mockClient.session.messages.mockResolvedValue({
      data: [
        { role: "assistant", parts: [{ type: "text", text: "작업 중..." }] },
      ],
      error: undefined,
    });
    mockClient.session.summarize.mockResolvedValue({
      data: "Summary",
      error: undefined,
    });

    // 첫 번째 호출: sess_1 -> sess_2 생성
    mockClient.session.create.mockResolvedValueOnce({
      data: { id: "sess_2" },
      error: undefined,
    });
    await pluginInstance.event({ event: { type: "session.idle", sessionId: "sess_1" } });

    // 두 번째 호출: sess_2 -> sess_3 생성
    mockClient.session.create.mockResolvedValueOnce({
      data: { id: "sess_3" },
      error: undefined,
    });
    await pluginInstance.event({ event: { type: "session.idle", sessionId: "sess_2" } });

    // maxRetries가 5(기본값)이므로 2회 호출로는 중단되지 않아야 함.
    // 하지만 retryCount가 유지되는지 확인하기 위해 maxRetries를 2로 설정해봄
    await pluginInstance.config({ maxRetries: 2 });

    // 세 번째 호출: sess_3에서 중단되어야 함 (retryCount가 1, 2 쌓여서 2에 도달)
    await pluginInstance.event({ event: { type: "session.idle", sessionId: "sess_3" } });

    expect(console.warn).toHaveBeenCalledWith(
      expect.stringContaining("최대 재시도 횟수에 도달했습니다"),
    );
  });

  it("새 세션의 첫 메시지에서 지시사항 중복 주입 방지 (QA 결함 2 재현)", async () => {
    // 세션 재생성 시나리오
    mockClient.session.messages.mockResolvedValue({
      data: [
        { role: "assistant", parts: [{ type: "text", text: "계속..." }] },
      ],
      error: undefined,
    });
    mockClient.session.create.mockResolvedValue({
      data: { id: "sess_new" },
      error: undefined,
    });

    // 1. 루프 발생 (sess_old -> sess_new)
    await pluginInstance.event({
      event: { type: "session.idle", sessionId: "sess_old" },
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

  it("사용자 메시지에 'ralph' 키워드가 포함되면 프롬프트 인젝션 발생", async () => {
    const input = "ralph에게 작업 상태를 물어봐";
    const output = {
      sessionId: "sess_789",
      parts: [{ type: "text", text: "기존 내용" }],
    };

    await pluginInstance["chat.message"](input, output);

    expect(output.parts.length).toBe(2);
    expect(output.parts[1].text).toContain(
      "[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 'DONE'을 출력하세요.",
    );
  });

  it("사용자 메시지에 'RALPH' 대문자 키워드가 포함되어도 프롬프트 인젝션 발생", async () => {
    const input = "RALPH에게 현재 진행 상황 보고";
    const output = {
      sessionId: "sess_790",
      parts: [{ type: "text", text: "기존 내용" }],
    };

    await pluginInstance["chat.message"](input, output);

    expect(output.parts.length).toBe(2);
    expect(output.parts[1].text).toContain(
      "[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 'DONE'을 출력하세요.",
    );
  });

  it("사용자 메시지에 'ralph' 키워드가 없으면 프롬프트 인젝션 미발생", async () => {
    const input = "작업 상태를 확인해줘";
    const output = {
      sessionId: "sess_791",
      parts: [{ type: "text", text: "기존 내용" }],
    };

    // 먼저 한 번 메시지를 보내서 messageCount를 1로 만듦
    await pluginInstance["chat.message"]("첫 메시지", {
      sessionId: "sess_791",
      parts: [{ type: "text", text: "첫 내용" }],
    });

    // 두 번째 메시지 (키워드 없음)
    await pluginInstance["chat.message"](input, output);

    // 첫 메시지가 아니고 키워드도 없으므로 인젝션되지 않아야 함
    expect(output.parts.length).toBe(1);
  });

  it("첫 메시지와 'ralph' 키워드가 중복되어도 중복 인젝션 방지", async () => {
    const input = "ralph에게 안녕";
    const output = {
      sessionId: "sess_792",
      parts: [{ type: "text", text: "첫 메시지" }],
    };

    await pluginInstance["chat.message"](input, output);

    // 첫 메시지이므로 인젝션되지만, 중복으로 추가되지는 않음
    expect(output.parts.length).toBe(2);
    expect(output.parts[1].text).toContain(
      "[Ralph Loop 플러그인] 모든 작업이 완료되면 반드시 'DONE'을 출력하세요.",
    );
  });
});
