import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import { FlowManager } from "./manager/FlowManager";
import { MockClient } from "./test/MockClient";
import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";

describe("Flow Integration Test", () => {
  let manager: FlowManager;
  let mockClient: MockClient;
  let tempDir: string;
  let flowsDir: string;

  beforeEach(async () => {
    // 임시 디렉토리 설정
    tempDir = await fs.mkdtemp(path.join(os.tmpdir(), "opencode-flows-test-"));
    flowsDir = path.join(tempDir, ".opencode/flows");
    await fs.mkdir(flowsDir, { recursive: true });

    // Mock Client 초기화
    mockClient = new MockClient();

    // Manager 초기화 (싱글톤 리셋 필요하지만 private이라 새 인스턴스처럼 동작하게 설정)
    // FlowManager.getInstance()는 싱글톤이므로, 테스트 간 상태 공유를 막기 위해
    // initialize를 매번 호출하여 설정을 덮어씁니다.
    manager = FlowManager.getInstance();
    manager.initialize(mockClient, tempDir, {
      flowsDir: ".opencode/flows",
      dataDir: path.join(tempDir, "data"),
      enableToasts: false,
      debugMode: true,
      tickInterval: 50,
      keepCompletedInstances: true
    });
    
    // 실행 중인 인스턴스 정리
    await manager.stopAll();
  });

  afterEach(async () => {
    await manager.stopAll();
    await fs.rm(tempDir, { recursive: true, force: true });
  });

  /**
   * 헬퍼: 플로우 완료 대기
   */
  async function waitForCompletion(instanceId: string, timeoutMs = 5000): Promise<string> {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
      const status = manager.getInstanceStatus(instanceId);
      if (status?.status === "completed" || status?.status === "failed") {
        return status.status;
      }
      await new Promise(resolve => setTimeout(resolve, 50));
    }
    return "timeout";
  }

  it("should execute a simple agent flow", async () => {
    // 1. 플로우 정의 파일 생성
    const flowJson = {
      name: "test-simple",
      version: "1.0.0",
      start: "step1",
      nodes: {
        step1: {
          agent: "general",
          prompt: "Hello",
          on: { success: "done" }
        },
        done: {
          end: true,
          message: "Finished"
        }
      }
    };
    await fs.writeFile(path.join(flowsDir, "test-simple.json"), JSON.stringify(flowJson));

    // 2. Mock 응답 설정
    mockClient.enqueueResponse("[RESULT:success] Agent worked!");

    // 3. 실행
    const instanceId = await manager.start("test-simple");
    
    // 4. 완료 대기
    const finalStatus = await waitForCompletion(instanceId, 15000);
    
    // 5. 검증
    expect(finalStatus).toBe("completed");
    
    const status = manager.getInstanceStatus(instanceId);
    expect(status?.currentNode).toBe("done");
    
    // Blackboard 검증 (private 접근을 위해 any 캐스팅 또는 getBlackboard 사용)
    // FlowManager -> instances -> get(id) -> getBlackboard()
    const instance = (manager as any).instances.get(instanceId);
    const blackboard = instance.getBlackboard();
    
    const history = blackboard.getNodeResult("step1");
    expect(history.message).toContain("Agent worked!");
  }, 20000);

  it("should handle conditional branching", async () => {
    // 1. 플로우 정의 (변수에 따라 분기)
    const flowJson = {
      name: "test-condition",
      version: "1.0.0",
      start: "check",
      nodes: {
        check: {
          conditions: [
            { field: "prompt", operator: "contains", value: "go_left", result: "left" }
          ],
          default: "right",
          on: {
            left: "node_left",
            right: "node_right"
          }
        },
        node_left: {
          end: true,
          message: "Went Left"
        },
        node_right: {
          end: true,
          message: "Went Right"
        }
      }
    };
    await fs.writeFile(path.join(flowsDir, "test-condition.json"), JSON.stringify(flowJson));

    // Case A: Go Left
    const id1 = await manager.start("test-condition", "please go_left");
    await waitForCompletion(id1, 15000);
    const status1 = manager.getInstanceStatus(id1);
    expect(status1?.currentNode).toBe("node_left");

    // Case B: Go Right (Default)
    const id2 = await manager.start("test-condition", "go somewhere else");
    await waitForCompletion(id2, 15000);
    const status2 = manager.getInstanceStatus(id2);
    expect(status2?.currentNode).toBe("node_right");
  }, 20000);

  it("should handle loops with max iterations", async () => {
    const flowJson = {
      name: "test-loop",
      version: "1.0.0",
      start: "loop_node",
      nodes: {
        loop_node: {
          loop: "dummy_action",
          max: 3,
          on: {
            continue: "dummy_action",
            max_reached: "done"
          }
        },
        dummy_action: {
          run: "echo 'looping'",
          on: { success: "loop_node" }
        },
        done: { end: true }
      }
    };
    await fs.writeFile(path.join(flowsDir, "test-loop.json"), JSON.stringify(flowJson));

    const id = await manager.start("test-loop");
    const result = await waitForCompletion(id, 15000);
    
    expect(result).toBe("completed");
    
    const instance = (manager as any).instances.get(id);
    const blackboard = instance.getBlackboard();
    
    // 루프가 3번 돌았는지 확인 (iteration 값은 0으로 초기화되므로 실행 횟수로 확인하거나 로그로 확인)
    // 여기서는 완료 여부만 확인
  }, 20000);

  it("should stop running flow", async () => {
    const flowJson = {
      name: "test-stop",
      version: "1.0.0",
      start: "wait",
      nodes: {
        wait: {
          wait: 5000,
          on: { success: "done" }
        },
        done: { end: true }
      }
    };
    await fs.writeFile(path.join(flowsDir, "test-stop.json"), JSON.stringify(flowJson));

    const id = await manager.start("test-stop");
    
    // Wait a bit
    await new Promise(r => setTimeout(r, 100));
    
    const statusBefore = manager.getInstanceStatus(id);
    expect(statusBefore?.status).toBe("running");

    // Stop
    await manager.stop(id);
    
    const statusAfter = manager.getInstanceStatus(id);
    // Stop removes instance immediately in current implementation?
    // No, stop() sets status to failed and then tick loop removes it.
    // But stop() calls instance.stop() then delete(id).
    expect(statusAfter).toBeUndefined();
  });

  it("should list available flows", async () => {
    const flowJson = {
      name: "test-list",
      version: "1.0.0",
      description: "Test Flow",
      start: "end",
      nodes: { end: { end: true } }
    };
    await fs.writeFile(path.join(flowsDir, "test-list.json"), JSON.stringify(flowJson));

    const flows = await manager.listAvailableFlows();
    expect(flows.length).toBeGreaterThan(0);
    expect(flows.find(f => f.name === "test-list")).toBeDefined();
  });
});
