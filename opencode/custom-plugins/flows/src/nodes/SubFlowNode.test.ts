import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import { Blackboard } from "../manager/Blackboard";
import { SubFlowNode } from "./SubFlowNode";
import { FlowManager } from "../manager/FlowManager";

describe("SubFlowNode", () => {
  let blackboard: Blackboard;
  let testInstanceId: string;

  beforeEach(async () => {
    testInstanceId = "test-subflow-" + Date.now();
    blackboard = new Blackboard(testInstanceId);
    await blackboard.load();
  });

  afterEach(async () => {
    await blackboard.cleanup();
    vi.restoreAllMocks();
  });

  it("should start and wait for sub-flow", async () => {
    const mockManager = {
      start: vi.fn().mockResolvedValue("child-id"),
      getInstanceStatus: vi.fn()
        .mockReturnValueOnce({ status: "running" }) // 1st check
        .mockReturnValueOnce({ status: "completed" }) // 2nd check
    };

    // Mock FlowManager.getInstance
    vi.spyOn(FlowManager, "getInstance").mockReturnValue(mockManager as any);

    const node = new SubFlowNode("sub", {
      flow_name: "child-flow"
    }, blackboard);

    // 1. Start
    let result = await node.execute("session");
    expect(result.name).toBe("running");
    expect(mockManager.start).toHaveBeenCalledWith("child-flow", "");

    // 2. Check (Running)
    result = await node.execute("session");
    expect(result.name).toBe("running");

    // 3. Check (Completed)
    result = await node.execute("session");
    expect(result.name).toBe("success");
  });
});
