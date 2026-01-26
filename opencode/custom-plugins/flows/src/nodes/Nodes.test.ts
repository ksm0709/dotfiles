import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import { Blackboard } from "../manager/Blackboard";
import { MockClient } from "../test/MockClient";
import { RetryNode } from "./RetryNode";
import { SkillNode } from "./SkillNode";
import { ParallelNode } from "./ParallelNode";
import { ToolNode } from "./ToolNode";
import { DelayNode } from "./DelayNode";
import { CommandNode } from "./CommandNode";
import { LoopNode } from "./LoopNode";
import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";

// Mock glob module
vi.mock("glob", () => ({
  glob: vi.fn().mockResolvedValue(["test-tool-node.txt"])
}));

// Mock child_process
vi.mock("node:child_process", () => ({
  exec: (cmd: string, opts: any, cb: any) => {
    if (cmd.includes("fail_command")) {
      cb(new Error("Command failed"), { stdout: "", stderr: "error output" });
    } else if (cmd.includes("exit_1")) {
      const err: any = new Error("Command failed");
      err.code = 1;
      cb(err, { stdout: "", stderr: "exit code 1" });
    } else {
      cb(null, { stdout: "mock stdout", stderr: "" });
    }
    return { kill: () => {} };
  }
}));

describe("Nodes Unit Tests", () => {
  let blackboard: Blackboard;
  let mockClient: MockClient;
  let testInstanceId: string;

  beforeEach(async () => {
    testInstanceId = "test-nodes-" + Date.now();
    blackboard = new Blackboard(testInstanceId);
    await blackboard.load();
    mockClient = new MockClient();
    
    // Mock client tools
    mockClient.tools = {
      call: vi.fn().mockResolvedValue("tool result")
    };
  });

  afterEach(async () => {
    await blackboard.cleanup();
  });

  describe("RetryNode", () => {
    it("should retry until max retries", async () => {
      const node = new RetryNode("retry_logic", {
        target_node: "target",
        max_retries: 2,
        retry_delay: 10 // fast test
      }, blackboard);

      // 1st attempt (wait)
      let result = await node.execute("session");
      expect(result.name).toBe("running");
      
      // Wait for delay
      await new Promise(r => setTimeout(r, 20));
      
      // 1st attempt (retry)
      result = await node.execute("session");
      expect(result.name).toBe("retry");
      expect(blackboard.get("retry_logic_retry_count")).toBe(1);

      // 2nd attempt (wait)
      result = await node.execute("session");
      expect(result.name).toBe("running");
      
      // Wait for delay
      await new Promise(r => setTimeout(r, 20));
      
      // 2nd attempt (retry)
      result = await node.execute("session");
      expect(result.name).toBe("retry");
      expect(blackboard.get("retry_logic_retry_count")).toBe(2);

      // 3rd attempt (max exceeded)
      result = await node.execute("session");
      expect(result.name).toBe("max_retries_exceeded");
      expect(blackboard.get("retry_logic_retry_count")).toBe(0); // reset
    });
  });

  describe("SkillNode", () => {
    it("should call skill tool", async () => {
      const node = new SkillNode("skill_node", {
        skill: "research",
        args: { query: "test" }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      
      expect(result.name).toBe("success");
      expect(mockClient.tools.call).toHaveBeenCalledWith("skill", {
        name: "research",
        query: "test"
      });
    });
  });

  describe("ToolNode", () => {
    const testFile = "test-tool-node.txt";
    
    beforeEach(async () => {
      await fs.writeFile(testFile, "file content");
    });
    
    afterEach(async () => {
      try {
        await fs.unlink(testFile);
      } catch {}
    });

    it("should execute read tool", async () => {
      const node = new ToolNode("read_file", {
        tool: "read",
        args: { filePath: path.resolve(testFile) }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.output).toBe("file content");
    });

    it("should execute write tool", async () => {
      const node = new ToolNode("write_file", {
        tool: "write",
        args: { filePath: path.resolve(testFile), content: "new content" }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      expect(result.name).toBe("success");
      
      const content = await fs.readFile(testFile, "utf-8");
      expect(content).toBe("new content");
    });

    it("should execute glob tool", async () => {
      const node = new ToolNode("glob_files", {
        tool: "glob",
        args: { pattern: "*.txt" }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.output).toContain("test-tool-node.txt");
    });

    it("should execute grep tool", async () => {
      const node = new ToolNode("grep_files", {
        tool: "grep",
        args: { pattern: "content", path: "." }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.output).toContain("test-tool-node.txt");
      expect(result.data?.output).toContain("file content");
    });

    it("should execute bash tool", async () => {
      const node = new ToolNode("bash_cmd", {
        tool: "bash",
        args: { command: "echo 'bash'" }
      }, blackboard, mockClient);

      const result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.output).toContain("mock stdout");
    });
  });

  describe("CommandNode", () => {
    it("should execute command", async () => {
      const node = new CommandNode("cmd", {
        command: "echo 'hello'"
      }, blackboard, mockClient);

      const result = await node.execute("session");
      
      expect(result.name).toBe("running");
      
      await new Promise(r => setTimeout(r, 100));
      
      const finalResult = await node.execute("session");
      expect(finalResult.name).toBe("success");
      expect(finalResult.data?.output).toContain("mock stdout");
    });

    it("should handle command failure", async () => {
      const node = new CommandNode("cmd_fail", {
        command: "fail_command"
      }, blackboard, mockClient);

      let result = await node.execute("session");
      expect(result.name).toBe("running");
      
      await new Promise(r => setTimeout(r, 100));
      
      result = await node.execute("session");
      expect(result.name).toBe("failed");
      expect(result.message).toContain("Command failed");
    });

    it("should handle expect_exit_code", async () => {
      const node = new CommandNode("cmd_exit", {
        command: "exit_1",
        expect_exit_code: 1
      }, blackboard, mockClient);

      let result = await node.execute("session");
      expect(result.name).toBe("running");
      
      await new Promise(r => setTimeout(r, 100));
      
      result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.exitCode).toBe(1);
    });
  });

  describe("DelayNode", () => {
    it("should wait for duration", async () => {
      const node = new DelayNode("wait", { duration: 50 }, blackboard);

      // Start
      let result = await node.execute("session");
      expect(result.name).toBe("running");

      // Not enough time
      result = await node.execute("session");
      expect(result.name).toBe("running");

      // Wait
      await new Promise(r => setTimeout(r, 60));

      // Done
      result = await node.execute("session");
      expect(result.name).toBe("success");
    });
  });

  describe("ParallelNode", () => {
    it("should execute children in parallel", async () => {
      const node = new ParallelNode("parallel", {
        nodes: ["d1", "d2"],
        nodeConfigs: {
          d1: { type: "delay", config: { duration: 10 } },
          d2: { type: "delay", config: { duration: 10 } }
        }
      }, blackboard, mockClient);

      // Start (initializes children)
      let result = await node.execute("session");
      expect(result.name).toBe("running");

      // Wait for children to finish
      await new Promise(r => setTimeout(r, 20));

      // Finish
      result = await node.execute("session");
      expect(result.name).toBe("success");
      expect(result.data?.d1.name).toBe("success");
      expect(result.data?.d2.name).toBe("success");
    });

    it("should handle child failure", async () => {
      // Mock createNode to return a failing node
      // Since createNode is imported, we can't easily mock it here without module mocking.
      // But we can use a real node that fails, e.g. CommandNode with fail command.
      
      const node = new ParallelNode("parallel_fail", {
        nodes: ["c1"],
        nodeConfigs: {
          c1: { type: "command", config: { command: "fail_command" } }
        }
      }, blackboard, mockClient);

      // Start
      let result = await node.execute("session");
      expect(result.name).toBe("running");

      // Wait
      await new Promise(r => setTimeout(r, 100));

      // Finish
      result = await node.execute("session");
      expect(result.name).toBe("failed");
      expect(result.message).toContain("One or more parallel nodes failed");
    });
  });

  describe("LoopNode", () => {
    it("should loop until max iterations", async () => {
      const node = new LoopNode("loop", {
        body_node: "body",
        max_iterations: 2
      }, blackboard);

      // 1st iteration
      let result = await node.execute("session");
      expect(result.name).toBe("continue");
      expect(blackboard.get("loop_iteration")).toBe(1);

      // 2nd iteration
      result = await node.execute("session");
      expect(result.name).toBe("continue");
      expect(blackboard.get("loop_iteration")).toBe(2);

      // 3rd iteration (max reached)
      result = await node.execute("session");
      expect(result.name).toBe("max_reached");
      expect(blackboard.get("loop_iteration")).toBe(0);
    });

    it("should stop when while condition is false", async () => {
      await blackboard.set("should_continue", true);
      
      const node = new LoopNode("loop_while", {
        body_node: "body",
        while_condition: {
          field: "should_continue",
          operator: "eq",
          value: true,
          result: "continue"
        }
      }, blackboard);

      // 1st iteration (condition true)
      let result = await node.execute("session");
      expect(result.name).toBe("continue");

      // Change condition
      await blackboard.set("should_continue", false);

      // 2nd iteration (condition false)
      result = await node.execute("session");
      expect(result.name).toBe("success"); // Loop completed
    });
  });
});
