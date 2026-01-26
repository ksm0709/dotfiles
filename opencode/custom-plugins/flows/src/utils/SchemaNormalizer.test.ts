import { describe, it, expect } from "vitest";
import { SchemaNormalizer } from "./SchemaNormalizer";
import { ConciseFlowDefinition } from "../types/schemas";

describe("SchemaNormalizer", () => {
  it("should normalize a concise flow definition", () => {
    const concise: ConciseFlowDefinition = {
      name: "test-flow",
      version: "1.0.0",
      start: "step1",
      nodes: {
        step1: {
          agent: "general",
          prompt: "Hello",
          on: {
            success: "step2",
          },
        },
        step2: {
          run: "echo 'World'",
          on: {
            success: "step3",
          },
        },
        step3: {
          end: true,
          message: "Done",
        },
      },
    };

    const normalized = SchemaNormalizer.normalize(concise);

    expect(normalized.initial_state).toBe("step1");
    
    // Step 1: Agent
    const step1 = normalized.nodes.step1;
    expect(step1.type).toBe("agent");
    expect(step1.config.agent_type).toBe("general");
    expect(step1.config.prompt).toBe("Hello");
    expect(step1.routes?.success).toBe("step2");

    // Step 2: Command
    const step2 = normalized.nodes.step2;
    expect(step2.type).toBe("command");
    expect(step2.config.command).toBe("echo 'World'");
    expect(step2.routes?.success).toBe("step3");

    // Step 3: End
    const step3 = normalized.nodes.step3;
    expect(step3.type).toBe("end");
    expect(step3.config.message).toBe("Done");
  });

  it("should handle already canonical flow definition", () => {
    const canonical = {
      name: "canonical-flow",
      version: "1.0.0",
      initial_state: "start",
      nodes: {
        start: {
          type: "end",
          config: { status: "success" },
        },
      },
    };

    const normalized = SchemaNormalizer.normalize(canonical);
    expect(normalized.initial_state).toBe("start");
    expect(normalized.nodes.start.type).toBe("end");
  });

  it("should normalize tool nodes", () => {
    const concise: ConciseFlowDefinition = {
      name: "tool-flow",
      version: "1.0.0",
      start: "tool1",
      nodes: {
        tool1: {
          tool: "read",
          args: { filePath: "test.txt" },
          on: { success: "end" }
        },
        end: { end: true }
      }
    };

    const normalized = SchemaNormalizer.normalize(concise);
    expect(normalized.nodes.tool1.type).toBe("tool");
    expect(normalized.nodes.tool1.config.tool).toBe("read");
    expect(normalized.nodes.tool1.config.args.filePath).toBe("test.txt");
  });

  it("should normalize delay nodes", () => {
    const concise: ConciseFlowDefinition = {
      name: "delay-flow",
      version: "1.0.0",
      start: "wait1",
      nodes: {
        wait1: {
          wait: 1000,
          on: { success: "end" }
        },
        end: { end: true }
      }
    };

    const normalized = SchemaNormalizer.normalize(concise);
    expect(normalized.nodes.wait1.type).toBe("delay");
    expect(normalized.nodes.wait1.config.duration).toBe(1000);
  });
});
