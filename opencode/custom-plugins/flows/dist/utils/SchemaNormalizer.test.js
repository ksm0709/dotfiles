"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const vitest_1 = require("vitest");
const SchemaNormalizer_1 = require("./SchemaNormalizer");
(0, vitest_1.describe)("SchemaNormalizer", () => {
    (0, vitest_1.it)("should normalize a concise flow definition", () => {
        const concise = {
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
        const normalized = SchemaNormalizer_1.SchemaNormalizer.normalize(concise);
        (0, vitest_1.expect)(normalized.initial_state).toBe("step1");
        // Step 1: Agent
        const step1 = normalized.nodes.step1;
        (0, vitest_1.expect)(step1.type).toBe("agent");
        (0, vitest_1.expect)(step1.config.agent_type).toBe("general");
        (0, vitest_1.expect)(step1.config.prompt).toBe("Hello");
        (0, vitest_1.expect)(step1.routes?.success).toBe("step2");
        // Step 2: Command
        const step2 = normalized.nodes.step2;
        (0, vitest_1.expect)(step2.type).toBe("command");
        (0, vitest_1.expect)(step2.config.command).toBe("echo 'World'");
        (0, vitest_1.expect)(step2.routes?.success).toBe("step3");
        // Step 3: End
        const step3 = normalized.nodes.step3;
        (0, vitest_1.expect)(step3.type).toBe("end");
        (0, vitest_1.expect)(step3.config.message).toBe("Done");
    });
    (0, vitest_1.it)("should handle already canonical flow definition", () => {
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
        const normalized = SchemaNormalizer_1.SchemaNormalizer.normalize(canonical);
        (0, vitest_1.expect)(normalized.initial_state).toBe("start");
        (0, vitest_1.expect)(normalized.nodes.start.type).toBe("end");
    });
    (0, vitest_1.it)("should normalize tool nodes", () => {
        const concise = {
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
        const normalized = SchemaNormalizer_1.SchemaNormalizer.normalize(concise);
        (0, vitest_1.expect)(normalized.nodes.tool1.type).toBe("tool");
        (0, vitest_1.expect)(normalized.nodes.tool1.config.tool).toBe("read");
        (0, vitest_1.expect)(normalized.nodes.tool1.config.args.filePath).toBe("test.txt");
    });
    (0, vitest_1.it)("should normalize delay nodes", () => {
        const concise = {
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
        const normalized = SchemaNormalizer_1.SchemaNormalizer.normalize(concise);
        (0, vitest_1.expect)(normalized.nodes.wait1.type).toBe("delay");
        (0, vitest_1.expect)(normalized.nodes.wait1.config.duration).toBe(1000);
    });
});
//# sourceMappingURL=SchemaNormalizer.test.js.map