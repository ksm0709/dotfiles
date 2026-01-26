"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const vitest_1 = require("vitest");
const Blackboard_1 = require("./Blackboard");
(0, vitest_1.describe)("Blackboard", () => {
    const testInstanceId = "test-instance-" + Date.now();
    let blackboard;
    (0, vitest_1.beforeEach)(async () => {
        blackboard = new Blackboard_1.Blackboard(testInstanceId);
        await blackboard.load();
    });
    (0, vitest_1.afterEach)(async () => {
        await blackboard.cleanup();
    });
    (0, vitest_1.it)("should set and get values", async () => {
        await blackboard.set("key1", "value1");
        await blackboard.set("key2", 123);
        await blackboard.set("key3", { nested: "obj" });
        (0, vitest_1.expect)(blackboard.get("key1")).toBe("value1");
        (0, vitest_1.expect)(blackboard.get("key2")).toBe(123);
        (0, vitest_1.expect)(blackboard.get("key3")).toEqual({ nested: "obj" });
    });
    (0, vitest_1.it)("should support nested key access via getValue", async () => {
        await blackboard.set("user", {
            profile: {
                name: "Taeho",
                age: 30
            }
        });
        (0, vitest_1.expect)(blackboard.getValue("user.profile.name")).toBe("Taeho");
        (0, vitest_1.expect)(blackboard.getValue("user.profile.age")).toBe(30);
        (0, vitest_1.expect)(blackboard.getValue("user.profile.unknown")).toBeUndefined();
    });
    (0, vitest_1.it)("should resolve variables in strings", async () => {
        await blackboard.set("name", "World");
        await blackboard.set("count", 5);
        const resolved = blackboard.resolveVariables("Hello ${name}, count is ${count}");
        (0, vitest_1.expect)(resolved).toBe("Hello World, count is 5");
    });
    (0, vitest_1.it)("should resolve nested variables", async () => {
        await blackboard.set("config", { env: "prod" });
        const resolved = blackboard.resolveVariables("Environment: ${config.env}");
        (0, vitest_1.expect)(resolved).toBe("Environment: prod");
    });
    (0, vitest_1.it)("should save and retrieve node results", async () => {
        const result = {
            name: "success",
            message: "Task completed",
            data: { output: "result.txt" },
            duration: 100
        };
        await blackboard.saveNodeResult("step1", result);
        const retrieved = blackboard.getNodeResult("step1");
        (0, vitest_1.expect)(retrieved).toEqual(result);
        const entry = blackboard.getNodeResultEntry("step1");
        (0, vitest_1.expect)(entry?.executionCount).toBe(1);
        (0, vitest_1.expect)(entry?.timestamp).toBeDefined();
    });
    (0, vitest_1.it)("should resolve history variables", async () => {
        const result = {
            name: "success",
            message: "Analysis done",
            data: { score: 95 }
        };
        await blackboard.saveNodeResult("analyze", result);
        // ${history.nodeName} -> message
        (0, vitest_1.expect)(blackboard.resolveVariables("Result: ${history.analyze}")).toBe("Result: Analysis done");
        // ${history.nodeName.message} -> message
        (0, vitest_1.expect)(blackboard.resolveVariables("Msg: ${history.analyze.message}")).toBe("Msg: Analysis done");
        // ${history.nodeName.data.field} -> data field
        (0, vitest_1.expect)(blackboard.resolveVariables("Score: ${history.analyze.data.score}")).toBe("Score: 95");
    });
    (0, vitest_1.it)("should persist data to file", async () => {
        await blackboard.set("persistent", "data");
        // Create a new instance with same ID to simulate reload
        const newBlackboard = new Blackboard_1.Blackboard(testInstanceId);
        await newBlackboard.load();
        (0, vitest_1.expect)(newBlackboard.get("persistent")).toBe("data");
    });
});
//# sourceMappingURL=Blackboard.test.js.map