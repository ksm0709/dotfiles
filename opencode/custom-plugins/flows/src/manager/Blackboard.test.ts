import { describe, it, expect, beforeEach, afterEach } from "vitest";
import { Blackboard } from "./Blackboard";
import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";

describe("Blackboard", () => {
  const testInstanceId = "test-instance-" + Date.now();
  let blackboard: Blackboard;

  beforeEach(async () => {
    blackboard = new Blackboard(testInstanceId);
    await blackboard.load();
  });

  afterEach(async () => {
    await blackboard.cleanup();
  });

  it("should set and get values", async () => {
    await blackboard.set("key1", "value1");
    await blackboard.set("key2", 123);
    await blackboard.set("key3", { nested: "obj" });

    expect(blackboard.get("key1")).toBe("value1");
    expect(blackboard.get("key2")).toBe(123);
    expect(blackboard.get("key3")).toEqual({ nested: "obj" });
  });

  it("should support nested key access via getValue", async () => {
    await blackboard.set("user", {
      profile: {
        name: "Taeho",
        age: 30
      }
    });

    expect(blackboard.getValue("user.profile.name")).toBe("Taeho");
    expect(blackboard.getValue("user.profile.age")).toBe(30);
    expect(blackboard.getValue("user.profile.unknown")).toBeUndefined();
  });

  it("should resolve variables in strings", async () => {
    await blackboard.set("name", "World");
    await blackboard.set("count", 5);
    
    const resolved = blackboard.resolveVariables("Hello ${name}, count is ${count}");
    expect(resolved).toBe("Hello World, count is 5");
  });

  it("should resolve nested variables", async () => {
    await blackboard.set("config", { env: "prod" });
    
    const resolved = blackboard.resolveVariables("Environment: ${config.env}");
    expect(resolved).toBe("Environment: prod");
  });

  it("should save and retrieve node results", async () => {
    const result = {
      name: "success",
      message: "Task completed",
      data: { output: "result.txt" },
      duration: 100
    };

    await blackboard.saveNodeResult("step1", result);

    const retrieved = blackboard.getNodeResult("step1");
    expect(retrieved).toEqual(result);
    
    const entry = blackboard.getNodeResultEntry("step1");
    expect(entry?.executionCount).toBe(1);
    expect(entry?.timestamp).toBeDefined();
  });

  it("should resolve history variables", async () => {
    const result = {
      name: "success",
      message: "Analysis done",
      data: { score: 95 }
    };

    await blackboard.saveNodeResult("analyze", result);

    // ${history.nodeName} -> message
    expect(blackboard.resolveVariables("Result: ${history.analyze}")).toBe("Result: Analysis done");
    
    // ${history.nodeName.message} -> message
    expect(blackboard.resolveVariables("Msg: ${history.analyze.message}")).toBe("Msg: Analysis done");
    
    // ${history.nodeName.data.field} -> data field
    expect(blackboard.resolveVariables("Score: ${history.analyze.data.score}")).toBe("Score: 95");
  });

  it("should persist data to file", async () => {
    await blackboard.set("persistent", "data");
    
    // Create a new instance with same ID to simulate reload
    const newBlackboard = new Blackboard(testInstanceId);
    await newBlackboard.load();
    
    expect(newBlackboard.get("persistent")).toBe("data");
  });

  it("should delete values", async () => {
    await blackboard.set("key", "value");
    expect(blackboard.has("key")).toBe(true);
    
    await blackboard.delete("key");
    expect(blackboard.has("key")).toBe(false);
    expect(blackboard.get("key")).toBeUndefined();
  });

  it("should get all values", async () => {
    await blackboard.set("a", 1);
    await blackboard.set("b", 2);
    await blackboard.set("_instance_id", testInstanceId);
    
    const all = blackboard.getAll();
    expect(all.a).toBe(1);
    expect(all.b).toBe(2);
    expect(all._instance_id).toBe(testInstanceId);
  });
});
