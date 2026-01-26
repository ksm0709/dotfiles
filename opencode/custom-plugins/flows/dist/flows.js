// opencode/custom-plugins/flows/src/manager/FlowManager.ts
import * as fs2 from "node:fs/promises";
import * as path2 from "node:path";
import * as os2 from "node:os";

// opencode/custom-plugins/flows/src/manager/Blackboard.ts
import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";
var Blackboard = class {
  cache = /* @__PURE__ */ new Map();
  dirty = false;
  filePath;
  saveDebounceTimer = null;
  constructor(instanceId) {
    const dataDir = path.join(
      os.homedir(),
      ".config/opencode/data/flows/instances"
    );
    this.filePath = path.join(dataDir, `${instanceId}.json`);
  }
  /**
   * 저장된 상태 로드 (있는 경우)
   */
  async load() {
    try {
      const content = await fs.readFile(this.filePath, "utf-8");
      const data = JSON.parse(content);
      this.cache = new Map(Object.entries(data));
    } catch {
      this.cache.set("_results", {});
      this.cache.set("_execution_order", []);
    }
  }
  /**
   * 현재 상태를 파일로 저장
   */
  async save() {
    if (!this.dirty) return;
    try {
      const data = Object.fromEntries(this.cache);
      await fs.mkdir(path.dirname(this.filePath), { recursive: true });
      await fs.writeFile(this.filePath, JSON.stringify(data, null, 2));
      this.dirty = false;
    } catch (error) {
      console.error(`[Blackboard] Save failed: ${error}`);
    }
  }
  /**
   * 디바운스된 저장 (연속 호출 최적화)
   */
  debouncedSave() {
    if (this.saveDebounceTimer) {
      clearTimeout(this.saveDebounceTimer);
    }
    this.saveDebounceTimer = setTimeout(() => {
      this.save();
    }, 100);
  }
  /**
   * 값 조회
   */
  get(key) {
    return this.cache.get(key);
  }
  /**
   * 값 조회 (중첩 키 지원)
   * 예: "history.analyze.data.score"
   */
  getValue(path3) {
    if (this.cache.has(path3)) {
      return this.cache.get(path3);
    }
    const parts = path3.split(".");
    if (parts[0] === "history" && parts.length >= 2) {
      const nodeName = parts[1];
      const result = this.getNodeResult(nodeName);
      if (!result) return void 0;
      if (parts.length === 2) {
        return result.message;
      }
      let value = result;
      for (let i = 2; i < parts.length; i++) {
        if (value && typeof value === "object") {
          value = value[parts[i]];
        } else {
          return void 0;
        }
      }
      return value;
    }
    let current = this.getAll();
    for (const part of parts) {
      if (current && typeof current === "object") {
        current = current[part];
      } else {
        return void 0;
      }
    }
    return current;
  }
  /**
   * 값 설정
   */
  async set(key, value) {
    this.cache.set(key, value);
    this.dirty = true;
    await this.save();
  }
  /**
   * 값 존재 여부 확인
   */
  has(key) {
    return this.cache.has(key);
  }
  /**
   * 값 삭제
   */
  async delete(key) {
    const result = this.cache.delete(key);
    if (result) {
      this.dirty = true;
      await this.save();
    }
    return result;
  }
  /**
   * 전체 데이터 반환
   */
  getAll() {
    return Object.fromEntries(this.cache);
  }
  /**
   * 노드 결과 저장 (개선된 방식)
   * 
   * - 노드 이름을 키로 하여 _results에 저장
   * - 같은 노드가 여러 번 실행되면 덮어쓰고 executionCount 증가
   * - 실행 순서는 _execution_order에 기록
   */
  async saveNodeResult(nodeName, result) {
    const results = this.get("_results") || {};
    const existingCount = results[nodeName]?.executionCount || 0;
    results[nodeName] = {
      result,
      timestamp: (/* @__PURE__ */ new Date()).toISOString(),
      executionCount: existingCount + 1
    };
    await this.set("_results", results);
    const executionOrder = this.get("_execution_order") || [];
    executionOrder.push(nodeName);
    await this.set("_execution_order", executionOrder);
  }
  /**
   * 특정 노드의 결과 조회
   */
  getNodeResult(nodeName) {
    const results = this.get("_results") || {};
    return results[nodeName]?.result;
  }
  /**
   * 특정 노드의 결과 엔트리 전체 조회 (메타데이터 포함)
   */
  getNodeResultEntry(nodeName) {
    const results = this.get("_results") || {};
    return results[nodeName];
  }
  /**
   * 모든 노드 결과 조회
   */
  getAllNodeResults() {
    return this.get("_results") || {};
  }
  /**
   * 실행 순서 조회
   */
  getExecutionOrder() {
    return this.get("_execution_order") || [];
  }
  /**
   * 변수 해석: ${key} 형태를 실제 값으로 치환
   * 
   * 지원 패턴:
   * - ${key}: blackboard의 직접 키
   * - ${prompt}: 초기 프롬프트
   * - ${history.nodeName}: 특정 노드의 결과 메시지 (단축)
   * - ${history.nodeName.message}: 특정 노드의 결과 메시지 (명시적)
   * - ${history.nodeName.data.field}: 특정 노드 결과의 데이터 필드
   * - ${_results.nodeName.result.message}: 직접 접근
   */
  resolveVariables(template) {
    return template.replace(/\$\{([^}]+)\}/g, (match, key) => {
      const parts = key.split(".");
      if (parts[0] === "history" && parts.length >= 2) {
        const nodeName = parts[1];
        const result = this.getNodeResult(nodeName);
        if (!result) {
          return match;
        }
        if (parts.length === 2) {
          return result.message;
        }
        let value2 = result;
        for (let i = 2; i < parts.length; i++) {
          if (value2 && typeof value2 === "object") {
            value2 = value2[parts[i]];
          } else {
            return match;
          }
        }
        return value2 !== void 0 ? String(value2) : match;
      }
      const value = this.get(key);
      if (value !== void 0) {
        if (typeof value === "object") {
          return JSON.stringify(value);
        }
        return String(value);
      }
      let current = this.getAll();
      for (const part of parts) {
        if (current && typeof current === "object") {
          current = current[part];
        } else {
          return match;
        }
      }
      if (current !== void 0) {
        if (typeof current === "object") {
          return JSON.stringify(current);
        }
        return String(current);
      }
      return match;
    });
  }
  /**
   * 객체 내 모든 문자열 값의 변수 해석
   */
  resolveObjectVariables(obj) {
    const result = {};
    for (const [key, value] of Object.entries(obj)) {
      if (typeof value === "string") {
        result[key] = this.resolveVariables(value);
      } else if (typeof value === "object" && value !== null) {
        result[key] = this.resolveObjectVariables(value);
      } else {
        result[key] = value;
      }
    }
    return result;
  }
  /**
   * 히스토리 컨텍스트 빌드 (에이전트 프롬프트용)
   * 실행 순서대로 모든 노드 결과를 포맷팅
   */
  buildHistoryContext(maxLength = 1e4) {
    const results = this.getAllNodeResults();
    const order = this.getExecutionOrder();
    if (order.length === 0) {
      return "";
    }
    const seen = /* @__PURE__ */ new Set();
    const uniqueOrder = order.filter((name) => {
      if (seen.has(name)) return false;
      seen.add(name);
      return true;
    });
    const lines = ["[\uC774\uC804 \uB178\uB4DC \uC2E4\uD589 \uD788\uC2A4\uD1A0\uB9AC]"];
    let currentLength = lines[0].length;
    for (const nodeName of uniqueOrder) {
      const entry = results[nodeName];
      if (!entry) continue;
      const statusIcon = entry.result.name === "success" ? "\u2713" : entry.result.name === "failed" ? "\u2717" : "\u2192";
      let message = entry.result.message;
      if (message.length > 500) {
        message = message.substring(0, 497) + "...";
      }
      const line = `
[${statusIcon} ${nodeName}] (${entry.result.name}):
${message}`;
      if (currentLength + line.length > maxLength) {
        lines.push("\n... (\uC774\uC804 \uD788\uC2A4\uD1A0\uB9AC \uC0DD\uB7B5)");
        break;
      }
      lines.push(line);
      currentLength += line.length;
    }
    return lines.join("\n");
  }
  /**
   * 인스턴스 데이터 삭제 (종료 시)
   */
  async cleanup() {
    try {
      await fs.unlink(this.filePath);
    } catch {
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/BaseNode.ts
var BaseNode = class {
  constructor(nodeName, config, blackboard) {
    this.nodeName = nodeName;
    this.config = config;
    this.blackboard = blackboard;
  }
  /**
   * 성공 결과 생성 헬퍼
   */
  success(message, data) {
    return {
      name: "success",
      message,
      data,
      duration: this.getDuration()
    };
  }
  /**
   * 실패 결과 생성 헬퍼
   */
  failed(message, data) {
    return {
      name: "failed",
      message,
      data,
      duration: this.getDuration()
    };
  }
  /**
   * 실행 중 결과 생성 헬퍼
   */
  running() {
    return {
      name: "running",
      message: "Node is still executing..."
    };
  }
  /**
   * 커스텀 결과 생성 헬퍼
   */
  result(name, message, data) {
    return {
      name,
      message,
      data,
      duration: this.getDuration()
    };
  }
  /**
   * 변수 해석 헬퍼
   */
  resolveVariables(template) {
    return this.blackboard.resolveVariables(template);
  }
  /**
   * 설정값 변수 해석 헬퍼
   */
  resolveConfig(config) {
    return this.blackboard.resolveObjectVariables(config);
  }
  // 실행 시간 측정용
  startTime;
  startTimer() {
    this.startTime = Date.now();
  }
  getDuration() {
    if (this.startTime) {
      return Date.now() - this.startTime;
    }
    return void 0;
  }
};

// opencode/custom-plugins/flows/src/nodes/AgentNode.ts
var AgentNode = class extends BaseNode {
  pendingPromise;
  storedResult;
  client;
  // OpenCode client
  constructor(nodeName, config, blackboard, client) {
    super(nodeName, config, blackboard);
    this.client = client;
  }
  async execute(sessionId) {
    console.log(`[AgentNode ${this.nodeName}] Execute`);
    if (this.pendingPromise) {
      if (this.storedResult) {
        const r = this.storedResult;
        this.storedResult = void 0;
        this.pendingPromise = void 0;
        return r;
      }
      return this.running();
    }
    this.startTimer();
    const resolvedPrompt = this.resolveVariables(this.config.prompt);
    const contextPrompt = this.buildContextPrompt(resolvedPrompt);
    this.pendingPromise = this.executeAgent(sessionId, contextPrompt);
    return this.running();
  }
  /**
   * 히스토리 기반 컨텍스트 프롬프트 구성
   */
  buildContextPrompt(basePrompt) {
    const historyContext = this.blackboard.buildHistoryContext?.(5e3) || "";
    if (!historyContext) {
      return basePrompt;
    }
    return `${historyContext}

---

${basePrompt}`;
  }
  /**
   * 에이전트 실행
   */
  async executeAgent(sessionId, prompt) {
    try {
      const resultGuide = this.buildResultGuide();
      const fullPrompt = `${prompt}

${resultGuide}`;
      await this.client.session.prompt(sessionId, fullPrompt);
      const messages = await this.client.session.messages(sessionId);
      const lastMessage = messages[messages.length - 1];
      const response = lastMessage?.parts.find((p) => p.type === "text")?.text || "";
      this.storedResult = await this.parseResult(response);
    } catch (error) {
      this.storedResult = this.failed(`Agent execution failed: ${error}`);
    }
  }
  /**
   * 결과 가이드 문자열 생성
   * 에이전트가 어떤 결과를 반환할지 선택하도록 안내
   * + 데이터 추출 문법 안내
   */
  buildResultGuide() {
    const results = this.config.results || {};
    let resultOptions;
    if (Array.isArray(results)) {
      resultOptions = results.map((r) => `- [RESULT:${r.name}] ${r.description}`).join("\n");
    } else {
      resultOptions = Object.entries(results).map(([name, desc]) => `- [RESULT:${name}] ${desc}`).join("\n");
    }
    if (!resultOptions) {
      resultOptions = "- [RESULT:success] \uC791\uC5C5 \uC131\uACF5\n- [RESULT:failed] \uC791\uC5C5 \uC2E4\uD328";
    }
    return `
---
[FLOW RESULT INSTRUCTION]
\uC791\uC5C5 \uC644\uB8CC \uD6C4, \uC544\uB798 \uACB0\uACFC \uC911 \uAC00\uC7A5 \uC801\uD569\uD55C \uAC83\uC744 \uB9C8\uC9C0\uB9C9\uC5D0 \uBC18\uB4DC\uC2DC \uCD9C\uB825\uD558\uC138\uC694:
${resultOptions}

\uD615\uC2DD \uC608\uC2DC: [RESULT:success] \uC791\uC5C5\uC774 \uC131\uACF5\uC801\uC73C\uB85C \uC644\uB8CC\uB418\uC5C8\uC2B5\uB2C8\uB2E4.

[DATA EXTRACTION - \uC120\uD0DD\uC0AC\uD56D]
\uD6C4\uC18D \uB178\uB4DC\uC5D0 \uB370\uC774\uD130\uB97C \uC804\uB2EC\uD558\uB824\uBA74 \uB2E4\uC74C \uD615\uC2DD\uC744 \uC0AC\uC6A9\uD558\uC138\uC694:
[DATA:key=value]

\uC608\uC2DC:
[DATA:file_path=/src/utils/helper.ts]
[DATA:error_count=3]
---`;
  }
  /**
   * 응답에서 결과 및 데이터 파싱
   */
  async parseResult(response) {
    const extractedData = await this.extractData(response);
    const match = response.match(/\[RESULT:(\w+)\]/);
    if (match) {
      const resultName = match[1];
      const validResults = this.getValidResultNames();
      if (validResults.includes(resultName) || ["success", "failed"].includes(resultName)) {
        return this.result(resultName, response, extractedData);
      }
    }
    if (response.toLowerCase().includes("error") || response.toLowerCase().includes("failed") || response.toLowerCase().includes("\uC2E4\uD328")) {
      return this.failed(response, extractedData);
    }
    return this.success(response, extractedData);
  }
  /**
   * 응답에서 [DATA:key=value] 패턴 추출
   */
  async extractData(response) {
    const data = {};
    const dataPattern = /\[DATA:(\w+)=([^\]]+)\]/g;
    let match;
    while ((match = dataPattern.exec(response)) !== null) {
      const key = match[1];
      let value = match[2];
      if (/^\d+$/.test(value)) {
        value = parseInt(value, 10);
      } else if (/^\d+\.\d+$/.test(value)) {
        value = parseFloat(value);
      } else if (value === "true") {
        value = true;
      } else if (value === "false") {
        value = false;
      }
      data[key] = value;
    }
    return data;
  }
  /**
   * 유효한 결과 이름 목록 조회
   */
  getValidResultNames() {
    const results = this.config.results || {};
    if (Array.isArray(results)) {
      return results.map((r) => r.name);
    } else {
      return Object.keys(results);
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/ToolNode.ts
var ToolNode = class extends BaseNode {
  client;
  constructor(nodeName, config, blackboard, client) {
    super(nodeName, config, blackboard);
    this.client = client;
  }
  async execute(_sessionId) {
    this.startTimer();
    try {
      const toolName = this.config.tool;
      const args = this.resolveArgs(this.config.args || {});
      let result;
      switch (toolName) {
        case "read":
          result = await this.executeRead(args);
          break;
        case "write":
          result = await this.executeWrite(args);
          break;
        case "glob":
          result = await this.executeGlob(args);
          break;
        case "grep":
          result = await this.executeGrep(args);
          break;
        case "bash":
          result = await this.executeBash(args);
          break;
        default:
          throw new Error(`Tool not supported: ${toolName}`);
      }
      await this.blackboard.set(`${this.nodeName}_output`, result);
      return this.success(result, { output: result });
    } catch (error) {
      return this.failed(`Tool execution failed: ${error}`);
    }
  }
  /**
   * 인자 내 변수 해석
   */
  resolveArgs(args) {
    return this.resolveConfig(args);
  }
  // 각 도구별 실행 메서드 (client API에 따라 조정 필요)
  async executeRead(args) {
    const fs3 = await import("node:fs/promises");
    return await fs3.readFile(args.filePath, "utf-8");
  }
  async executeWrite(args) {
    const fs3 = await import("node:fs/promises");
    await fs3.writeFile(args.filePath, args.content, "utf-8");
    return `File written: ${args.filePath}`;
  }
  async executeGlob(args) {
    const { glob } = await import("glob");
    const files = await glob(args.pattern, { cwd: args.path || process.cwd() });
    return files.join("\n");
  }
  async executeGrep(args) {
    const fs3 = await import("node:fs/promises");
    const path3 = await import("node:path");
    const { glob } = await import("glob");
    const pattern = new RegExp(args.pattern);
    const files = await glob(args.include || "**/*", { cwd: args.path || process.cwd() });
    const results = [];
    for (const file of files.slice(0, 100)) {
      try {
        const content = await fs3.readFile(path3.join(args.path || process.cwd(), file), "utf-8");
        const lines = content.split("\n");
        lines.forEach((line, i) => {
          if (pattern.test(line)) {
            results.push(`${file}:${i + 1}: ${line}`);
          }
        });
      } catch {
      }
    }
    return results.join("\n");
  }
  async executeBash(args) {
    const { exec: exec2 } = await import("node:child_process");
    const { promisify: promisify2 } = await import("node:util");
    const execAsync2 = promisify2(exec2);
    const { stdout, stderr } = await execAsync2(args.command, {
      cwd: args.workdir || process.cwd(),
      timeout: args.timeout || 6e4
    });
    return stdout || stderr;
  }
};

// opencode/custom-plugins/flows/src/nodes/CommandNode.ts
import { exec } from "node:child_process";
import { promisify } from "node:util";
var execAsync = promisify(exec);
var CommandNode = class extends BaseNode {
  constructor(nodeName, config, blackboard, client) {
    super(nodeName, config, blackboard);
    this.client = client;
  }
  pendingPromise;
  storedResult;
  async execute(_sessionId) {
    if (this.pendingPromise) {
      if (this.storedResult) {
        const r = this.storedResult;
        this.storedResult = void 0;
        this.pendingPromise = void 0;
        return r;
      }
      return this.running();
    }
    this.startTimer();
    const command = this.resolveVariables(this.config.command);
    const workdir = this.config.workdir ? this.resolveVariables(this.config.workdir) : process.cwd();
    this.pendingPromise = this.executeCommand(command, workdir);
    return this.running();
  }
  async executeCommand(command, workdir) {
    try {
      const { stdout, stderr } = await execAsync(command, {
        cwd: workdir,
        timeout: 6e4,
        // 1분 타임아웃
        maxBuffer: 10 * 1024 * 1024
        // 10MB
      });
      const output = stdout || stderr;
      await this.blackboard.set(`${this.nodeName}_output`, output);
      await this.blackboard.set(`${this.nodeName}_exit_code`, 0);
      if (this.config.expect_exit_code !== void 0) {
        if (this.config.expect_exit_code !== 0) {
          this.storedResult = this.failed(
            `Expected exit code ${this.config.expect_exit_code}, got 0

Output:
${output}`
          );
          return;
        }
      }
      this.storedResult = this.success(output, {
        output,
        exitCode: 0
      });
    } catch (error) {
      const exitCode = error.code || 1;
      const output = error.stdout || error.stderr || error.message;
      await this.blackboard.set(`${this.nodeName}_output`, output);
      await this.blackboard.set(`${this.nodeName}_exit_code`, exitCode);
      if (this.config.expect_exit_code !== void 0) {
        if (this.config.expect_exit_code === exitCode) {
          this.storedResult = this.success(output, {
            output,
            exitCode
          });
          return;
        }
      }
      this.storedResult = this.failed(
        `Command failed with exit code ${exitCode}

Output:
${output}`,
        { output, exitCode }
      );
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/ConditionalNode.ts
var ConditionalNode = class extends BaseNode {
  async execute(_sessionId) {
    this.startTimer();
    const conditions = this.config.conditions || [];
    for (const condition of conditions) {
      const value = this.blackboard.getValue(condition.field);
      if (this.evaluateCondition(value, condition.operator, condition.value)) {
        return this.result(
          condition.result,
          `Condition matched: ${condition.field} ${condition.operator} ${condition.value}`
        );
      }
    }
    const defaultResult = this.config.default || "failed";
    return this.result(
      defaultResult,
      "No condition matched, using default"
    );
  }
  /**
   * 조건 평가
   */
  evaluateCondition(value, operator, expected) {
    switch (operator) {
      case "eq":
        return value === expected;
      case "ne":
        return value !== expected;
      case "gt":
        return typeof value === "number" && value > expected;
      case "lt":
        return typeof value === "number" && value < expected;
      case "contains":
        if (typeof value === "string") {
          return value.includes(expected);
        }
        if (Array.isArray(value)) {
          return value.includes(expected);
        }
        return false;
      case "exists":
        return value !== void 0 && value !== null;
      default:
        return false;
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/EndNode.ts
var EndNode = class extends BaseNode {
  async execute(_sessionId) {
    const status = this.config.status || "success";
    const message = this.config.message ? this.resolveVariables(this.config.message) : `Flow completed with status: ${status}`;
    await this.blackboard.set("_final_status", status);
    await this.blackboard.set("_final_message", message);
    await this.blackboard.set("_completed_at", (/* @__PURE__ */ new Date()).toISOString());
    return this.result(status, message);
  }
};

// opencode/custom-plugins/flows/src/nodes/DelayNode.ts
var DelayNode = class extends BaseNode {
  duration = 0;
  async execute(_sessionId) {
    if (!this.startTime) {
      this.startTimer();
      this.duration = this.config.duration || 1e3;
    }
    const elapsed = Date.now() - (this.startTime || 0);
    if (elapsed >= this.duration) {
      this.startTime = void 0;
      return this.success(`Delayed for ${this.duration}ms`);
    }
    return this.running();
  }
};

// opencode/custom-plugins/flows/src/nodes/LoopNode.ts
var LoopNode = class extends BaseNode {
  async execute(_sessionId) {
    this.startTimer();
    const maxIterations = this.config.max_iterations || 10;
    const iteration = this.blackboard.get(`${this.nodeName}_iteration`) || 0;
    if (this.config.while_condition) {
      const shouldContinue = this.evaluateCondition(this.config.while_condition);
      if (!shouldContinue) {
        await this.blackboard.set(`${this.nodeName}_iteration`, 0);
        return this.success(`Loop completed after ${iteration} iterations`);
      }
    }
    if (iteration >= maxIterations) {
      await this.blackboard.set(`${this.nodeName}_iteration`, 0);
      return this.result(
        "max_reached",
        `Max iterations (${maxIterations}) reached`
      );
    }
    await this.blackboard.set(`${this.nodeName}_iteration`, iteration + 1);
    return this.result(
      "continue",
      `Iteration ${iteration + 1} of max ${maxIterations}`
    );
  }
  /**
   * 조건 평가
   */
  evaluateCondition(condition) {
    const value = this.blackboard.getValue(condition.field);
    switch (condition.operator) {
      case "eq":
        return value === condition.value;
      case "ne":
        return value !== condition.value;
      case "gt":
        return typeof value === "number" && value > condition.value;
      case "lt":
        return typeof value === "number" && value < condition.value;
      case "contains":
        return typeof value === "string" && value.includes(condition.value);
      case "exists":
        return value !== void 0 && value !== null;
      default:
        return false;
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/RetryNode.ts
var RetryNode = class extends BaseNode {
  startTime;
  isWaiting = false;
  async execute(_sessionId) {
    const targetNode = this.config.target_node;
    const maxRetries = this.config.max_retries || 3;
    const retryDelay = this.config.retry_delay || 1e3;
    const retryKey = `${this.nodeName}_retry_count`;
    const currentRetries = this.blackboard.get(retryKey) || 0;
    if (this.isWaiting) {
      const elapsed = Date.now() - (this.startTime || 0);
      if (elapsed >= retryDelay) {
        this.isWaiting = false;
        this.startTime = void 0;
        await this.blackboard.set(retryKey, currentRetries + 1);
        return this.result("retry", `Retrying ${targetNode} (${currentRetries + 1}/${maxRetries})`);
      }
      return this.running();
    }
    if (currentRetries >= maxRetries) {
      await this.blackboard.set(retryKey, 0);
      return this.result("max_retries_exceeded", `Max retries (${maxRetries}) exceeded for ${targetNode}`);
    }
    this.isWaiting = true;
    this.startTimer();
    return this.running();
  }
};

// opencode/custom-plugins/flows/src/nodes/SkillNode.ts
var SkillNode = class extends BaseNode {
  client;
  constructor(nodeName, config, blackboard, client) {
    super(nodeName, config, blackboard);
    this.client = client;
  }
  async execute(_sessionId) {
    this.startTimer();
    const skillName = this.resolveVariables(this.config.skill);
    const args = this.config.args ? this.resolveConfig(this.config.args) : {};
    try {
      const result = await this.client.tools.call("skill", {
        name: skillName,
        ...args
      });
      return this.success(`Skill '${skillName}' executed successfully`, { result });
    } catch (error) {
      return this.failed(`Skill '${skillName}' execution failed: ${error.message}`);
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/ParallelNode.ts
var ParallelNode = class extends BaseNode {
  client;
  childNodes = /* @__PURE__ */ new Map();
  childResults = /* @__PURE__ */ new Map();
  initialized = false;
  constructor(nodeName, config, blackboard, client) {
    super(nodeName, config, blackboard);
    this.client = client;
  }
  async execute(sessionId) {
    if (!this.initialized) {
      this.startTimer();
      this.initializeChildren();
      this.initialized = true;
    }
    const targetNodes = this.config.nodes || [];
    let allCompleted = true;
    let hasFailure = false;
    for (const childName of targetNodes) {
      if (this.childResults.has(childName)) {
        continue;
      }
      const childNode = this.childNodes.get(childName);
      if (!childNode) {
        this.childResults.set(childName, {
          name: "failed",
          message: `Child node instance not found: ${childName}`
        });
        hasFailure = true;
        continue;
      }
      try {
        const result = await childNode.execute(sessionId);
        if (result.name === "running") {
          allCompleted = false;
        } else {
          this.childResults.set(childName, result);
          await this.blackboard.saveNodeResult(childName, result);
          if (result.name === "failed") {
            hasFailure = true;
          }
        }
      } catch (error) {
        this.childResults.set(childName, {
          name: "failed",
          message: `Child node execution error: ${error.message}`
        });
        hasFailure = true;
      }
    }
    if (!allCompleted) {
      return this.running();
    }
    const resultsObj = Object.fromEntries(this.childResults);
    if (hasFailure) {
      return this.failed("One or more parallel nodes failed", resultsObj);
    }
    return this.success("All parallel nodes completed successfully", resultsObj);
  }
  initializeChildren() {
    const targetNodes = this.config.nodes || [];
    const nodeConfigs = this.config.nodeConfigs || {};
    for (const childName of targetNodes) {
      const childDef = nodeConfigs[childName];
      if (!childDef) {
        console.warn(`[ParallelNode] No config found for child node: ${childName}`);
        continue;
      }
      const nodeInstance = createNode(
        childName,
        childDef,
        // NodeDefinition
        this.blackboard,
        this.client
      );
      this.childNodes.set(childName, nodeInstance);
    }
  }
};

// opencode/custom-plugins/flows/src/nodes/SubFlowNode.ts
var SubFlowNode = class extends BaseNode {
  childInstanceId;
  async execute(_sessionId) {
    const flowName = this.resolveVariables(this.config.flow_name);
    const manager = FlowManager.getInstance();
    if (!this.childInstanceId) {
      this.startTimer();
      let prompt = "";
      if (this.config.input_mapping && this.config.input_mapping.prompt) {
        prompt = this.resolveVariables(this.config.input_mapping.prompt);
      }
      try {
        this.childInstanceId = await manager.start(flowName, prompt);
        await this.blackboard.set(`${this.nodeName}_child_id`, this.childInstanceId);
      } catch (error) {
        return this.failed(`Failed to start sub-flow '${flowName}': ${error.message}`);
      }
      return this.running();
    }
    const status = manager.getInstanceStatus(this.childInstanceId);
    if (!status) {
      return this.failed(`Sub-flow instance ${this.childInstanceId} not found`);
    }
    if (status.status === "completed") {
      return this.success(`Sub-flow '${flowName}' completed`);
    }
    if (status.status === "failed") {
      return this.failed(`Sub-flow '${flowName}' failed`);
    }
    return this.running();
  }
};

// opencode/custom-plugins/flows/src/nodes/NodeFactory.ts
function createNode(nodeName, nodeDef, blackboard, client) {
  const config = nodeDef.config || {};
  switch (nodeDef.type) {
    case "agent":
      return new AgentNode(nodeName, config, blackboard, client);
    case "tool":
      return new ToolNode(nodeName, config, blackboard, client);
    case "command":
      return new CommandNode(nodeName, config, blackboard, client);
    case "conditional":
      return new ConditionalNode(nodeName, config, blackboard);
    case "delay":
      return new DelayNode(nodeName, config, blackboard);
    case "loop":
      return new LoopNode(nodeName, config, blackboard);
    case "end":
      return new EndNode(nodeName, config, blackboard);
    case "retry":
      return new RetryNode(nodeName, config, blackboard);
    case "skill":
      return new SkillNode(nodeName, config, blackboard, client);
    case "parallel":
      return new ParallelNode(nodeName, config, blackboard, client);
    case "sub-flow":
      return new SubFlowNode(nodeName, config, blackboard);
    default:
      throw new Error(`Unknown node type: ${nodeDef.type}`);
  }
}

// opencode/custom-plugins/flows/src/manager/FlowInstance.ts
var FlowInstance = class {
  constructor(instanceId, flowDef, initialPrompt, client) {
    this.instanceId = instanceId;
    this.flowDef = flowDef;
    this.initialPrompt = initialPrompt;
    this.client = client;
    this.blackboard = new Blackboard(instanceId);
    this.currentNode = flowDef.initial_state;
    this.startTime = Date.now();
  }
  status = "initializing";
  blackboard;
  currentNode;
  retryCount = 0;
  stepCount = 0;
  currentNodeInstance = null;
  sessionId;
  startTime;
  /**
   * 인스턴스 초기화
   */
  async initialize() {
    await this.blackboard.load();
    if (!this.blackboard.has("_instance_id")) {
      await this.blackboard.set("prompt", this.initialPrompt || "");
      await this.blackboard.set("_instance_id", this.instanceId);
      await this.blackboard.set("_flow_name", this.flowDef.name);
      await this.blackboard.set("_current_state", this.currentNode);
      await this.blackboard.set("_started_at", (/* @__PURE__ */ new Date()).toISOString());
      await this.blackboard.set("_history", []);
      const config = this.flowDef.config;
      await this.blackboard.set("_config_history_depth", config?.history_depth ?? 5);
      await this.blackboard.set("_config_timeout", config?.timeout ?? 3e5);
      await this.blackboard.set("_config_max_retries", config?.max_retries ?? 3);
      await this.blackboard.set("_config_max_steps", config?.max_steps ?? 100);
    } else {
      this.currentNode = this.blackboard.get("_current_state") || this.currentNode;
    }
    if (!this.blackboard.has("_session_id")) {
      const session = await this.client.session.create();
      this.sessionId = session.id;
      await this.blackboard.set("_session_id", session.id);
    } else {
      this.sessionId = this.blackboard.get("_session_id");
    }
    this.status = "running";
  }
  /**
   * Tick - 매니저에 의해 주기적으로 호출됨
   */
  async tick() {
    if (this.status !== "running") return;
    console.log(`[FlowInstance] Tick: ${this.currentNode}`);
    const timeout = this.blackboard.get("_config_timeout") || 3e5;
    if (Date.now() - this.startTime > timeout) {
      this.status = "failed";
      await this.blackboard.set("error", `Flow timed out after ${timeout}ms`);
      return;
    }
    const maxSteps = this.blackboard.get("_config_max_steps") || 100;
    if (this.stepCount >= maxSteps) {
      this.status = "failed";
      await this.blackboard.set("error", `Max steps (${maxSteps}) exceeded`);
      return;
    }
    const nodeDef = this.flowDef.nodes[this.currentNode];
    if (!nodeDef) {
      this.status = "failed";
      await this.blackboard.set("error", `Node not found: ${this.currentNode}`);
      return;
    }
    try {
      if (!this.currentNodeInstance) {
        this.currentNodeInstance = createNode(
          this.currentNode,
          nodeDef,
          this.blackboard,
          this.client
        );
      }
      const result = await this.currentNodeInstance.execute(this.sessionId);
      if (result.name === "running") {
        return;
      }
      await this.blackboard.saveNodeResult(this.currentNode, result);
      if (result.data) {
        for (const [key, value] of Object.entries(result.data)) {
          await this.blackboard.set(`${this.currentNode}_${key}`, value);
        }
      }
      const nextNode = this.getNextNode(nodeDef, result.name);
      if (!nextNode) {
        if (nodeDef.type === "end") {
          const endStatus = nodeDef.config?.status || "success";
          this.status = endStatus === "success" ? "completed" : "failed";
          await this.blackboard.set("_final_status", endStatus);
          await this.blackboard.set("_final_message", result.message);
        } else {
          this.status = "failed";
          await this.blackboard.set("error", `No route for result: ${result.name}`);
        }
        return;
      }
      this.currentNode = nextNode;
      this.currentNodeInstance = null;
      this.retryCount = 0;
      this.stepCount++;
      await this.blackboard.set("_current_state", nextNode);
    } catch (error) {
      const maxRetries = this.blackboard.get("_config_max_retries") || 3;
      this.retryCount++;
      if (this.retryCount >= maxRetries) {
        this.status = "failed";
        await this.blackboard.set("error", `Max retries exceeded: ${error}`);
      } else {
        await this.blackboard.set("_retry_count", this.retryCount);
      }
    }
  }
  /**
   * 다음 노드 결정
   */
  getNextNode(nodeDef, resultName) {
    const routes = nodeDef.routes || {};
    if (resultName in routes) {
      return routes[resultName];
    }
    if (resultName !== "success" && resultName !== "failed") {
      if ("success" in routes && !resultName.includes("fail")) {
        return routes["success"];
      }
      if ("failed" in routes) {
        return routes["failed"];
      }
    }
    return null;
  }
  /**
   * 인스턴스 상태 조회
   */
  getStatus() {
    return {
      instanceId: this.instanceId,
      flowName: this.flowDef.name,
      currentNode: this.currentNode,
      status: this.status,
      startedAt: this.blackboard.get("_started_at") || (/* @__PURE__ */ new Date()).toISOString(),
      elapsedMs: Date.now() - this.startTime,
      retryCount: this.retryCount
    };
  }
  /**
   * 인스턴스 중단
   */
  async stop() {
    this.status = "failed";
    await this.blackboard.set("_stopped_at", (/* @__PURE__ */ new Date()).toISOString());
    await this.blackboard.set("_stopped_reason", "Manual stop requested");
    if (this.sessionId) {
      try {
        await this.client.session.delete(this.sessionId);
      } catch {
      }
    }
  }
  /**
   * Blackboard 직접 접근 (디버깅용)
   */
  getBlackboard() {
    return this.blackboard;
  }
};

// opencode/custom-plugins/flows/src/types/schemas.ts
import { z } from "zod";
var NodeResultSchema = z.object({
  name: z.string(),
  message: z.string(),
  data: z.record(z.any()).optional(),
  duration: z.number().optional()
});
var AgentResultDefSchema = z.object({
  name: z.string(),
  description: z.string()
});
var ConditionSchema = z.object({
  field: z.string(),
  operator: z.enum(["eq", "ne", "gt", "lt", "contains", "exists"]),
  value: z.any(),
  result: z.string()
});
var AgentNodeConfigSchema = z.object({
  prompt: z.string(),
  agent_type: z.enum([
    "general",
    "senior-sw-engineer",
    "py-code-reviewer",
    "cpp-review",
    "research-analyst",
    "qa",
    "plan",
    "build"
  ]).default("general"),
  results: z.record(z.string()).optional()
  // Array -> Record로 변경 (name: desc)
});
var ToolNodeConfigSchema = z.object({
  tool: z.enum([
    "read",
    "write",
    "edit",
    "glob",
    "grep",
    "bash",
    "webfetch",
    "task"
  ]),
  args: z.record(z.any()).optional()
});
var CommandNodeConfigSchema = z.object({
  command: z.string(),
  workdir: z.string().optional(),
  expect_exit_code: z.number().optional()
});
var SkillNodeConfigSchema = z.object({
  skill: z.string(),
  args: z.record(z.any()).optional()
});
var ConditionalNodeConfigSchema = z.object({
  conditions: z.array(ConditionSchema),
  default: z.string().optional()
});
var ParallelNodeConfigSchema = z.object({
  nodes: z.array(z.string()),
  nodeConfigs: z.record(z.any()).optional()
});
var DelayNodeConfigSchema = z.object({
  duration: z.number()
  // ms
});
var LoopNodeConfigSchema = z.object({
  max_iterations: z.number().default(10),
  while_condition: ConditionSchema.optional(),
  body_node: z.string()
});
var RetryNodeConfigSchema = z.object({
  target_node: z.string(),
  max_retries: z.number().default(3),
  retry_delay: z.number().default(1e3)
});
var SubFlowNodeConfigSchema = z.object({
  flow_name: z.string(),
  input_mapping: z.record(z.string()).optional(),
  output_mapping: z.record(z.string()).optional()
});
var EndNodeConfigSchema = z.object({
  status: z.enum(["success", "failed"]).default("success"),
  message: z.string().optional()
});
var NodeDefinitionSchema = z.object({
  type: z.enum([
    "agent",
    "tool",
    "command",
    "skill",
    "conditional",
    "parallel",
    "delay",
    "loop",
    "retry",
    "sub-flow",
    "end"
  ]),
  config: z.any(),
  // 노드 타입별로 다른 스키마 적용
  routes: z.record(z.string().nullable()).optional()
});
var FlowConfigSchema = z.object({
  timeout: z.number().default(3e5),
  // 5분
  max_retries: z.number().default(3),
  retry_delay: z.number().default(1e3),
  tick_interval: z.number().default(500),
  history_depth: z.number().default(5),
  max_steps: z.number().default(100)
  // 최대 실행 단계 수
});
var FlowDefinitionSchema = z.object({
  name: z.string().regex(/^[a-z][a-z0-9-]*$/),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  description: z.string().optional(),
  config: FlowConfigSchema.optional(),
  initial_state: z.string(),
  nodes: z.record(NodeDefinitionSchema)
});
var ConciseBase = z.object({
  on: z.record(z.string().nullable()).optional()
});
var ConciseAgentSchema = ConciseBase.extend({
  agent: z.string(),
  // agent_type
  prompt: z.string(),
  results: z.record(z.string()).optional()
});
var ConciseCommandSchema = ConciseBase.extend({
  run: z.string(),
  // command
  workdir: z.string().optional(),
  expect_exit_code: z.number().optional()
});
var ConciseToolSchema = ConciseBase.extend({
  tool: z.string(),
  args: z.record(z.any()).optional()
});
var ConciseDelaySchema = ConciseBase.extend({
  wait: z.number()
  // duration
});
var ConciseEndSchema = z.object({
  end: z.boolean().optional(),
  // true면 end 노드
  status: z.enum(["success", "failed"]).optional(),
  message: z.string().optional()
});
var ConciseLoopSchema = ConciseBase.extend({
  loop: z.string(),
  // body_node
  max: z.number().optional(),
  while: ConditionSchema.optional()
});
var ConciseConditionalSchema = ConciseBase.extend({
  conditions: z.array(ConditionSchema),
  default: z.string().optional()
});
var ConciseNodeSchema = z.union([
  ConciseAgentSchema,
  ConciseCommandSchema,
  ConciseToolSchema,
  ConciseDelaySchema,
  ConciseEndSchema,
  ConciseLoopSchema,
  ConciseConditionalSchema,
  // Fallback to Canonical-like structure but with 'on'
  z.object({
    type: z.string(),
    config: z.any(),
    on: z.record(z.string().nullable()).optional()
  })
]);
var ConciseFlowDefinitionSchema = z.object({
  name: z.string().regex(/^[a-z][a-z0-9-]*$/),
  version: z.string().regex(/^\d+\.\d+\.\d+$/),
  description: z.string().optional(),
  config: FlowConfigSchema.optional(),
  start: z.string(),
  // initial_state 대체
  nodes: z.record(ConciseNodeSchema)
});
var PluginConfigSchema = z.object({
  flowsDir: z.string().default(".opencode/flows"),
  globalNodesDir: z.string().default("~/.config/opencode/shared/flows/nodes"),
  dataDir: z.string().default("~/.config/opencode/data/flows"),
  enableToasts: z.boolean().default(true),
  debugMode: z.boolean().default(false),
  tickInterval: z.number().default(500),
  keepCompletedInstances: z.boolean().default(false)
});

// opencode/custom-plugins/flows/src/utils/SchemaNormalizer.ts
var SchemaNormalizer = class {
  /**
   * 플로우 정의를 정규화합니다.
   * 입력이 이미 정규 형식이면 유효성 검사 후 반환하고,
   * 단축 형식이면 변환 후 반환합니다.
   */
  static normalize(input) {
    if (input.initial_state && !input.start) {
      return FlowDefinitionSchema.parse(input);
    }
    const concise = input;
    const canonical = {
      name: concise.name,
      version: concise.version,
      description: concise.description,
      config: concise.config,
      initial_state: concise.start,
      // start -> initial_state
      nodes: {}
    };
    for (const [nodeName, nodeBody] of Object.entries(concise.nodes)) {
      canonical.nodes[nodeName] = this.normalizeNode(nodeBody);
    }
    return FlowDefinitionSchema.parse(canonical);
  }
  /**
   * 단일 노드를 정규화합니다.
   */
  static normalizeNode(node) {
    if ("type" in node && "config" in node) {
      const anyNode = node;
      return {
        type: anyNode.type,
        config: anyNode.config,
        routes: anyNode.routes || anyNode.on
        // routes가 없으면 on을 사용
      };
    }
    const routes = "on" in node ? node.on : void 0;
    let type;
    let config = {};
    if ("agent" in node) {
      type = "agent";
      config = {
        agent_type: node.agent,
        prompt: node.prompt,
        results: node.results
      };
    } else if ("run" in node) {
      type = "command";
      config = {
        command: node.run,
        workdir: node.workdir,
        expect_exit_code: node.expect_exit_code
      };
    } else if ("tool" in node) {
      type = "tool";
      config = {
        tool: node.tool,
        args: node.args
      };
    } else if ("wait" in node) {
      type = "delay";
      config = {
        duration: node.wait
      };
    } else if ("loop" in node) {
      type = "loop";
      config = {
        body_node: node.loop,
        max_iterations: node.max,
        while_condition: node.while
      };
    } else if ("end" in node || "status" in node && !("type" in node)) {
      type = "end";
      config = {
        status: node.status || "success",
        message: node.message
      };
    } else if ("conditions" in node) {
      type = "conditional";
      config = {
        conditions: node.conditions,
        default: node.default
      };
    } else {
      throw new Error(`Unknown node type or missing required fields: ${JSON.stringify(node)}`);
    }
    return {
      type,
      config,
      routes
    };
  }
};

// opencode/custom-plugins/flows/src/manager/FlowManager.ts
import { v4 as uuidv4 } from "uuid";
var FlowManager = class _FlowManager {
  static instance = null;
  instances = /* @__PURE__ */ new Map();
  tickInterval = null;
  config;
  client;
  projectDir = "";
  constructor() {
    this.config = PluginConfigSchema.parse({});
  }
  /**
   * 싱글톤 인스턴스 획득
   */
  static getInstance() {
    if (!_FlowManager.instance) {
      _FlowManager.instance = new _FlowManager();
    }
    return _FlowManager.instance;
  }
  /**
   * 매니저 초기화
   */
  initialize(client, projectDir, config) {
    this.client = client;
    this.projectDir = projectDir;
    if (config) {
      this.config = PluginConfigSchema.parse({ ...this.config, ...config });
    }
  }
  /**
   * 새 플로우 인스턴스 시작
   */
  async start(flowName, prompt) {
    const flowDef = await this.loadFlowDefinition(flowName);
    const instanceId = this.generateInstanceId();
    const instance = new FlowInstance(instanceId, flowDef, prompt, this.client);
    this.instances.set(instanceId, instance);
    await instance.initialize();
    this.ensureTickLoop();
    if (this.config.enableToasts) {
      await this.client.toast(`[Flow] ${flowName} \uC2DC\uC791\uB428 (ID: ${instanceId.substring(0, 8)})`);
    }
    return instanceId;
  }
  /**
   * 플로우 정의 로드
   */
  async loadFlowDefinition(flowName) {
    const searchPaths = [
      // 1. 프로젝트 로컬
      path2.join(this.projectDir, ".opencode/flows", `${flowName}.json`),
      // 2. 글로벌 사용자 설정
      path2.join(os2.homedir(), ".config/opencode/flows", `${flowName}.json`),
      // 3. 글로벌 shared
      path2.join(os2.homedir(), ".config/opencode/shared/flows", `${flowName}.json`)
    ];
    for (const filePath of searchPaths) {
      try {
        const content = await fs2.readFile(filePath, "utf-8");
        const parsed = JSON.parse(content);
        return SchemaNormalizer.normalize(parsed);
      } catch {
        continue;
      }
    }
    throw new Error(`Flow definition not found: ${flowName}`);
  }
  /**
   * 사용 가능한 플로우 목록 조회
   */
  async listAvailableFlows() {
    const flows = [];
    const searchDirs = [
      path2.join(this.projectDir, ".opencode/flows"),
      path2.join(os2.homedir(), ".config/opencode/flows"),
      path2.join(os2.homedir(), ".config/opencode/shared/flows")
    ];
    for (const dir of searchDirs) {
      try {
        const files = await fs2.readdir(dir);
        for (const file of files) {
          if (!file.endsWith(".json")) continue;
          try {
            const content = await fs2.readFile(path2.join(dir, file), "utf-8");
            const parsed = JSON.parse(content);
            flows.push({
              name: parsed.name || file.replace(".json", ""),
              description: parsed.description,
              path: path2.join(dir, file)
            });
          } catch {
          }
        }
      } catch {
      }
    }
    const seen = /* @__PURE__ */ new Set();
    return flows.filter((f) => {
      if (seen.has(f.name)) return false;
      seen.add(f.name);
      return true;
    });
  }
  /**
   * Tick 루프 시작/유지
   */
  ensureTickLoop() {
    if (this.tickInterval) return;
    this.tickInterval = setInterval(async () => {
      if (this.instances.size === 0) {
        this.stopTickLoop();
        return;
      }
      const toRemove = [];
      for (const [id, instance] of this.instances) {
        const statusBefore = instance.status;
        if (instance.status === "running") {
          await instance.tick();
        }
        if (instance.status !== statusBefore && instance.status !== "running") {
          if (this.config.enableToasts) {
            const status = instance.getStatus();
            await this.client.toast(
              `[Flow] ${status.flowName} ${instance.status} (${status.currentNode})`
            );
          }
        }
        if (!this.config.keepCompletedInstances && (instance.status === "completed" || instance.status === "failed")) {
          toRemove.push(id);
        }
      }
      for (const id of toRemove) {
        this.instances.delete(id);
      }
    }, this.config.tickInterval);
  }
  /**
   * Tick 루프 중지
   */
  stopTickLoop() {
    if (this.tickInterval) {
      clearInterval(this.tickInterval);
      this.tickInterval = null;
    }
  }
  /**
   * 인스턴스 ID 생성
   */
  generateInstanceId() {
    return uuidv4();
  }
  /**
   * 모든 인스턴스 상태 조회
   */
  getStatus() {
    return Array.from(this.instances.values()).map((i) => i.getStatus());
  }
  /**
   * 특정 인스턴스 상태 조회
   */
  getInstanceStatus(instanceId) {
    const instance = this.instances.get(instanceId);
    return instance?.getStatus();
  }
  /**
   * 인스턴스 중단
   */
  async stop(instanceId) {
    const instance = this.instances.get(instanceId);
    if (!instance) {
      return false;
    }
    await instance.stop();
    this.instances.delete(instanceId);
    if (this.config.enableToasts) {
      await this.client.toast(`[Flow] \uC778\uC2A4\uD134\uC2A4 \uC911\uB2E8\uB428: ${instanceId.substring(0, 8)}`);
    }
    return true;
  }
  /**
   * 모든 인스턴스 중단
   */
  async stopAll() {
    const count = this.instances.size;
    for (const instance of this.instances.values()) {
      await instance.stop();
    }
    this.instances.clear();
    this.stopTickLoop();
    return count;
  }
  /**
   * 실행 중인 인스턴스 수
   */
  get runningCount() {
    return Array.from(this.instances.values()).filter(
      (i) => i.status === "running"
    ).length;
  }
  /**
   * 전체 인스턴스 수
   */
  get totalCount() {
    return this.instances.size;
  }
};

// opencode/custom-plugins/flows/src/index.ts
var FlowsPlugin = async ({ directory, client }) => {
  const manager = FlowManager.getInstance();
  manager.initialize(client, directory);
  const formatStatus = (statuses) => {
    if (statuses.length === 0) {
      return "\uC2E4\uD589 \uC911\uC778 \uD50C\uB85C\uC6B0\uAC00 \uC5C6\uC2B5\uB2C8\uB2E4.";
    }
    return statuses.map((s) => {
      const elapsed = s.elapsedMs ? `${Math.round(s.elapsedMs / 1e3)}s` : "N/A";
      return `[${s.instanceId.substring(0, 8)}] ${s.flowName} - ${s.status} @ ${s.currentNode} (${elapsed})`;
    }).join("\n");
  };
  const formatFlowList = (flows) => {
    if (flows.length === 0) {
      return "\uC0AC\uC6A9 \uAC00\uB2A5\uD55C \uD50C\uB85C\uC6B0\uAC00 \uC5C6\uC2B5\uB2C8\uB2E4.\n.opencode/flows/ \uB514\uB809\uD1A0\uB9AC\uC5D0 JSON \uD30C\uC77C\uC744 \uCD94\uAC00\uD558\uC138\uC694.";
    }
    return flows.map((f) => `\u2022 ${f.name}${f.description ? ` - ${f.description}` : ""}`).join("\n");
  };
  return {
    commands: {
      flow: {
        description: "\uC6CC\uD06C\uD50C\uB85C\uC6B0 \uAD00\uB9AC (start|stop|status|list)",
        args: {
          action: {
            type: "string",
            description: "\uC561\uC158: start, stop, status, list"
          },
          name: {
            type: "string",
            optional: true,
            description: "\uD50C\uB85C\uC6B0 \uC774\uB984 \uB610\uB294 \uC778\uC2A4\uD134\uC2A4 ID"
          },
          prompt: {
            type: "string",
            optional: true,
            description: "\uD50C\uB85C\uC6B0\uC5D0 \uC804\uB2EC\uD560 \uD504\uB86C\uD504\uD2B8"
          }
        },
        async execute(args) {
          try {
            switch (args.action) {
              case "start": {
                if (!args.name) {
                  return "\uD50C\uB85C\uC6B0 \uC774\uB984\uC744 \uC9C0\uC815\uD558\uC138\uC694: /flow start <flow-name> [prompt]";
                }
                const id = await manager.start(args.name, args.prompt);
                return `\u2705 \uD50C\uB85C\uC6B0 \uC2DC\uC791\uB428
\u2022 ID: ${id}
\u2022 \uC774\uB984: ${args.name}`;
              }
              case "stop": {
                if (!args.name) {
                  const count = await manager.stopAll();
                  return `${count}\uAC1C\uC758 \uD50C\uB85C\uC6B0\uAC00 \uC911\uB2E8\uB418\uC5C8\uC2B5\uB2C8\uB2E4.`;
                }
                const stopped = await manager.stop(args.name);
                if (stopped) {
                  return `\uD50C\uB85C\uC6B0 \uC911\uB2E8\uB428: ${args.name}`;
                }
                return `\uD50C\uB85C\uC6B0\uB97C \uCC3E\uC744 \uC218 \uC5C6\uC2B5\uB2C8\uB2E4: ${args.name}`;
              }
              case "status": {
                if (args.name) {
                  const status = manager.getInstanceStatus(args.name);
                  if (status) {
                    return formatStatus([status]);
                  }
                  return `\uC778\uC2A4\uD134\uC2A4\uB97C \uCC3E\uC744 \uC218 \uC5C6\uC2B5\uB2C8\uB2E4: ${args.name}`;
                }
                const statuses = manager.getStatus();
                return formatStatus(statuses);
              }
              case "list": {
                const flows = await manager.listAvailableFlows();
                return formatFlowList(flows);
              }
              default:
                return `\uC54C \uC218 \uC5C6\uB294 \uC561\uC158: ${args.action}
\uC0AC\uC6A9\uBC95: /flow <start|stop|status|list> [name] [prompt]`;
            }
          } catch (error) {
            return `\uC624\uB958: ${error}`;
          }
        }
      }
    },
    // 이벤트 훅
    event: async ({ event }) => {
      if (event.type === "session.deleted") {
      }
    }
  };
};
var index_default = FlowsPlugin;
export {
  AgentNodeConfigSchema,
  AgentResultDefSchema,
  BaseNode,
  Blackboard,
  CommandNodeConfigSchema,
  ConciseAgentSchema,
  ConciseCommandSchema,
  ConciseConditionalSchema,
  ConciseDelaySchema,
  ConciseEndSchema,
  ConciseFlowDefinitionSchema,
  ConciseLoopSchema,
  ConciseNodeSchema,
  ConciseToolSchema,
  ConditionSchema,
  ConditionalNodeConfigSchema,
  DelayNodeConfigSchema,
  EndNodeConfigSchema,
  FlowConfigSchema,
  FlowDefinitionSchema,
  FlowInstance,
  FlowManager,
  FlowsPlugin,
  LoopNodeConfigSchema,
  NodeDefinitionSchema,
  NodeResultSchema,
  ParallelNodeConfigSchema,
  PluginConfigSchema,
  RetryNodeConfigSchema,
  SkillNodeConfigSchema,
  SubFlowNodeConfigSchema,
  ToolNodeConfigSchema,
  index_default as default
};
