/**
 * OpenCode Flows Plugin - Tool Node
 * 
 * OpenCode 도구(tool)를 호출하는 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, ToolNodeConfig } from "../types/schemas";

export class ToolNode extends BaseNode {
  private client: any;

  constructor(
    nodeName: string,
    config: ToolNodeConfig,
    blackboard: any,
    client: any
  ) {
    super(nodeName, config, blackboard);
    this.client = client;
  }

  async execute(_sessionId: string): Promise<NodeResult> {
    this.startTimer();
    
    try {
      const toolName = this.config.tool;
      const args = this.resolveArgs(this.config.args || {});

      // OpenCode 도구 호출
      // client.tool.execute 가 없을 수 있으므로 세션을 통해 호출
      let result: string;
      
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
      
      // 결과를 blackboard에 저장
      await this.blackboard.set(`${this.nodeName}_output`, result);
      
      return this.success(result, { output: result });
    } catch (error) {
      return this.failed(`Tool execution failed: ${error}`);
    }
  }

  /**
   * 인자 내 변수 해석
   */
  private resolveArgs(args: Record<string, any>): Record<string, any> {
    return this.resolveConfig(args);
  }

  // 각 도구별 실행 메서드 (client API에 따라 조정 필요)
  
  private async executeRead(args: any): Promise<string> {
    // 실제로는 client.tool.execute 또는 fs 직접 사용
    const fs = await import("node:fs/promises");
    return await fs.readFile(args.filePath, "utf-8");
  }

  private async executeWrite(args: any): Promise<string> {
    const fs = await import("node:fs/promises");
    await fs.writeFile(args.filePath, args.content, "utf-8");
    return `File written: ${args.filePath}`;
  }

  private async executeGlob(args: any): Promise<string> {
    // glob 패턴 매칭
    const { glob } = await import("glob");
    const files = await glob(args.pattern, { cwd: args.path || process.cwd() });
    return files.join("\n");
  }

  private async executeGrep(args: any): Promise<string> {
    // 간단한 grep 구현
    const fs = await import("node:fs/promises");
    const path = await import("node:path");
    const { glob } = await import("glob");
    
    const pattern = new RegExp(args.pattern);
    const files = await glob(args.include || "**/*", { cwd: args.path || process.cwd() });
    
    const results: string[] = [];
    for (const file of files.slice(0, 100)) { // 최대 100개 파일
      try {
        const content = await fs.readFile(path.join(args.path || process.cwd(), file), "utf-8");
        const lines = content.split("\n");
        lines.forEach((line, i) => {
          if (pattern.test(line)) {
            results.push(`${file}:${i + 1}: ${line}`);
          }
        });
      } catch {
        // 읽기 실패 무시
      }
    }
    
    return results.join("\n");
  }

  private async executeBash(args: any): Promise<string> {
    const { exec } = await import("node:child_process");
    const { promisify } = await import("node:util");
    const execAsync = promisify(exec);
    
    const { stdout, stderr } = await execAsync(args.command, {
      cwd: args.workdir || process.cwd(),
      timeout: args.timeout || 60000,
    });
    
    return stdout || stderr;
  }
}
