/**
 * OpenCode Flows Plugin - Command Node
 * 
 * 쉘 명령어를 실행하는 노드
 */

import { BaseNode } from "./BaseNode";
import type { NodeResult, CommandNodeConfig } from "../types/schemas";
import { exec } from "node:child_process";
import { promisify } from "node:util";

const execAsync = promisify(exec);

export class CommandNode extends BaseNode {
  private pendingPromise?: Promise<void>;
  private storedResult?: NodeResult;

  constructor(
    nodeName: string,
    config: CommandNodeConfig,
    blackboard: any,
    private client: any
  ) {
    super(nodeName, config, blackboard);
  }

  async execute(_sessionId: string): Promise<NodeResult> {
    // 이미 실행 중이면 결과 확인
    if (this.pendingPromise) {
      if (this.storedResult) {
        const r = this.storedResult;
        this.storedResult = undefined;
        this.pendingPromise = undefined;
        return r;
      }
      return this.running();
    }

    this.startTimer();

    // 명령어 변수 해석
    const command = this.resolveVariables(this.config.command);
    const workdir = this.config.workdir 
      ? this.resolveVariables(this.config.workdir)
      : process.cwd();

    // 비동기 실행 시작
    this.pendingPromise = this.executeCommand(command, workdir);
    
    return this.running();
  }

  private async executeCommand(command: string, workdir: string): Promise<void> {
    try {
      const { stdout, stderr } = await execAsync(command, {
        cwd: workdir,
        timeout: 60000, // 1분 타임아웃
        maxBuffer: 10 * 1024 * 1024, // 10MB
      });

      const output = stdout || stderr;
      
      // 결과를 blackboard에 저장
      await this.blackboard.set(`${this.nodeName}_output`, output);
      await this.blackboard.set(`${this.nodeName}_exit_code`, 0);

      // expect_exit_code 체크
      if (this.config.expect_exit_code !== undefined) {
        // exit code 0인데 다른 값을 기대했다면
        if (this.config.expect_exit_code !== 0) {
          this.storedResult = this.failed(
            `Expected exit code ${this.config.expect_exit_code}, got 0\n\nOutput:\n${output}`
          );
          return;
        }
      }

      this.storedResult = this.success(output, { 
        output, 
        exitCode: 0 
      });
    } catch (error: any) {
      const exitCode = error.code || 1;
      const output = error.stdout || error.stderr || error.message;
      
      await this.blackboard.set(`${this.nodeName}_output`, output);
      await this.blackboard.set(`${this.nodeName}_exit_code`, exitCode);

      // expect_exit_code 체크
      if (this.config.expect_exit_code !== undefined) {
        if (this.config.expect_exit_code === exitCode) {
          // 기대한 에러 코드와 일치 → success
          this.storedResult = this.success(output, { 
            output, 
            exitCode 
          });
          return;
        }
      }

      this.storedResult = this.failed(
        `Command failed with exit code ${exitCode}\n\nOutput:\n${output}`,
        { output, exitCode }
      );
    }
  }
}
