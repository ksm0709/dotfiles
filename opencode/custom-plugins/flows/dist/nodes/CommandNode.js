"use strict";
/**
 * OpenCode Flows Plugin - Command Node
 *
 * 쉘 명령어를 실행하는 노드
 */
Object.defineProperty(exports, "__esModule", { value: true });
exports.CommandNode = void 0;
const BaseNode_1 = require("./BaseNode");
const node_child_process_1 = require("node:child_process");
const node_util_1 = require("node:util");
const execAsync = (0, node_util_1.promisify)(node_child_process_1.exec);
class CommandNode extends BaseNode_1.BaseNode {
    client;
    pendingPromise;
    storedResult;
    constructor(nodeName, config, blackboard, client) {
        super(nodeName, config, blackboard);
        this.client = client;
    }
    async execute(_sessionId) {
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
    async executeCommand(command, workdir) {
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
                    this.storedResult = this.failed(`Expected exit code ${this.config.expect_exit_code}, got 0\n\nOutput:\n${output}`);
                    return;
                }
            }
            this.storedResult = this.success(output, {
                output,
                exitCode: 0
            });
        }
        catch (error) {
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
            this.storedResult = this.failed(`Command failed with exit code ${exitCode}\n\nOutput:\n${output}`, { output, exitCode });
        }
    }
}
exports.CommandNode = CommandNode;
//# sourceMappingURL=CommandNode.js.map