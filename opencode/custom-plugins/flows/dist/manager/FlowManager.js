"use strict";
/**
 * OpenCode Flows Plugin - Flow Manager
 *
 * 싱글톤 매니저: 모든 플로우 인스턴스의 라이프사이클 관리
 * 500ms 간격으로 tick loop 실행
 */
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.FlowManager = void 0;
const fs = __importStar(require("node:fs/promises"));
const path = __importStar(require("node:path"));
const os = __importStar(require("node:os"));
const FlowInstance_1 = require("./FlowInstance");
const schemas_1 = require("../types/schemas");
const SchemaNormalizer_1 = require("../utils/SchemaNormalizer");
const uuid_1 = require("uuid");
class FlowManager {
    static instance = null;
    instances = new Map();
    tickInterval = null;
    config;
    client;
    projectDir = "";
    constructor() {
        this.config = schemas_1.PluginConfigSchema.parse({});
    }
    /**
     * 싱글톤 인스턴스 획득
     */
    static getInstance() {
        if (!FlowManager.instance) {
            FlowManager.instance = new FlowManager();
        }
        return FlowManager.instance;
    }
    /**
     * 매니저 초기화
     */
    initialize(client, projectDir, config) {
        this.client = client;
        this.projectDir = projectDir;
        if (config) {
            this.config = schemas_1.PluginConfigSchema.parse({ ...this.config, ...config });
        }
    }
    /**
     * 새 플로우 인스턴스 시작
     */
    async start(flowName, prompt) {
        const flowDef = await this.loadFlowDefinition(flowName);
        const instanceId = this.generateInstanceId();
        const instance = new FlowInstance_1.FlowInstance(instanceId, flowDef, prompt, this.client);
        this.instances.set(instanceId, instance);
        await instance.initialize();
        this.ensureTickLoop();
        // 시작 알림
        if (this.config.enableToasts) {
            await this.client.toast(`[Flow] ${flowName} 시작됨 (ID: ${instanceId.substring(0, 8)})`);
        }
        return instanceId;
    }
    /**
     * 플로우 정의 로드
     */
    async loadFlowDefinition(flowName) {
        // 검색 경로 (우선순위 순)
        const searchPaths = [
            // 1. 프로젝트 로컬
            path.join(this.projectDir, ".opencode/flows", `${flowName}.json`),
            // 2. 글로벌 사용자 설정
            path.join(os.homedir(), ".config/opencode/flows", `${flowName}.json`),
            // 3. 글로벌 shared
            path.join(os.homedir(), ".config/opencode/shared/flows", `${flowName}.json`),
        ];
        for (const filePath of searchPaths) {
            try {
                const content = await fs.readFile(filePath, "utf-8");
                const parsed = JSON.parse(content);
                return SchemaNormalizer_1.SchemaNormalizer.normalize(parsed);
            }
            catch {
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
            path.join(this.projectDir, ".opencode/flows"),
            path.join(os.homedir(), ".config/opencode/flows"),
            path.join(os.homedir(), ".config/opencode/shared/flows"),
        ];
        for (const dir of searchDirs) {
            try {
                const files = await fs.readdir(dir);
                for (const file of files) {
                    if (!file.endsWith(".json"))
                        continue;
                    try {
                        const content = await fs.readFile(path.join(dir, file), "utf-8");
                        const parsed = JSON.parse(content);
                        flows.push({
                            name: parsed.name || file.replace(".json", ""),
                            description: parsed.description,
                            path: path.join(dir, file),
                        });
                    }
                    catch {
                        // 파싱 실패 무시
                    }
                }
            }
            catch {
                // 디렉토리 없음 무시
            }
        }
        // 중복 제거 (이름 기준, 첫 번째 우선)
        const seen = new Set();
        return flows.filter((f) => {
            if (seen.has(f.name))
                return false;
            seen.add(f.name);
            return true;
        });
    }
    /**
     * Tick 루프 시작/유지
     */
    ensureTickLoop() {
        if (this.tickInterval)
            return;
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
                // 상태 변경 알림
                if (instance.status !== statusBefore && instance.status !== "running") {
                    if (this.config.enableToasts) {
                        const status = instance.getStatus();
                        await this.client.toast(`[Flow] ${status.flowName} ${instance.status} (${status.currentNode})`);
                    }
                }
                // 완료/실패 인스턴스 제거 예약
                if (!this.config.keepCompletedInstances && (instance.status === "completed" || instance.status === "failed")) {
                    toRemove.push(id);
                }
            }
            // 완료된 인스턴스 정리
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
        return (0, uuid_1.v4)();
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
            await this.client.toast(`[Flow] 인스턴스 중단됨: ${instanceId.substring(0, 8)}`);
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
        return Array.from(this.instances.values()).filter((i) => i.status === "running").length;
    }
    /**
     * 전체 인스턴스 수
     */
    get totalCount() {
        return this.instances.size;
    }
}
exports.FlowManager = FlowManager;
//# sourceMappingURL=FlowManager.js.map