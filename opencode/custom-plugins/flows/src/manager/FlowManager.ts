/**
 * OpenCode Flows Plugin - Flow Manager
 * 
 * 싱글톤 매니저: 모든 플로우 인스턴스의 라이프사이클 관리
 * 500ms 간격으로 tick loop 실행
 */

import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";
import { FlowInstance } from "./FlowInstance";
import type { FlowDefinition, FlowInstanceStatus, PluginConfig } from "../types/schemas";
import { PluginConfigSchema } from "../types/schemas";
import { SchemaNormalizer } from "../utils/SchemaNormalizer";
import { v4 as uuidv4 } from "uuid";

export class FlowManager {
  private static instance: FlowManager | null = null;
  
  private instances: Map<string, FlowInstance> = new Map();
  private tickInterval: NodeJS.Timeout | null = null;
  private config: PluginConfig;
  private client: any;
  private projectDir: string = "";
  
  private constructor() {
    this.config = PluginConfigSchema.parse({});
  }

  /**
   * 싱글톤 인스턴스 획득
   */
  static getInstance(): FlowManager {
    if (!FlowManager.instance) {
      FlowManager.instance = new FlowManager();
    }
    return FlowManager.instance;
  }

  /**
   * 매니저 초기화
   */
  initialize(client: any, projectDir: string, config?: Partial<PluginConfig>): void {
    this.client = client;
    this.projectDir = projectDir;
    if (config) {
      this.config = PluginConfigSchema.parse({ ...this.config, ...config });
    }
  }

  /**
   * 새 플로우 인스턴스 시작
   */
  async start(flowName: string, prompt?: string): Promise<string> {
    const flowDef = await this.loadFlowDefinition(flowName);
    const instanceId = this.generateInstanceId();
    
    const instance = new FlowInstance(instanceId, flowDef, prompt, this.client);
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
  private async loadFlowDefinition(flowName: string): Promise<FlowDefinition> {
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
  async listAvailableFlows(): Promise<Array<{ name: string; description?: string; path: string }>> {
    const flows: Array<{ name: string; description?: string; path: string }> = [];
    
    const searchDirs = [
      path.join(this.projectDir, ".opencode/flows"),
      path.join(os.homedir(), ".config/opencode/flows"),
      path.join(os.homedir(), ".config/opencode/shared/flows"),
    ];

    for (const dir of searchDirs) {
      try {
        const files = await fs.readdir(dir);
        for (const file of files) {
          if (!file.endsWith(".json")) continue;
          
          try {
            const content = await fs.readFile(path.join(dir, file), "utf-8");
            const parsed = JSON.parse(content);
            flows.push({
              name: parsed.name || file.replace(".json", ""),
              description: parsed.description,
              path: path.join(dir, file),
            });
          } catch {
            // 파싱 실패 무시
          }
        }
      } catch {
        // 디렉토리 없음 무시
      }
    }

    // 중복 제거 (이름 기준, 첫 번째 우선)
    const seen = new Set<string>();
    return flows.filter((f) => {
      if (seen.has(f.name)) return false;
      seen.add(f.name);
      return true;
    });
  }

  /**
   * Tick 루프 시작/유지
   */
  private ensureTickLoop(): void {
    if (this.tickInterval) return;
    
    this.tickInterval = setInterval(async () => {
      if (this.instances.size === 0) {
        this.stopTickLoop();
        return;
      }
      
      const toRemove: string[] = [];
      
      for (const [id, instance] of this.instances) {
        const statusBefore = instance.status;
        
        if (instance.status === "running") {
          await instance.tick();
        }
        
        // 상태 변경 알림
        if (instance.status !== statusBefore && instance.status !== "running") {
          if (this.config.enableToasts) {
            const status = instance.getStatus();
            await this.client.toast(
              `[Flow] ${status.flowName} ${instance.status} (${status.currentNode})`
            );
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
  private stopTickLoop(): void {
    if (this.tickInterval) {
      clearInterval(this.tickInterval);
      this.tickInterval = null;
    }
  }

  /**
   * 인스턴스 ID 생성
   */
  private generateInstanceId(): string {
    return uuidv4();
  }

  /**
   * 모든 인스턴스 상태 조회
   */
  getStatus(): FlowInstanceStatus[] {
    return Array.from(this.instances.values()).map((i) => i.getStatus());
  }

  /**
   * 특정 인스턴스 상태 조회
   */
  getInstanceStatus(instanceId: string): FlowInstanceStatus | undefined {
    const instance = this.instances.get(instanceId);
    return instance?.getStatus();
  }

  /**
   * 인스턴스 중단
   */
  async stop(instanceId: string): Promise<boolean> {
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
  async stopAll(): Promise<number> {
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
  get runningCount(): number {
    return Array.from(this.instances.values()).filter(
      (i) => i.status === "running"
    ).length;
  }

  /**
   * 전체 인스턴스 수
   */
  get totalCount(): number {
    return this.instances.size;
  }
}
