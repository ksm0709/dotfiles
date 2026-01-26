/**
 * OpenCode Flows Plugin - Blackboard
 * 
 * 플로우 인스턴스의 상태를 관리하는 Key-Value 저장소
 * 파일 기반 영속화를 지원하여 crash recovery 가능
 * 
 * ## 히스토리 관리 정책 (개선됨)
 * - 각 노드의 결과는 노드 이름을 키로 하여 `_results`에 저장
 * - 노드 이름은 플로우 내에서 유일하므로 충돌 없음
 * - 같은 노드가 여러 번 실행되면 최신 결과로 덮어쓰고 executionCount 증가
 * - 실행 순서는 `_execution_order` 배열에 별도 기록
 */

import * as fs from "node:fs/promises";
import * as path from "node:path";
import * as os from "node:os";
import type { BlackboardData, NodeResult } from "../types/schemas";

/** 노드 결과 저장 구조 */
interface NodeResultEntry {
  result: NodeResult;
  timestamp: string;
  executionCount: number;
}

/** 노드 결과 맵 */
interface ResultsMap {
  [nodeName: string]: NodeResultEntry;
}

export class Blackboard {
  private cache: Map<string, any> = new Map();
  private dirty: boolean = false;
  private readonly filePath: string;
  private saveDebounceTimer: NodeJS.Timeout | null = null;
  
  constructor(instanceId: string) {
    const dataDir = path.join(
      os.homedir(),
      ".config/opencode/data/flows/instances"
    );
    this.filePath = path.join(dataDir, `${instanceId}.json`);
  }

  /**
   * 저장된 상태 로드 (있는 경우)
   */
  async load(): Promise<void> {
    try {
      const content = await fs.readFile(this.filePath, "utf-8");
      const data = JSON.parse(content) as BlackboardData;
      this.cache = new Map(Object.entries(data));
    } catch {
      // 새 인스턴스 - 파일이 없음
      // 기본 구조 초기화
      this.cache.set("_results", {});
      this.cache.set("_execution_order", []);
    }
  }

  /**
   * 현재 상태를 파일로 저장
   */
  async save(): Promise<void> {
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
  private debouncedSave(): void {
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
  get<T = any>(key: string): T | undefined {
    return this.cache.get(key) as T | undefined;
  }

  /**
   * 값 조회 (중첩 키 지원)
   * 예: "history.analyze.data.score"
   */
  getValue(path: string): any {
    // 1. 직접 키 조회
    if (this.cache.has(path)) {
      return this.cache.get(path);
    }

    // 2. 중첩 키 조회
    const parts = path.split(".");
    
    // history.nodeName 형태 처리 (단축 문법)
    if (parts[0] === "history" && parts.length >= 2) {
      const nodeName = parts[1];
      const result = this.getNodeResult(nodeName);
      
      if (!result) return undefined;
      
      // history.nodeName -> message
      if (parts.length === 2) {
        return result.message;
      }
      
      // history.nodeName.message 또는 history.nodeName.data.field
      let value: any = result;
      for (let i = 2; i < parts.length; i++) {
        if (value && typeof value === "object") {
          value = value[parts[i]];
        } else {
          return undefined;
        }
      }
      return value;
    }

    // 일반 중첩 키 조회
    let current: any = this.getAll();
    for (const part of parts) {
      if (current && typeof current === "object") {
        current = current[part];
      } else {
        return undefined;
      }
    }
    
    return current;
  }

  /**
   * 값 설정
   */
  async set(key: string, value: any): Promise<void> {
    this.cache.set(key, value);
    this.dirty = true;
    await this.save(); // 즉시 저장 (crash recovery)
  }

  /**
   * 값 존재 여부 확인
   */
  has(key: string): boolean {
    return this.cache.has(key);
  }

  /**
   * 값 삭제
   */
  async delete(key: string): Promise<boolean> {
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
  getAll(): BlackboardData {
    return Object.fromEntries(this.cache) as BlackboardData;
  }

  /**
   * 노드 결과 저장 (개선된 방식)
   * 
   * - 노드 이름을 키로 하여 _results에 저장
   * - 같은 노드가 여러 번 실행되면 덮어쓰고 executionCount 증가
   * - 실행 순서는 _execution_order에 기록
   */
  async saveNodeResult(nodeName: string, result: NodeResult): Promise<void> {
    // 결과 맵 가져오기
    const results = this.get<ResultsMap>("_results") || {};
    
    // 기존 실행 횟수 확인
    const existingCount = results[nodeName]?.executionCount || 0;
    
    // 결과 저장 (덮어쓰기)
    results[nodeName] = {
      result,
      timestamp: new Date().toISOString(),
      executionCount: existingCount + 1,
    };
    
    await this.set("_results", results);
    
    // 실행 순서 기록
    const executionOrder = this.get<string[]>("_execution_order") || [];
    executionOrder.push(nodeName);
    await this.set("_execution_order", executionOrder);
  }

  /**
   * 특정 노드의 결과 조회
   */
  getNodeResult(nodeName: string): NodeResult | undefined {
    const results = this.get<ResultsMap>("_results") || {};
    return results[nodeName]?.result;
  }

  /**
   * 특정 노드의 결과 엔트리 전체 조회 (메타데이터 포함)
   */
  getNodeResultEntry(nodeName: string): NodeResultEntry | undefined {
    const results = this.get<ResultsMap>("_results") || {};
    return results[nodeName];
  }

  /**
   * 모든 노드 결과 조회
   */
  getAllNodeResults(): ResultsMap {
    return this.get<ResultsMap>("_results") || {};
  }

  /**
   * 실행 순서 조회
   */
  getExecutionOrder(): string[] {
    return this.get<string[]>("_execution_order") || [];
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
  resolveVariables(template: string): string {
    return template.replace(/\$\{([^}]+)\}/g, (match, key: string) => {
      const parts = key.split(".");
      
      // history.nodeName 형태 처리 (단축 문법)
      if (parts[0] === "history" && parts.length >= 2) {
        const nodeName = parts[1];
        const result = this.getNodeResult(nodeName);
        
        if (!result) {
          return match; // 결과 없으면 원본 유지
        }
        
        // ${history.nodeName} → message 반환
        if (parts.length === 2) {
          return result.message;
        }
        
        // ${history.nodeName.message} 또는 ${history.nodeName.data.field}
        let value: any = result;
        for (let i = 2; i < parts.length; i++) {
          if (value && typeof value === "object") {
            value = value[parts[i]];
          } else {
            return match;
          }
        }
        
        return value !== undefined ? String(value) : match;
      }
      
      // 직접 키 조회
      const value = this.get(key);
      if (value !== undefined) {
        if (typeof value === "object") {
          return JSON.stringify(value);
        }
        return String(value);
      }
      
      // 중첩 키 조회 (a.b.c 형태)
      let current: any = this.getAll();
      for (const part of parts) {
        if (current && typeof current === "object") {
          current = current[part];
        } else {
          return match; // 값 없으면 원본 유지
        }
      }
      
      if (current !== undefined) {
        if (typeof current === "object") {
          return JSON.stringify(current);
        }
        return String(current);
      }
      
      return match; // 값 없으면 원본 유지
    });
  }

  /**
   * 객체 내 모든 문자열 값의 변수 해석
   */
  resolveObjectVariables<T extends Record<string, any>>(obj: T): T {
    const result: any = {};
    
    for (const [key, value] of Object.entries(obj)) {
      if (typeof value === "string") {
        result[key] = this.resolveVariables(value);
      } else if (typeof value === "object" && value !== null) {
        result[key] = this.resolveObjectVariables(value);
      } else {
        result[key] = value;
      }
    }
    
    return result as T;
  }

  /**
   * 히스토리 컨텍스트 빌드 (에이전트 프롬프트용)
   * 실행 순서대로 모든 노드 결과를 포맷팅
   */
  buildHistoryContext(maxLength: number = 10000): string {
    const results = this.getAllNodeResults();
    const order = this.getExecutionOrder();
    
    if (order.length === 0) {
      return "";
    }
    
    // 중복 제거 (같은 노드는 한 번만)
    const seen = new Set<string>();
    const uniqueOrder = order.filter(name => {
      if (seen.has(name)) return false;
      seen.add(name);
      return true;
    });
    
    const lines: string[] = ["[이전 노드 실행 히스토리]"];
    let currentLength = lines[0].length;
    
    for (const nodeName of uniqueOrder) {
      const entry = results[nodeName];
      if (!entry) continue;
      
      const statusIcon = entry.result.name === "success" ? "✓" : 
                         entry.result.name === "failed" ? "✗" : "→";
      
      // 메시지 길이 제한 (노드당 최대 500자)
      let message = entry.result.message;
      if (message.length > 500) {
        message = message.substring(0, 497) + "...";
      }
      
      const line = `\n[${statusIcon} ${nodeName}] (${entry.result.name}):\n${message}`;
      
      if (currentLength + line.length > maxLength) {
        lines.push("\n... (이전 히스토리 생략)");
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
  async cleanup(): Promise<void> {
    try {
      await fs.unlink(this.filePath);
    } catch {
      // 파일이 없어도 무시
    }
  }
}
