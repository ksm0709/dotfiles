"use strict";
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
exports.Blackboard = void 0;
const fs = __importStar(require("node:fs/promises"));
const path = __importStar(require("node:path"));
const os = __importStar(require("node:os"));
class Blackboard {
    cache = new Map();
    dirty = false;
    filePath;
    saveDebounceTimer = null;
    constructor(instanceId) {
        const dataDir = path.join(os.homedir(), ".config/opencode/data/flows/instances");
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
        }
        catch {
            // 새 인스턴스 - 파일이 없음
            // 기본 구조 초기화
            this.cache.set("_results", {});
            this.cache.set("_execution_order", []);
        }
    }
    /**
     * 현재 상태를 파일로 저장
     */
    async save() {
        if (!this.dirty)
            return;
        try {
            const data = Object.fromEntries(this.cache);
            await fs.mkdir(path.dirname(this.filePath), { recursive: true });
            await fs.writeFile(this.filePath, JSON.stringify(data, null, 2));
            this.dirty = false;
        }
        catch (error) {
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
    getValue(path) {
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
            if (!result)
                return undefined;
            // history.nodeName -> message
            if (parts.length === 2) {
                return result.message;
            }
            // history.nodeName.message 또는 history.nodeName.data.field
            let value = result;
            for (let i = 2; i < parts.length; i++) {
                if (value && typeof value === "object") {
                    value = value[parts[i]];
                }
                else {
                    return undefined;
                }
            }
            return value;
        }
        // 일반 중첩 키 조회
        let current = this.getAll();
        for (const part of parts) {
            if (current && typeof current === "object") {
                current = current[part];
            }
            else {
                return undefined;
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
        await this.save(); // 즉시 저장 (crash recovery)
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
        // 결과 맵 가져오기
        const results = this.get("_results") || {};
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
                let value = result;
                for (let i = 2; i < parts.length; i++) {
                    if (value && typeof value === "object") {
                        value = value[parts[i]];
                    }
                    else {
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
            let current = this.getAll();
            for (const part of parts) {
                if (current && typeof current === "object") {
                    current = current[part];
                }
                else {
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
    resolveObjectVariables(obj) {
        const result = {};
        for (const [key, value] of Object.entries(obj)) {
            if (typeof value === "string") {
                result[key] = this.resolveVariables(value);
            }
            else if (typeof value === "object" && value !== null) {
                result[key] = this.resolveObjectVariables(value);
            }
            else {
                result[key] = value;
            }
        }
        return result;
    }
    /**
     * 히스토리 컨텍스트 빌드 (에이전트 프롬프트용)
     * 실행 순서대로 모든 노드 결과를 포맷팅
     */
    buildHistoryContext(maxLength = 10000) {
        const results = this.getAllNodeResults();
        const order = this.getExecutionOrder();
        if (order.length === 0) {
            return "";
        }
        // 중복 제거 (같은 노드는 한 번만)
        const seen = new Set();
        const uniqueOrder = order.filter(name => {
            if (seen.has(name))
                return false;
            seen.add(name);
            return true;
        });
        const lines = ["[이전 노드 실행 히스토리]"];
        let currentLength = lines[0].length;
        for (const nodeName of uniqueOrder) {
            const entry = results[nodeName];
            if (!entry)
                continue;
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
    async cleanup() {
        try {
            await fs.unlink(this.filePath);
        }
        catch {
            // 파일이 없어도 무시
        }
    }
}
exports.Blackboard = Blackboard;
//# sourceMappingURL=Blackboard.js.map