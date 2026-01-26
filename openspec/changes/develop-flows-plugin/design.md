# Design: Flows Plugin - Multi-Agent Workflow Management

## Context

### 배경
OpenCode에서 복잡한 멀티스텝 작업(코드 리뷰, TDD 사이클, 배포 파이프라인 등)을 수동으로 관리하는 것은 비효율적입니다. 사용자가 각 단계를 일일이 지시해야 하며, 중간 상태 관리도 수동으로 이루어집니다.

### 제약사항
- OpenCode 플러그인 API 제약 (비동기 이벤트 기반)
- 세션 관리의 한계 (컨텍스트 길이 제한)
- 장시간 실행 작업의 안정성 요구

### 이해관계자
- 개발자: 반복적인 워크플로우 자동화 필요
- DevOps: CI/CD 스타일 파이프라인 구성 필요

---

## Goals / Non-Goals

### Goals
- ✅ JSON 기반의 선언적 워크플로우 정의
- ✅ FSM 기반의 안정적인 상태 관리
- ✅ 다중 인스턴스 동시 실행 지원
- ✅ 파일 기반 영속화로 crash recovery 지원
- ✅ 에이전트 노드의 arbitrary results 지원
- ✅ 확장 가능한 노드 타입 아키텍처

### Non-Goals
- ❌ 비주얼 워크플로우 에디터 (향후 고려)
- ❌ 분산 실행 (단일 프로세스 내 실행)
- ❌ 실시간 협업 기능
- ❌ 버전 관리 및 롤백 (v1에서는 제외)

---

## Architecture

### 전체 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Flows Plugin                                 │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────────────────────────────────────┐ │
│  │   /flow     │───▶│            FlowManager (Singleton)          │ │
│  │   Command   │    │  ┌─────────────────────────────────────────┐│ │
│  └─────────────┘    │  │  Tick Loop (500ms interval)             ││ │
│                     │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐    ││ │
│                     │  │  │Instance1│ │Instance2│ │Instance3│    ││ │
│                     │  │  └────┬────┘ └────┬────┘ └────┬────┘    ││ │
│                     │  └───────┼───────────┼───────────┼─────────┘│ │
│                     └──────────┼───────────┼───────────┼──────────┘ │
│                                ▼           ▼           ▼            │
│                     ┌──────────────────────────────────────────────┐│
│                     │              FlowInstance (FSM)              ││
│                     │  ┌────────────────┐ ┌─────────────────────┐  ││
│                     │  │  Blackboard    │ │    Node Executor    │  ││
│                     │  │  (State Store) │ │                     │  ││
│                     │  │  ┌───────────┐ │ │  ┌───────────────┐  │  ││
│                     │  │  │ _results  │ │ │  │  AgentNode    │  │  ││
│                     │  │  │  .step1   │ │ │  │  ToolNode     │  │  ││
│                     │  │  │  .step2   │ │ │  │  CommandNode  │  │  ││
│                     │  │  │  .step3   │ │ │  │  CondNode     │  │  ││
│                     │  │  │  ...      │ │ │  │  EndNode      │  │  ││
│                     │  │  └───────────┘ │ │  └───────────────┘  │  ││
│                     │  └────────────────┘ └─────────────────────┘  ││
│                     └──────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────┘
```

### 컴포넌트 설명

| 컴포넌트 | 책임 | 상태 |
|----------|------|------|
| **FlowManager** | 싱글톤, 인스턴스 라이프사이클 관리, Tick 루프 | Stateless (인스턴스 참조만 보유) |
| **FlowInstance** | 개별 FSM, 상태 전이 관리 | `status`, `currentNode`, `retryCount` |
| **Blackboard** | Key-Value 저장소, 영속화 | 파일 기반 JSON |
| **NodeExecutor** | 노드 실행 위임 | Stateless |
| **Nodes** | 개별 노드 로직 | 노드별 상태 (pendingPromise 등) |

---

## Decisions

### Decision 1: 히스토리 관리 - 노드별 전체 저장

**결정**: N개 유지 대신 모든 노드의 결과를 노드 이름을 키로 저장

**이유**:
- 노드 이름이 플로우 내에서 유일하므로 키 충돌 없음
- 컨텍스트 손실 방지 (N개 유지 시 이전 노드 결과 참조 불가능할 수 있음)
- `${history.nodeName.message}` 형태로 직관적 참조 가능

**구현**:
```typescript
interface BlackboardData {
  // 노드별 결과 저장 (노드 이름이 키)
  _results: {
    [nodeName: string]: {
      result: NodeResult;
      timestamp: string;
      executionCount: number; // 같은 노드가 여러 번 실행된 경우
    };
  };
  
  // 실행 순서 기록 (디버깅 및 순서 추적용)
  _execution_order: string[];
}
```

**대안 고려**:
- ❌ N개 유지: 컨텍스트 손실 위험, 오래된 노드 참조 불가
- ❌ 전체 히스토리 배열: 중복 저장, 조회 비효율

---

### Decision 2: Tick 기반 비동기 실행

**결정**: 500ms 고정 간격의 Tick 루프

**이유**:
- 예측 가능한 실행 패턴
- 비동기 노드 (agent, command) 완료 대기 용이
- CPU 사용량 제어 가능

**구현**:
```typescript
class FlowManager {
  private readonly TICK_MS = 500;
  
  private ensureTickLoop(): void {
    this.tickInterval = setInterval(async () => {
      for (const instance of this.instances.values()) {
        if (instance.status === 'running') {
          await instance.tick();
        }
      }
    }, this.TICK_MS);
  }
}
```

**대안 고려**:
- ❌ 이벤트 기반: 복잡도 증가, 동시성 제어 어려움
- ❌ 동적 간격: 구현 복잡도 증가, 예측 어려움

---

### Decision 3: 플로우 스키마 가독성 개선

**결정**: 간결하고 직관적인 스키마 구조 채택

**기존 스키마** (복잡):
```json
{
  "nodes": {
    "review": {
      "type": "agent",
      "config": {
        "prompt": "...",
        "agent_type": "py-code-reviewer",
        "results": [
          { "name": "approved", "description": "승인" }
        ]
      },
      "routes": {
        "approved": "success_end"
      }
    }
  }
}
```

**개선된 스키마** (가독성 향상):
```json
{
  "nodes": {
    "review": {
      "type": "agent",
      "agent": "py-code-reviewer",
      "prompt": "코드를 리뷰하세요: ${history.analyze.message}",
      
      "results": {
        "approved": "코드 승인",
        "rejected": "수정 필요"
      },
      
      "on": {
        "approved": "success_end",
        "rejected": "request_changes"
      }
    }
  }
}
```

**개선 포인트**:
1. `config` 래퍼 제거 → 플랫한 구조
2. `agent_type` → `agent` (간결한 키 이름)
3. `results` 배열 → 객체 (name: description 직접 매핑)
4. `routes` → `on` (이벤트 핸들러 스타일, 더 직관적)

---

### Decision 4: 노드 타입별 단축 문법

**결정**: 자주 사용되는 노드 타입에 단축 문법 제공

**일반 문법**:
```json
{
  "run_tests": {
    "type": "command",
    "command": "npm test",
    "on": { "success": "next", "failed": "error" }
  }
}
```

**단축 문법 (command)**:
```json
{
  "run_tests": {
    "run": "npm test",
    "on": { "success": "next", "failed": "error" }
  }
}
```

**단축 문법 매핑**:
| 단축 키 | 노드 타입 | 예시 |
|---------|-----------|------|
| `run` | command | `"run": "npm test"` |
| `agent` | agent | `"agent": "senior-sw-engineer"` |
| `read` | tool (read) | `"read": "${file_path}"` |
| `write` | tool (write) | `"write": { "path": "...", "content": "..." }` |
| `wait` | delay | `"wait": 5000` |
| `if` | conditional | `"if": { "status": "approved" }` |

---

### Decision 5: 변수 참조 문법 확장

**결정**: `${key}` 문법에 다양한 참조 패턴 지원

**지원 패턴**:
```
${prompt}                      # 초기 프롬프트
${custom_key}                  # 사용자 정의 변수
${history.nodeName}            # 노드 결과 메시지 (단축)
${history.nodeName.message}    # 노드 결과 메시지 (명시적)
${history.nodeName.data.field} # 노드 결과 데이터의 특정 필드
${_current_state}              # 현재 노드 이름
${_instance_id}                # 인스턴스 ID
${env.VAR_NAME}                # 환경 변수 (선택적)
```

---

### Decision 6: 파일 기반 영속화

**결정**: 각 인스턴스 상태를 개별 JSON 파일로 저장

**저장 위치**: `~/.config/opencode/data/flows/instances/<instance-id>.json`

**저장 시점**:
- 모든 Blackboard 쓰기 연산 시 즉시 저장 (crash recovery)
- 노드 전이 시 저장
- 인스턴스 완료/실패 시 선택적 보관

**파일 구조**:
```json
{
  "_instance_id": "abc123",
  "_flow_name": "code-review",
  "_current_state": "review",
  "_started_at": "2026-01-26T10:00:00Z",
  "_session_id": "sess_xyz",
  "_execution_order": ["analyze", "gather_context", "analyze"],
  "_results": {
    "analyze": {
      "result": { "name": "success", "message": "..." },
      "timestamp": "2026-01-26T10:01:00Z",
      "executionCount": 2
    }
  },
  "prompt": "src/ 폴더 리뷰해주세요",
  "custom_var": "value"
}
```

---

## Data Model

### Flow Definition (개선된 스키마)

```typescript
interface FlowDefinition {
  name: string;              // 플로우 식별자
  version: string;           // 시맨틱 버전
  description?: string;      // 설명
  
  config?: {
    timeout?: number;        // 노드 타임아웃 (ms, 기본 300000)
    max_retries?: number;    // 최대 재시도 (기본 3)
    retry_delay?: number;    // 재시도 간격 (ms, 기본 1000)
  };
  
  start: string;             // 시작 노드 (initial_state 대신)
  
  nodes: {
    [name: string]: NodeDefinition;
  };
}

// 개선된 노드 정의
interface NodeDefinition {
  // 기본 필드
  type?: NodeType;           // 명시적 타입 (단축 문법 사용 시 생략 가능)
  on?: RouteMap;             // 결과별 라우팅
  
  // Agent 노드
  agent?: string;            // 에이전트 타입
  prompt?: string;           // 프롬프트
  results?: ResultMap;       // 가능한 결과
  
  // Command 노드
  run?: string;              // 실행할 명령어
  workdir?: string;          // 작업 디렉토리
  expect?: number;           // 기대 종료 코드
  
  // Tool 노드
  read?: string;             // 파일 읽기
  write?: WriteConfig;       // 파일 쓰기
  glob?: string;             // 패턴 매칭
  
  // Control 노드
  if?: ConditionMap;         // 조건 분기
  wait?: number;             // 대기 시간 (ms)
  loop?: LoopConfig;         // 반복
  
  // End 노드
  end?: EndConfig | boolean; // 종료 (true면 success)
}

type RouteMap = { [result: string]: string | null };
type ResultMap = { [name: string]: string }; // name: description
type ConditionMap = { [field: string]: any }; // field: expectedValue
```

### Blackboard Data (개선)

```typescript
interface BlackboardData {
  // 시스템 필드 (언더스코어 접두사)
  _instance_id: string;
  _flow_name: string;
  _current_state: string;
  _started_at: string;
  _session_id?: string;
  _final_status?: 'success' | 'failed';
  _final_message?: string;
  
  // 노드 결과 저장 (노드 이름 = 키)
  _results: {
    [nodeName: string]: {
      result: NodeResult;
      timestamp: string;
      executionCount: number;
    };
  };
  
  // 실행 순서 (디버깅용)
  _execution_order: string[];
  
  // 초기 입력
  prompt?: string;
  
  // 사용자 정의 변수
  [key: string]: any;
}
```

---

## Risks / Trade-offs

### Risk 1: 세션 컨텍스트 길이 초과
- **위험**: 장시간 플로우에서 세션 컨텍스트가 초과될 수 있음
- **완화**: 
  - 에이전트 노드는 필요한 히스토리만 선택적으로 주입
  - 컨텍스트 요약 메커니즘 도입 (향후)

### Risk 2: 동시 실행 경합
- **위험**: 같은 파일을 수정하는 플로우들 간 충돌
- **완화**:
  - 사용자에게 경고 표시
  - 향후 락 메커니즘 도입 고려

### Risk 3: 무한 루프
- **위험**: 잘못된 라우팅으로 무한 루프 발생
- **완화**:
  - 글로벌 타임아웃 적용
  - 최대 상태 전이 횟수 제한 (기본 1000)
  - 루프 노드의 max_iterations 강제

### Trade-off: 동기 vs 비동기 실행
- **선택**: 비동기 (Tick 기반)
- **장점**: 다중 인스턴스 지원, 응답성 유지
- **단점**: 약간의 지연 (최대 500ms)

---

## Migration Plan

### Phase 1: Core (Week 1)
- FlowManager 싱글톤 구현
- FlowInstance FSM 구현
- Blackboard 영속화 구현
- 개선된 스키마 파서 구현

### Phase 2: Basic Nodes (Week 1-2)
- AgentNode (arbitrary results 포함)
- CommandNode
- ToolNode (read, write, glob, bash)
- EndNode

### Phase 3: Control Nodes (Week 2)
- ConditionalNode
- LoopNode
- DelayNode

### Phase 4: Plugin Integration (Week 2-3)
- /flow 커맨드 구현
- 토스트 알림 연동
- 에러 핸들링 및 재시도

### Phase 5: Testing & Polish (Week 3)
- 통합 테스트
- 예제 워크플로우 검증
- 문서화

---

## Open Questions

1. **서브플로우 구현 범위**: v1에서 sub-flow 노드를 지원할 것인가?
   - 현재 결정: v1에서는 기본 구현만, 고급 기능은 v2로 연기

2. **병렬 노드 동기화**: parallel 노드에서 일부만 실패했을 때 처리 방식?
   - 현재 결정: any-fail (하나라도 실패하면 전체 실패)
   - 향후: `strategy: all-success | any-success | partial` 옵션 추가

3. **사용자 입력 대기**: 플로우 중간에 사용자 입력을 받을 수 있는가?
   - 현재 결정: v1에서는 미지원
   - 향후: `prompt` 노드 타입 추가 고려
