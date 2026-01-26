# OpenCode Flows Plugin - 설계 문서

> OpenCode를 위한 멀티 에이전트 워크플로우 관리 플러그인

## 1. 개요 (Overview)

### 1.1 목적
OpenCode를 위한 **멀티 에이전트 워크플로우 관리 플러그인**입니다. JSON 기반의 FSM(Finite State Machine)을 정의하여 복잡한 멀티스텝 작업을 자동화합니다.

### 1.2 핵심 컨셉
```
┌─────────────────────────────────────────────────────────────┐
│                    Flows Plugin                              │
├─────────────────────────────────────────────────────────────┤
│  /flow <flow-name> [prompt]                                 │
│       ↓                                                      │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ FlowManager (Singleton)                             │    │
│  │  - 500ms tick loop                                  │    │
│  │  - 다중 인스턴스 관리                               │    │
│  │  - 상태 영속화                                      │    │
│  └─────────────────────────────────────────────────────┘    │
│       ↓                                                      │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ FlowInstance (per flow execution)                   │    │
│  │  - FSM State Machine                                │    │
│  │  - Blackboard (local memory)                        │    │
│  │  - Session (per flow)                               │    │
│  │  - Node History                                     │    │
│  └─────────────────────────────────────────────────────┘    │
│       ↓                                                      │
│  ┌─────────────────────────────────────────────────────┐    │
│  │ Nodes (실행 단위)                                   │    │
│  │  - agent, tool, command, skill                      │    │
│  │  - conditional, parallel, delay                     │    │
│  │  - loop, retry, sub-flow, end                       │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 핵심 설계 결정 (Design Decisions)

| 항목 | 결정 | 비고 |
|------|------|------|
| **Tick 주기** | 고정 500ms | 예측 가능한 실행 |
| **에러 핸들링** | 재시도 + 타임아웃 | 설정 가능한 max_retries |
| **Blackboard** | 파일 기반 영속화 | JSON 저장, 재시작 가능 |
| **인스턴스** | 다중 인스턴스 지원 | 같은 플로우 동시 실행 가능 |
| **Arbitrary Results** | JSON에 사전 정의 | 명시적 라우팅 |
| **타임아웃** | 글로벌 설정 | 기본 5분, 설정 변경 가능 |
| **컨텍스트 전달** | 노드 히스토리 기반 | 이전 N개 결과 주입 |
| **중단** | 수동 중단 (명령어) | `/flow stop <id>` |
| **세션** | 플로우당 단일 세션 | 컨텍스트 유지 |
| **노드 라이브러리** | 글로벌 라이브러리 | 중앙집중 관리 |
| **트리거** | `/flow` 커맨드 | 명시적 시작 |
| **종료** | 명시적 `end` 노드 | 무한 루프 방지 |
| **변수 문법** | `${key}` | blackboard 참조 |

---

## 3. 디렉토리 구조

```
~/.config/opencode/                # 글로벌 설정
├── plugins/
│   └── flows/                     # Flows 플러그인
│       ├── src/
│       │   ├── index.ts           # 플러그인 엔트리포인트
│       │   ├── manager/
│       │   │   ├── FlowManager.ts     # 싱글톤 매니저
│       │   │   ├── FlowInstance.ts    # 개별 인스턴스
│       │   │   └── Blackboard.ts      # 상태 저장소
│       │   ├── nodes/
│       │   │   ├── BaseNode.ts        # 추상 노드
│       │   │   ├── AgentNode.ts       # 에이전트 실행
│       │   │   ├── ToolNode.ts        # 도구 호출
│       │   │   ├── CommandNode.ts     # 명령 실행
│       │   │   ├── SkillNode.ts       # 스킬 실행
│       │   │   ├── ConditionalNode.ts # 분기
│       │   │   ├── ParallelNode.ts    # 병렬 실행
│       │   │   ├── DelayNode.ts       # 대기
│       │   │   ├── LoopNode.ts        # 반복
│       │   │   ├── RetryNode.ts       # 재시도 래퍼
│       │   │   ├── SubFlowNode.ts     # 서브플로우
│       │   │   └── EndNode.ts         # 종료
│       │   ├── utils/
│       │   │   ├── VariableResolver.ts # ${key} 해석
│       │   │   └── HistoryInjector.ts  # 히스토리 주입
│       │   └── types/
│       │       └── schemas.ts          # Zod 스키마
│       ├── package.json
│       └── tsconfig.json
│
├── shared/
│   └── flows/                     # 글로벌 노드 라이브러리
│       └── nodes/                 # 사용자 정의 노드
│           └── custom-node.ts
│
└── data/
    └── flows/                     # 런타임 데이터
        └── instances/             # 인스턴스 상태 저장
            └── <instance-id>.json

(project)/.opencode/flows/         # 프로젝트별 플로우 정의
├── code-review.json               # 예: 코드 리뷰 워크플로우
├── tdd-cycle.json                 # 예: TDD 사이클
└── deploy.json                    # 예: 배포 워크플로우
```

---

## 4. 커맨드 사용법

```bash
# 플로우 시작
/flow start <flow-name> [prompt]
예: /flow start code-review "src/ 폴더의 모든 Python 파일을 리뷰해주세요"

# 상태 확인
/flow status [instance-id]

# 플로우 중단
/flow stop <instance-id>

# 사용 가능한 플로우 목록
/flow list
```

---

## 5. 다음 단계

1. **Phase 1 - Core**: FlowManager, FlowInstance, Blackboard 구현
2. **Phase 2 - Basic Nodes**: AgentNode, ToolNode, EndNode 구현
3. **Phase 3 - Control Nodes**: ConditionalNode, ParallelNode, LoopNode 구현
4. **Phase 4 - Commands**: /flow 커맨드 및 모니터링 구현
5. **Phase 5 - Testing**: 통합 테스트 및 예제 워크플로우 검증
