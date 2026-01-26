# OpenCode Flows Plugin

OpenCode를 위한 멀티 에이전트 워크플로우 관리 플러그인입니다. JSON 기반의 FSM(Finite State Machine)을 정의하여 복잡한 멀티스텝 작업(코드 리뷰, TDD 사이클 등)을 자동화할 수 있습니다.

## ✨ 주요 기능

- **JSON 기반 워크플로우 정의**: 복잡한 로직을 선언적인 JSON 파일로 정의
- **멀티 에이전트 오케스트레이션**: 여러 전문 에이전트(`general`, `senior-sw-engineer` 등)를 조율
- **상태 관리 및 복구**: 실행 상태를 파일에 저장하여 충돌 시에도 복구 가능 (Blackboard 패턴)
- **강력한 제어 흐름**: 조건 분기, 반복문, 지연, 재시도 등 지원
- **단축 문법 지원**: 간결하고 직관적인 스키마 제공

## 🚀 설치 및 설정

이 플러그인은 OpenCode 환경에 포함되어 있습니다. 별도의 설치가 필요하지 않습니다.

## 📖 사용법

### 1. 플로우 정의하기

`.opencode/flows/` 디렉토리에 JSON 파일을 생성합니다. (예: `my-task.json`)

```json
{
  "name": "my-task",
  "version": "1.0.0",
  "start": "step1",
  "nodes": {
    "step1": {
      "agent": "general",
      "prompt": "안녕하세요! 간단한 인사를 해주세요.",
      "on": {
        "success": "step2"
      }
    },
    "step2": {
      "run": "echo '작업 완료'",
      "on": {
        "success": "done"
      }
    },
    "done": {
      "end": true,
      "message": "모든 작업이 완료되었습니다."
    }
  }
}
```

### 2. 플로우 실행하기

OpenCode 채팅창에서 다음 명령어를 입력합니다:

```bash
/flow start my-task
```

프롬프트를 전달하려면:

```bash
/flow start my-task "추가 지시사항입니다"
```

### 3. 상태 확인 및 제어

- **목록 조회**: `/flow list`
- **상태 확인**: `/flow status` (또는 `/flow status <instance-id>`)
- **중단**: `/flow stop <instance-id>`

## 📝 스키마 레퍼런스 (단축 문법)

### 노드 타입

#### 1. Agent Node (에이전트)
OpenCode 에이전트를 호출합니다.

```json
"node_name": {
  "agent": "general", // agent_type (general, senior-sw-engineer, ...)
  "prompt": "프롬프트 내용 (${variable} 사용 가능)",
  "results": {
    "success": "성공 시 설명",
    "failed": "실패 시 설명"
  },
  "on": {
    "success": "next_node",
    "failed": "error_node"
  }
}
```

#### 2. Command Node (명령어)
쉘 명령어를 실행합니다.

```json
"node_name": {
  "run": "npm test",
  "workdir": "./src", // 선택사항
  "expect_exit_code": 0, // 선택사항 (기본값: 0)
  "on": {
    "success": "next_node",
    "failed": "error_node"
  }
}
```

#### 3. Tool Node (도구)
OpenCode 도구를 직접 호출합니다.

```json
"node_name": {
  "tool": "read",
  "args": { "filePath": "README.md" },
  "on": {
    "success": "next_node"
  }
}
```

#### 4. Conditional Node (조건 분기)
변수 값에 따라 분기합니다.

```json
"node_name": {
  "conditions": [
    {
      "field": "history.analyze.data.score",
      "operator": "gt", // eq, ne, gt, lt, contains, exists
      "value": 80,
      "result": "high_score"
    }
  ],
  "default": "low_score",
  "on": {
    "high_score": "pass_node",
    "low_score": "fail_node"
  }
}
```

#### 5. Loop Node (반복)
특정 노드를 반복 실행합니다.

```json
"node_name": {
  "loop": "target_node", // 반복할 노드 이름 (보통 자기 자신이나 이전 노드)
  "max": 5, // 최대 반복 횟수
  "while": { // 선택사항: 반복 조건
    "field": "retry_count",
    "operator": "lt",
    "value": 5
  }
}
```

#### 6. Delay Node (지연)
일정 시간 대기합니다.

```json
"node_name": {
  "wait": 1000, // 밀리초
  "on": { "success": "next_node" }
}
```

#### 7. End Node (종료)
플로우를 종료합니다.

```json
"node_name": {
  "end": true,
  "status": "success", // success 또는 failed
  "message": "종료 메시지"
}
```

### 변수 사용

- `${key}`: Blackboard에 저장된 변수
- `${prompt}`: 초기 입력 프롬프트
- `${history.nodeName}`: 특정 노드의 결과 메시지
- `${history.nodeName.data.field}`: 특정 노드가 추출한 데이터 필드

## 📂 예제

`docs/examples/` 디렉토리에서 다양한 예제를 확인할 수 있습니다.
- `simple-task.json`: 기본 구조
- `code-review.json`: 에이전트 간 협업 및 데이터 전달
- `tdd-cycle.json`: 복잡한 루프 및 조건 분기

## 라이선스

MIT
