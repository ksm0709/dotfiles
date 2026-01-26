# Change: Develop Flows Plugin - Multi-Agent Workflow Management

## Why

OpenCode에서 복잡한 멀티스텝 작업을 수동으로 관리하는 것은 비효율적이며, 반복적인 워크플로우(코드 리뷰, TDD 사이클, 배포 등)를 자동화할 수 있는 메커니즘이 필요합니다. JSON 기반의 FSM(Finite State Machine)을 정의하여 에이전트, 도구, 명령어를 조합한 워크플로우를 선언적으로 구성하고 자동 실행할 수 있는 플러그인이 필요합니다.

## What Changes

### 핵심 변경사항
- **ADDED**: `flows` 플러그인 - 멀티 에이전트 워크플로우 관리 시스템
- **ADDED**: `/flow` 커맨드 - 워크플로우 시작, 중단, 상태 조회
- **ADDED**: JSON 기반 플로우 정의 스키마 - 선언적 워크플로우 구성
- **ADDED**: FSM 기반 실행 엔진 - 상태 전이 및 노드 실행 관리
- **ADDED**: Blackboard 시스템 - 노드 간 상태 공유 및 영속화

### 지원 노드 타입
- `agent`: OpenCode 에이전트 호출 (arbitrary results 지원)
- `tool`: 도구 실행 (read, write, glob, grep, bash 등)
- `command`: 쉘 명령어 실행
- `skill`: 스킬 실행
- `conditional`: 조건 분기
- `parallel`: 병렬 실행
- `delay`: 시간 대기
- `loop`: 반복 실행
- `retry`: 재시도 래퍼
- `sub-flow`: 서브플로우 호출
- `end`: 플로우 종료

## Impact

- **Affected specs**: 새로운 capability 추가 (`flows`)
- **Affected code**: 
  - `opencode/custom-plugins/flows/` - 플러그인 전체 구현
  - `~/.config/opencode/plugins/` - 플러그인 설치 위치
- **Dependencies**: `@opencode-ai/plugin`, `@opencode-ai/sdk`, `zod`, `uuid`, `glob`
