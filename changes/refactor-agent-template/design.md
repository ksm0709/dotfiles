# Design: 에이전트 템플릿 리팩토링 설계

## 1. 파일 위치 변경
- **Source**: `opencode/config/agent/agent-template.md`
- **Destination**: `opencode/config/agent-template.md`
- **이유**: `install_agents.sh`가 `config/agent/*.md`를 복사하므로, 상위 디렉토리로 옮기면 배포 대상에서 자연스럽게 제외됨.

## 2. 템플릿 내용 수정 (`agent-template.md`)

### Frontmatter 및 제목
- `description: [에이전트의 간단한 설명]` -> `description: 에이전트의 간단한 설명` (대괄호 제거)
- `# Role: [에이전트 이름]` -> `# Role: 에이전트 이름` (대괄호 제거)

### 섹션 추가: 가이드라인 (Guidelines)
`Reference` 섹션 앞 또는 뒤에 다음 구조 추가:

```markdown
## 가이드라인 (Guidelines)

### Boundary
- **Must**: 반드시 수행해야 하는 작업이나 규칙.
- **Never**: 절대 수행하지 말아야 하는 작업이나 안티패턴.

### Commands
- **Preferred Tools**: 우선적으로 사용해야 할 도구.
- **Restricted Commands**: 사용이 제한된 명령어.

### Conventions
- **Code Style**: 따를 코딩 컨벤션 (PEP 8, Google Style 등).
- **Documentation**: 문서 작성 형식.
```

## 3. 에이전트 크리에이터 수정 (`agent-creator.md`)
- 워크플로우 내 템플릿 읽기 단계의 경로를 `opencode/config/agent/agent-template.md`에서 `opencode/config/agent-template.md`로 수정.

## ADDED Requirements
- 템플릿 파일 내의 모든 예시용 대괄호(`[ ]`)는 제거하거나 다른 기호(예: `< >`)로 대체해야 한다.
- `Guidelines` 섹션은 `Boundary`, `Commands`, `Conventions` 하위 섹션을 반드시 포함해야 한다.

#### Scenario: 템플릿 이동 및 정리 확인
1. `git mv`로 파일 이동.
2. `install_agents.sh` 실행 시 `agent-template.md`가 복사되지 않음 확인.
3. 기존에 설치된 `~/.config/opencode/agent/agent-template.md`가 삭제됨 확인.
