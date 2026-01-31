<!-- TASKS_TOOLS_GUIDE_START -->
<!-- 이 섹션은 Tasks Plugin에 의해 자동으로 관리됩니다. 수동으로 수정하지 마세요. -->

### 📋 Task Management Tools

Tasks Plugin은 에이전트가 작업을 체계적으로 관리할 수 있도록 **통합 `tasks` 도구**를 제공합니다.

#### 사용 가능한 도구

**통합 도구:**
- **`tasks(operations)`** - 단일 도구로 모든 작업 관리
  - 반환값: `{ title, output, metadata }`
  - metadata: { results, summary, taskList, operation }
  - 항상 현재 세션의 작업 현황 자동 표시
  - 세션 격리: 다른 세션의 작업은 보이지 않음

**사용 예시:**
```typescript
// 작업 목록 초기화
tasks({
  operations: [
    { type: 'init', agent: 'senior-sw-engineer', title: 'API-구현' }
  ]
})

// 배치 작업으로 여러 작업 한 번에 처리
tasks({
  operations: [
    { type: 'add', title: '요구사항 분석' },
    { type: 'add', title: '설계' },
    { type: 'add', title: '구현', parent: '2' },
    { type: 'update', id: '1', status: 'in_progress' },
    { type: 'complete', id: '2' }
  ]
})
```

**Operation 타입:**
- `init`: 작업 목록 초기화 (필수: agent, title)
- `add`: 작업 추가 (필수: title, 선택: parent)
- `update`: 상태 업데이트 (필수: id, status)
- `complete`: 작업 완료 (필수: id)
- `remove`: 작업 제거 (필수: id)

**주요 특징:**
- 단일 `tasks` 도구로 모든 작업 관리
- 한 번에 최대 50개 operation 처리 (배치 작업)
- 부분적 실패 허용: 일부 실패 시 성공한 작업은 유지
- 세션 격리: 현재 세션의 작업만 표시
- 자동 현황 표시: 모든 작업 후 현재 세션 상태 자동 출력
- OpenCode 네이티브 UI 렌더링 지원
- 작업 파일은 `~/.local/share/opencode/tasks/{session-id}/`에 저장

**설치 구조:**
- 플러그인 코드: `~/.config/opencode/plugins/tasks/`
- 문서 및 가이드: `~/.config/opencode/shared/tasks/`

#### 에이전트 설정

에이전트가 Tasks 도구를 사용하려면 frontmatter에 다음을 추가하세요:

```yaml
---
tools:
  tasks: true
---
```

#### 자세한 사용법

자세한 사용법은 다음 문서를 참조하세요:
- `~/.config/opencode/shared/tasks/README.md`
- `~/.config/opencode/shared/tasks/docs/MIGRATION-v2-to-v3.md`

<!-- TASKS_TOOLS_GUIDE_END -->
