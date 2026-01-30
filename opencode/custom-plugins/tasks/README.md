# tasks - Task Management Plugin for OpenCode

OpenCode 에이전트를 위한 작업 관리 플러그인입니다. 서브에이전트가 체계적으로 작업을 관리할 수 있도록 지원합니다.

## 📋 기능

- ✅ 작업 목록 초기화 및 관리
- ✅ 작업 상태 추적 (pending → in_progress → completed)
- ✅ 작업 세부사항 저장 (짧은 문장 리스트)
- ✅ 하위 작업(subtask) 지원
- ✅ 다양한 출력 포맷 (markdown, json, table)
- ✅ 진행 상황 요약 및 완료율 계산

## 🚀 설치

### 1. 의존성 설치

```bash
cd ~/.config/opencode/custom-plugins/tasks
npm install
```

### 2. TypeScript 컴파일

```bash
npm run build
```

### 3. OpenCode 플러그인 등록

`~/.config/opencode/config.json`에 플러그인을 등록합니다:

```json
{
  "plugins": [
    {
      "name": "tasks",
      "path": "~/.config/opencode/custom-plugins/tasks"
    }
  ]
}
```

## 📖 사용법

### 작업 목록 초기화

```bash
tasks init \
  --agent senior-sw-engineer \
  --title "implement-auth" \
  --file ./tasks.md
```

### 작업 목록 조회

```bash
# 마크다운 형식
tasks list --agent senior-sw-engineer --format markdown

# JSON 형식
tasks list --agent senior-sw-engineer --format json

# 테이블 형식
tasks list --agent senior-sw-engineer --format table
```

### 작업 상태 업데이트

```bash
tasks update \
  --agent senior-sw-engineer \
  --id "2.1" \
  --status in_progress
```

### 작업 완료 처리

```bash
tasks complete \
  --agent senior-sw-engineer \
  --id "2.1"
```

### 작업 제거

```bash
# 확인 메시지 후 삭제
tasks remove --agent senior-sw-engineer --id "2.1"

# 강제 삭제 (확인 없음)
tasks remove --agent senior-sw-engineer --id "2.1" --force
```

### 진행 상황 요약

```bash
tasks status --agent senior-sw-engineer
```

### 새 작업 추가

```bash
# 세부사항과 함께 추가
tasks add-task \
  --agent senior-sw-engineer \
  --parent "2" \
  --title "에러 처리 구현" \
  --details "예외 케이스 정의,에러 로깅 구현,사용자 친화적 메시지"
```

## 📁 파일 저장 구조

```
~/.config/opencode/tasks/
├── senior-sw-engineer/
│   ├── implement-user-auth.md
│   └── fix-bug-123.md
├── py-code-reviewer/
│   ├── review-pr-456.md
│   └── review-pr-789.md
└── qa/
    ├── verify-login.md
    └── test-payment.md
```

## 📝 마크다운 파일 형식

```markdown
# Task List: implement-user-auth

**에이전트**: senior-sw-engineer  
**생성일**: 2026-01-30 14:30:00  
**세션 ID**: abc-123-def

---

## 작업 목록

- [x] 1. 요구사항 분석
  - 사용자 인증 요구사항 정의
  - 보안 표준 및 규제 준수 확인
- [ ] 2. 기능 구현
  - [x] 2.1 핵심 로직 개발
    - JWT 토큰 생성 및 검증 구현
    - 비밀번호 해싱 로직 구현
  - [ ] 2.2 테스트 코드 작성
    - 단위 테스트 커버리지 80% 이상
    - 통합 테스트 시나리오 작성

---

## 진행 상황 요약

**현재 단계**: 2. 기능 구현  
**상태**: in_progress  
**완료율**: 40% (2/5)
```

## 🔧 개발

### 빌드

```bash
npm run build
```

### 개발 모드 (watch)

```bash
npm run dev
```

### 클린

```bash
npm run clean
```

## 📄 라이선스

MIT
