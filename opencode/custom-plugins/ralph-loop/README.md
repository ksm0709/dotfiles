# Ralph Loop Plugin for OpenCode

오픈코드(OpenCode) 에이전트의 장기 작업을 자동화하고 컨텍스트를 효율적으로 관리하기 위한 루프 제어 플러그인입니다.

## 주요 기능

- **자동 프롬프트 주입**: 세션 시작 시 에이전트에게 작업 완료 지침(Promise Word 출력)을 자동으로 전달합니다.
- **Idle 상태 감지**: 에이전트가 작업을 마쳤을 때 약속된 단어(Promise Word)가 포함되어 있는지 확인합니다.
- **자동 요약 및 세션 재시작**: 작업이 미완료된 경우(Promise Word 부재 시), 현재까지의 진행 상황을 요약하여 저장하고 세션을 초기화하여 작업을 이어서 수행합니다.
- **무한 루프 방지**: 최대 반복 횟수(`maxRetries`)를 설정하여 안전하게 작업을 제어합니다.

## 설치 방법

제공된 `install.sh` 스크립트를 사용하면 오픈코드 표준 설정 디렉토리에 자동으로 설치됩니다.

```bash
cd opencode/plugin/ralph-loop
./install.sh
```

### 설치 위치 (OpenCode 공식 문서 기준)

| 항목 | 경로 |
| :--- | :--- |
| **플러그인 파일** | `~/.config/opencode/plugins/ralph-loop.ts` |
| **의존성** | `~/.config/opencode/package.json` |
| **설정 파일** | `~/.config/opencode/ralph-loop.json` |

> **참고**: OpenCode는 `~/.config/opencode/plugins/` 디렉토리(복수형)에서 `.ts`, `.js` 파일을 자동으로 로드합니다.

설치 후 별도의 `opencode.json` 수정 없이 오픈코드를 실행하면 플러그인이 자동으로 로드됩니다.

## 설정 방법

플러그인은 **`ralph-loop.json`** 파일을 통해 설정을 관리합니다. 설정 파일은 다음 우선순위로 로드됩니다:

1.  **프로젝트별 설정**: `{project-dir}/.opencode/ralph-loop.json`
2.  **사용자 전역 설정**: `~/.config/opencode/ralph-loop.json`

### 설정 예시 (`ralph-loop.json`)

```json
{
  "promiseWord": "DONE",
  "maxRetries": 5,
  "summaryPath": "./.opencode/sessions/",
  "autoRestart": true
}
```

### 설정 항목 설명

| 항목 | 기본값 | 설명 |
| :--- | :--- | :--- |
| `promiseWord` | `"DONE"` | 작업 완료를 판단하는 키워드입니다. |
| `maxRetries` | `5` | 최대 자동 반복 횟수입니다. |
| `summaryPath` | `"./.opencode/sessions/"` | 요약 파일이 저장될 기본 경로입니다. |
| `autoRestart` | `true` | 루프 발생 시 자동으로 세션을 재시작할지 여부입니다. |

## 사용 방법

1. 오픈코드를 실행하고 작업을 시작합니다.
2. 첫 번째 메시지를 보내면 플러그인이 자동으로 "작업 완료 시 `promiseWord`를 출력하라"는 지시사항을 추가합니다.
3. 에이전트가 작업을 수행하다가 `idle` 상태가 되었을 때, 메시지에 `promiseWord`가 없으면 플러그인이 개입합니다.
4. 플러그인은 현재 세션을 요약하여 `.opencode/sessions/(session-id)/ralph_summary.md`에 저장하고, 새 세션을 생성하여 요약본과 함께 작업을 계속하도록 지시합니다.

## 개발 및 테스트

```bash
# 단위 테스트 실행
npm test

# CI 파이프라인 실행 (포맷팅, 린팅, 테스트)
npm run ci
```

## 트러블슈팅

### 플러그인이 로드되지 않는 경우

1. **디렉토리 확인**: `~/.config/opencode/plugins/` (복수형)에 `ralph-loop.ts`가 있는지 확인
   ```bash
   ls -la ~/.config/opencode/plugins/
   ```

2. **의존성 확인**: `~/.config/opencode/package.json`에 `zod` 의존성이 있는지 확인
   ```bash
   cat ~/.config/opencode/package.json
   ```

3. **기존 설치 정리**: 이전 버전이 잘못된 위치에 설치되었다면 재설치
   ```bash
   rm -rf ~/.config/opencode/plugin/ralph-loop  # 구버전 정리
   ./install.sh  # 재설치
   ```

## 라이선스

ISC
