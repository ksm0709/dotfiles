# OpenCode Skill 구조 Reference

> **참고**: 이 문서는 정적 참조용입니다. 최신 정보는 다음 URL에서 확인하세요:
> - https://opencode.ai/docs/skills/

## Skill 디렉토리 구조

```
.opencode/skills/my-skill/
├── SKILL.md              # 필수: 메인 스킬 파일
├── scripts/              # 선택: 실행 스크립트
│   ├── setup.sh
│   ├── validate.sh
│   └── helper.py
├── references/           # 선택: 참조 문서
│   ├── guide.md
│   └── checklist.md
└── assets/               # 선택: 기타 파일
    ├── logo.png
    └── template.json
```

## SKILL.md Frontmatter

```yaml
---
name: my-skill                    # 필수, 디렉토리명과 일치
description: 스킬 설명 (20자 이상)  # 필수, 검색/발견에 사용
license: MIT                      # 선택, 라이선스
allowed-tools:                    # 선택, 허용할 도구 목록
  - read
  - write
  - bash
  - edit
  - glob
  - grep
  - webfetch
metadata:                         # 선택, 사용자 정의 메타데이터
  version: "1.0.0"
  author: "Your Name"
  tags: ["tag1", "tag2"]
---
```

## 네이밍 규칙

| 항목 | 규칙 | 예시 | 결과 도구명 |
|------|------|------|-------------|
| 디렉토리 | 소문자, 하이픈 사용 | `my-skill` | - |
| name | 디렉토리명과 정확히 일치 | `my-skill` | - |
| 중첩 디렉토리 | 하위 디렉토리 가능 | `tools/analyzer` | `skills_tools_analyzer` |

## Skill 등록 프로세스

1. **디렉토리 생성**: `.opencode/skills/my-skill/`
2. **SKILL.md 작성**: Frontmatter + Markdown 내용
3. **OpenCode 재시작**: 자동으로 스킬 등록
4. **사용**: `skills_my_skill`로 호출

## Skill 실행 시 컨텍스트

Skill이 실행될 때 에이전트는 다음 정보를 받습니다:

```
Base directory for this skill: /path/to/.opencode/skills/my-skill/

[SKILL.md의 Markdown 내용]
```

## 지원 파일 참조

Skill 내용에서 상대 경로로 지원 파일을 참조할 수 있습니다:

```markdown
## 사용 방법

1. 체크리스트 확인: `references/checklist.md`
2. 검증 실행: `scripts/validate.sh`
3. 템플릿 사용: `assets/template.json`
```

## Allowed Tools

일반적으로 Skill에서 사용되는 도구들:

- `read` - 파일 읽기
- `write` - 파일 쓰기
- `edit` - 파일 수정
- `bash` - 명령어 실행
- `glob` - 파일 검색
- `grep` - 내용 검색
- `webfetch` - 웹 콘텐츠 가져오기

## Skill 예제

### 기본 Skill

```markdown
---
name: code-reviewer
description: 코드 리뷰를 수행하는 스킬. 품질, 보안, 성능을 검토합니다.
license: MIT
allowed-tools:
  - read
  - grep
---

# 코드 리뷰어

코드 변경사항을 검토하고 개선점을 제안합니다.

## 검토 항목

- [ ] 코드 품질
- [ ] 보안 취약점
- [ ] 성능 병목
- [ ] 테스트 커버리지

## 출력 형식

각 이슈에 대해 다음 형식으로 보고:
- 심각도: High/Medium/Low
- 위치: 파일 경로 및 라인
- 설명: 문제 설명
- 제안: 개선 방안
```

### 고급 Skill (지원 파일 포함)

```markdown
---
name: deployment-helper
description: 프로덕션 배포 자동화 스킬. 검증, 배포, 롤백을 지원합니다.
license: MIT
allowed-tools:
  - read
  - write
  - bash
metadata:
  version: "2.0"
  team: "DevOps"
---

# 배포 도우미

프로덕션 배포를 안전하게 수행합니다.

## 워크플로우

1. `references/checklist.md` 읽기
2. `scripts/validate.sh` 실행
3. `scripts/deploy.sh` 실행
4. 배포 상태 확인
5. 문제 발생 시 `scripts/rollback.sh` 실행

## 주의사항

- 모든 스크립트는 프로젝트 루트에서 실행
- `references/troubleshooting.md` 참조
```

## 문제 해결

### Skill이 등록되지 않음
- OpenCode 재시작
- name이 디렉토리명과 일치하는지 확인
- description이 20자 이상인지 확인
- YAML frontmatter 문법 확인

### Skill이 보이지 않음
- `allowed-tools`에 필요한 도구가 포함되어 있는지 확인
- 글로벌 스킬 경로 확인: `~/.config/opencode/skills/`
- 프로젝트 스킬 경로 확인: `.opencode/skills/`

## Context7에서 Skill 예제 조회

```
libraryId: /malhashemi/opencode-skills
query: skill structure, SKILL.md examples, frontmatter schema
```
