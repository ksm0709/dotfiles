# Tasks: 템플릿 리팩토링 및 적용

## 1. 파일 이동 및 수정
- [ ] `git mv opencode/config/agent/agent-template.md opencode/config/agent-template.md`
- [ ] `opencode/config/agent-template.md` 내용 수정 (대괄호 제거, Guideline 추가)
- [ ] `opencode/config/agent/agent-creator.md` 경로 참조 수정

## 2. 배포 및 정리
- [ ] `opencode/install_agents.sh` 실행 (업데이트 반영)
- [ ] `rm ~/.config/opencode/agent/agent-template.md` (잘못된 파일 제거)

## 3. 검증
- [ ] `ls ~/.config/opencode/agent/` 확인 (템플릿 파일 없어야 함)
- [ ] `agent-creator` 워크플로우 확인 (새 경로 인식 여부 - 코드 리뷰로 확인)
