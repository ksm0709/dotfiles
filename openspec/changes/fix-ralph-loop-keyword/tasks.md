## 1. Specification
- [x] 1.1 이벤트 타입 및 입력 추출 기준 확정 (message.updated + message.part.updated)
- [x] 1.2 키워드 감지 범위(부분 일치/단어 경계) 확정 (부분 일치, 사용자 메시지만)

## 2. Testing
- [ ] 2.1 시나리오별 테스트 추가/수정 (키워드 감지, 비감지, 활성화 유지)
- [ ] 2.2 실패 테스트(Red) 확인

## 3. Implementation
- [ ] 3.1 플러그인 이벤트 핸들러를 실제 이벤트로 변경 (message.updated + message.part.updated)
- [ ] 3.2 메시지 텍스트 추출 로직 반영 (user role만)
- [ ] 3.3 중복 이벤트 대비 디-듀핑 처리
- [ ] 3.4 기존 동작(활성화 후 session.idle 루프) 유지

## 4. Verification
- [ ] 4.1 테스트 통과 확인
- [ ] 4.2 린트/포맷 확인
