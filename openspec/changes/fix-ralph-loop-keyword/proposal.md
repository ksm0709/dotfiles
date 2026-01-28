# Change: Fix ralph-loop keyword activation

## Why
ralph-loop 플러그인이 키워드에 반응하지 않아 자동 재시작 루프가 동작하지 않습니다. 로그 분석 결과 이벤트 구독 불일치가 원인으로 확인되어, 키워드 감지 로직이 실행되지 않는 상태입니다.

## What Changes
- ralph-loop 플러그인이 실제 발생하는 메시지 이벤트에 반응하도록 이벤트 핸들러를 조정합니다.
- 키워드 감지 입력(source) 추출 방식을 명확히 정의합니다.
- 키워드 감지/활성화 동작을 테스트 시나리오로 명시합니다.

## Impact
- Affected specs: ralph-loop
- Affected code: opencode/custom-plugins/ralph-loop/src/index.ts, tests/ralph-loop.test.ts
