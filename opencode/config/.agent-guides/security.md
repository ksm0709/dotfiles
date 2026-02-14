# Security

모든 구현/리팩터링/자동화 작업에 적용되는 보안 규칙입니다.

## MUST Rules

- **No hardcoded secrets**: 비밀번호, API 키, 토큰 등 민감 정보를 코드에 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.
- **Parameterized queries only**: SQL 인젝션 방지를 위해 파라미터화된 쿼리를 사용합니다.
