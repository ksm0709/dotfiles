# Deep Research Skill 안정화 설계

## 아키텍처 개요

deep-research 스킬의 안정성을 개선하기 위한 아키텍처 설계입니다.

## 핵심 개선 사항

### 1. 환경 변수 로딩 계층 구조

```
우선순위 (높음 → 낮음):
1. 직접 설정된 환경 변수 (export GEMINI_API_KEY=...)
2. ~/.local/bin/env 스크립트
3. ~/.bashrc export 구문
4. ~/.config/opencode/.env 파일
5. 기본값/Mock 모드
```

### 2. 의존성 검증 시스템

- **런타임 검증**: 스크립트 실행 시 필요한 패키지 확인
- **그레이스풀 디그레이이션**: 패키지 부족 시 기능 제한 안내
- **자동 설치 제안**: missing 패키지에 대한 설치 명령어 제공

### 3. 에러 핸들링 전략

- **사용자 친화적 메시지**: 기술적 에러를 이해하기 쉽게 변환
- **해결 가이드**: 에러 발생 시 구체적인 해결 단계 제시
- **Fallback 모드**: API 키 부족 시 mock 모드로 동작

## 구현 상세

### 환경 변수 관리자 (EnvManager)

```python
class EnvManager:
    def load_api_keys(self) -> Dict[str, str]
    def validate_keys(self) -> Tuple[bool, List[str]]
    def get_preferred_provider(self) -> str
```

### 의존성 검증기 (DependencyChecker)

```python
class DependencyChecker:
    def check_python_packages(self) -> Dict[str, bool]
    def suggest_install_commands(self, missing: List[str]) -> str
    def verify_api_connectivity(self) -> Dict[str, bool]
```

### 에러 핸들러 (ErrorHandler)

```python
class ErrorHandler:
    def format_user_message(self, error: Exception) -> str
    def provide_resolution_guide(self, error_type: str) -> str
    def should_fallback(self, error: Exception) -> bool
```

## 트레이드오프

### 복잡성 vs 안정성
- **선택**: 안정성 우선
- **이유**: 스킬의 신뢰성이 사용자 경험에 직접적 영향
- **대가**: 약간의 코드 복잡성 증가

### 성능 vs 검증
- **선택**: 검증 우선 (startup 시간 약간 증가)
- **이유**: 런타임 실패보다 사전 검증이 더 효율적
- **대가**: 초기 로딩 시간 100-200ms 증가

## 호환성

- **기존 API**: 현재 사용 방식 유지
- **환경 변수**: 기존 변수명 그대로 지원
- **출력 형식**: 기존 출력 구조 유지

## 보안 고려사항

- API 키 노출 방지 (로그에서 마스킹)
- 환경 파일 권한 검증
- 민감 정보 메모리에서 적절히 정리