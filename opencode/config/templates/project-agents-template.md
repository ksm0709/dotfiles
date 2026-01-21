# [Project Name] Agent Configuration (AGENTS.md)

이 파일은 **[Project Name]** 프로젝트의 AI 에이전트 설정을 정의합니다. 프로젝트의 문맥, 구조, 규칙, 워크플로우를 명시하여 에이전트가 일관되고 안전하게 작업을 수행하도록 돕습니다.

---

## 1. Project Context (프로젝트 문맥)

### Overview (개요)
- **Description**: [프로젝트의 목적과 주요 기능을 한 문장으로 요약]
- **Goals**: [프로젝트의 핵심 목표]
- **Architecture**: [시스템 아키텍처 요약 (예: MSA, Monolithic, Serverless)]
- **Domain Knowledge**: [프로젝트 이해에 필요한 도메인 지식]

### Tech Stack (기술 스택)
- **Language**: [주요 언어 및 버전]
- **Framework**: [주요 프레임워크]
- **Database**: [데이터베이스]
- **Infrastructure**: [인프라 및 배포 환경]

---

## 2. Folder Structure (폴더 구조)

이 프로젝트의 디렉토리 구조와 각 폴더의 역할입니다. 에이전트는 코드를 생성하거나 수정할 때 이 구조를 준수해야 합니다.

```
[Project Root]/
├── src/                   # 소스 코드
│   ├── api/               # API 엔드포인트
│   ├── core/              # 핵심 비즈니스 로직
│   └── utils/             # 유틸리티 함수
├── tests/                 # 테스트 코드
├── docs/                  # 문서
├── config/                # 설정 파일
├── .env.example           # 환경 변수 예시
├── AGENTS.md              # 이 파일
└── README.md              # 프로젝트 설명
```

---

## 3. Rules & Guidelines (규칙 및 가이드라인)

### Coding Standards (코딩 표준)
- **Style Guide**: [사용하는 스타일 가이드 (예: PEP 8, Google Style Guide)]
- **Naming Convention**: [변수, 함수, 클래스 명명 규칙 (예: snake_case, PascalCase)]
- **Formatting**: [사용하는 포매터 (예: Black, Prettier)]
- **Linting**: [사용하는 린터 (예: ESLint, Ruff)]

### Security (보안)
- **Authentication**: [인증 방식]
- **Input Validation**: 모든 사용자 입력은 검증되어야 합니다.
- **Secrets**: API 키나 비밀번호는 코드에 하드코딩하지 않고 환경 변수를 사용합니다.
- **No hardcoded secrets**: 코드 내에 비밀번호, API 키, 토큰 등을 직접 작성하지 않습니다.
- **Environment variables**: 민감한 데이터는 반드시 환경 변수로 관리합니다.
- **Validate all user inputs**: 모든 사용자 입력에 대해 유효성 검사를 수행합니다.
- **Parameterized queries only**: SQL 인젝션 방지를 위해 반드시 파라미터화된 쿼리를 사용합니다.

---

## 4. Guardrails (가드레일)

에이전트가 반드시 지켜야 할 안전 수칙과 제한 사항입니다.

- **Input Safety**: 프롬프트 인젝션 공격을 방지하기 위해 사용자 입력을 검증합니다.
- **Output Safety**: 개인정보(PII), 독성 콘텐츠, 보안에 민감한 정보를 출력하지 않습니다.
- **Hallucination**: 사실이 아닌 정보를 생성하지 않도록 주의하며, 불확실한 경우 사용자에게 확인합니다.
- **Destructive Actions**: 파일 삭제, 강제 푸시 등 파괴적인 작업은 사용자의 명시적 승인 없이 수행하지 않습니다.

---

## 5. Workflows (워크플로우)

### Development Workflow
1. **Task Analysis**: 요구사항 분석 및 계획 수립
2. **Implementation**: 코드 작성 (TDD 권장)
3. **Testing**: 단위 테스트 및 통합 테스트 수행
4. **Review**: 코드 리뷰 및 리팩토링

#⁹## Deployment Workflow
- [배포 절차 및 자동화 도구 설명]

---

## 6. Environment Variables (환경 변수)

프로젝트 실행에 필요한 환경 변수 목록입니다. (`.env` 파일 관리)

- `DATABASE_URL`: 데이터베이스 연결 문자열
- `API_KEY`: 외부 서비스 API 키
- `DEBUG`: 디버그 모드 활성화 여부

---

## 7. Agent Registry (에이전트 명부)

이 프로젝트에서 활용 가능한 전문 에이전트 목록입니다. (글로벌 에이전트 외 프로젝트 전용 에이전트만 기술)

### [Custom Agent Name]
- **Role**: [역할]
- **Description**: [설명]
- **Trigger**: [호출 시점]
