---
name: quick-research
description: |
  DuckDuckGo 기반 빠른 웹 검색 스킬.
  특정 사실 확인, 최신 뉴스 조회, 간단한 정보 수집에 사용.
  
  사용 시점:
  (1) 특정 사실을 빠르게 확인해야 할 때
  (2) 최신 뉴스나 이벤트를 조회할 때
  (3) 심층 분석 전 초기 정보 수집 시
  (4) 예: 슬래시 커맨드 /quick-search 사용 시
metadata:
  trigger_keywords:
    - quick search
    - fact check
---

# Research Skill (Quick Search)

DuckDuckGo 기반 빠른 웹 검색. API 키 불필요.

## 디렉토리 구조

```
quick-research/
├── SKILL.md           # 이 문서
└── scripts/
    ├── load_env.sh    # 환경 변수 로더
    ├── run.py         # 메인 실행 스크립트
    └── test_search.py # 테스트 코드
```

## 빠른 시작

```bash
# 스킬 디렉토리로 이동
cd ~/.config/opencode/skills/quick-research

# 테스트 실행
python scripts/test_search.py

# 검색 실행
python scripts/run.py "Python best practices"
```

## 환경 준비 (venv + 의존성)

런타임 의존성이 누락된 환경에서는 먼저 아래를 한 번 실행하세요.

```bash
# 스킬 디렉토리로 이동
cd ~/.config/opencode/skills/quick-research

# venv 생성 및 활성화
python -m venv .venv
source .venv/bin/activate

# 필수 패키지 설치
pip install -U ddgs duckduckgo-search

# 준비 상태 확인(네트워크 없는 빠른 체크)
python scripts/test_search.py --skip-network
```

선택 사항: 실제 네트워크 통합 테스트가 필요하면 `python scripts/test_search.py`를 실행하세요.

## 사용법

### CLI

```bash
# 예시: 슬래시 커맨드(legacy usage)
/quick-search "검색어"

# 스킬 스크립트 직접 실행
python ~/.config/opencode/skills/quick-research/scripts/run.py "검색어" [max_results]
```

## Activation hints

- "최신 AI 뉴스 빠르게 찾아줘"
- "이 사실 맞는지 quick search 해줘"

### Python API

```python
from scripts.run import search

results = search("machine learning trends", max_results=5)
for r in results:
    print(f"{r['title']}: {r['url']}")
```

## 출력 형식

```
🔍 Searching for: Python frameworks
Title: Top Python Web Frameworks in 2026
URL: https://example.com/article/...
Snippet: Django, FastAPI, and Flask continue to dominate...
----------------------------------------
```

## 의존성

```
ddgs>=9.0.0
duckduckgo-search>=3.0.0
```

호환성 메모: 기본 권장 패키지는 `ddgs`이며, 기존 환경 호환을 위해 `duckduckgo-search`는 legacy fallback으로만 유지합니다.

## 관련 스킬

- **deep-research**: 심층 분석이 필요한 경우
