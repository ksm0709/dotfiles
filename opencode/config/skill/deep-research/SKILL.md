---
name: deep-research
description: |
  LLM 기반 심층 리서치 스킬.
  주제에 대해 자동으로 검색 계획을 수립하고, 웹 스크래핑 후 종합 리포트를 생성.
  
  사용 시점:
  (1) 특정 주제에 대한 포괄적인 분석이 필요할 때
  (2) 다양한 소스에서 정보를 수집하여 종합해야 할 때
  (3) 마크다운 형식의 상세 리포트가 필요할 때
  (4) 슬래시 커맨드 /deep-research 실행 시
  
  Prerequisites: GEMINI_API_KEY 또는 OPENAI_API_KEY 환경 변수 필요
---

# Deep Research Skill

LLM 기반 자동 심층 리서치. Plan → Execute → Report 파이프라인.

## 디렉토리 구조

```
deep-research/
├── SKILL.md              # 이 문서
└── scripts/
    ├── load_env.sh       # 환경 변수 로더 (API 키)
    ├── run.py            # 메인 실행 스크립트
    ├── search_engine.py  # 검색 엔진 모듈
    ├── scraper.py        # 웹 스크래퍼 모듈
    ├── llm_client.py     # LLM 클라이언트 모듈
    └── test_research.py  # 테스트 코드
```

## 빠른 시작

```bash
# 스킬 디렉토리로 이동
cd ~/.config/opencode/skill/deep-research

# 환경 변수 로드 (API 키)
source scripts/load_env.sh

# 테스트 실행 (mock mode)
python scripts/test_research.py

# 리서치 실행
python scripts/run.py "AI trends in 2026"
```

## 사용법

### CLI

```bash
# 슬래시 커맨드
/deep-research "주제"

# 스킬 스크립트 직접 실행
source ~/.config/opencode/skill/deep-research/scripts/load_env.sh
python ~/.config/opencode/skill/deep-research/scripts/run.py "주제" --depth 3
```

### Python API

```python
from scripts.run import DeepResearch, llm_complete

researcher = DeepResearch(llm_callback=llm_complete)
session = researcher.create_plan("Climate change solutions")
results = researcher.execute_plan(session)
report = researcher.generate_report(session, results)
```

## 환경 설정

```bash
# ~/.bashrc에 추가
export GEMINI_API_KEY="your-api-key"
# 또는
export OPENAI_API_KEY="your-api-key"

# 로드
source scripts/load_env.sh
```

## 출력

- 리포트: `~/.cache/opencode/research/{session_id}/final_report.md`
- 원본 데이터: `~/.cache/opencode/research/{session_id}/sources/`

## 의존성

```
duckduckgo-search>=3.0.0
beautifulsoup4>=4.9.0
requests>=2.25.0
google-generativeai>=0.3.0  # Gemini
openai>=1.0.0               # OpenAI (선택)
```

## 관련 스킬

- **research**: 빠른 검색이 필요한 경우
- **memory**: 리서치 결과 장기 저장 시
