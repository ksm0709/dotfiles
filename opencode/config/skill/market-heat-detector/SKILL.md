---
name: market-heat-detector
description: 선행지표 기반 시장 과열 예측 및 조기 경고 스킬. VIX Put-Call Ratio, A/D Breadth, New Highs/Lows, 투자자 심리 지표를 조합하여 시장 과열 상태를 실시간 분석하고 예측 신호 생성. 다이버전스 감지, 예측 신뢰도 제공, 백테스팅 검증 지원.
---

# Market Heat Detector

## Overview

다중 선행지표를 조합하여 시장 과열 상태를 예측하고 조기 경고를 생성하는 전문 분석 스킬입니다. VIX Put-Call Ratio, 시장 참여도, 신고가/신저가 비율, 투자자 심리 지표의 다이버전스를 감지하여 시장 천정을 예측합니다.

## Core Capabilities

### 1. 실시간 시장 과열 분석
현재 시장의 과열 상태를 다중 지표로 종합 평가합니다:

```python
# 메인 분석 실행
from scripts.market_heat_detector import MarketHeatDetector

detector = MarketHeatDetector()
heat_score = detector.analyze_current_heat()
print(f"시장 과열 점수: {heat_score}/100")
```

### 2. 선행지표 다이버전스 감지
가격과 선행지표 간의 다이버전스를 자동으로 감지합니다:

```python
# 다이버전스 분석
divergence_signals = detector.detect_divergence()
for signal in divergence_signals:
    print(f"다이버전스 신호: {signal['type']} - {signal['strength']}")
```

### 3. 예측 신호 생성 및 신뢰도 평가
과열 예측 신호와 신뢰도, 리드 타임을 제공합니다:

```python
# 예측 신호 생성
prediction = detector.generate_prediction()
print(f"예측: {prediction['signal']}")
print(f"신뢰도: {prediction['confidence']}%")
print(f"예상 리드 타임: {prediction['lead_time']}일")
```

### 4. 백테스팅 및 정확도 검증
역사적 데이터로 예측 모델의 정확도를 검증합니다:

```python
# 백테스팅 실행
from scripts.backtest_engine import BacktestEngine

backtest = BacktestEngine()
accuracy = backtest.run_backtest(start_date="2020-01-01")
print(f"예측 정확도: {accuracy['overall']}%")
```

## Analysis Workflow

### 단계 1: 데이터 수집 및 전처리
실시간 시장 데이터를 수집하고 정규화합니다:

```bash
# 데이터 수집 스크립트 실행
python3 scripts/data_collector.py --collect-all --normalize
```

### 단계 2: 지표 계산 및 다이버전스 분석
핵심 선행지표를 계산하고 다이버전스를 분석합니다:

```python
# 지표별 분석
indicators = {
    'vix_put_call': detector.calculate_vix_put_call(),
    'ad_breadth': detector.calculate_ad_breadth(),
    'new_highs_lows': detector.calculate_new_highs_lows(),
    'sentiment': detector.calculate_sentiment_index()
}
```

### 단계 3: 종합 과열 점수 계산
가중치를 적용하여 종합 과열 점수를 계산합니다:

```python
# 가중치 적용 종합 점수
heat_score = detector.calculate_composite_score(indicators)
if heat_score > 80:
    alert_level = "CRITICAL"
elif heat_score > 60:
    alert_level = "WARNING"
else:
    alert_level = "NORMAL"
```

### 단계 4: 예측 신호 생성 및 경고
분석 결과를 바탕으로 예측 신호와 조기 경고를 생성합니다:

```python
# 경고 생성
if alert_level in ["WARNING", "CRITICAL"]:
    alert = detector.generate_alert(heat_score, indicators)
    print(alert['message'])
```

## Key Indicators

### VIX Put-Call Ratio
- **해석**: 옵션 투자자 심리 지표
- **과열 신호**: 0.7 이하 (극도 낙관)
- **공포 신호**: 1.0 이상 (공포 우세)
- **리드 타임**: 1-4주

### Advance/Decline Breadth
- **해석**: 시장 참여도 및 내구성
- **다이버전스**: 가격 상승 & A-D 라인 하락
- **과열 신호**: 베어리시 다이버전스 발생
- **리드 타임**: 2-6주

### New Highs/New Lows Ratio
- **해석**: 시장 내부 강도
- **과열 신호**: 신고가 감소 & 신저가 증가
- **리드 타임**: 1-3주

### Sentiment Indicators
- **해석**: 투자자 심리 (AAII 등)
- **과열 신호**: 극도 낙관 (Bullish > 60%)
- **리드 타임**: 2-8주

## Configuration Parameters

### 가중치 설정
```json
{
  "vix_put_call_weight": 0.30,
  "ad_breadth_weight": 0.25,
  "new_highs_lows_weight": 0.25,
  "sentiment_weight": 0.20
}
```

### 임계값 설정
```json
{
  "heat_threshold_warning": 60,
  "heat_threshold_critical": 80,
  "divergence_threshold": 0.15,
  "confidence_threshold": 0.70
}
```

## Resources

### scripts/
핵심 분석 및 자동화 스크립트들입니다:

- **`market_heat_detector.py`** - 메인 분석 엔진
- **`data_collector.py`** - 실시간 데이터 수집기
- **`divergence_analyzer.py`** - 다이버전스 감지 알고리즘
- **`backtest_engine.py`** - 백테스팅 및 정확도 검증
- **`alert_generator.py`** - 조기 경고 생성기

### references/
상세 기술 문서 및 설정 참조 자료들입니다:

- **`indicators.md`** - 선행지표 상세 설명 및 계산 방법
- **`parameters.md`** - 설정 가능한 파라미터 및 가중치 최적화 가이드
- **`historical_patterns.md`** - 역사적 과열 패턴 및 사례 분석
- **`api_documentation.md`** - 데이터 소스 API 문서 및 연동 가이드

### assets/
템플릿 및 설정 파일들입니다:

- **`templates/alert_template.md`** - 경고 보고서 템플릿
- **`config/default_config.json`** - 기본 설정 및 파라미터
- **`examples/sample_analysis.md`** - 분석 결과 예시

## Usage Examples

### 실시간 모니터링
```
"현재 시장 과열 상태를 분석해줘"
```

### 다이버전스 확인
```
"VIX Put-Call Ratio와 AD 브레드th 다이버전스를 분석해줘"
```

### 역사적 패턴 분석
```
"과거 시장 천정 형성 시 선행지표 패턴을 분석해줘"
```

### 조기 경고 생성
```
"현재 과열 신호가 있는지 확인하고 조기 경고를 생성해줘"
```

### 정확도 검증
```
"이 선행지표 조합의 예측 정확도를 백테스팅으로 검증해줘"
```