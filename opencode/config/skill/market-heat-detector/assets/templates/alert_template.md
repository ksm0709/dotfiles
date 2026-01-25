# 시장 과열 경고 보고서

## 📊 경고 개요

**경고 시각**: {{TIMESTAMP}}  
**경고 레벨**: {{ALERT_LEVEL}}  
**경고 타입**: {{ALERT_TYPE}}  
**과열 점수**: {{HEAT_SCORE}}/100  
**신뢰도**: {{CONFIDENCE}}%  
**예상 리드 타임**: {{LEAD_TIME}}

---

## 🚨 주요 경고 메시지

{{ALERT_MESSAGE}}

---

## 📈 지표별 현황

| 선행지표 | 현재 값 | 신호 | 해석 |
|----------|----------|------|------|
{{INDICATOR_TABLE}}

---

## 🔄 다이버전스 신호 분석

{{#if DIVERGENCE_SIGNALS}}
### 감지된 다이버전스

{{#each DIVERGENCE_SIGNALS}}
- **{{type}}**: {{description}}
  - 강도: {{strength}}
  - 신뢰도: {{confidence}}%
  - 리드 타임: {{lead_time_estimate}}

{{/each}}
{{else}}
현재 감지된 다이버전스 신호가 없습니다.
{{/if}}

---

## 🎯 예측 분석

### 예측 신호
- **신호**: {{PREDICTION_SIGNAL}}
- **신뢰도**: {{PREDICTION_CONFIDENCE}}%
- **리드 타임**: {{PREDICTION_LEAD_TIME}}

### 시나리오 분석
{{#if IS_CRITICAL}}
#### 🔴 시나리오: 시장 조정 가능성 높음
- 확률: {{PROBABILITY}}%
- 예상 조정 폭: 5-15%
- 기간: 1-4주
{{/if}}

{{#if IS_WARNING}}
#### 🟡 시나리오: 시장 변동성 확대 가능성
- 확률: {{PROBABILITY}}%
- 예상 변동성: 10-20%
- 기간: 2-6주
{{/if}}

---

## 💡 투자 권고사항

### 즉시 조치 사항
{{#each IMMEDIATE_RECOMMENDATIONS}}
- {{this}}
{{/each}}

### 중기 전략
{{#each MEDIUM_TERM_RECOMMENDATIONS}}
- {{this}}
{{/each}}

### 리스크 관리
{{#each RISK_MANAGEMENT_RECOMMENDATIONS}}
- {{this}}
{{/each}}

---

## 📊 역사적 참고 자료

### 유사한 과열 패턴
{{#each HISTORICAL_PATTERNS}}
- **기간**: {{period}}
- **사례**: {{case}}
- **결과**: {{outcome}}
- **정확도**: {{accuracy}}%
{{/each}}

### 지표별 성능 통계
{{#each INDICATOR_PERFORMANCE}}
- **{{indicator}}**: 정확도 {{accuracy}}%, 리드 타임 {{lead_time}}
{{/each}}

---

## ⚠️ 면책 조항

본 경고 보고서는 참고용 정보로 제공되며, 투자 결정은 투자자 본인의 판단과 책임하에 이루어져야 합니다. 시장 예측의 불확실성을 고려하여 충분한 리스크 관리를 수행하시기 바랍니다.

---

## 📞 추가 정보

- **데이터 업데이트**: 실시간 (VIX), 일간 (A/D Breadth, New Highs/Lows)
- **다음 분석 예정**: {{NEXT_ANALYSIS_TIME}}
- **문의**: support@marketheatdetector.com

---

*보고서 생성 시각: {{REPORT_GENERATION_TIME}}*  
*분석 엔진 버전: v1.0.0*