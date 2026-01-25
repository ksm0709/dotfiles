# 설정 가능한 파라미터 및 가중치 최적화 가이드

## 기본 설정 구조

### 전역 설정
```json
{
  "version": "1.0.0",
  "last_updated": "2024-01-21",
  "model_type": "market_heat_detector",
  "data_sources": {
    "primary": ["yahoo", "cboe", "aaii"],
    "backup": ["alpha_vantage", "quandl"]
  }
}
```

---

## 지표 가중치 설정

### 기본 가중치
```json
{
  "weights": {
    "vix_put_call": 0.30,
    "ad_breadth": 0.25,
    "new_highs_lows": 0.25,
    "sentiment": 0.20
  },
  "weight_constraints": {
    "min_weight": 0.10,
    "max_weight": 0.40,
    "sum_to_one": true
  }
}
```

### 시장 상황별 가중치

#### 상승장 (Bull Market)
```json
{
  "market_condition": "bull_market",
  "weights": {
    "vix_put_call": 0.25,
    "ad_breadth": 0.30,
    "new_highs_lows": 0.25,
    "sentiment": 0.20
  },
  "rationale": "A/D Breadth의 추세 지속성 확인 강화"
}
```

#### 하락장 (Bear Market)
```json
{
  "market_condition": "bear_market",
  "weights": {
    "vix_put_call": 0.35,
    "ad_breadth": 0.25,
    "new_highs_lows": 0.20,
    "sentiment": 0.20
  },
  "rationale": "VIX Put-Call Ratio의 공포 지표 강화"
}
```

#### 고변동성 (High Volatility)
```json
{
  "market_condition": "high_volatility",
  "weights": {
    "vix_put_call": 0.40,
    "ad_breadth": 0.20,
    "new_highs_lows": 0.20,
    "sentiment": 0.20
  },
  "rationale": "변동성 지표의 비중 확대"
}
```

#### 횡보장 (Sideways Market)
```json
{
  "market_condition": "sideways_market",
  "weights": {
    "vix_put_call": 0.30,
    "ad_breadth": 0.25,
    "new_highs_lows": 0.25,
    "sentiment": 0.20
  },
  "rationale": "기본 가중치 유지"
}
```

---

## 임계값 설정

### 과열 점수 임계값
```json
{
  "heat_thresholds": {
    "warning": {
      "value": 60,
      "description": "시장 과열 가능성 주의"
    },
    "critical": {
      "value": 80,
      "description": "시장 과열 신호 강함"
    },
    "emergency": {
      "value": 90,
      "description": "시장 붕괴 가능성 높음"
    }
  }
}
```

### 개별 지표 임계값

#### VIX Put-Call Ratio
```json
{
  "vix_put_call": {
    "thresholds": {
      "overheated": 0.7,
      "normal_min": 0.7,
      "normal_max": 1.0,
      "fear": 1.0
    },
    "extreme_levels": {
      "extreme_overheated": 0.5,
      "extreme_fear": 1.5
    },
    "smoothing": {
      "use_moving_average": true,
      "ma_period": 5
    }
  }
}
```

#### A/D Breadth
```json
{
  "ad_breadth": {
    "thresholds": {
      "strong_bullish": 0.6,
      "normal_min": 0.4,
      "normal_max": 0.6,
      "strong_bearish": 0.4
    },
    "divergence": {
      "min_periods": 10,
      "trend_threshold": 0.1,
      "confirmation_required": true
    },
    "smoothing": {
      "use_moving_average": true,
      "ma_period": 10
    }
  }
}
```

#### New Highs/New Lows
```json
{
  "new_highs_lows": {
    "thresholds": {
      "strong_bullish": 0.7,
      "normal_min": 0.3,
      "normal_max": 0.7,
      "strong_bearish": 0.3
    },
    "trend_confirmation": {
      "min_consecutive_days": 3,
      "volume_confirmation": true
    },
    "market_cap_weighting": {
      "enabled": true,
      "large_cap_weight": 0.6,
      "mid_cap_weight": 0.3,
      "small_cap_weight": 0.1
    }
  }
}
```

#### Sentiment Indicators
```json
{
  "sentiment": {
    "aaii": {
      "bullish_thresholds": {
        "extreme_optimism": 60,
        "normal_max": 40,
        "normal_min": 20,
        "extreme_pessimism": 20
      },
      "bull_bear_spread_threshold": 30
    },
    "fear_greed": {
      "thresholds": {
        "extreme_greed": 75,
        "greed": 55,
        "neutral_max": 55,
        "neutral_min": 45,
        "fear": 25,
        "extreme_fear": 25
      }
    }
  }
}
```

---

## 다이버전스 감지 파라미터

### 기본 다이버전스 설정
```json
{
  "divergence_detection": {
    "enabled": true,
    "min_lookback_periods": 10,
    "max_lookback_periods": 50,
    "min_points": 3,
    "trend_threshold": 0.1,
    "strength_threshold": 0.15
  }
}
```

### 다이버전스 타입별 설정
```json
{
  "divergence_types": {
    "bearish_divergence": {
      "enabled": true,
      "weight_multiplier": 1.2,
      "min_strength": 0.1,
      "confirmation_required": true
    },
    "bullish_divergence": {
      "enabled": true,
      "weight_multiplier": 1.0,
      "min_strength": 0.1,
      "confirmation_required": false
    },
    "hidden_divergence": {
      "enabled": true,
      "weight_multiplier": 0.8,
      "min_strength": 0.15,
      "confirmation_required": true
    }
  }
}
```

### 종합 다이버전스 설정
```json
{
  "composite_divergence": {
    "enabled": true,
    "min_indicators": 2,
    "same_direction_required": true,
    "strength_aggregation": "weighted_average",
    "confidence_boost": 10
  }
}
```

---

## 예측 모델 파라미터

### 신뢰도 설정
```json
{
  "confidence": {
    "minimum_threshold": 0.60,
    "high_confidence": 0.80,
    "maximum_confidence": 0.95,
    "calculation_method": "weighted_average"
  }
}
```

### 리드 타임 추정
```json
{
  "lead_time_estimation": {
    "vix_put_call": {
      "bearish": "1-3주",
      "bullish": "1-2주"
    },
    "ad_breadth": {
      "bearish": "2-6주",
      "bullish": "2-4주"
    },
    "new_highs_lows": {
      "bearish": "1-3주",
      "bullish": "1-2주"
    },
    "sentiment": {
      "bearish": "2-8주",
      "bullish": "2-6주"
    }
  }
}
```

---

## 데이터 수집 파라미터

### 업데이트 주기
```json
{
  "data_update_frequency": {
    "vix_put_call": "real_time",
    "ad_breadth": "daily",
    "new_highs_lows": "daily",
    "sentiment": {
      "aaii": "weekly",
      "fear_greed": "daily"
    }
  }
}
```

### 데이터 품질
```json
{
  "data_quality": {
    "max_delay_minutes": 15,
    "min_accuracy": 0.95,
    "missing_data_handling": "interpolation",
    "outlier_detection": {
      "enabled": true,
      "method": "iqr",
      "threshold": 1.5
    }
  }
}
```

### 캐시 설정
```json
{
  "cache": {
    "enabled": true,
    "duration_minutes": 5,
    "max_size_mb": 100,
    "cleanup_policy": "lru"
  }
}
```

---

## 백테스팅 파라미터

### 테스트 기간
```json
{
  "backtest_periods": {
    "short_term": {
      "duration": "6M",
      "description": "단기 성능 검증"
    },
    "medium_term": {
      "duration": "1Y",
      "description": "중기 성능 검증"
    },
    "long_term": {
      "duration": "2Y",
      "description": "장기 성능 검증"
    }
  }
}
```

### 성능 평가 지표
```json
{
  "performance_metrics": {
    "primary": ["accuracy", "precision", "recall", "f1_score"],
    "secondary": ["hit_rate", "false_positive_rate", "lead_time_accuracy"],
    "weights": {
      "accuracy": 0.3,
      "precision": 0.25,
      "recall": 0.25,
      "f1_score": 0.2
    }
  }
}
```

---

## 경고 시스템 파라미터

### 경고 레벨 설정
```json
{
  "alert_system": {
    "levels": {
      "normal": {
        "enabled": false,
        "color": "green"
      },
      "warning": {
        "enabled": true,
        "color": "yellow",
        "min_heat_score": 60
      },
      "critical": {
        "enabled": true,
        "color": "red",
        "min_heat_score": 80
      },
      "emergency": {
        "enabled": true,
        "color": "purple",
        "min_heat_score": 90
      }
    }
  }
}
```

### 발송 제한
```json
{
  "rate_limiting": {
    "max_alerts_per_hour": 10,
    "min_interval_minutes": 30,
    "quiet_hours": {
      "enabled": true,
      "start": 22,
      "end": 6,
      "timezone": "UTC"
    }
  }
}
```

---

## 최적화 알고리즘

### 가중치 최적화
```json
{
  "weight_optimization": {
    "method": "genetic_algorithm",
    "parameters": {
      "population_size": 100,
      "generations": 50,
      "mutation_rate": 0.1,
      "crossover_rate": 0.8,
      "elitism_rate": 0.1
    },
    "objective_function": "maximize_f1_score",
    "constraints": {
      "sum_to_one": true,
      "min_weight": 0.1,
      "max_weight": 0.4
    }
  }
}
```

### 임계값 최적화
```json
{
  "threshold_optimization": {
    "method": "grid_search",
    "parameters": {
      "vix_put_call_range": [0.5, 0.6, 0.7, 0.8],
      "ad_breadth_range": [0.3, 0.4, 0.5, 0.6],
      "heat_score_range": [55, 60, 65, 70, 75, 80]
    },
    "objective_function": "maximize_accuracy",
    "cross_validation": {
      "enabled": true,
      "folds": 5
    }
  }
}
```

---

## 리스크 관리 파라미터

### 포지션 사이징
```json
{
  "position_sizing": {
    "base_allocation": 0.02,
    "risk_adjusted": true,
    "max_allocation": 0.10,
    "scaling_factors": {
      "warning": 1.0,
      "critical": 0.5,
      "emergency": 0.2
    }
  }
}
```

### 손실 한미 설정
```json
{
  "stop_loss": {
    "enabled": true,
    "method": "percentage",
    "parameters": {
      "warning_level": 0.05,
      "critical_level": 0.03,
      "emergency_level": 0.02
    },
    "trailing_stop": {
      "enabled": true,
      "distance": 0.02
    }
  }
}
```

---

## 모니터링 및 로깅

### 성능 모니터링
```json
{
  "monitoring": {
    "enabled": true,
    "metrics": [
      "prediction_accuracy",
      "alert_frequency",
      "data_quality",
      "system_latency"
    ],
    "alert_thresholds": {
      "accuracy_drop": 0.1,
      "latency_increase": 1000,
      "data_quality_drop": 0.05
    }
  }
}
```

### 로깅 설정
```json
{
  "logging": {
    "level": "INFO",
    "format": "json",
    "retention_days": 30,
    "max_file_size_mb": 100,
    "categories": {
      "predictions": true,
      "alerts": true,
      "errors": true,
      "performance": true
    }
  }
}
```

---

## 설정 파일 예시

### 완전 설정 파일 (config.json)
```json
{
  "version": "1.0.0",
  "last_updated": "2024-01-21",
  "weights": {
    "vix_put_call": 0.30,
    "ad_breadth": 0.25,
    "new_highs_lows": 0.25,
    "sentiment": 0.20
  },
  "heat_thresholds": {
    "warning": 60,
    "critical": 80,
    "emergency": 90
  },
  "vix_put_call": {
    "thresholds": {
      "overheated": 0.7,
      "normal_min": 0.7,
      "normal_max": 1.0,
      "fear": 1.0
    }
  },
  "divergence_detection": {
    "enabled": true,
    "min_lookback_periods": 10,
    "min_points": 3,
    "trend_threshold": 0.1
  },
  "confidence": {
    "minimum_threshold": 0.60,
    "high_confidence": 0.80
  },
  "alert_system": {
    "enabled": true,
    "rate_limiting": {
      "max_alerts_per_hour": 10,
      "min_interval_minutes": 30
    }
  },
  "backtest": {
    "period": "1Y",
    "metrics": ["accuracy", "precision", "recall", "f1_score"]
  }
}
```

---

## 파라미터 튜닝 가이드

### 1단계: 기본 설정 검증
- 기본 가중치로 1년 백테스팅 실행
- 정확도 65% 이상 확인
- 주요 지표별 성능 분석

### 2단계: 가중치 최적화
- 유전 알고리즘으로 가중치 튜닝
- 시장 상황별 가중치 테스트
- 과적합 방지를 위한 교차 검증

### 3단계: 임계값 최적화
- 그리드 서치로 최적 임계값 탐색
- 거짓 신호 최소화 목표
- 리스크/보상 균형 조정

### 4단계: 실제 환경 검증
- 실시간 데이터로 모델 테스트
- 3개월간 성능 모니터링
- 필요시 파라미터 재조정

---

## 주의사항

### 파라미터 조정 시 주의점
1. **과적합 방지**: 과거 데이터에만 최적화하지 않기
2. **시장 변화 고려**: 시장 구조 변화에 따른 조정 필요
3. **리스크 관리**: 높은 정확도도 리스크 고려 필수
4. **지속적 모니터링**: 정기적인 성능 검토 및 조정

### 백테스팅 한계
- 과거 데이터가 미래를 보장하지 않음
- 시장 구조 변화로 성능 저하 가능성
- 거래 비용 및 세금 미반영
- 실제 거래와의 차이 고려 필요