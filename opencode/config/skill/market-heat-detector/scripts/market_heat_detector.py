#!/usr/bin/env python3
"""
Market Heat Detector - Main Analysis Engine

선행지표 기반 시장 과열 예측 메인 분석 엔진입니다.
다중 지표를 조합하여 시장 과열 상태를 실시간으로 분석합니다.
"""

import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional
import json
import warnings
warnings.filterwarnings('ignore')

class MarketHeatDetector:
    """시장 과열 감지 메인 클래스"""
    
    def __init__(self, config_path: str = None):
        """
        초기화
        
        Args:
            config_path: 설정 파일 경로
        """
        self.config = self._load_config(config_path)
        self.indicators = {}
        self.heat_score = 0
        self.alert_level = "NORMAL"
        
    def _load_config(self, config_path: str) -> Dict:
        """설정 파일 로드"""
        if config_path is None:
            # 기본 설정
            return {
                "weights": {
                    "vix_put_call": 0.30,
                    "ad_breadth": 0.25,
                    "new_highs_lows": 0.25,
                    "sentiment": 0.20
                },
                "thresholds": {
                    "heat_warning": 60,
                    "heat_critical": 80,
                    "divergence": 0.15,
                    "confidence": 0.70
                },
                "data_sources": {
                    "vix": "CBOE",
                    "sentiment": "AAII",
                    "market_data": "YAHOO"
                }
            }
        else:
            with open(config_path, 'r') as f:
                return json.load(f)
    
    def analyze_current_heat(self) -> Dict:
        """
        현재 시장 과열 상태 분석
        
        Returns:
            Dict: 분석 결과
        """
        try:
            # 1. 데이터 수집
            self._collect_market_data()
            
            # 2. 지표 계산
            self._calculate_indicators()
            
            # 3. 다이버전스 분석
            divergence_signals = self._detect_divergence()
            
            # 4. 종합 과열 점수 계산
            self.heat_score = self._calculate_composite_score()
            
            # 5. 경고 레벨 결정
            self.alert_level = self._determine_alert_level()
            
            # 6. 예측 신호 생성
            prediction = self._generate_prediction()
            
            return {
                "timestamp": datetime.now().isoformat(),
                "heat_score": self.heat_score,
                "alert_level": self.alert_level,
                "indicators": self.indicators,
                "divergence_signals": divergence_signals,
                "prediction": prediction
            }
            
        except Exception as e:
            return {
                "error": f"분석 중 오류 발생: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }
    
    def _collect_market_data(self):
        """시장 데이터 수집"""
        # 실제 구현에서는 data_collector.py 사용
        # 여기서는 모의 데이터 생성
        dates = pd.date_range(end=datetime.now(), periods=252, freq='D')
        
        self.market_data = {
            'spy': pd.Series(np.random.randn(252).cumsum() + 100, index=dates),
            'vix': pd.Series(np.abs(np.random.randn(252) * 5 + 20), index=dates),
            'volume': pd.Series(np.random.randint(1000000, 5000000, 252), index=dates)
        }
    
    def _calculate_indicators(self):
        """핵심 선행지표 계산"""
        self.indicators = {
            'vix_put_call': self._calculate_vix_put_call(),
            'ad_breadth': self._calculate_ad_breadth(),
            'new_highs_lows': self._calculate_new_highs_lows(),
            'sentiment': self._calculate_sentiment_index()
        }
    
    def _calculate_vix_put_call(self) -> Dict:
        """VIX Put-Call Ratio 계산"""
        # 모의 데이터
        vix_current = self.market_data['vix'].iloc[-1]
        put_call_ratio = np.random.uniform(0.5, 1.2)
        
        signal = "NEUTRAL"
        if put_call_ratio < 0.7:
            signal = "OVERHEATED"
        elif put_call_ratio > 1.0:
            signal = "FEAR"
        
        return {
            "value": put_call_ratio,
            "signal": signal,
            "vix_level": vix_current,
            "interpretation": self._interpret_vix_put_call(put_call_ratio)
        }
    
    def _calculate_ad_breadth(self) -> Dict:
        """Advance/Decline Breadth 계산"""
        # 모의 데이터
        ad_ratio = np.random.uniform(0.3, 0.8)
        ad_line = np.random.randn(50).cumsum()
        
        # 다이버전스 확인
        price_trend = "UP"  # 단순화
        divergence = self._check_ad_divergence(ad_line, price_trend)
        
        return {
            "ad_ratio": ad_ratio,
            "ad_line_trend": "UP" if ad_line[-1] > ad_line[0] else "DOWN",
            "divergence": divergence,
            "signal": "WARNING" if divergence else "NORMAL"
        }
    
    def _calculate_new_highs_lows(self) -> Dict:
        """New Highs/New Lows 비율 계산"""
        # 모의 데이터
        new_highs = np.random.randint(50, 200)
        new_lows = np.random.randint(10, 100)
        total_issues = new_highs + new_lows
        
        if total_issues > 0:
            high_low_ratio = new_highs / total_issues
        else:
            high_low_ratio = 0.5
        
        signal = "NORMAL"
        if high_low_ratio < 0.3 and new_lows > new_highs:
            signal = "WEAKNESS"
        elif high_low_ratio > 0.7:
            signal = "STRENGTH"
        
        return {
            "new_highs": new_highs,
            "new_lows": new_lows,
            "ratio": high_low_ratio,
            "signal": signal
        }
    
    def _calculate_sentiment_index(self) -> Dict:
        """투자자 심리 지표 계산"""
        # 모의 데이터 (AAII 스타일)
        bullish = np.random.uniform(20, 60)
        bearish = np.random.uniform(15, 45)
        neutral = 100 - bullish - bearish
        
        sentiment_score = (bullish - bearish) / 100
        
        signal = "NEUTRAL"
        if sentiment_score > 0.3:
            signal = "OVERHEATED"
        elif sentiment_score < -0.3:
            signal = "OVERLY_BEARISH"
        
        return {
            "bullish": bullish,
            "bearish": bearish,
            "neutral": neutral,
            "sentiment_score": sentiment_score,
            "signal": signal
        }
    
    def _detect_divergence(self) -> List[Dict]:
        """다이버전스 신호 감지"""
        signals = []
        
        # VIX 다이버전스
        if self.indicators['vix_put_call']['signal'] == "OVERHEATED":
            signals.append({
                "type": "VIX_DIVERGENCE",
                "strength": "HIGH",
                "description": "VIX Put-Call Ratio가 낙관적이면서 시장은 과열될 수 있습니다"
            })
        
        # A/D 브레드th 다이버전스
        if self.indicators['ad_breadth']['divergence']:
            signals.append({
                "type": "AD_BREADTH_DIVERGENCE",
                "strength": "MEDIUM",
                "description": "가격 상승과 A/D 라인 하락 간 다이버전스 감지"
            })
        
        return signals
    
    def _calculate_composite_score(self) -> float:
        """종합 과열 점수 계산"""
        weights = self.config['weights']
        
        # 각 지표별 과열 점수 (0-100)
        scores = {
            'vix_put_call': self._score_vix_put_call(),
            'ad_breadth': self._score_ad_breadth(),
            'new_highs_lows': self._score_new_highs_lows(),
            'sentiment': self._score_sentiment()
        }
        
        # 가중치 적용 종합 점수
        composite_score = sum(
            scores[indicator] * weights[indicator]
            for indicator in scores
        )
        
        return round(composite_score, 2)
    
    def _score_vix_put_call(self) -> float:
        """VIX Put-Call Ratio 과열 점수"""
        ratio = self.indicators['vix_put_call']['value']
        
        if ratio < 0.7:
            return 80 + (0.7 - ratio) * 100  # 과열
        elif ratio > 1.0:
            return 20  # 공포 상태는 과열 반대
        else:
            return 50  # 중립
    
    def _score_ad_breadth(self) -> float:
        """A/D 브레드th 과열 점수"""
        if self.indicators['ad_breadth']['divergence']:
            return 75  # 다이버전스는 과열 신호
        else:
            return 40  # 정상
    
    def _score_new_highs_lows(self) -> float:
        """New Highs/Lows 과열 점수"""
        ratio = self.indicators['new_highs_lows']['ratio']
        
        if ratio < 0.3:
            return 70  # 신저가 증가는 약세 신호
        elif ratio > 0.8:
            return 60  # 신고가 과도도 과열 가능성
        else:
            return 45  # 정상
    
    def _score_sentiment(self) -> float:
        """심리 지표 과열 점수"""
        score = self.indicators['sentiment']['sentiment_score']
        
        if score > 0.3:
            return 75 + score * 25  # 과도한 낙관
        elif score < -0.3:
            return 30  # 과도한 비관
        else:
            return 50  # 중립
    
    def _determine_alert_level(self) -> str:
        """경고 레벨 결정"""
        if self.heat_score >= self.config['thresholds']['heat_critical']:
            return "CRITICAL"
        elif self.heat_score >= self.config['thresholds']['heat_warning']:
            return "WARNING"
        else:
            return "NORMAL"
    
    def _generate_prediction(self) -> Dict:
        """예측 신호 생성"""
        confidence = min(95, max(50, self.heat_score + 10))
        
        if self.alert_level == "CRITICAL":
            signal = "STRONG_OVERHEAT_WARNING"
            lead_time = "1-3주"
        elif self.alert_level == "WARNING":
            signal = "MODERATE_OVERHEAT_WARNING"
            lead_time = "2-6주"
        else:
            signal = "MARKET_NORMAL"
            lead_time = "N/A"
        
        return {
            "signal": signal,
            "confidence": confidence,
            "lead_time": lead_time,
            "recommendation": self._get_recommendation()
        }
    
    def _get_recommendation(self) -> str:
        """투자 권고사항"""
        if self.alert_level == "CRITICAL":
            return "시장 과열 신호가 강합니다. 포지션 축소 및 리스크 관리를 고려하세요."
        elif self.alert_level == "WARNING":
            return "시장 과열 가능성이 있습니다. 주의 깊게 모니터링하세요."
        else:
            return "현재 시장은 정상 범위 내에 있습니다."
    
    def _interpret_vix_put_call(self, ratio: float) -> str:
        """VIX Put-Call Ratio 해석"""
        if ratio < 0.7:
            return f"극도 낙관 ({ratio:.2f}). 과열 가능성 있음."
        elif ratio > 1.0:
            return f"공포 우세 ({ratio:.2f}). 하락 두려움."
        else:
            return f"중립 ({ratio:.2f}). 정상 범위."
    
    def _check_ad_divergence(self, ad_line: np.ndarray, price_trend: str) -> bool:
        """A/D 브레드th 다이버전스 확인"""
        # 단순화된 다이버전스 로직
        ad_trend = "UP" if ad_line[-1] > ad_line[0] else "DOWN"
        return price_trend == "UP" and ad_trend == "DOWN"

def main():
    """메인 실행 함수"""
    detector = MarketHeatDetector()
    result = detector.analyze_current_heat()
    
    print("=== 시장 과열 분석 결과 ===")
    print(f"분석 시각: {result['timestamp']}")
    print(f"과열 점수: {result['heat_score']}/100")
    print(f"경고 레벨: {result['alert_level']}")
    
    print("\n=== 지표별 현황 ===")
    for name, indicator in result['indicators'].items():
        print(f"{name}: {indicator.get('signal', 'N/A')}")
    
    if result['divergence_signals']:
        print("\n=== 다이버전스 신호 ===")
        for signal in result['divergence_signals']:
            print(f"{signal['type']}: {signal['description']}")
    
    print(f"\n=== 예측 ===")
    prediction = result['prediction']
    print(f"신호: {prediction['signal']}")
    print(f"신뢰도: {prediction['confidence']}%")
    print(f"예상 리드 타임: {prediction['lead_time']}")
    print(f"권고: {prediction['recommendation']}")

if __name__ == "__main__":
    main()