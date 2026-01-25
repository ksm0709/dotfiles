#!/usr/bin/env python3
"""
Divergence Analyzer - Market Divergence Detection

가격과 선행지표 간의 다이버전스를 감지하는 분석 엔진입니다.
베어리시 다이버전스, 불리시 다이버전스 등 다양한 패턴을 식별합니다.
"""

import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional
import warnings
warnings.filterwarnings('ignore')

class DivergenceAnalyzer:
    """다이버전스 분석 클래스"""
    
    def __init__(self, config: Dict = None):
        """
        초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config or self._default_config()
        
    def _default_config(self) -> Dict:
        """기본 설정"""
        return {
            "lookback_periods": {
                "short": 10,
                "medium": 20,
                "long": 50
            },
            "thresholds": {
                "divergence_strength": 0.15,
                "trend_confirmation": 0.10,
                "min_points": 3
            },
            "indicators": [
                "vix_put_call",
                "ad_breadth", 
                "new_highs_lows",
                "sentiment"
            ]
        }
    
    def detect_all_divergences(self, price_data: Dict, indicator_data: Dict) -> List[Dict]:
        """
        모든 다이버전스 신호 감지
        
        Args:
            price_data: 가격 데이터
            indicator_data: 지표 데이터
            
        Returns:
            List[Dict]: 다이버전스 신호 목록
        """
        divergences = []
        
        for indicator in self.config["indicators"]:
            if indicator in indicator_data:
                indicator_divergences = self._detect_indicator_divergence(
                    price_data, indicator_data[indicator], indicator
                )
                divergences.extend(indicator_divergences)
        
        # 종합 다이버전스 분석
        composite_divergence = self._analyze_composite_divergence(divergences)
        
        if composite_divergence:
            divergences.append(composite_divergence)
        
        return divergences
    
    def _detect_indicator_divergence(self, price_data: Dict, indicator_data: Dict, 
                                   indicator_name: str) -> List[Dict]:
        """
        개별 지표 다이버전스 감지
        
        Args:
            price_data: 가격 데이터
            indicator_data: 지표 데이터
            indicator_name: 지표 이름
            
        Returns:
            List[Dict]: 다이버전스 신호 목록
        """
        try:
            # 데이터 전처리
            price_series = self._extract_price_series(price_data)
            indicator_series = self._extract_indicator_series(indicator_data, indicator_name)
            
            if price_series.empty or indicator_series.empty:
                return []
            
            # 시계열 정렬 및 동기화
            aligned_data = self._align_time_series(price_series, indicator_series)
            
            if len(aligned_data) < self.config["thresholds"]["min_points"]:
                return []
            
            # 추세 추정
            price_trend = self._estimate_trend(aligned_data['price'])
            indicator_trend = self._estimate_trend(aligned_data['indicator'])
            
            # 다이버전스 확인
            divergences = []
            
            # 베어리시 다이버전스 (가격 상승 vs 지표 하락)
            if price_trend > 0 and indicator_trend < -self.config["thresholds"]["divergence_strength"]:
                bearish_div = self._create_bearish_divergence(
                    aligned_data, indicator_name, price_trend, indicator_trend
                )
                divergences.append(bearish_div)
            
            # 불리시 다이버전스 (가격 하락 vs 지표 상승)
            elif price_trend < 0 and indicator_trend > self.config["thresholds"]["divergence_strength"]:
                bullish_div = self._create_bullish_divergence(
                    aligned_data, indicator_name, price_trend, indicator_trend
                )
                divergences.append(bullish_div)
            
            # 히든 다이버전스 확인
            hidden_divergences = self._detect_hidden_divergences(aligned_data, indicator_name)
            divergences.extend(hidden_divergences)
            
            return divergences
            
        except Exception as e:
            return []
    
    def _extract_price_series(self, price_data: Dict) -> pd.Series:
        """가격 시리즈 추출"""
        try:
            if 'close' in price_data:
                return pd.Series(price_data['close'])
            elif 'spy' in price_data and 'close' in price_data['spy']:
                return pd.Series(price_data['spy']['close'])
            else:
                return pd.Series()
        except:
            return pd.Series()
    
    def _extract_indicator_series(self, indicator_data: Dict, indicator_name: str) -> pd.Series:
        """지표 시리즈 추출"""
        try:
            if indicator_name == "vix_put_call":
                if 'ratio' in indicator_data:
                    return pd.Series(indicator_data['ratio'])
                elif 'current' in indicator_data:
                    return pd.Series([indicator_data['current']])
                    
            elif indicator_name == "ad_breadth":
                if 'ad_ratio' in indicator_data:
                    return pd.Series(indicator_data['ad_ratio'])
                    
            elif indicator_name == "new_highs_lows":
                if 'ratio' in indicator_data:
                    return pd.Series(indicator_data['ratio'])
                    
            elif indicator_name == "sentiment":
                if 'sentiment_score' in indicator_data:
                    return pd.Series([indicator_data['sentiment_score']])
                elif 'bull_bear_spread' in indicator_data:
                    return pd.Series(indicator_data['bull_bear_spread'])
                    
            return pd.Series()
            
        except:
            return pd.Series()
    
    def _align_time_series(self, price_series: pd.Series, 
                          indicator_series: pd.Series) -> pd.DataFrame:
        """시계열 데이터 동기화"""
        try:
            # 데이터프레임 생성
            df = pd.DataFrame({
                'price': price_series,
                'indicator': indicator_series
            })
            
            # 결측치 제거
            df = df.dropna()
            
            return df
            
        except:
            return pd.DataFrame()
    
    def _estimate_trend(self, series: pd.Series) -> float:
        """추세 추정 (선형 회귀 기울기)"""
        try:
            if len(series) < 2:
                return 0.0
            
            x = np.arange(len(series))
            y = series.values
            
            # 선형 회귀
            slope = np.polyfit(x, y, 1)[0]
            
            # 정규화
            normalized_slope = slope / np.mean(np.abs(y)) if np.mean(np.abs(y)) > 0 else 0
            
            return normalized_slope
            
        except:
            return 0.0
    
    def _create_bearish_divergence(self, data: pd.DataFrame, indicator_name: str,
                                 price_trend: float, indicator_trend: float) -> Dict:
        """베어리시 다이버전스 생성"""
        strength = self._calculate_divergence_strength(price_trend, indicator_trend)
        
        return {
            "type": "BEARISH_DIVERGENCE",
            "indicator": indicator_name,
            "strength": strength,
            "price_trend": price_trend,
            "indicator_trend": indicator_trend,
            "description": f"가격 상승({price_trend:.3f})과 {indicator_name} 하락({indicator_trend:.3f}) 간 베어리시 다이버전스",
            "signal": "SELL_WARNING",
            "confidence": min(95, max(50, abs(strength) * 100)),
            "lead_time_estimate": self._estimate_lead_time(indicator_name, "bearish"),
            "timestamp": datetime.now().isoformat()
        }
    
    def _create_bullish_divergence(self, data: pd.DataFrame, indicator_name: str,
                                 price_trend: float, indicator_trend: float) -> Dict:
        """불리시 다이버전스 생성"""
        strength = self._calculate_divergence_strength(price_trend, indicator_trend)
        
        return {
            "type": "BULLISH_DIVERGENCE",
            "indicator": indicator_name,
            "strength": strength,
            "price_trend": price_trend,
            "indicator_trend": indicator_trend,
            "description": f"가격 하락({price_trend:.3f})과 {indicator_name} 상승({indicator_trend:.3f}) 간 불리시 다이버전스",
            "signal": "BUY_OPPORTUNITY",
            "confidence": min(95, max(50, abs(strength) * 100)),
            "lead_time_estimate": self._estimate_lead_time(indicator_name, "bullish"),
            "timestamp": datetime.now().isoformat()
        }
    
    def _detect_hidden_divergences(self, data: pd.DataFrame, 
                                  indicator_name: str) -> List[Dict]:
        """히든 다이버전스 감지"""
        hidden_divergences = []
        
        try:
            # 히든 베어리시 다이버전스 (고점 갱신 vs 지표 고점 미갱신)
            if self._detect_hidden_bearish(data):
                hidden_divergences.append({
                    "type": "HIDDEN_BEARISH_DIVERGENCE",
                    "indicator": indicator_name,
                    "strength": "MEDIUM",
                    "description": f"히든 베어리시 다이버전스: {indicator_name}",
                    "signal": "WEAKENING_UPTREND",
                    "timestamp": datetime.now().isoformat()
                })
            
            # 히든 불리시 다이버전스 (저점 갱신 vs 지표 저점 미갱신)
            if self._detect_hidden_bullish(data):
                hidden_divergences.append({
                    "type": "HIDDEN_BULLISH_DIVERGENCE",
                    "indicator": indicator_name,
                    "strength": "MEDIUM",
                    "description": f"히든 불리시 다이버전스: {indicator_name}",
                    "signal": "STRENGTHENING_DOWNTREND",
                    "timestamp": datetime.now().isoformat()
                })
                
        except:
            pass
        
        return hidden_divergences
    
    def _detect_hidden_bearish(self, data: pd.DataFrame) -> bool:
        """히든 베어리시 다이버전스 감지"""
        try:
            # 고점 추출
            price_highs = self._find_peaks(data['price'])
            indicator_highs = self._find_peaks(data['indicator'])
            
            # 가격 고점 갱신 여부 확인
            if len(price_highs) >= 2:
                price_making_higher_highs = price_highs[-1] > price_highs[-2]
                
                # 지표 고점 미갱신 여부 확인
                if len(indicator_highs) >= 2:
                    indicator_not_making_higher_highs = indicator_highs[-1] <= indicator_highs[-2]
                    
                    return price_making_higher_highs and indicator_not_making_higher_highs
            
            return False
            
        except:
            return False
    
    def _detect_hidden_bullish(self, data: pd.DataFrame) -> bool:
        """히든 불리시 다이버전스 감지"""
        try:
            # 저점 추출
            price_lows = self._find_troughs(data['price'])
            indicator_lows = self._find_troughs(data['indicator'])
            
            # 가격 저점 갱신 여부 확인
            if len(price_lows) >= 2:
                price_making_lower_lows = price_lows[-1] < price_lows[-2]
                
                # 지표 저점 미갱신 여부 확인
                if len(indicator_lows) >= 2:
                    indicator_not_making_lower_lows = indicator_lows[-1] >= indicator_lows[-2]
                    
                    return price_making_lower_lows and indicator_not_making_lower_lows
            
            return False
            
        except:
            return False
    
    def _find_peaks(self, series: pd.Series) -> List[float]:
        """고점 찾기"""
        try:
            peaks = []
            
            for i in range(1, len(series) - 1):
                if series.iloc[i] > series.iloc[i-1] and series.iloc[i] > series.iloc[i+1]:
                    peaks.append(series.iloc[i])
            
            return peaks
            
        except:
            return []
    
    def _find_troughs(self, series: pd.Series) -> List[float]:
        """저점 찾기"""
        try:
            troughs = []
            
            for i in range(1, len(series) - 1):
                if series.iloc[i] < series.iloc[i-1] and series.iloc[i] < series.iloc[i+1]:
                    troughs.append(series.iloc[i])
            
            return troughs
            
        except:
            return []
    
    def _calculate_divergence_strength(self, price_trend: float, 
                                      indicator_trend: float) -> float:
        """다이버전스 강도 계산"""
        # 추세 차이의 절대값으로 강도 계산
        strength = abs(price_trend - indicator_trend)
        
        # 정규화
        normalized_strength = min(1.0, strength / 0.5)
        
        if normalized_strength > 0.7:
            return "STRONG"
        elif normalized_strength > 0.4:
            return "MEDIUM"
        else:
            return "WEAK"
    
    def _estimate_lead_time(self, indicator_name: str, divergence_type: str) -> str:
        """리드 타임 추정"""
        lead_times = {
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
        
        return lead_times.get(indicator_name, {}).get(divergence_type, "1-4주")
    
    def _analyze_composite_divergence(self, divergences: List[Dict]) -> Optional[Dict]:
        """종합 다이버전스 분석"""
        try:
            if len(divergences) < 2:
                return None
            
            # 동일 방향 다이버전스 그룹화
            bearish_count = sum(1 for d in divergences if d["type"] == "BEARISH_DIVERGENCE")
            bullish_count = sum(1 for d in divergences if d["type"] == "BULLISH_DIVERGENCE")
            
            # 다수결로 방향 결정
            if bearish_count > bullish_count:
                return self._create_composite_divergence(divergences, "BEARISH")
            elif bullish_count > bearish_count:
                return self._create_composite_divergence(divergences, "BULLISH")
            else:
                return None  # 신호 모순
                
        except:
            return None
    
    def _create_composite_divergence(self, divergences: List[Dict], 
                                   direction: str) -> Dict:
        """종합 다이버전스 생성"""
        involved_indicators = [d["indicator"] for d in divergences]
        avg_confidence = np.mean([d["confidence"] for d in divergences])
        
        return {
            "type": f"COMPOSITE_{direction}_DIVERGENCE",
            "involved_indicators": involved_indicators,
            "indicator_count": len(divergences),
            "strength": "STRONG" if len(divergences) >= 3 else "MEDIUM",
            "description": f"{len(divergences)}개 지표에서 {direction.lower()} 다이버전스 동시 감지",
            "signal": f"STRONG_{direction}_WARNING" if direction == "BEARISH" else f"STRONG_{direction}_OPPORTUNITY",
            "confidence": min(95, avg_confidence + 10),
            "lead_time_estimate": "1-4주",
            "timestamp": datetime.now().isoformat()
        }

def main():
    """메인 실행 함수"""
    analyzer = DivergenceAnalyzer()
    
    # 모의 데이터 생성
    dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
    price_data = {
        "close": dict(zip(dates, 100 + np.random.randn(30).cumsum()))
    }
    
    indicator_data = {
        "vix_put_call": {
            "ratio": dict(zip(dates, np.random.uniform(0.5, 1.2, 30)))
        },
        "ad_breadth": {
            "ad_ratio": dict(zip(dates, np.random.uniform(0.3, 0.8, 30)))
        }
    }
    
    # 다이버전스 분석
    divergences = analyzer.detect_all_divergences(price_data, indicator_data)
    
    print("=== 다이버전스 분석 결과 ===")
    if divergences:
        for i, divergence in enumerate(divergences, 1):
            print(f"\n{i}. {divergence['type']}")
            print(f"   설명: {divergence['description']}")
            print(f"   강도: {divergence['strength']}")
            print(f"   신뢰도: {divergence['confidence']}%")
            print(f"   신호: {divergence['signal']}")
    else:
        print("감지된 다이버전스가 없습니다.")

if __name__ == "__main__":
    main()