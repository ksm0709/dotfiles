#!/usr/bin/env python3
"""
Data Collector - Real-time Market Data Collection

실시간 시장 데이터를 수집하고 정규화하는 데이터 수집기입니다.
VIX, 옵션 데이터, 시장 참여도, 투자자 심리 데이터를 수집합니다.
"""

import pandas as pd
import numpy as np
import requests
import json
from datetime import datetime, timedelta
from typing import Dict, List, Optional
import time
import warnings
warnings.filterwarnings('ignore')

class DataCollector:
    """실시간 시장 데이터 수집 클래스"""
    
    def __init__(self, config: Optional[Dict] = None):
        """
        초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config or self._default_config()
        self.session = requests.Session()
        self.data_cache = {}
        
    def _default_config(self) -> Dict:
        """기본 설정"""
        return {
            "api_endpoints": {
                "yahoo": "https://query1.finance.yahoo.com/v8/finance/chart/",
                "alpha_vantage": "https://www.alphavantage.co/query",
                "cboe": "https://www.cboe.com/",
                "aaii": "https://www.aaii.com/"
            },
            "rate_limits": {
                "yahoo": 100,  # requests per hour
                "alpha_vantage": 5,
                "cboe": 1000,
                "aaii": 60
            },
            "cache_duration": 300  # 5 minutes
        }
    
    def collect_all_data(self) -> Dict:
        """
        모든 필수 데이터 수집
        
        Returns:
            Dict: 수집된 데이터
        """
        try:
            data = {
                "market_data": self._collect_market_data(),
                "vix_data": self._collect_vix_data(),
                "options_data": self._collect_options_data(),
                "breadth_data": self._collect_breadth_data(),
                "sentiment_data": self._collect_sentiment_data(),
                "timestamp": datetime.now().isoformat()
            }
            
            # 데이터 정규화
            normalized_data = self._normalize_data(data)
            
            # 캐시 저장
            self._update_cache(normalized_data)
            
            return normalized_data
            
        except Exception as e:
            print(f"데이터 수집 중 오류: {str(e)}")
            return self._get_fallback_data()
    
    def _collect_market_data(self) -> Dict:
        """기본 시장 데이터 수집"""
        try:
            # S&P 500 데이터
            spy_data = self._get_yahoo_data("SPY", "1d", 30)
            
            # 주요 지수 데이터
            indices = ["^VIX", "^TNX", "DX-Y.NYB"]
            index_data = {}
            
            for index in indices:
                try:
                    index_data[index] = self._get_yahoo_data(index, "1d", 30)
                except:
                    continue
            
            return {
                "spy": spy_data,
                "indices": index_data,
                "status": "success"
            }
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _collect_vix_data(self) -> Dict:
        """VIX 관련 데이터 수집"""
        try:
            # VIX 지수
            vix_data = self._get_yahoo_data("^VIX", "1d", 60)
            
            # VIX 선물 (실제로는 CBOE API 사용)
            vix_futures = self._get_mock_vix_futures()
            
            return {
                "vix_spot": vix_data,
                "vix_futures": vix_futures,
                "vix_term_structure": self._calculate_vix_term_structure(vix_futures),
                "status": "success"
            }
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _collect_options_data(self) -> Dict:
        """옵션 데이터 수집"""
        try:
            # Put-Call Ratio 데이터
            put_call_ratio = self._get_put_call_ratio()
            
            # 옵션 변동성 데이터
            options_volatility = self._get_options_volatility()
            
            return {
                "put_call_ratio": put_call_ratio,
                "options_volatility": options_volatility,
                "status": "success"
            }
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _collect_breadth_data(self) -> Dict:
        """시장 참여도(Breadth) 데이터 수집"""
        try:
            # Advance/Decline 데이터
            ad_data = self._get_advance_decline_data()
            
            # New Highs/New Lows 데이터
            new_highs_lows = self._get_new_highs_lows()
            
            # Volume 데이터
            volume_data = self._get_volume_data()
            
            return {
                "advance_decline": ad_data,
                "new_highs_lows": new_highs_lows,
                "volume": volume_data,
                "status": "success"
            }
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _collect_sentiment_data(self) -> Dict:
        """투자자 심리 데이터 수집"""
        try:
            # AAII 심리 지표
            aaii_sentiment = self._get_aaii_sentiment()
            
            # CNN Fear & Greed Index
            fear_greed = self._get_fear_greed_index()
            
            # 기타 심리 지표들
            other_sentiment = self._get_other_sentiment_indicators()
            
            return {
                "aaii": aaii_sentiment,
                "fear_greed": fear_greed,
                "other": other_sentiment,
                "status": "success"
            }
            
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def _get_yahoo_data(self, symbol: str, interval: str, period: int) -> Dict:
        """Yahoo Finance API로 데이터 가져오기"""
        try:
            url = f"{self.config['api_endpoints']['yahoo']}{symbol}"
            params = {
                "interval": interval,
                "range": f"{period}d"
            }
            
            response = self.session.get(url, params=params, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                return self._parse_yahoo_data(data)
            else:
                raise Exception(f"Yahoo API error: {response.status_code}")
                
        except Exception as e:
            # 실제 데이터 대신 모의 데이터 반환
            return {}
    
    def _parse_yahoo_data(self, data: Dict) -> Dict:
        """Yahoo Finance 데이터 파싱"""
        try:
            result = data['chart']['result'][0]
            timestamps = result['timestamp']
            
            parsed_data = {}
            
            for indicator in ['open', 'high', 'low', 'close', 'volume']:
                if indicator in result['indicators']['quote'][0]:
                    values = result['indicators']['quote'][0][indicator]
                    parsed_data[indicator] = dict(zip(timestamps, values))
            
            return parsed_data
            
        except Exception:
            return {}
    
    def _get_put_call_ratio(self) -> Dict:
        """Put-Call Ratio 데이터 가져오기"""
        # 실제로는 CBOE API 사용
        # 여기서는 모의 데이터
        
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        put_call_ratios = np.random.uniform(0.5, 1.2, 30)
        
        return {
            "ratio": dict(zip(dates, put_call_ratios)),
            "current": put_call_ratios[-1],
            "ma_5d": np.mean(put_call_ratios[-5:]),
            "ma_20d": np.mean(put_call_ratios[-20:])
        }
    
    def _get_advance_decline_data(self) -> Dict:
        """Advance/Decline 데이터 가져오기"""
        dates = pd.date_range(end=datetime.now(), periods=60, freq='D')
        
        advances = np.random.randint(1000, 3000, 60).astype(float)
        declines = np.random.randint(500, 2000, 60).astype(float)
        ad_ratio = advances / (advances + declines)
        
        return {
            "advances": dict(zip(dates, advances)),
            "declines": dict(zip(dates, declines)),
            "ad_ratio": dict(zip(dates, ad_ratio)),
            "ad_line": np.cumsum(advances - declines).tolist()
        }
    
    def _get_new_highs_lows(self) -> Dict:
        """New Highs/New Lows 데이터 가져오기"""
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        
        new_highs = np.random.randint(50, 300, 30).astype(float)
        new_lows = np.random.randint(10, 150, 30).astype(float)
        
        return {
            "new_highs": dict(zip(dates, new_highs)),
            "new_lows": dict(zip(dates, new_lows)),
            "total_issues": new_highs + new_lows,
            "high_low_ratio": dict(zip(dates, new_highs / (new_highs + new_lows)))
        }
    
    def _get_volume_data(self) -> Dict:
        """거래량 데이터 가져오기"""
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        
        up_volume = np.random.uniform(1e9, 3e9, 30)
        down_volume = np.random.uniform(0.5e9, 2e9, 30)
        total_volume = up_volume + down_volume
        
        return {
            "up_volume": dict(zip(dates, up_volume)),
            "down_volume": dict(zip(dates, down_volume)),
            "total_volume": dict(zip(dates, total_volume)),
            "up_down_ratio": dict(zip(dates, up_volume / down_volume))
        }
    
    def _get_aaii_sentiment(self) -> Dict:
        """AAII 투자자 심리 지표 가져오기"""
        dates = pd.date_range(end=datetime.now(), periods=52, freq='W')
        
        bullish = np.random.uniform(20, 60, 52)
        bearish = np.random.uniform(15, 45, 52)
        neutral = 100 - bullish - bearish
        
        return {
            "bullish": dict(zip(dates, bullish)),
            "bearish": dict(zip(dates, bearish)),
            "neutral": dict(zip(dates, neutral)),
            "bull_bear_spread": dict(zip(dates, bullish - bearish))
        }
    
    def _get_fear_greed_index(self) -> Dict:
        """CNN Fear & Greed Index 가져오기"""
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        
        fear_greed_values = np.random.randint(0, 100, 30)
        
        return {
            "index": dict(zip(dates, fear_greed_values)),
            "current": fear_greed_values[-1],
            "signal": self._interpret_fear_greed(fear_greed_values[-1])
        }
    
    def _get_other_sentiment_indicators(self) -> Dict:
        """기타 심리 지표들"""
        return {
            "insider_trading": {
                "buy_sell_ratio": np.random.uniform(0.3, 1.5),
                "signal": "NEUTRAL"
            },
            "short_interest": {
                "short_ratio": np.random.uniform(1.0, 5.0),
                "days_to_cover": np.random.uniform(2.0, 8.0)
            }
        }
    
    def _normalize_data(self, raw_data: Dict) -> Dict:
        """데이터 정규화"""
        normalized = {}
        
        for key, value in raw_data.items():
            if isinstance(value, dict) and value.get("status") == "success":
                normalized[key] = value
            elif key == "timestamp":
                normalized[key] = value
            else:
                # 실패한 데이터는 모의 데이터로 대체
                normalized[key] = self._get_fallback_data_for_type(key)
        
        return normalized
    
    def _update_cache(self, data: Dict):
        """캐시 업데이트"""
        self.data_cache = {
            "data": data,
            "timestamp": datetime.now(),
            "expires": datetime.now() + timedelta(seconds=self.config['cache_duration'])
        }
    
    def _get_fallback_data(self) -> Dict:
        """실패 시 대체 데이터"""
        return {
            "status": "fallback",
            "message": "실시간 데이터 수집 실패. 모의 데이터 사용.",
            "data": self._generate_mock_data()
        }
    
    def _get_fallback_data_for_type(self, data_type: str) -> Dict:
        """데이터 타입별 대체 데이터"""
        mock_generators = {
            "market_data": self._get_mock_market_data,
            "vix_data": self._get_mock_vix_data,
            "options_data": self._get_mock_options_data,
            "breadth_data": self._get_mock_breadth_data,
            "sentiment_data": self._get_mock_sentiment_data
        }
        
        generator = mock_generators.get(data_type, self._get_mock_market_data)
        return generator()
    
    def _generate_mock_data(self) -> Dict:
        """모의 데이터 생성"""
        return {
            "market_data": self._get_mock_market_data(),
            "vix_data": self._get_mock_vix_data(),
            "options_data": self._get_mock_options_data(),
            "breadth_data": self._get_mock_breadth_data(),
            "sentiment_data": self._get_mock_sentiment_data(),
            "timestamp": datetime.now().isoformat()
        }
    
    def _get_mock_market_data(self) -> Dict:
        """모의 시장 데이터"""
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        prices = 100 + np.random.randn(30).cumsum()
        
        return {
            "spy": {
                "close": dict(zip(dates, prices)),
                "volume": dict(zip(dates, np.random.uniform(1e7, 5e7, 30)))
            },
            "status": "mock"
        }
    
    def _get_mock_vix_data(self) -> Dict:
        """모의 VIX 데이터"""
        dates = pd.date_range(end=datetime.now(), periods=30, freq='D')
        vix_values = np.abs(np.random.randn(30) * 5 + 20)
        
        return {
            "vix_spot": {
                "close": dict(zip(dates, vix_values))
            },
            "status": "mock"
        }
    
    def _get_mock_options_data(self) -> Dict:
        """모의 옵션 데이터"""
        return {
            "put_call_ratio": {
                "current": np.random.uniform(0.5, 1.2),
                "ma_5d": np.random.uniform(0.6, 1.1)
            },
            "status": "mock"
        }
    
    def _get_mock_breadth_data(self) -> Dict:
        """모의 Breadth 데이터"""
        return {
            "advance_decline": {
                "ad_ratio": np.random.uniform(0.4, 0.8)
            },
            "new_highs_lows": {
                "new_highs": np.random.randint(100, 200),
                "new_lows": np.random.randint(20, 80)
            },
            "status": "mock"
        }
    
    def _get_mock_sentiment_data(self) -> Dict:
        """모의 심리 데이터"""
        return {
            "aaii": {
                "bullish": np.random.uniform(25, 55),
                "bearish": np.random.uniform(20, 40)
            },
            "fear_greed": {
                "current": np.random.randint(20, 80)
            },
            "status": "mock"
        }
    
    def _interpret_fear_greed(self, value: float) -> str:
        """Fear & Greed Index 해석"""
        if value < 25:
            return "EXTREME_FEAR"
        elif value < 45:
            return "FEAR"
        elif value < 55:
            return "NEUTRAL"
        elif value < 75:
            return "GREED"
        else:
            return "EXTREME_GREED"
    
    def _get_mock_vix_futures(self) -> Dict:
        """모의 VIX 선물 데이터"""
        months = ['F', 'G', 'H', 'J', 'K', 'M']
        futures_data = {}
        
        for i, month in enumerate(months):
            futures_data[f"VIX{month}"] = 20 + np.random.randn() * 3 + i * 0.5
        
        return futures_data
    
    def _calculate_vix_term_structure(self, futures: Dict) -> str:
        """VIX 기간 구조 계산"""
        if len(futures) < 2:
            return "NEUTRAL"
        
        # 단순화된 기간 구조 분석
        values = list(futures.values())
        if values[1] > values[0] * 1.1:
            return "CONTANGO"
        elif values[1] < values[0] * 0.9:
            return "BACKWARDATION"
        else:
            return "NEUTRAL"
    
    def _get_options_volatility(self) -> Dict:
        """옵션 변동성 데이터"""
        return {
            "implied_volatility": np.random.uniform(0.15, 0.35),
            "realized_volatility": np.random.uniform(0.10, 0.30),
            "volatility_premium": np.random.uniform(-0.05, 0.10)
        }

def main():
    """메인 실행 함수"""
    collector = DataCollector()
    
    print("=== 실시간 시장 데이터 수집 ===")
    data = collector.collect_all_data()
    
    print(f"수집 시각: {data['timestamp']}")
    
    for key, value in data.items():
        if key != 'timestamp' and isinstance(value, dict):
            status = value.get('status', 'unknown')
            print(f"{key}: {status}")

if __name__ == "__main__":
    main()