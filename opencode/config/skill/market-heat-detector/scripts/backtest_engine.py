#!/usr/bin/env python3
"""
Backtest Engine - Prediction Accuracy Validation

선행지표 기반 예측 모델의 정확도를 검증하는 백테스팅 엔진입니다.
역사적 데이터를 사용하여 예측 성능을 평가하고 최적화합니다.
"""

import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Tuple, Optional
import json
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score
import warnings
warnings.filterwarnings('ignore')

class BacktestEngine:
    """백테스팅 엔진 클래스"""
    
    def __init__(self, config: Dict = None):
        """
        초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config or self._default_config()
        self.results = {}
        
    def _default_config(self) -> Dict:
        """기본 설정"""
        return {
            "backtest_periods": {
                "short": "6M",  # 6개월
                "medium": "1Y",  # 1년
                "long": "2Y"  # 2년
            },
            "evaluation_metrics": [
                "accuracy",
                "precision", 
                "recall",
                "f1_score",
                "hit_rate",
                "false_positive_rate"
            ],
            "prediction_thresholds": {
                "warning": 60,
                "critical": 80
            },
            "lead_time_windows": [5, 10, 20]  # trading days
        }
    
    def run_comprehensive_backtest(self, start_date: str, end_date: str = None) -> Dict:
        """
        종합 백테스팅 실행
        
        Args:
            start_date: 시작일 (YYYY-MM-DD)
            end_date: 종료일 (YYYY-MM-DD), 기본값: 현재
            
        Returns:
            Dict: 백테스팅 결과
        """
        try:
            if end_date is None:
                end_date = datetime.now().strftime("%Y-%m-%d")
            
            # 1. 역사적 데이터 로드
            historical_data = self._load_historical_data(start_date, end_date)
            
            # 2. 예측 모델 백테스팅
            prediction_results = self._backtest_prediction_model(historical_data)
            
            # 3. 지표별 성능 평가
            indicator_performance = self._evaluate_indicator_performance(historical_data)
            
            # 4. 리드 타임 분석
            lead_time_analysis = self._analyze_lead_times(historical_data)
            
            # 5. 시장 상황별 성능
            market_condition_performance = self._analyze_market_conditions(historical_data)
            
            # 6. 종합 결과 집계
            self.results = self._aggregate_results({
                "prediction_model": prediction_results,
                "indicator_performance": indicator_performance,
                "lead_time_analysis": lead_time_analysis,
                "market_conditions": market_condition_performance,
                "period": {
                    "start": start_date,
                    "end": end_date,
                    "trading_days": len(historical_data.get('dates', []))
                }
            })
            
            return self.results
            
        except Exception as e:
            return {
                "error": f"백테스팅 실행 중 오류: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }
    
    def _load_historical_data(self, start_date: str, end_date: str) -> Dict:
        """역사적 데이터 로드"""
        try:
            # 날짜 범위 생성
            dates = pd.date_range(start=start_date, end=end_date, freq='D')
            trading_days = self._filter_trading_days(dates)
            
            # 모의 역사적 데이터 생성
            historical_data = {
                "dates": trading_days,
                "spy_prices": self._generate_price_series(trading_days),
                "vix_data": self._generate_vix_series(trading_days),
                "put_call_ratios": self._generate_put_call_series(trading_days),
                "ad_breadth": self._generate_ad_breadth_series(trading_days),
                "new_highs_lows": self._generate_new_highs_lows_series(trading_days),
                "sentiment_data": self._generate_sentiment_series(trading_days),
                "market_events": self._generate_market_events(trading_days)
            }
            
            return historical_data
            
        except Exception as e:
            print(f"역사적 데이터 로드 실패: {str(e)}")
            return {}
    
    def _filter_trading_days(self, dates: pd.DatetimeIndex) -> List[pd.Timestamp]:
        """거래일 필터링"""
        trading_days = []
        
        for date in dates:
            # 주말 제외
            if date.weekday() < 5:
                # 간단한 휴장일 제거 (실제로는 휴장일 캘린더 사용)
                if not self._is_holiday(date):
                    trading_days.append(date)
        
        return trading_days
    
    def _is_holiday(self, date: pd.Timestamp) -> bool:
        """휴장일 확인 (간단화)"""
        # 주요 휴장일들 (실제로는 더 정확한 캘린더 필요)
        major_holidays = [
            (1, 1),   # 신정
            (7, 4),   # 미국 독립기념일
            (12, 25), # 크리스마스
        ]
        
        for month, day in major_holidays:
            if date.month == month and date.day == day:
                return True
        
        return False
    
    def _generate_price_series(self, dates: List[pd.Timestamp]) -> pd.Series:
        """가격 시리즈 생성 (모의 역사적 데이터)"""
        n_days = len(dates)
        
        # 기본 추세 + 변동성
        trend = np.random.normal(0.0005, 0.001, n_days)  # 일일 수익률
        volatility = np.random.normal(0, 0.02, n_days)    # 변동성
        
        # 시장 붕괴 시점 추가
        crash_points = np.random.choice(n_days, size=max(1, n_days // 500), replace=False)
        for crash_point in crash_points:
            if crash_point < n_days - 5:
                trend[crash_point:crash_point+5] = np.random.normal(-0.05, 0.02, 5)
        
        # 가격 계산
        returns = trend + volatility
        prices = 100 * np.exp(np.cumsum(returns))
        
        return pd.Series(prices, index=dates)
    
    def _generate_vix_series(self, dates: List[pd.Timestamp]) -> pd.Series:
        """VIX 시리즈 생성"""
        n_days = len(dates)
        
        # VIX는 역사적 변동성과 상관관계
        base_vix = 20
        vix_values = []
        
        for i in range(n_days):
            if i == 0:
                vix = base_vix
            else:
                # 평균 회귀 특성
                change = np.random.normal(0, 2)
                vix = max(5, min(80, vix_values[-1] * 0.95 + base_vix * 0.05 + change))
            
            vix_values.append(vix)
        
        return pd.Series(vix_values, index=dates)
    
    def _generate_put_call_series(self, dates: List[pd.Timestamp]) -> pd.Series:
        """Put-Call Ratio 시리즈 생성"""
        n_days = len(dates)
        
        # VIX와 상관관계
        vix_series = self._generate_vix_series(dates)
        
        put_call_ratios = []
        for i, date in enumerate(dates):
            vix = vix_series.iloc[i]
            
            # VIX가 높을수록 Put-Call Ratio가 높아짐
            base_ratio = 0.3 + (vix / 80) * 1.2
            noise = np.random.normal(0, 0.1)
            
            ratio = max(0.2, min(2.0, base_ratio + noise))
            put_call_ratios.append(ratio)
        
        return pd.Series(put_call_ratios, index=dates)
    
    def _generate_ad_breadth_series(self, dates: List[pd.Timestamp]) -> pd.Series:
        """Advance/Decline Breadth 시리즈 생성"""
        n_days = len(dates)
        
        ad_ratios = []
        for i in range(n_days):
            # 시장 상태에 따른 A/D 비율
            if i == 0:
                ratio = 0.6
            else:
                change = np.random.normal(0, 0.1)
                ratio = max(0.2, min(0.9, ad_ratios[-1] + change))
            
            ad_ratios.append(ratio)
        
        return pd.Series(ad_ratios, index=dates)
    
    def _generate_new_highs_lows_series(self, dates: List[pd.Timestamp]) -> Dict:
        """New Highs/New Lows 시리즈 생성"""
        n_days = len(dates)
        
        new_highs = []
        new_lows = []
        
        for i in range(n_days):
            # 시장 상태에 따른 신고가/신저가 수
            if i == 0:
                high, low = 150, 50
            else:
                # 이전 값 기반 변화
                high_change = np.random.normal(0, 30)
                low_change = np.random.normal(0, 20)
                
                high = max(10, min(500, new_highs[-1] + high_change))
                low = max(5, min(300, new_lows[-1] + low_change))
            
            new_highs.append(int(high))
            new_lows.append(int(low))
        
        return {
            "new_highs": pd.Series(new_highs, index=dates),
            "new_lows": pd.Series(new_lows, index=dates)
        }
    
    def _generate_sentiment_series(self, dates: List[pd.Timestamp]) -> Dict:
        """투자자 심리 시리즈 생성"""
        n_days = len(dates)
        
        bullish = []
        bearish = []
        
        for i in range(n_days):
            # 심리 지표는 서서히 변화
            if i == 0:
                bull, bear = 40, 30
            else:
                bull_change = np.random.normal(0, 3)
                bear_change = np.random.normal(0, 3)
                
                bull = max(10, min(70, bullish[-1] + bull_change))
                bear = max(10, min(60, bearish[-1] + bear_change))
            
            bullish.append(bull)
            bearish.append(bear)
        
        return {
            "bullish": pd.Series(bullish, index=dates),
            "bearish": pd.Series(bearish, index=dates)
        }
    
    def _generate_market_events(self, dates: List[pd.Timestamp]) -> pd.Series:
        """시장 이벤트 생성 (붕괴/회복)"""
        n_days = len(dates)
        
        events = []
        for i in range(n_days):
            # 대부분은 정상
            event = "NORMAL"
            
            # 무작위로 시장 붕괴/회복 이벤트 생성
            if np.random.random() < 0.02:  # 2% 확률
                event = np.random.choice(["CRASH", "RECOVERY"])
            
            events.append(event)
        
        return pd.Series(events, index=dates)
    
    def _backtest_prediction_model(self, historical_data: Dict) -> Dict:
        """예측 모델 백테스팅"""
        try:
            if not historical_data:
                return {"error": "역사적 데이터 없음"}
            
            dates = historical_data["dates"]
            predictions = []
            actuals = []
            
            for i, date in enumerate(dates):
                if i < 20:  # 초기 20일은 분석 제외
                    continue
                
                # 과거 데이터 기반 예측 생성
                prediction = self._generate_historical_prediction(
                    historical_data, i
                )
                
                # 실제 시장 결과
                actual = self._determine_actual_outcome(
                    historical_data, i
                )
                
                predictions.append(prediction)
                actuals.append(actual)
            
            # 성능 평가
            performance = self._evaluate_prediction_performance(predictions, actuals)
            
            return {
                "predictions": len(predictions),
                "performance": performance,
                "prediction_accuracy": performance.get("accuracy", 0)
            }
            
        except Exception as e:
            return {"error": f"예측 모델 백테스팅 실패: {str(e)}"}
    
    def _generate_historical_prediction(self, historical_data: Dict, 
                                      index: int) -> Dict:
        """역사적 데이터 기반 예측 생성"""
        try:
            # 과거 20일 데이터 사용
            window_start = max(0, index - 20)
            window_end = index
            
            # 지표 데이터 추출
            indicators = self._extract_historical_indicators(
                historical_data, window_start, window_end
            )
            
            # 과열 점수 계산
            heat_score = self._calculate_historical_heat_score(indicators)
            
            # 예측 신호 생성
            if heat_score >= self.config["prediction_thresholds"]["critical"]:
                prediction = "CRITICAL_WARNING"
            elif heat_score >= self.config["prediction_thresholds"]["warning"]:
                prediction = "MODERATE_WARNING"
            else:
                prediction = "NORMAL"
            
            return {
                "prediction": prediction,
                "heat_score": heat_score,
                "indicators": indicators
            }
            
        except:
            return {"prediction": "NORMAL", "heat_score": 50}
    
    def _extract_historical_indicators(self, historical_data: Dict, 
                                     start: int, end: int) -> Dict:
        """역사적 지표 추출"""
        try:
            dates = historical_data["dates"][start:end+1]
            
            indicators = {}
            
            # VIX Put-Call Ratio
            if "put_call_ratios" in historical_data:
                pc_ratios = historical_data["put_call_ratios"].iloc[start:end+1]
                indicators["vix_put_call"] = {
                    "current": pc_ratios.iloc[-1],
                    "ma_5d": pc_ratios.tail(5).mean(),
                    "trend": self._calculate_trend(pc_ratios)
                }
            
            # A/D Breadth
            if "ad_breadth" in historical_data:
                ad_ratios = historical_data["ad_breadth"].iloc[start:end+1]
                indicators["ad_breadth"] = {
                    "current": ad_ratios.iloc[-1],
                    "trend": self._calculate_trend(ad_ratios)
                }
            
            # New Highs/Lows
            if "new_highs_lows" in historical_data:
                new_highs = historical_data["new_highs_lows"]["new_highs"].iloc[start:end+1]
                new_lows = historical_data["new_highs_lows"]["new_lows"].iloc[start:end+1]
                
                total_issues = new_highs + new_lows
                high_low_ratio = new_highs / total_issues
                
                indicators["new_highs_lows"] = {
                    "ratio": high_low_ratio.iloc[-1],
                    "trend": self._calculate_trend(high_low_ratio)
                }
            
            # Sentiment
            if "sentiment_data" in historical_data:
                bullish = historical_data["sentiment_data"]["bullish"].iloc[start:end+1]
                bearish = historical_data["sentiment_data"]["bearish"].iloc[start:end+1]
                
                sentiment_score = (bullish - bearish) / 100
                
                indicators["sentiment"] = {
                    "score": sentiment_score.iloc[-1],
                    "trend": self._calculate_trend(sentiment_score)
                }
            
            return indicators
            
        except:
            return {}
    
    def _calculate_trend(self, series: pd.Series) -> float:
        """추세 계산"""
        try:
            if len(series) < 2:
                return 0.0
            
            x = np.arange(len(series))
            y = series.values
            
            slope = np.polyfit(x, y, 1)[0]
            
            # 정규화
            normalized_slope = slope / np.mean(np.abs(y)) if np.mean(np.abs(y)) > 0 else 0
            
            return normalized_slope
            
        except:
            return 0.0
    
    def _calculate_historical_heat_score(self, indicators: Dict) -> float:
        """역사적 과열 점수 계산"""
        try:
            scores = []
            weights = {"vix_put_call": 0.3, "ad_breadth": 0.25, "new_highs_lows": 0.25, "sentiment": 0.2}
            
            for indicator, data in indicators.items():
                weight = weights.get(indicator, 0.25)
                
                if indicator == "vix_put_call":
                    score = self._score_vix_put_call(data["current"])
                elif indicator == "ad_breadth":
                    score = self._score_ad_breadth(data["trend"])
                elif indicator == "new_highs_lows":
                    score = self._score_new_highs_lows(data["ratio"])
                elif indicator == "sentiment":
                    score = self._score_sentiment(data["score"])
                else:
                    score = 50
                
                scores.append(score * weight)
            
            return sum(scores)
            
        except:
            return 50
    
    def _score_vix_put_call(self, ratio: float) -> float:
        """VIX Put-Call Ratio 점수"""
        if ratio < 0.7:
            return 80 + (0.7 - ratio) * 100
        elif ratio > 1.0:
            return 20
        else:
            return 50
    
    def _score_ad_breadth(self, trend: float) -> float:
        """A/D Breadth 점수"""
        if trend < -0.1:  # 하락 추세
            return 75
        else:
            return 40
    
    def _score_new_highs_lows(self, ratio: float) -> float:
        """New Highs/Lows 점수"""
        if ratio < 0.3:
            return 70
        elif ratio > 0.8:
            return 60
        else:
            return 45
    
    def _score_sentiment(self, score: float) -> float:
        """심리 지표 점수"""
        if score > 0.3:
            return 75 + score * 25
        elif score < -0.3:
            return 30
        else:
            return 50
    
    def _determine_actual_outcome(self, historical_data: Dict, 
                                index: int) -> str:
        """실제 시장 결과 확인"""
        try:
            # 예측 후 5-20일 내 시장 붕괴 확인
            future_window = min(20, len(historical_data["dates"]) - index - 1)
            
            if future_window < 5:
                return "NORMAL"
            
            # 미래 가격 변화 확인
            prices = historical_data["spy_prices"]
            current_price = prices.iloc[index]
            
            # 5일 후, 10일 후, 20일 후 가격
            future_prices = []
            for days in [5, 10, 20]:
                if index + days < len(prices):
                    future_prices.append(prices.iloc[index + days])
            
            if not future_prices:
                return "NORMAL"
            
            # 최대 하락률 확인
            max_decline = max((current_price - fp) / current_price for fp in future_prices)
            
            # 시장 이벤트 확인
            events = historical_data.get("market_events", pd.Series())
            if index < len(events):
                future_events = events.iloc[index+1:index+6]
                if "CRASH" in future_events.values:
                    return "CRASH_OCCURRED"
            
            # 하락률 기반 판정
            if max_decline > 0.10:  # 10% 이상 하락
                return "CRASH_OCCURRED"
            elif max_decline > 0.05:  # 5% 이상 하락
                return "DECLINE_OCCURRED"
            else:
                return "NORMAL"
                
        except:
            return "NORMAL"
    
    def _evaluate_prediction_performance(self, predictions: List[Dict], 
                                      actuals: List[str]) -> Dict:
        """예측 성능 평가"""
        try:
            # 예측 및 실제 결과 변환
            pred_labels = []
            actual_labels = []
            
            for pred, actual in zip(predictions, actuals):
                # 예측 레이블
                if pred["prediction"] in ["CRITICAL_WARNING", "MODERATE_WARNING"]:
                    pred_label = 1  # 경고
                else:
                    pred_label = 0  # 정상
                
                # 실제 레이블
                if actual in ["CRASH_OCCURRED", "DECLINE_OCCURRED"]:
                    actual_label = 1  # 붕괴/하락
                else:
                    actual_label = 0  # 정상
                
                pred_labels.append(pred_label)
                actual_labels.append(actual_label)
            
            # 성능 지표 계산
            accuracy = accuracy_score(actual_labels, pred_labels)
            precision = precision_score(actual_labels, pred_labels, zero_division=0)
            recall = recall_score(actual_labels, pred_labels, zero_division=0)
            f1 = f1_score(actual_labels, pred_labels, zero_division=0)
            
            # 히트율 계산
            hit_rate = sum(1 for p, a in zip(pred_labels, actual_labels) if p == a) / len(pred_labels)
            
            # False Positive Rate
            fp = sum(1 for p, a in zip(pred_labels, actual_labels) if p == 1 and a == 0)
            tn = sum(1 for p, a in zip(pred_labels, actual_labels) if p == 0 and a == 0)
            fpr = fp / (fp + tn) if (fp + tn) > 0 else 0
            
            return {
                "accuracy": accuracy,
                "precision": precision,
                "recall": recall,
                "f1_score": f1,
                "hit_rate": hit_rate,
                "false_positive_rate": fpr,
                "total_predictions": len(predictions),
                "correct_predictions": sum(1 for p, a in zip(pred_labels, actual_labels) if p == a)
            }
            
        except Exception as e:
            return {"error": f"성능 평가 실패: {str(e)}"}
    
    def _evaluate_indicator_performance(self, historical_data: Dict) -> Dict:
        """지표별 성능 평가"""
        try:
            indicator_performance = {}
            
            indicators = ["vix_put_call", "ad_breadth", "new_highs_lows", "sentiment"]
            
            for indicator in indicators:
                # 개별 지표 성능 평가
                performance = self._evaluate_single_indicator(
                    historical_data, indicator
                )
                indicator_performance[indicator] = performance
            
            return indicator_performance
            
        except Exception as e:
            return {"error": f"지표 성능 평가 실패: {str(e)}"}
    
    def _evaluate_single_indicator(self, historical_data: Dict, 
                                 indicator_name: str) -> Dict:
        """개별 지표 성능 평가"""
        try:
            # 지표 데이터 추출
            indicator_data = self._extract_indicator_series(
                historical_data, indicator_name
            )
            
            if indicator_data.empty:
                return {"error": f"{indicator_name} 데이터 없음"}
            
            # 지표 기반 예측 성능 평가
            predictions = []
            actuals = []
            
            for i in range(20, len(indicator_data)):
                # 과열 신호 확인
                signal = self._check_indicator_overheat_signal(
                    indicator_data.iloc[i-20:i+1], indicator_name
                )
                
                # 실제 결과 확인
                actual = self._determine_actual_outcome(historical_data, i)
                
                predictions.append(signal)
                actuals.append(actual)
            
            # 성능 계산
            performance = self._calculate_indicator_performance(predictions, actuals)
            
            return performance
            
        except Exception as e:
            return {"error": f"{indicator_name} 평가 실패: {str(e)}"}
    
    def _extract_indicator_series(self, historical_data: Dict, 
                                 indicator_name: str) -> pd.Series:
        """지표 시리즈 추출"""
        try:
            if indicator_name == "vix_put_call":
                return historical_data.get("put_call_ratios", pd.Series())
            elif indicator_name == "ad_breadth":
                return historical_data.get("ad_breadth", pd.Series())
            elif indicator_name == "new_highs_lows":
                new_highs = historical_data["new_highs_lows"]["new_highs"]
                new_lows = historical_data["new_highs_lows"]["new_lows"]
                return new_highs / (new_highs + new_lows)
            elif indicator_name == "sentiment":
                bullish = historical_data["sentiment_data"]["bullish"]
                bearish = historical_data["sentiment_data"]["bearish"]
                return (bullish - bearish) / 100
            else:
                return pd.Series()
                
        except:
            return pd.Series()
    
    def _check_indicator_overheat_signal(self, indicator_series: pd.Series, 
                                        indicator_name: str) -> str:
        """지표 과열 신호 확인"""
        try:
            current_value = indicator_series.iloc[-1]
            
            if indicator_name == "vix_put_call":
                return "WARNING" if current_value < 0.7 else "NORMAL"
            elif indicator_name == "ad_breadth":
                trend = self._calculate_trend(indicator_series)
                return "WARNING" if trend < -0.1 else "NORMAL"
            elif indicator_name == "new_highs_lows":
                return "WARNING" if current_value < 0.3 else "NORMAL"
            elif indicator_name == "sentiment":
                return "WARNING" if current_value > 0.3 else "NORMAL"
            else:
                return "NORMAL"
                
        except:
            return "NORMAL"
    
    def _calculate_indicator_performance(self, predictions: List[str], 
                                      actuals: List[str]) -> Dict:
        """지표 성능 계산"""
        try:
            # 이진 분류로 변환
            pred_binary = [1 if p == "WARNING" else 0 for p in predictions]
            actual_binary = [1 if a in ["CRASH_OCCURRED", "DECLINE_OCCURRED"] else 0 for a in actuals]
            
            # 성능 지표
            accuracy = accuracy_score(actual_binary, pred_binary)
            precision = precision_score(actual_binary, pred_binary, zero_division=0)
            recall = recall_score(actual_binary, pred_binary, zero_division=0)
            
            return {
                "accuracy": accuracy,
                "precision": precision,
                "recall": recall,
                "total_signals": len(predictions),
                "warning_signals": sum(pred_binary),
                "actual_crashes": sum(actual_binary)
            }
            
        except:
            return {"error": "성능 계산 실패"}
    
    def _analyze_lead_times(self, historical_data: Dict) -> Dict:
        """리드 타임 분석"""
        try:
            lead_time_results = {}
            
            for window in self.config["lead_time_windows"]:
                # 각 리드 타임 윈도우별 성능 분석
                performance = self._analyze_lead_time_window(
                    historical_data, window
                )
                lead_time_results[f"{window}_days"] = performance
            
            return lead_time_results
            
        except Exception as e:
            return {"error": f"리드 타임 분석 실패: {str(e)}"}
    
    def _analyze_lead_time_window(self, historical_data: Dict, 
                                 lead_days: int) -> Dict:
        """특정 리드 타임 윈도우 분석"""
        try:
            predictions = []
            actuals = []
            
            dates = historical_data["dates"]
            
            for i in range(20, len(dates) - lead_days):
                # 예측 생성
                prediction = self._generate_historical_prediction(
                    historical_data, i
                )
                
                # 리드 타임 후 실제 결과 확인
                future_index = i + lead_days
                actual = self._determine_actual_outcome(historical_data, future_index)
                
                predictions.append(prediction["prediction"])
                actuals.append(actual)
            
            # 성능 평가
            performance = self._evaluate_prediction_performance(predictions, actuals)
            
            return {
                "lead_days": lead_days,
                "performance": performance,
                "sample_size": len(predictions)
            }
            
        except:
            return {"error": f"리드 타임 {lead_days}일 분석 실패"}
    
    def _analyze_market_conditions(self, historical_data: Dict) -> Dict:
        """시장 상황별 성능 분석"""
        try:
            market_conditions = {
                "bull_market": self._analyze_bull_market_performance(historical_data),
                "bear_market": self._analyze_bear_market_performance(historical_data),
                "sideways_market": self._analyze_sideways_market_performance(historical_data),
                "high_volatility": self._analyze_high_volatility_performance(historical_data)
            }
            
            return market_conditions
            
        except Exception as e:
            return {"error": f"시장 상황 분석 실패: {str(e)}"}
    
    def _analyze_bull_market_performance(self, historical_data: Dict) -> Dict:
        """상승장 성능 분석"""
        try:
            # 상승장 기간 식별
            bull_periods = self._identify_bull_market_periods(historical_data)
            
            if not bull_periods:
                return {"error": "상승장 기간 없음"}
            
            # 상승장 기간별 성능 평가
            performances = []
            
            for start, end in bull_periods:
                period_data = self._extract_period_data(historical_data, start, end)
                performance = self._backtest_prediction_model(period_data)
                performances.append(performance.get("prediction_accuracy", 0))
            
            return {
                "periods_identified": len(bull_periods),
                "average_accuracy": np.mean(performances) if performances else 0,
                "performance_range": [min(performances), max(performances)] if performances else [0, 0]
            }
            
        except:
            return {"error": "상승장 분석 실패"}
    
    def _identify_bull_market_periods(self, historical_data: Dict) -> List[Tuple[int, int]]:
        """상승장 기간 식별"""
        try:
            prices = historical_data["spy_prices"]
            bull_periods = []
            
            # 50일 이동평균 기준 상승장 식별
            ma_50 = prices.rolling(50).mean()
            
            in_bull_market = False
            start_idx = None
            
            for i in range(50, len(prices)):
                if prices.iloc[i] > ma_50.iloc[i] and not in_bull_market:
                    # 상승장 시작
                    in_bull_market = True
                    start_idx = i
                elif prices.iloc[i] <= ma_50.iloc[i] and in_bull_market:
                    # 상승장 종료
                    in_bull_market = False
                    if start_idx is not None:
                        bull_periods.append((start_idx, i))
                        start_idx = None
            
            return bull_periods
            
        except:
            return []
    
    def _extract_period_data(self, historical_data: Dict, 
                            start: int, end: int) -> Dict:
        """특정 기간 데이터 추출"""
        try:
            dates = historical_data["dates"][start:end+1]
            
            period_data = {
                "dates": dates,
                "spy_prices": historical_data["spy_prices"].iloc[start:end+1],
                "vix_data": historical_data["vix_data"].iloc[start:end+1],
                "put_call_ratios": historical_data["put_call_ratios"].iloc[start:end+1],
                "ad_breadth": historical_data["ad_breadth"].iloc[start:end+1],
                "new_highs_lows": {
                    "new_highs": historical_data["new_highs_lows"]["new_highs"].iloc[start:end+1],
                    "new_lows": historical_data["new_highs_lows"]["new_lows"].iloc[start:end+1]
                },
                "sentiment_data": {
                    "bullish": historical_data["sentiment_data"]["bullish"].iloc[start:end+1],
                    "bearish": historical_data["sentiment_data"]["bearish"].iloc[start:end+1]
                },
                "market_events": historical_data["market_events"].iloc[start:end+1]
            }
            
            return period_data
            
        except:
            return {}
    
    def _analyze_bear_market_performance(self, historical_data: Dict) -> Dict:
        """하락장 성능 분석"""
        # 상승장 분석과 유사한 로직 (간단화)
        return {"message": "하락장 분석은 상승장과 유사한 로직으로 구현"}
    
    def _analyze_sideways_market_performance(self, historical_data: Dict) -> Dict:
        """횡보장 성능 분석"""
        return {"message": "횡보장 분석은 유사한 로직으로 구현"}
    
    def _analyze_high_volatility_performance(self, historical_data: Dict) -> Dict:
        """고변동성 성능 분석"""
        return {"message": "고변동성 분석은 유사한 로직으로 구현"}
    
    def _aggregate_results(self, all_results: Dict) -> Dict:
        """종합 결과 집계"""
        try:
            # 전체 정확도 계산
            overall_accuracy = 0
            
            if "prediction_model" in all_results:
                pred_accuracy = all_results["prediction_model"].get("prediction_accuracy", 0)
                overall_accuracy = pred_accuracy
            
            # 지표별 평균 정확도
            indicator_accuracies = []
            if "indicator_performance" in all_results:
                for indicator, performance in all_results["indicator_performance"].items():
                    if "accuracy" in performance:
                        indicator_accuracies.append(performance["accuracy"])
            
            avg_indicator_accuracy = np.mean(indicator_accuracies) if indicator_accuracies else 0
            
            # 종합 평가
            if overall_accuracy >= 0.75:
                grade = "A"
                assessment = "우수"
            elif overall_accuracy >= 0.65:
                grade = "B"
                assessment = "양호"
            elif overall_accuracy >= 0.55:
                grade = "C"
                assessment = "보통"
            else:
                grade = "D"
                assessment = "개선 필요"
            
            return {
                "overall_accuracy": overall_accuracy,
                "average_indicator_accuracy": avg_indicator_accuracy,
                "grade": grade,
                "assessment": assessment,
                "detailed_results": all_results,
                "recommendations": self._generate_recommendations(overall_accuracy),
                "timestamp": datetime.now().isoformat()
            }
            
        except Exception as e:
            return {"error": f"결과 집계 실패: {str(e)}"}
    
    def _generate_recommendations(self, accuracy: float) -> List[str]:
        """개선 권고사항 생성"""
        recommendations = []
        
        if accuracy < 0.55:
            recommendations.extend([
                "지표 가중치 재조정 필요",
                "추가 선행지표 고려",
                "예측 모델 개선"
            ])
        elif accuracy < 0.65:
            recommendations.extend([
                "파라미터 튜닝 고려",
                "시장 상황별 모델 분리 검토"
            ])
        elif accuracy < 0.75:
            recommendations.extend([
                "리드 타임 최적화",
                "거래 비용 고려"
            ])
        else:
            recommendations.extend([
                "실제 거래 환경 검증",
                "리스크 관리 강화"
            ])
        
        return recommendations

def main():
    """메인 실행 함수"""
    engine = BacktestEngine()
    
    print("=== 선행지표 예측 모델 백테스팅 ===")
    
    # 1년간 백테스팅 실행
    results = engine.run_comprehensive_backtest("2023-01-01")
    
    if "error" in results:
        print(f"백테스팅 실패: {results['error']}")
        return
    
    print(f"\n=== 종합 결과 ===")
    print(f"전체 정확도: {results['overall_accuracy']:.2%}")
    print(f"평가 등급: {results['grade']} ({results['assessment']})")
    print(f"지표별 평균 정확도: {results['average_indicator_accuracy']:.2%}")
    
    print(f"\n=== 권고사항 ===")
    for i, rec in enumerate(results['recommendations'], 1):
        print(f"{i}. {rec}")
    
    print(f"\n=== 상세 성능 ===")
    if "prediction_model" in results["detailed_results"]:
        pred_perf = results["detailed_results"]["prediction_model"]["performance"]
        print(f"정확도: {pred_perf.get('accuracy', 0):.2%}")
        print(f"정밀도: {pred_perf.get('precision', 0):.2%}")
        print(f"재현율: {pred_perf.get('recall', 0):.2%}")
        print(f"F1 점수: {pred_perf.get('f1_score', 0):.2%}")

if __name__ == "__main__":
    main()