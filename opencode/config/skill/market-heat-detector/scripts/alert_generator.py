#!/usr/bin/env python3
"""
Alert Generator - Early Warning System

시장 과열 예측 신호를 바탕으로 조기 경고를 생성하는 시스템입니다.
다양한 경고 레벨과 포맷으로 사용자에게 알림을 제공합니다.
"""

import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Union
import json
from dataclasses import dataclass
from enum import Enum
import warnings
warnings.filterwarnings('ignore')

class AlertLevel(Enum):
    """경고 레벨"""
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    EMERGENCY = "EMERGENCY"

class AlertType(Enum):
    """경고 타입"""
    HEAT_WARNING = "HEAT_WARNING"
    DIVERGENCE_WARNING = "DIVERGENCE_WARNING"
    COMPOSITE_WARNING = "COMPOSITE_WARNING"
    MARKET_CRASH_WARNING = "MARKET_CRASH_WARNING"

@dataclass
class AlertSignal:
    """경고 신호 데이터 클래스"""
    alert_type: AlertType
    alert_level: AlertLevel
    confidence: float
    heat_score: float
    indicators: Dict
    divergence_signals: List[Dict]
    prediction: Dict
    timestamp: str
    message: str
    recommendations: List[str]
    lead_time: str

class AlertGenerator:
    """경고 생성기 클래스"""
    
    def __init__(self, config: Dict = None):
        """
        초기화
        
        Args:
            config: 설정 딕셔너리
        """
        self.config = config or self._default_config()
        self.alert_history = []
        
    def _default_config(self) -> Dict:
        """기본 설정"""
        return {
            "thresholds": {
                "heat_warning": 60,
                "heat_critical": 80,
                "heat_emergency": 90,
                "confidence_minimum": 0.60,
                "divergence_minimum": 2
            },
            "alert_channels": ["console", "file", "email"],
            "alert_formats": ["text", "json", "markdown"],
            "notification_settings": {
                "rate_limit_minutes": 30,
                "max_alerts_per_hour": 10,
                "quiet_hours": {"start": 22, "end": 6}
            }
        }
    
    def generate_alert(self, analysis_result: Dict) -> Optional[AlertSignal]:
        """
        분석 결과 기반 경고 생성
        
        Args:
            analysis_result: 시장 과열 분석 결과
            
        Returns:
            AlertSignal: 생성된 경고 신호
        """
        try:
            # 1. 경고 레벨 결정
            alert_level = self._determine_alert_level(analysis_result)
            
            # 2. 경고 타입 결정
            alert_type = self._determine_alert_type(analysis_result)
            
            # 3. 신뢰도 확인
            if not self._check_confidence_threshold(analysis_result):
                return None
            
            # 4. 경고 메시지 생성
            alert_message = self._generate_alert_message(
                alert_level, alert_type, analysis_result
            )
            
            # 5. 권고사항 생성
            recommendations = self._generate_recommendations(
                alert_level, analysis_result
            )
            
            # 6. 경고 신호 생성
            alert_signal = AlertSignal(
                alert_type=alert_type,
                alert_level=alert_level,
                confidence=analysis_result.get("prediction", {}).get("confidence", 0),
                heat_score=analysis_result.get("heat_score", 0),
                indicators=analysis_result.get("indicators", {}),
                divergence_signals=analysis_result.get("divergence_signals", []),
                prediction=analysis_result.get("prediction", {}),
                timestamp=analysis_result.get("timestamp", datetime.now().isoformat()),
                message=alert_message,
                recommendations=recommendations,
                lead_time=analysis_result.get("prediction", {}).get("lead_time", "N/A")
            )
            
            # 7. 경고 기록 저장
            self.alert_history.append(alert_signal)
            
            return alert_signal
            
        except Exception as e:
            print(f"경고 생성 중 오류: {str(e)}")
            return None
    
    def _determine_alert_level(self, analysis_result: Dict) -> AlertLevel:
        """경고 레벨 결정"""
        heat_score = analysis_result.get("heat_score", 0)
        confidence = analysis_result.get("prediction", {}).get("confidence", 0)
        
        # 다이버전스 신호 확인
        divergence_count = len(analysis_result.get("divergence_signals", []))
        
        # 종합적인 경고 레벨 결정
        if heat_score >= self.config["thresholds"]["heat_emergency"] and confidence >= 80:
            return AlertLevel.EMERGENCY
        elif heat_score >= self.config["thresholds"]["heat_critical"] and confidence >= 70:
            return AlertLevel.CRITICAL
        elif heat_score >= self.config["thresholds"]["heat_warning"] and confidence >= 60:
            return AlertLevel.WARNING
        elif divergence_count >= self.config["thresholds"]["divergence_minimum"]:
            return AlertLevel.WARNING
        else:
            return AlertLevel.NORMAL
    
    def _determine_alert_type(self, analysis_result: Dict) -> AlertType:
        """경고 타입 결정"""
        heat_score = analysis_result.get("heat_score", 0)
        divergence_signals = analysis_result.get("divergence_signals", [])
        
        # 다이버전스 신호 확인
        composite_divergence = any(
            signal["type"] == "COMPOSITE_BEARISH_DIVERGENCE" 
            for signal in divergence_signals
        )
        
        if heat_score >= 90:
            return AlertType.MARKET_CRASH_WARNING
        elif composite_divergence:
            return AlertType.COMPOSITE_WARNING
        elif divergence_signals:
            return AlertType.DIVERGENCE_WARNING
        elif heat_score >= 60:
            return AlertType.HEAT_WARNING
        else:
            return AlertType.HEAT_WARNING  # 기본값
    
    def _check_confidence_threshold(self, analysis_result: Dict) -> bool:
        """신뢰도 임계값 확인"""
        confidence = analysis_result.get("prediction", {}).get("confidence", 0)
        return confidence >= self.config["thresholds"]["confidence_minimum"]
    
    def _generate_alert_message(self, alert_level: AlertLevel, 
                              alert_type: AlertType, 
                              analysis_result: Dict) -> str:
        """경고 메시지 생성"""
        heat_score = analysis_result.get("heat_score", 0)
        confidence = analysis_result.get("prediction", {}).get("confidence", 0)
        
        # 기본 메시지 템플릿
        message_templates = {
            AlertLevel.WARNING: f"⚠️ 시장 과열 경고 (과열 점수: {heat_score}/100, 신뢰도: {confidence}%)",
            AlertLevel.CRITICAL: f"🚨 긴급 시장 과열 경고 (과열 점수: {heat_score}/100, 신뢰도: {confidence}%)",
            AlertLevel.EMERGENCY: f"🆘 시장 붕괴 경고 (과열 점수: {heat_score}/100, 신뢰도: {confidence}%)",
            AlertLevel.NORMAL: f"ℹ️ 시장 정상 상태 (과열 점수: {heat_score}/100)"
        }
        
        base_message = message_templates.get(alert_level, message_templates[AlertLevel.WARNING])
        
        # 경고 타입별 추가 정보
        type_specific_info = self._get_type_specific_info(alert_type, analysis_result)
        
        return f"{base_message}\n{type_specific_info}"
    
    def _get_type_specific_info(self, alert_type: AlertType, 
                              analysis_result: Dict) -> str:
        """경고 타입별 특정 정보"""
        if alert_type == AlertType.DIVERGENCE_WARNING:
            divergences = analysis_result.get("divergence_signals", [])
            divergence_count = len(divergences)
            
            if divergence_count > 0:
                divergence_types = [d["type"] for d in divergences]
                return f"📊 {divergence_count}개 다이버전스 신호 감지: {', '.join(divergence_types)}"
        
        elif alert_type == AlertType.COMPOSITE_WARNING:
            return "🔄 다중 지표에서 동시에 다이버전스 감지. 강력한 반전 신호 가능성."
        
        elif alert_type == AlertType.MARKET_CRASH_WARNING:
            return "💥 역사적 데이터와 유사한 시장 붕괴 패턴 감지. 즉각적인 주의 필요."
        
        elif alert_type == AlertType.HEAT_WARNING:
            indicators = analysis_result.get("indicators", {})
            overheat_indicators = []
            
            for name, data in indicators.items():
                if data.get("signal") in ["OVERHEATED", "WARNING"]:
                    overheat_indicators.append(name)
            
            if overheat_indicators:
                return f"🌡️ 과열 지표: {', '.join(overheat_indicators)}"
        
        return ""
    
    def _generate_recommendations(self, alert_level: AlertLevel, 
                                 analysis_result: Dict) -> List[str]:
        """권고사항 생성"""
        recommendations = []
        
        if alert_level == AlertLevel.EMERGENCY:
            recommendations.extend([
                "🛑 모든 롱 포지션 즉시 청산 고려",
                "🛡️ 캐시 비중 50% 이상 확보",
                "📉 방어적 자산(금, 채권) 비중 증가",
                "⏰ 시장 안정화까지 관망"
            ])
        
        elif alert_level == AlertLevel.CRITICAL:
            recommendations.extend([
                "⚖️ 포트폴리오 리밸런싱 실행",
                "📊 레버리지 비중 축소",
                "🎯 손실 한미(Stop Loss) 설정 강화",
                "📈 변동성 확대에 대비한 헷지 고려"
            ])
        
        elif alert_level == AlertLevel.WARNING:
            recommendations.extend([
                "🔍 시장 상황 면밀히 관찰",
                "📝 새로운 진입 자제",
                "⚖️ 기존 포지션 재평가",
                "📊 추가 확인 지표 모니터링"
            ])
        
        # 다이버전스 특정 권고사항
        divergence_signals = analysis_result.get("divergence_signals", [])
        if divergence_signals:
            bearish_divergences = [
                d for d in divergence_signals 
                if "BEARISH" in d["type"]
            ]
            
            if bearish_divergences:
                recommendations.append("🐻 베어리시 다이버전스: 상승 추세 약화 가능성")
        
        # 리드 타임 기반 권고사항
        lead_time = analysis_result.get("prediction", {}).get("lead_time", "")
        if "1-2주" in lead_time:
            recommendations.append("⏰ 단기적 조정 필요: 1-2주 내 변동성 확대 가능성")
        elif "2-4주" in lead_time:
            recommendations.append("📅 중기적 대비 필요: 2-4주 내 추세 전환 가능성")
        
        return recommendations
    
    def format_alert(self, alert_signal: AlertSignal, 
                     format_type: str = "text") -> str:
        """
        경고 포맷팅
        
        Args:
            alert_signal: 경고 신호
            format_type: 포맷 타입 (text, json, markdown)
            
        Returns:
            str: 포맷팅된 경고
        """
        if format_type == "json":
            return self._format_json_alert(alert_signal)
        elif format_type == "markdown":
            return self._format_markdown_alert(alert_signal)
        else:
            return self._format_text_alert(alert_signal)
    
    def _format_text_alert(self, alert_signal: AlertSignal) -> str:
        """텍스트 포맷 경고"""
        lines = [
            "=" * 60,
            f"🚨 시장 과열 경고 시스템 🚨",
            "=" * 60,
            f"경고 시각: {alert_signal.timestamp}",
            f"경고 레벨: {alert_signal.alert_level.value}",
            f"경고 타입: {alert_signal.alert_type.value}",
            f"과열 점수: {alert_signal.heat_score}/100",
            f"신뢰도: {alert_signal.confidence}%",
            f"예상 리드 타임: {alert_signal.lead_time}",
            "",
            "📋 주요 메시지:",
            alert_signal.message,
            "",
            "📊 지표 현황:",
        ]
        
        # 지표 정보 추가
        for name, data in alert_signal.indicators.items():
            signal = data.get("signal", "N/A")
            lines.append(f"  • {name}: {signal}")
        
        # 다이버전스 신호 추가
        if alert_signal.divergence_signals:
            lines.extend([
                "",
                "🔄 다이버전스 신호:",
            ])
            for signal in alert_signal.divergence_signals:
                lines.append(f"  • {signal['type']}: {signal.get('description', '')}")
        
        # 권고사항 추가
        if alert_signal.recommendations:
            lines.extend([
                "",
                "💡 권고사항:",
            ])
            for rec in alert_signal.recommendations:
                lines.append(f"  {rec}")
        
        lines.extend([
            "",
            "=" * 60,
            "⚠️ 본 경고는 참고용 정보이며, 투자 결정은 본인의 판단에 따라야 합니다.",
            "=" * 60
        ])
        
        return "\n".join(lines)
    
    def _format_json_alert(self, alert_signal: AlertSignal) -> str:
        """JSON 포맷 경고"""
        alert_dict = {
            "alert_system": "Market Heat Detector",
            "timestamp": alert_signal.timestamp,
            "alert_level": alert_signal.alert_level.value,
            "alert_type": alert_signal.alert_type.value,
            "heat_score": alert_signal.heat_score,
            "confidence": alert_signal.confidence,
            "lead_time": alert_signal.lead_time,
            "message": alert_signal.message,
            "indicators": alert_signal.indicators,
            "divergence_signals": alert_signal.divergence_signals,
            "prediction": alert_signal.prediction,
            "recommendations": alert_signal.recommendations
        }
        
        return json.dumps(alert_dict, indent=2, ensure_ascii=False)
    
    def _format_markdown_alert(self, alert_signal: AlertSignal) -> str:
        """Markdown 포맷 경고"""
        lines = [
            f"# 🚨 시장 과열 경고",
            "",
            f"**경고 시각**: {alert_signal.timestamp}",
            f"**경고 레벨**: `{alert_signal.alert_level.value}`",
            f"**경고 타입**: `{alert_signal.alert_type.value}`",
            f"**과열 점수**: {alert_signal.heat_score}/100",
            f"**신뢰도**: {alert_signal.confidence}%",
            f"**예상 리드 타임**: {alert_signal.lead_time}",
            "",
            "## 📋 주요 메시지",
            alert_signal.message,
            "",
            "## 📊 지표 현황",
            "| 지표 | 신호 |",
            "|------|------|",
        ]
        
        # 지표 표 추가
        for name, data in alert_signal.indicators.items():
            signal = data.get("signal", "N/A")
            lines.append(f"| {name} | {signal} |")
        
        # 다이버전스 신호 추가
        if alert_signal.divergence_signals:
            lines.extend([
                "",
                "## 🔄 다이버전스 신호",
            ])
            for signal in alert_signal.divergence_signals:
                lines.extend([
                    f"### {signal['type']}",
                    signal.get('description', ''),
                    ""
                ])
        
        # 권고사항 추가
        if alert_signal.recommendations:
            lines.extend([
                "",
                "## 💡 권고사항",
            ])
            for rec in alert_signal.recommendations:
                lines.append(f"- {rec}")
        
        lines.extend([
            "",
            "---",
            "*⚠️ 본 경고는 참고용 정보이며, 투자 결정은 본인의 판단에 따라야 합니다.*"
        ])
        
        return "\n".join(lines)
    
    def send_alert(self, alert_signal: AlertSignal, 
                  channels: List[str] = None) -> bool:
        """
        경고 발송
        
        Args:
            alert_signal: 경고 신호
            channels: 발송 채널 목록
            
        Returns:
            bool: 발송 성공 여부
        """
        try:
            if channels is None:
                channels = self.config["alert_channels"]
            
            success = True
            
            for channel in channels:
                if channel == "console":
                    success &= self._send_console_alert(alert_signal)
                elif channel == "file":
                    success &= self._send_file_alert(alert_signal)
                elif channel == "email":
                    success &= self._send_email_alert(alert_signal)
            
            return success
            
        except Exception as e:
            print(f"경고 발송 중 오류: {str(e)}")
            return False
    
    def _send_console_alert(self, alert_signal: AlertSignal) -> bool:
        """콘솔 경고 발송"""
        try:
            formatted_alert = self.format_alert(alert_signal, "text")
            print(formatted_alert)
            return True
        except:
            return False
    
    def _send_file_alert(self, alert_signal: AlertSignal) -> bool:
        """파일 경고 발송"""
        try:
            # 경고 로그 파일에 저장
            log_filename = f"alerts_{datetime.now().strftime('%Y%m%d')}.log"
            
            with open(log_filename, 'a', encoding='utf-8') as f:
                formatted_alert = self.format_alert(alert_signal, "json")
                f.write(formatted_alert + "\n")
            
            return True
        except:
            return False
    
    def _send_email_alert(self, alert_signal: AlertSignal) -> bool:
        """이메일 경고 발송 (모의 구현)"""
        try:
            # 실제 이메일 발송 로직 (SMTP 등)
            print(f"📧 이메일 경고 발송: {alert_signal.alert_level.value}")
            return True
        except:
            return False
    
    def check_rate_limit(self) -> bool:
        """발송 빈도 제한 확인"""
        try:
            if not self.alert_history:
                return True
            
            # 마지막 경고 시간 확인
            last_alert = self.alert_history[-1]
            last_time = datetime.fromisoformat(last_alert.timestamp.replace('Z', '+00:00'))
            current_time = datetime.now()
            
            # 최소 발송 간격 확인
            min_interval = timedelta(minutes=self.config["notification_settings"]["rate_limit_minutes"])
            
            if current_time - last_time < min_interval:
                return False
            
            # 시간당 최대 발송 횟수 확인
            hour_ago = current_time - timedelta(hours=1)
            recent_alerts = [
                alert for alert in self.alert_history
                if datetime.fromisoformat(alert.timestamp.replace('Z', '+00:00')) > hour_ago
            ]
            
            max_per_hour = self.config["notification_settings"]["max_alerts_per_hour"]
            
            return len(recent_alerts) < max_per_hour
            
        except:
            return True
    
    def get_alert_statistics(self) -> Dict:
        """경고 통계 정보"""
        try:
            if not self.alert_history:
                return {"message": "경고 기록 없음"}
            
            # 기간별 통계
            total_alerts = len(self.alert_history)
            
            # 레벨별 통계
            level_counts = {}
            for alert in self.alert_history:
                level = alert.alert_level.value
                level_counts[level] = level_counts.get(level, 0) + 1
            
            # 타입별 통계
            type_counts = {}
            for alert in self.alert_history:
                alert_type = alert.alert_type.value
                type_counts[alert_type] = type_counts.get(alert_type, 0) + 1
            
            # 최근 24시간 경고
            day_ago = datetime.now() - timedelta(days=1)
            recent_alerts = [
                alert for alert in self.alert_history
                if datetime.fromisoformat(alert.timestamp.replace('Z', '+00:00')) > day_ago
            ]
            
            return {
                "total_alerts": total_alerts,
                "last_24_hours": len(recent_alerts),
                "alert_levels": level_counts,
                "alert_types": type_counts,
                "average_confidence": np.mean([alert.confidence for alert in self.alert_history]),
                "average_heat_score": np.mean([alert.heat_score for alert in self.alert_history])
            }
            
        except Exception as e:
            return {"error": f"통계 계산 실패: {str(e)}"}

def main():
    """메인 실행 함수"""
    generator = AlertGenerator()
    
    # 모의 분석 결과 생성
    mock_analysis_result = {
        "timestamp": datetime.now().isoformat(),
        "heat_score": 75,
        "alert_level": "WARNING",
        "indicators": {
            "vix_put_call": {
                "signal": "OVERHEATED",
                "value": 0.65
            },
            "ad_breadth": {
                "signal": "WARNING",
                "divergence": True
            }
        },
        "divergence_signals": [
            {
                "type": "BEARISH_DIVERGENCE",
                "description": "가격 상승과 A/D 라인 하락 간 다이버전스"
            }
        ],
        "prediction": {
            "signal": "MODERATE_OVERHEAT_WARNING",
            "confidence": 72,
            "lead_time": "2-4주"
        }
    }
    
    print("=== 시장 과열 경고 생성 시스템 ===")
    
    # 경고 생성
    alert_signal = generator.generate_alert(mock_analysis_result)
    
    if alert_signal:
        print(f"\n🚨 경고 생성 완료!")
        print(f"레벨: {alert_signal.alert_level.value}")
        print(f"타입: {alert_signal.alert_type.value}")
        print(f"신뢰도: {alert_signal.confidence}%")
        
        # 발송 빈도 확인
        if generator.check_rate_limit():
            print(f"\n📤 경고 발송 중...")
            success = generator.send_alert(alert_signal, ["console"])
            print(f"발송 결과: {'성공' if success else '실패'}")
        else:
            print(f"\n⏰ 발송 빈도 제한으로 경고 발송 생략")
        
        # 통계 정보
        print(f"\n📊 경고 통계:")
        stats = generator.get_alert_statistics()
        for key, value in stats.items():
            print(f"  {key}: {value}")
    else:
        print("경고 생성 실패 또는 조건 미충족")

if __name__ == "__main__":
    main()