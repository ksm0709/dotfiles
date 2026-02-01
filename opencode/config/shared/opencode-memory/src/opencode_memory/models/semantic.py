"""
Semantic Memory Data Models

Pydantic 모델 정의: SemanticRecord, Decision, ProblemResolution, IntentInfo

Task 1.1: 데이터 모델 정의
"""

import uuid
from datetime import datetime
from typing import List, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field


class IntentInfo(BaseModel):
    """작업 의도 정보 (에이전트 제공 또는 규칙 기반 추론)"""

    goal: str = Field(..., description="현재 작업의 목표 (1-2문장)")
    user_request_summary: Optional[str] = Field(
        default=None, description="사용자 요청 요약"
    )
    context: Optional[str] = Field(default=None, description="배경 정보, 제약 사항")
    assumptions: List[str] = Field(default_factory=list, description="세운 가정들")
    agent_thoughts: List[str] = Field(
        default_factory=list,
        description="에이전트가 명시적으로 기록한 사고 흐름/계획",
    )
    source: Literal["agent", "inferred", "unknown"] = Field(
        default="unknown",
        description="의도 출처: agent=에이전트 명시, inferred=규칙 기반 추론, unknown=알 수 없음",
    )
    confidence: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="신뢰도: agent=0.9, inferred=0.5, unknown=0.2",
    )


class Decision(BaseModel):
    """의사결정 기록"""

    decision_type: str = Field(
        ...,
        description="결정 유형: library, architecture, approach, user_preference 등",
    )
    choice: str = Field(..., description="최종 선택")
    alternatives: List[str] = Field(default_factory=list, description="고려한 대안들")
    rationale: str = Field(..., description="선택 이유")
    is_user_preference: bool = Field(False, description="사용자 직접 선택 여부")
    timestamp: datetime = Field(default_factory=datetime.now)


class ProblemResolution(BaseModel):
    """문제 해결 추적"""

    id: str = Field(..., description="문제 ID")
    error_type: str = Field(..., description="에러 유형")
    error_message: str = Field(..., description="에러 메시지")
    attempted_solutions: List[str] = Field(
        default_factory=list, description="시도한 해결책들"
    )
    successful_solution: Optional[str] = Field(None, description="성공한 해결책")
    path_to_solution: List[str] = Field(default_factory=list, description="해결 경로")
    status: Literal["active", "resolved", "abandoned"] = Field(
        "active", description="상태"
    )


class SemanticRecord(BaseModel):
    """의미 기반 도구 실행 기록

    핵심 삼중체(Intent-Action-Outcome)와 함께
    의사결정, 문제 해결, 학습 사항을 포함한 풍부한 메타데이터를 저장합니다.
    """

    # 식별자
    id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="레코드 고유 ID",
    )
    session_id: str = Field(..., description="세션 ID")
    episode_id: Optional[str] = Field(default=None, description="에피소드 ID")
    timestamp: datetime = Field(default_factory=datetime.now)

    # 핵심 삼중체 (Intent-Action-Outcome)
    intent: str = Field(..., description="작업 의도 (규칙 기반 폴백)")
    intent_info: Optional[IntentInfo] = Field(
        default=None, description="상세 의도 정보 (에이전트 제공 시)"
    )
    action: str = Field(..., description="수행한 작업")
    outcome: str = Field(..., description="작업 결과")

    # 도구 정보
    tool_name: str = Field(..., description="사용한 도구명")
    tool_args: dict = Field(default_factory=dict, description="도구 인자 (JSON)")
    success: bool = Field(default=True, description="성공 여부")

    # 풍부화 필드 (에이전트 제공)
    decision: Optional[Decision] = Field(
        default=None, description="의사결정 정보 (context_decision에서)"
    )
    problem_resolution: Optional[ProblemResolution] = Field(
        default=None, description="문제 해결 정보"
    )
    learnings: List[str] = Field(
        default_factory=list, description="학습 사항 (context_learning에서)"
    )
    related_records: List[str] = Field(
        default_factory=list, description="관련 레코드 ID들"
    )

    # 중요도
    importance: Literal["low", "medium", "high", "critical"] = Field(
        default="low", description="중요도"
    )

    model_config = ConfigDict(
        ser_json_timedelta="iso8601",
        json_schema_extra={"examples": []},
    )

    def model_dump_json_safe(self) -> dict:
        """JSON 직렬화 안전한 dict 반환"""
        data = self.model_dump()
        if self.timestamp:
            data["timestamp"] = self.timestamp.isoformat()
        if self.decision and self.decision.timestamp:
            data["decision"]["timestamp"] = self.decision.timestamp.isoformat()
        return data


# ═══════════════════════════════════════════════════════════════
# API Request/Response 모델
# ═══════════════════════════════════════════════════════════════


class RecordIntentRequest(BaseModel):
    """POST /record-intent 요청"""

    session_id: str
    goal: str
    user_request_summary: Optional[str] = None
    context: Optional[str] = None
    assumptions: List[str] = Field(default_factory=list)
    agent_thoughts: List[str] = Field(
        default_factory=list,
        description="에이전트가 제공한 사고 흐름 (LLM 자동 생성 금지)",
    )


class RecordDecisionRequest(BaseModel):
    """POST /record-decision 요청"""

    session_id: str
    decision_type: str
    choice: str
    alternatives: List[str] = Field(default_factory=list)
    rationale: str
    is_user_preference: bool = False


class RecordLearningRequest(BaseModel):
    """POST /record-learning 요청"""

    session_id: str
    learning: str
    category: Optional[str] = None  # project, pattern, preference, error_solution


class RecordSemanticRequest(BaseModel):
    """POST /record-semantic 요청"""

    session_id: str
    intent: str
    action: str
    outcome: str
    tool_name: str
    tool_args: dict = Field(default_factory=dict)
    success: bool = True
    decision: Optional[dict] = None
    importance: Literal["low", "medium", "high", "critical"] = "low"


class SemanticRecordResponse(BaseModel):
    """Semantic API 응답"""

    status: str
    record_id: Optional[str] = None
    intent_id: Optional[str] = None
    decision_id: Optional[str] = None
    learning_id: Optional[str] = None
    episode_id: Optional[str] = None
    importance: Optional[str] = None
    message: Optional[str] = None
    agent_thoughts: Optional[List[str]] = None


# ═══════════════════════════════════════════════════════════════
# Episode 모델 (Phase 3: Task 3.1)
# ═══════════════════════════════════════════════════════════════


class EpisodeContext(BaseModel):
    """에피소드 컨텍스트 - 에이전트 제공 + 자동 수집"""

    # 에이전트 제공 (context_intent에서)
    goal: str = Field(..., description="작업 목표")
    user_request_summary: Optional[str] = Field(
        default=None, description="사용자 요청 요약"
    )
    assumptions: List[str] = Field(default_factory=list, description="세운 가정들")
    agent_thoughts: List[str] = Field(
        default_factory=list,
        description="에이전트가 기록한 사고 흐름 목록",
    )

    # 자동 수집
    initial_tool: Optional[str] = Field(default=None, description="첫 번째 도구")
    tools_used: List[str] = Field(default_factory=list, description="사용된 도구들")


class Episode(BaseModel):
    """에피소드 - 하나의 목표 달성을 위한 작업 그룹

    Phase 3: 에피소드 기반 그룹화
    - 목표(goal)를 중심으로 관련 SemanticRecord들을 그룹화
    - 문제 해결, 학습 사항을 에피소드 단위로 추적
    """

    id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="에피소드 고유 ID",
    )
    session_id: str = Field(..., description="세션 ID")
    goal: str = Field(..., description="에피소드 목표")
    status: Literal["active", "completed", "failed"] = Field(
        default="active", description="상태: active, completed, failed"
    )

    # 컨텍스트
    context: EpisodeContext = Field(..., description="에피소드 컨텍스트")

    # 연결된 레코드들 (런타임 시 채워짐, DB에는 ID만 저장)
    records: List[SemanticRecord] = Field(
        default_factory=list, description="연결된 SemanticRecord들"
    )
    problems_solved: List[ProblemResolution] = Field(
        default_factory=list, description="해결된 문제들"
    )

    # 결과
    outcome: Optional[str] = Field(default=None, description="에피소드 결과")
    learnings: List[str] = Field(default_factory=list, description="학습 사항들")

    # 시간
    start_time: datetime = Field(default_factory=datetime.now)
    last_updated: datetime = Field(default_factory=datetime.now)
    end_time: Optional[datetime] = Field(default=None, description="종료 시각")

    # 메타
    tools_used: List[str] = Field(default_factory=list, description="사용된 도구 목록")
    record_count: int = Field(default=0, description="레코드 수")

    model_config = ConfigDict(
        ser_json_timedelta="iso8601",
    )


# ═══════════════════════════════════════════════════════════════
# Episode API Request/Response 모델 (Phase 3: Task 3.4)
# ═══════════════════════════════════════════════════════════════


class StartEpisodeRequest(BaseModel):
    """POST /episode/start 요청"""

    session_id: str
    goal: str
    context: dict = Field(default_factory=dict)


class CompleteEpisodeRequest(BaseModel):
    """POST /episode/complete 요청"""

    session_id: str
    outcome: str = "completed"
    summary: Optional[str] = None


class EpisodeResponse(BaseModel):
    """Episode API 응답"""

    status: str
    episode_id: Optional[str] = None
    goal: Optional[str] = None
    message: Optional[str] = None


class Reflection(BaseModel):
    """에피소드 성찰 결과 (Task 4.1)"""

    episode_id: str = Field(..., description="에피소드 ID")
    summary: str = Field(..., description="에피소드 요약")
    key_learnings: List[str] = Field(default_factory=list, description="핵심 학습 내용")
    user_preferences_discovered: List[str] = Field(
        default_factory=list, description="발견된 사용자 선호"
    )
    reusable_patterns: List[str] = Field(
        default_factory=list, description="재사용 가능한 패턴"
    )
    improvements: List[str] = Field(default_factory=list, description="향후 개선점")
