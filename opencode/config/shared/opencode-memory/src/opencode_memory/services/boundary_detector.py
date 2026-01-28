"""
Episode Boundary Detector

에피소드 경계 감지 알고리즘

Task 3.3: 에피소드 경계 감지 구현
- 규칙 기반 경계 감지:
  - context_start 호출 시 → 새 에피소드
  - 30분 이상 간격 → 새 에피소드
  - 현재 에피소드 완료/실패 → 새 에피소드
- LLM 기반 목표 변경 감지 (Phase 5에서 구현 예정)
"""

import asyncio
import logging
from datetime import datetime, timedelta
from typing import Optional, Any, Coroutine, TypeVar

from opencode_memory.models.semantic import Episode
from opencode_memory.services.semantic_extractor import SemanticExtractor

logger = logging.getLogger(__name__)

T = TypeVar("T")


class EpisodeBoundaryDetector:
    """에피소드 경계 감지 알고리즘

    규칙 기반으로 새 에피소드 시작 여부를 결정합니다.
    Phase 5에서 LLM 기반 목표 변경 감지가 추가될 예정입니다.
    """

    # 기본 시간 임계값 (분)
    DEFAULT_TIME_THRESHOLD_MINUTES = 30

    def __init__(
        self,
        time_threshold_minutes: int = DEFAULT_TIME_THRESHOLD_MINUTES,
        use_llm: bool = False,
        semantic_extractor: Optional[SemanticExtractor] = None,
    ):
        """
        Args:
            time_threshold_minutes: 시간 간격 임계값 (분)
            use_llm: LLM 기반 목표 변경 감지 사용 여부 (Phase 5)
        """
        self.time_threshold_minutes = time_threshold_minutes
        self.use_llm = use_llm
        self.semantic_extractor = semantic_extractor

    def should_start_new_episode(
        self,
        current_episode: Optional[Episode],
        new_input: str,
        tool_name: Optional[str] = None,
    ) -> bool:
        """새 에피소드 시작 여부 결정

        Args:
            current_episode: 현재 활성 에피소드 (없으면 None)
            new_input: 새로운 입력 (작업 설명)
            tool_name: 호출된 도구명 (context_start 감지용)

        Returns:
            True면 새 에피소드 시작 필요
        """
        # 1. 명시적 시작 신호: context_start 호출
        if tool_name == "context_start":
            logger.debug("Boundary detected: context_start called")
            return True

        # 2. 현재 에피소드 없음
        if current_episode is None:
            logger.debug("Boundary detected: no current episode")
            return True

        # 3. 현재 에피소드가 완료/실패 상태
        if current_episode.status != "active":
            logger.debug(f"Boundary detected: episode status is {current_episode.status}")
            return True

        # 4. 시간 기반 휴리스틱 (30분 이상 간격)
        if self._time_gap_exceeded(current_episode):
            logger.debug(
                f"Boundary detected: time gap exceeded "
                f"(threshold: {self.time_threshold_minutes}min)"
            )
            return True

        # 5. LLM 기반 목표 변경 감지 (Phase 5)
        if self._llm_goal_change_available():
            try:
                goal_changed = self._run_async(
                    self._detect_goal_change_async(current_episode.goal, new_input)
                )
                if goal_changed:
                    logger.debug("Boundary detected: goal change (LLM)")
                    return True
            except Exception as e:
                logger.warning(f"LLM goal change detection failed, fallback: {e}")

        # 기본: 에피소드 계속
        return False

    async def ashould_start_new_episode(
        self,
        current_episode: Optional[Episode],
        new_input: str,
        tool_name: Optional[str] = None,
    ) -> bool:
        """새 에피소드 시작 여부 결정 (비동기)"""
        # 1. 명시적 시작 신호: context_start 호출
        if tool_name == "context_start":
            logger.debug("Boundary detected: context_start called")
            return True

        # 2. 현재 에피소드 없음
        if current_episode is None:
            logger.debug("Boundary detected: no current episode")
            return True

        # 3. 현재 에피소드가 완료/실패 상태
        if current_episode.status != "active":
            logger.debug(f"Boundary detected: episode status is {current_episode.status}")
            return True

        # 4. 시간 기반 휴리스틱 (30분 이상 간격)
        if self._time_gap_exceeded(current_episode):
            logger.debug(
                f"Boundary detected: time gap exceeded "
                f"(threshold: {self.time_threshold_minutes}min)"
            )
            return True

        # 5. LLM 기반 목표 변경 감지 (Phase 5)
        if self._llm_goal_change_available():
            try:
                goal_changed = await self._detect_goal_change_async(
                    current_episode.goal,
                    new_input,
                )
                if goal_changed:
                    logger.debug("Boundary detected: goal change (LLM)")
                    return True
            except Exception as e:
                logger.warning(f"LLM goal change detection failed, fallback: {e}")

        # 기본: 에피소드 계속
        return False

    def _time_gap_exceeded(self, episode: Episode) -> bool:
        """시간 간격 임계값 초과 여부 확인

        Args:
            episode: 현재 에피소드

        Returns:
            True면 임계값 초과
        """
        if not episode.start_time:
            return False

        now = datetime.now()
        threshold = timedelta(minutes=self.time_threshold_minutes)

        # 마지막 활동 시간 계산 (last_updated 사용)
        last_activity = episode.last_updated or episode.start_time

        gap = now - last_activity
        return gap > threshold

    def _llm_goal_change_available(self) -> bool:
        if not self.use_llm:
            return False
        if not self.semantic_extractor:
            return False
        return bool(getattr(self.semantic_extractor, "llm", None) and self.semantic_extractor.llm.enabled)

    async def _detect_goal_change_async(self, current_goal: str, new_input: str) -> bool:
        semantic_extractor = self.semantic_extractor
        if semantic_extractor is None:
            return False
        return await semantic_extractor.detect_goal_change(current_goal, new_input)

    def _run_async(self, coro: Coroutine[Any, Any, T]) -> T:
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.run(coro)

        raise RuntimeError(
            "비동기 컨텍스트에서 동기 EpisodeBoundaryDetector를 호출할 수 없습니다. "
            "ashould_start_new_episode를 사용하세요."
        )


# ═══════════════════════════════════════════════════════════════
# 유틸리티 함수
# ═══════════════════════════════════════════════════════════════


def create_detector(
    time_threshold_minutes: int = 30,
    use_llm: bool = False,
    semantic_extractor: Optional[SemanticExtractor] = None,
) -> EpisodeBoundaryDetector:
    """BoundaryDetector 팩토리 함수

    Args:
        time_threshold_minutes: 시간 임계값 (분)
        use_llm: LLM 사용 여부

    Returns:
        EpisodeBoundaryDetector 인스턴스
    """
    return EpisodeBoundaryDetector(
        time_threshold_minutes=time_threshold_minutes,
        use_llm=use_llm,
        semantic_extractor=semantic_extractor,
    )
