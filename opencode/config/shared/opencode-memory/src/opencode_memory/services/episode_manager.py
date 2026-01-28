"""
Episode Manager Service

에피소드 생명주기 관리 서비스

Task 3.2: EpisodeManager 서비스 구현
- start_episode: 새 에피소드 시작
- add_record: 현재 에피소드에 기록 추가
- get_active_episode: 활성 에피소드 조회
- complete_episode: 에피소드 완료
"""

import asyncio
import logging
from datetime import datetime
from typing import Dict, Optional, TYPE_CHECKING

from opencode_memory.models.semantic import (
    Episode,
    EpisodeContext,
    ProblemResolution,
    SemanticRecord,
)
from opencode_memory.storage.episode_store import EpisodeStore

if TYPE_CHECKING:
    from opencode_memory.services.reflection_engine import ReflectionEngine

logger = logging.getLogger(__name__)


class EpisodeManager:
    """에피소드 생명주기 관리
    
    세션별 활성 에피소드를 추적하고,
    레코드 추가, 학습 기록, 에피소드 완료 등을 처리합니다.
    """

    def __init__(self, store: EpisodeStore, reflection_engine: Optional["ReflectionEngine"] = None):
        """
        Args:
            store: EpisodeStore 인스턴스
            reflection_engine: ReflectionEngine 인스턴스 (Task 4.5)
        """
        self.store = store
        self.reflection_engine = reflection_engine
        # 메모리 캐시: session_id -> Episode
        self._active_episodes: Dict[str, Episode] = {}

    def start_episode(
        self,
        session_id: str,
        goal: str,
        context: EpisodeContext,
    ) -> Episode:
        """새 에피소드 시작

        기존 활성 에피소드가 있으면 'interrupted'로 완료 처리 후
        새 에피소드를 생성합니다.

        Args:
            session_id: 세션 ID
            goal: 에피소드 목표
            context: 에피소드 컨텍스트

        Returns:
            생성된 Episode
        """
        # 1. 기존 활성 에피소드 완료 처리
        active = self.get_active_episode(session_id)
        if active:
            logger.info(f"Interrupting active episode {active.id} for session {session_id}")
            self.complete_episode(session_id, outcome="interrupted")

        # 2. 새 에피소드 생성
        episode = Episode(
            session_id=session_id,
            goal=goal,
            context=context,
            status="active",
            start_time=datetime.now(),
        )

        # 3. 저장
        result = self.store.save(episode)
        if result["status"] != "success":
            logger.error(f"Failed to save episode: {result.get('message')}")
            raise RuntimeError(f"Failed to save episode: {result.get('message')}")

        # 4. 캐시 업데이트
        self._active_episodes[session_id] = episode

        logger.info(f"Started new episode {episode.id} for session {session_id}: {goal}")
        return episode

    def get_active_episode(self, session_id: str) -> Optional[Episode]:
        """활성 에피소드 조회

        먼저 메모리 캐시를 확인하고, 없으면 DB에서 조회합니다.

        Args:
            session_id: 세션 ID

        Returns:
            활성 Episode or None
        """
        # 캐시 확인
        if session_id in self._active_episodes:
            episode = self._active_episodes[session_id]
            if episode.status == "active":
                return episode
            else:
                # 캐시 정리
                del self._active_episodes[session_id]

        # DB에서 조회
        episode = self.store.get_active(session_id)
        if episode:
            self._active_episodes[session_id] = episode
            return episode

        return None

    def complete_episode(
        self,
        session_id: str,
        outcome: str = "completed",
    ) -> Optional[Episode]:
        """에피소드 완료 처리

        Args:
            session_id: 세션 ID
            outcome: 결과 (completed, failed, interrupted)

        Returns:
            완료된 Episode or None
        """
        episode = self.get_active_episode(session_id)
        if not episode:
            logger.warning(f"No active episode found for session {session_id}")
            return None

        # 상태 업데이트
        episode.status = "completed"
        episode.outcome = outcome
        episode.end_time = datetime.now()
        episode.last_updated = episode.end_time

        # 저장
        result = self.store.update(episode)
        if result["status"] != "success":
            logger.error(f"Failed to update episode: {result.get('message')}")

        # 캐시에서 제거
        if session_id in self._active_episodes:
            del self._active_episodes[session_id]

        logger.info(f"Completed episode {episode.id} with outcome: {outcome}")
        
        # Task 4.5: 자동 성찰 트리거 (비동기)
        if self.reflection_engine:
            self._trigger_reflection(episode)

        return episode

    def _trigger_reflection(self, episode: Episode):
        """성찰 비동기 실행"""
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self.reflection_engine.areflect_and_persist(episode))
            logger.debug(f"Triggered async reflection for episode {episode.id}")
        except RuntimeError:
            logger.warning("No running event loop, skipping async reflection.")
        except Exception as e:
            logger.error(f"Failed to trigger reflection: {e}")

    def fail_episode(self, session_id: str, reason: str = "failed") -> Optional[Episode]:
        """에피소드 실패 처리

        Args:
            session_id: 세션 ID
            reason: 실패 사유

        Returns:
            실패한 Episode or None
        """
        episode = self.get_active_episode(session_id)
        if not episode:
            return None

        episode.status = "failed"
        episode.outcome = reason
        episode.end_time = datetime.now()
        episode.last_updated = episode.end_time

        self.store.update(episode)

        if session_id in self._active_episodes:
            del self._active_episodes[session_id]

        logger.info(f"Failed episode {episode.id} with reason: {reason}")
        return episode

    def add_record(
        self,
        session_id: str,
        record: SemanticRecord,
    ) -> bool:
        """에피소드에 레코드 추가

        활성 에피소드가 있으면 레코드의 episode_id를 설정하고
        tools_used, record_count를 업데이트합니다.

        Args:
            session_id: 세션 ID
            record: 추가할 SemanticRecord

        Returns:
            성공 여부
        """
        episode = self.get_active_episode(session_id)
        if not episode:
            logger.debug(f"No active episode for session {session_id}")
            return False

        # episode_id 할당
        record.episode_id = episode.id

        # tools_used 업데이트 (중복 방지)
        if record.tool_name and record.tool_name not in episode.tools_used:
            episode.tools_used.append(record.tool_name)

        # record_count 증가
        episode.record_count += 1

        # context_tools_used도 업데이트
        if record.tool_name and record.tool_name not in episode.context.tools_used:
            episode.context.tools_used.append(record.tool_name)

        # last_updated 갱신
        episode.last_updated = datetime.now()

        # 저장 (비동기로 처리하는 것이 좋지만, 일단 동기로)
        self.store.update(episode)

        logger.debug(f"Added record to episode {episode.id}: {record.tool_name}")
        return True

    def add_learning(self, session_id: str, learning: str) -> bool:
        """에피소드에 학습 추가

        Args:
            session_id: 세션 ID
            learning: 학습 내용

        Returns:
            성공 여부
        """
        episode = self.get_active_episode(session_id)
        if not episode:
            return False

        # 중복 방지
        if learning not in episode.learnings:
            episode.learnings.append(learning)
            episode.last_updated = datetime.now()
            self.store.update(episode)
            logger.debug(f"Added learning to episode {episode.id}: {learning[:50]}...")

        return True

    def add_problem_solved(
        self,
        session_id: str,
        problem: ProblemResolution,
    ) -> bool:
        """에피소드에 해결된 문제 추가

        Args:
            session_id: 세션 ID
            problem: 해결된 문제

        Returns:
            성공 여부
        """
        episode = self.get_active_episode(session_id)
        if not episode:
            return False

        # problems_solved는 런타임 필드
        episode.problems_solved.append(problem)

        logger.debug(f"Added problem solved to episode {episode.id}")
        return True

    def get_episode(self, episode_id: str) -> Optional[Episode]:
        """에피소드 ID로 조회

        Args:
            episode_id: 에피소드 ID

        Returns:
            Episode or None
        """
        return self.store.get(episode_id)


# ═══════════════════════════════════════════════════════════════
# 싱글톤 인스턴스 관리
# ═══════════════════════════════════════════════════════════════

_global_manager: Optional[EpisodeManager] = None


def get_episode_manager(
    store: Optional[EpisodeStore] = None, 
    reflection_engine: Optional["ReflectionEngine"] = None
) -> EpisodeManager:
    """전역 EpisodeManager 인스턴스 반환

    Args:
        store: EpisodeStore 인스턴스 (첫 호출 시 필수)
        reflection_engine: ReflectionEngine 인스턴스 (Task 4.5, 선택)

    Returns:
        EpisodeManager 인스턴스
    """
    global _global_manager
    if _global_manager is None:
        if store is None:
            raise ValueError("EpisodeStore must be provided on first call")
        _global_manager = EpisodeManager(store, reflection_engine)
    return _global_manager


def reset_episode_manager():
    """테스트용: 전역 인스턴스 리셋"""
    global _global_manager
    _global_manager = None
