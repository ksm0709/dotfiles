"""
Reflection Engine Service

에피소드 완료 시 자동으로 성찰하고 학습 내용을 추출하여 장기 기억에 저장합니다.
Task 4.1 ~ 4.4 구현
"""

import asyncio
import logging
from typing import Any, Coroutine, List, Optional, TypeVar

from opencode_memory.context_memory import ContextMemory, is_meaningful_content
from opencode_memory.models.semantic import Episode, Reflection
from opencode_memory.services.semantic_extractor import SemanticExtractor

logger = logging.getLogger(__name__)

T = TypeVar("T")


class ReflectionEngine:
    """자동 성찰 및 학습 엔진"""

    def __init__(
        self,
        context_memory: ContextMemory,
        semantic_extractor: Optional[SemanticExtractor] = None,
        enable_llm_reflection: bool = True,
    ):
        self.context_memory = context_memory
        self.semantic_extractor = semantic_extractor
        self.enable_llm_reflection = enable_llm_reflection

    def reflect(self, episode: Episode) -> Reflection:
        """에피소드 성찰 (동기)

        LLM이 있으면 LLM을 사용하고, 없으면 규칙 기반으로 추출합니다.
        """
        if self._llm_reflection_available():
            reflection = self._run_async(self._reflect_with_llm_async(episode))
            if reflection:
                return reflection

        return self._reflect_rule_based(episode)

    async def areflect(self, episode: Episode) -> Reflection:
        """에피소드 성찰 (비동기)

        비동기 컨텍스트에서는 이 메서드를 사용합니다.
        """
        if self._llm_reflection_available():
            reflection = await self._reflect_with_llm_async(episode)
            if reflection:
                return reflection

        return self._reflect_rule_based(episode)

    async def areflect_and_persist(self, episode: Episode) -> Reflection:
        """비동기 성찰 및 저장 (Non-blocking wrapper)"""
        reflection = await self.areflect(episode)

        all_learnings = (
            reflection.key_learnings
            + reflection.user_preferences_discovered
            + reflection.reusable_patterns
        )
        await asyncio.to_thread(self.persist_learnings, all_learnings, episode.id)

        return reflection

    def _reflect_and_persist_sync(self, episode: Episode) -> Reflection:
        """동기 성찰 및 저장 로직"""
        # 1. 성찰
        reflection = self.reflect(episode)

        # 2. 학습 내용 저장
        all_learnings = (
            reflection.key_learnings
            + reflection.user_preferences_discovered
            + reflection.reusable_patterns
        )

        self.persist_learnings(all_learnings, episode.id)

        return reflection

    def _reflect_rule_based(self, episode: Episode) -> Reflection:
        """규칙 기반 성찰 (Fallback)"""

        # 1. 요약 생성
        summary = (
            f"Episode '{episode.goal}' completed with {episode.record_count} records."
        )
        if episode.outcome:
            summary += f" Outcome: {episode.outcome}."

        # 2. 학습 추출
        learnings = self.extract_learnings(episode)

        # 3. Reflection 객체 생성
        return Reflection(
            episode_id=episode.id,
            summary=summary,
            key_learnings=learnings,
            user_preferences_discovered=[],  # 규칙 기반으로는 추출 어려움
            reusable_patterns=[],
            improvements=[],
        )

    def _llm_reflection_available(self) -> bool:
        if not self.enable_llm_reflection:
            return False
        if not self.semantic_extractor:
            return False
        return bool(
            getattr(self.semantic_extractor, "llm", None)
            and self.semantic_extractor.llm.enabled
        )

    async def _reflect_with_llm_async(self, episode: Episode) -> Optional[Reflection]:
        """LLM 기반 성찰 (비동기)"""
        try:
            semantic_extractor = self.semantic_extractor
            if not semantic_extractor:
                return None
            events = self._build_event_log(episode)
            result = await semantic_extractor.reflect_on_episode(events)
            if not result:
                return None

            summary = result.get("summary") or f"Episode '{episode.goal}' completed."
            key_learnings = result.get("key_learnings") or []
            improvements = result.get("next_steps") or []

            return Reflection(
                episode_id=episode.id,
                summary=summary,
                key_learnings=key_learnings,
                user_preferences_discovered=[],
                reusable_patterns=[],
                improvements=improvements,
            )
        except Exception as e:
            logger.warning(f"LLM reflection failed, fallback to rule-based: {e}")
            return None

    def _build_event_log(self, episode: Episode) -> List[dict]:
        events: List[dict] = []
        for record in episode.records:
            content = (
                f"Intent: {record.intent}\n"
                f"Action: {record.action}\n"
                f"Outcome: {record.outcome}\n"
                f"Tool: {record.tool_name}"
            )
            events.append({"role": "tool", "content": content})

        if episode.learnings:
            for learning in episode.learnings:
                events.append({"role": "learning", "content": learning})

        if not events:
            events.append({"role": "goal", "content": episode.goal})

        return events

    def _run_async(self, coro: Coroutine[Any, Any, T]) -> T:
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.run(coro)

        raise RuntimeError(
            "비동기 컨텍스트에서 동기 ReflectionEngine.reflect를 호출할 수 없습니다. "
            "areflect를 사용하세요."
        )

    def extract_learnings(self, episode: Episode) -> List[str]:
        """학습 내용 추출 (Task 4.2)"""
        learnings = []

        # 1. 해결된 문제에서 학습 추출
        for problem in episode.problems_solved:
            if problem.status == "resolved" and problem.successful_solution:
                learning = f"Problem '{problem.error_type}' solved by: {problem.successful_solution}"
                learnings.append(learning)

        # 2. 에피소드 내 명시적 학습 사항 포함
        learnings.extend(episode.learnings)

        # 3. Successful actions (추가)
        IMPORTANT_KEYWORDS = [
            "install",
            "setup",
            "config",
            "create",
            "add",
            "update",
            "fix",
        ]
        for r in episode.records:
            if r.success and any(k in r.action.lower() for k in IMPORTANT_KEYWORDS):
                learnings.append(f"{r.action} (Auto-extracted)")

        # 4. 중복 제거
        return list(set(learnings))

    def should_persist(self, learning: str) -> bool:
        """저장 가치 판단 (Task 4.3)"""
        return is_meaningful_content(learning)

    def persist_learnings(self, learnings: List[str], episode_id: str):
        """LTM 저장 (Task 4.4)"""
        count = 0
        for learning in learnings:
            if self.should_persist(learning):
                tags = ["learning", f"episode:{episode_id}"]

                # ContextMemory.add 호출
                result = self.context_memory.add(
                    content=learning,
                    tags=tags,
                    metadata={
                        "type": "learning",
                        "source": "reflection_engine",
                        "episode_id": episode_id,
                    },
                )
                if result.get("status") == "success":
                    count += 1

        if count > 0:
            logger.info(f"Persisted {count} learnings for episode {episode_id}")
