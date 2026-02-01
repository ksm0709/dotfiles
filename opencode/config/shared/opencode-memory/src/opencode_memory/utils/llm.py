"""
Async LLM Client Utility + Usage Metrics
"""

import asyncio
import logging
import os
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Coroutine, Dict, List, Optional, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")


@dataclass
class LLMUsageStats:
    total_calls: int = 0
    total_errors: int = 0
    total_prompt_chars: int = 0
    total_response_chars: int = 0
    total_duration_sec: float = 0.0
    provider_calls: Dict[str, int] = field(default_factory=lambda: defaultdict(int))

    def record(
        self,
        provider: str,
        prompt_chars: int,
        response_chars: int,
        duration: float,
        success: bool,
    ) -> None:
        self.total_calls += 1
        self.total_prompt_chars += prompt_chars
        self.total_response_chars += response_chars
        self.total_duration_sec += duration
        self.provider_calls[provider] += 1
        if not success:
            self.total_errors += 1

    def snapshot(self) -> Dict[str, Any]:
        avg_latency = (
            (self.total_duration_sec / self.total_calls) if self.total_calls else 0.0
        )
        return {
            "total_calls": self.total_calls,
            "total_errors": self.total_errors,
            "avg_latency_ms": round(avg_latency * 1000, 2),
            "prompt_chars": self.total_prompt_chars,
            "response_chars": self.total_response_chars,
            "provider_calls": dict(self.provider_calls),
        }


class AsyncLLMClient:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.llm_config = config.get("llm", {})
        self.provider = self.llm_config.get("provider", "gemini")
        self.model_name = self.llm_config.get("model", "gemini-1.5-flash")
        self.enabled = self.llm_config.get("enabled", False)
        self.api_key = None

        if self.provider == "gemini":
            self.api_key = os.environ.get("GEMINI_API_KEY")
        elif self.provider == "openai":
            self.api_key = os.environ.get("OPENAI_API_KEY")
        else:
            logger.warning(
                "지원되지 않는 LLM provider '%s'가 설정되었습니다.",
                self.provider,
            )

        if self.enabled and not self.api_key:
            logger.warning(
                "LLM enabled but no API key found for provider %s", self.provider
            )

        monitoring_cfg = self.llm_config.get("monitoring", {})
        self.monitoring_enabled = monitoring_cfg.get("enabled", False)
        self.monitoring_interval = max(1, monitoring_cfg.get("log_interval", 25))
        self.usage = LLMUsageStats()

    async def generate(
        self, prompt: str, system_instruction: Optional[str] = None
    ) -> Optional[str]:
        """Generate text using the configured LLM provider."""
        if not self.enabled or not self.api_key:
            return None

        start = time.perf_counter()
        prompt_chars = len(prompt) + len(system_instruction or "")
        response_text: Optional[str] = None
        try:
            if self.provider == "gemini":
                response_text = await self._call_gemini(prompt, system_instruction)
            elif self.provider == "openai":
                response_text = await self._call_openai(prompt, system_instruction)
            else:
                logger.warning("LLM provider '%s' is not supported", self.provider)
                return None
        except Exception as e:
            logger.error(f"LLM generation failed: {e}")
            self._record_usage(prompt_chars, 0, time.perf_counter() - start, False)
            return None

        self._record_usage(
            prompt_chars,
            len(response_text or ""),
            time.perf_counter() - start,
            response_text is not None,
        )
        return response_text

    def _record_usage(
        self, prompt_chars: int, response_chars: int, duration: float, success: bool
    ) -> None:
        if not self.monitoring_enabled:
            return
        self.usage.record(
            provider=self.provider,
            prompt_chars=prompt_chars,
            response_chars=response_chars,
            duration=duration,
            success=success,
        )
        if self.usage.total_calls % self.monitoring_interval == 0:
            snapshot = self.usage.snapshot()
            logger.info(
                "LLM usage summary: calls=%s errors=%s avg_latency_ms=%s",  # noqa: G004
                snapshot["total_calls"],
                snapshot["total_errors"],
                snapshot["avg_latency_ms"],
            )

    def get_usage_snapshot(self) -> Dict[str, Any]:
        return self.usage.snapshot()

    async def _call_gemini(
        self, prompt: str, system_instruction: Optional[str]
    ) -> Optional[str]:
        try:
            import google.generativeai as genai

            genai.configure(api_key=self.api_key)

            # Note: system_instruction support depends on library version/model
            # For simplicity in this integration, we prepend it if provided
            full_prompt = prompt
            if system_instruction:
                full_prompt = f"{system_instruction}\n\n{prompt}"

            model = genai.GenerativeModel(self.model_name)
            response = await model.generate_content_async(full_prompt)
            return response.text
        except ImportError:
            logger.error("google-generativeai package not installed")
            return None
        except Exception as e:
            logger.error(f"Gemini call failed: {e}")
            return None

    async def _call_openai(
        self, prompt: str, system_instruction: Optional[str]
    ) -> Optional[str]:
        try:
            from openai import AsyncOpenAI

            client = AsyncOpenAI(api_key=self.api_key)

            messages = []
            if system_instruction:
                messages.append({"role": "system", "content": system_instruction})
            messages.append({"role": "user", "content": prompt})

            response = await client.chat.completions.create(
                model=self.model_name, messages=messages
            )
            return response.choices[0].message.content
        except ImportError:
            logger.error("openai package not installed")
            return None
        except Exception as e:
            logger.error(f"OpenAI call failed: {e}")
            return None

    async def generate_queries(self, task: str) -> List[str]:
        """Generate expanded queries for HyDE (Async)."""
        if not self.enabled:
            return []

        prompt = f"""
        You are an expert search assistant. Given the following task description,
        generate 3 specific search queries that would help find relevant past memories
        to solve this task.

        Task: {task}

        Return only the queries, one per line, without any other text.
        """

        response = await self.generate(prompt)
        if response:
            return [q.strip() for q in response.split("\n") if q.strip()]
        return []

    async def rerank(
        self, task: str, candidates: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Rerank candidates based on task relevance (Async)."""
        if not self.enabled or not candidates:
            return candidates

        # Prepare candidates for prompt
        candidate_texts = []
        for i, c in enumerate(candidates):
            content = c.get("content", "") or c.get("summary", "") or str(c)
            candidate_texts.append(f"[{i}] {content[:300]}")

        prompt = f"""
        Task: {task}

        Below are some past memories. Rate each memory's relevance to the current task
        on a scale of 0 to 10, where 10 is extremely relevant and 0 is irrelevant.

        Memories:
        {chr(10).join(candidate_texts)}

        Return the results as a comma-separated list of scores in the same order as the memories.
        Example: 8, 2, 9, 5
        """

        response = await self.generate(prompt)
        if response:
            try:
                scores = [float(s.strip()) for s in response.split(",") if s.strip()]

                # Apply scores and filter
                reranked = []
                for i, score in enumerate(scores):
                    if i < len(candidates):
                        candidates[i]["relevance_score"] = score
                        if score >= 7.0:
                            reranked.append(candidates[i])

                # Sort by relevance score
                reranked.sort(key=lambda x: x.get("relevance_score", 0), reverse=True)
                return reranked
            except Exception as e:
                logger.error(f"Failed to parse rerank response: {e}")

        return candidates

    async def summarize_memories(
        self, task: str, memories: List[Dict[str, Any]]
    ) -> Optional[str]:
        """Summarize retrieved memories for context briefing (Async)."""
        if not self.enabled or not memories:
            return None

        # Prepare memories text
        memory_texts = []
        for i, m in enumerate(memories):
            content = m.get("content", "") or m.get("summary", "") or str(m)
            # Add metadata if available
            tags = m.get("tags", [])
            tag_str = f" [Tags: {', '.join(tags)}]" if tags else ""
            memory_texts.append(f"[{i}] {content[:500]}{tag_str}")

        memories_block = "\n".join(memory_texts)

        prompt = f"""
        You are a helpful assistant briefing another AI agent.
        Analyze the following retrieved memories relevant to the current task: "{task}"

        <memories>
        {memories_block}
        </memories>

        Create a concise "Context Briefing" in Markdown.
        Focus on actionable insights, user preferences, and technical constraints.
        Do NOT just list the memories; synthesize them.
        **The output must be written in Korean.**

        Format:
        ## 🧠 Context Briefing
        - **💡 Insight**: ...
        - **⚠️ Constraint**: ...
        - **📂 Context**: ...
        """

        return await self.generate(prompt)


class LLMClient:
    """동기 래퍼: AsyncLLMClient 사용"""

    def __init__(self, config: dict):
        self._client = AsyncLLMClient(config)
        self.enabled = self._client.enabled

    def generate_queries(self, task: str) -> List[str]:
        return self._run_async(self._client.generate_queries(task))

    def rerank(self, task: str, candidates: List[dict]) -> List[dict]:
        return self._run_async(self._client.rerank(task, candidates))

    def _run_async(self, coro: Coroutine[Any, Any, T]) -> T:
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return asyncio.run(coro)

        raise RuntimeError(
            "비동기 컨텍스트에서 동기 LLMClient를 호출할 수 없습니다. "
            "AsyncLLMClient를 사용하세요."
        )
