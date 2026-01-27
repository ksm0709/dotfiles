"""
LLM Client Utility
"""

import logging
import os
from typing import List, Optional

logger = logging.getLogger(__name__)


class LLMClient:
    def __init__(self, config: dict):
        self.config = config
        self.provider = config.get("embeddings", {}).get("provider", "synthetic")
        self.api_key = None

        if self.provider == "gemini":
            self.api_key = os.environ.get("GEMINI_API_KEY")
        elif self.provider == "openai":
            self.api_key = os.environ.get("OPENAI_API_KEY")

    def generate_queries(self, task: str) -> List[str]:
        """Generate expanded queries for HyDE."""
        if not self.api_key:
            return []

        prompt = f"""
        You are an expert search assistant. Given the following task description,
        generate 3 specific search queries that would help find relevant past memories
        to solve this task.

        Task: {task}

        Return only the queries, one per line, without any other text.
        """

        try:
            response = self._call_llm(prompt)
            if response:
                return [q.strip() for q in response.split("\n") if q.strip()]
        except Exception as e:
            logger.error(f"Failed to generate queries: {e}")

        return []

    def rerank(self, task: str, candidates: List[dict]) -> List[dict]:
        """Rerank candidates based on task relevance."""
        if not self.api_key or not candidates:
            return candidates

        # Prepare candidates for prompt
        candidate_texts = []
        for i, c in enumerate(candidates):
            candidate_texts.append(f"[{i}] {c['content'][:300]}")

        prompt = f"""
        Task: {task}

        Below are some past memories. Rate each memory's relevance to the current task
        on a scale of 0 to 10, where 10 is extremely relevant and 0 is irrelevant.

        Memories:
        {chr(10).join(candidate_texts)}

        Return the results as a comma-separated list of scores in the same order as the memories.
        Example: 8, 2, 9, 5
        """

        try:
            response = self._call_llm(prompt)
            if response:
                scores = [float(s.strip()) for s in response.split(",") if s.strip()]

                # Apply scores and filter
                reranked = []
                for i, score in enumerate(scores):
                    if i < len(candidates):
                        candidates[i]["relevance_score"] = score
                        if score >= 7.0:  # REQ-SR-03: Threshold 7
                            reranked.append(candidates[i])

                # Sort by relevance score
                reranked.sort(key=lambda x: x.get("relevance_score", 0), reverse=True)
                return reranked
        except Exception as e:
            logger.error(f"Failed to rerank: {e}")

        return candidates

    def _call_llm(self, prompt: str) -> Optional[str]:
        """Call the configured LLM provider."""
        if self.provider == "gemini":
            return self._call_gemini(prompt)
        elif self.provider == "openai":
            return self._call_openai(prompt)
        return None

    def _call_gemini(self, prompt: str) -> Optional[str]:
        try:
            from google import genai

            client = genai.Client(api_key=self.api_key)
            response = client.models.generate_content(
                model="gemini-2.0-flash", contents=prompt
            )
            return str(response.text)
        except Exception as e:
            logger.error(f"Gemini call failed: {e}")
            return None

    def _call_openai(self, prompt: str) -> Optional[str]:
        try:
            from openai import OpenAI

            client = OpenAI(api_key=self.api_key)
            response = client.chat.completions.create(
                model="gpt-4o-mini", messages=[{"role": "user", "content": prompt}]
            )
            return str(response.choices[0].message.content)
        except Exception as e:
            logger.error(f"OpenAI call failed: {e}")
            return None
