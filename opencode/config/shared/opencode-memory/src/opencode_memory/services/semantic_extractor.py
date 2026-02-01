"""
Semantic Extractor Service

Encapsulates LLM calls for semantic analysis of agent activities.
"""

import json
import logging
from typing import Any, Dict, List

from ..utils.llm import AsyncLLMClient

logger = logging.getLogger(__name__)


class SemanticExtractor:
    def __init__(self, llm_client: AsyncLLMClient):
        self.llm = llm_client

    async def extract_intent(
        self, tool: str, args: Dict[str, Any], context: str = ""
    ) -> str:
        """
        Extract the high-level intent from a tool execution.
        """
        if not self.llm.enabled:
            return f"Execute {tool}"

        prompt = f"""
        Analyze the following tool execution and extract the high-level user intent.

        Tool: {tool}
        Arguments: {json.dumps(args, indent=2)}
        Context: {context}

        Summarize the intent in one clear, concise sentence (e.g., "Fix the bug in the login handler" or "Search for documentation on API").
        Do not include the tool name or technical details unless necessary for understanding the goal.
        """

        intent = await self.llm.generate(prompt)
        return intent.strip() if intent else f"Execute {tool}"

    async def reflect_on_episode(self, events: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Reflect on a sequence of events (episode) to extract key learnings and summary.
        """
        if not self.llm.enabled or not events:
            return {}

        # Format events for the prompt
        event_log = []
        for e in events:
            role = e.get("role", "unknown")
            content = e.get("content", "")
            # Truncate content if too long
            if isinstance(content, str) and len(content) > 500:
                content = content[:500] + "..."
            event_log.append(f"[{role}] {content}")

        prompt = f"""
        Analyze the following development session (episode) and provide a structured reflection.

        Episode Log:
        {chr(10).join(event_log)}

        Provide the output in JSON format with the following keys:
        - summary: A concise summary of what was accomplished (1-2 sentences).
        - success: Boolean, whether the apparent goal was achieved.
        - key_learnings: A list of string bullet points containing technical facts, patterns, or mistakes learned.
        - next_steps: Suggested next steps if the task is incomplete.

        Return ONLY valid JSON.
        """

        response = await self.llm.generate(
            prompt, system_instruction="You are a JSON-speaking developer assistant."
        )

        if response:
            try:
                # Clean up markdown code blocks if present
                clean_response = (
                    response.replace("```json", "").replace("```", "").strip()
                )
                return json.loads(clean_response)
            except json.JSONDecodeError:
                logger.error("Failed to parse reflection JSON")

        return {}

    async def detect_goal_change(self, current_goal: str, new_input: str) -> bool:
        """
        Detect if the user's new input represents a significant change in goal.
        """
        if not self.llm.enabled:
            return False

        prompt = f"""
        Current Goal: {current_goal}
        New Input: {new_input}

        Does the 'New Input' represent a completely new task or a significant pivot away from the 'Current Goal'?
        Reply with 'YES' or 'NO'.
        """

        response = await self.llm.generate(prompt)
        if response:
            return response.strip().upper().startswith("YES")

        return False
