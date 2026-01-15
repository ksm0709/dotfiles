#!/usr/bin/env python3
"""
LLM Client Module - Unified interface for Gemini/OpenAI

Standalone module for deep-research skill.
Loads API keys from environment variables.
"""

import os
import json
import logging
from typing import Optional

logger = logging.getLogger(__name__)

# Optional imports
_gemini_available = False
_openai_available = False

try:
    from google import genai

    _gemini_available = True
except ImportError:
    genai = None

try:
    from openai import OpenAI

    _openai_available = True
except ImportError:
    OpenAI = None


def gemini_complete(prompt: str, model: str = "gemini-2.0-flash") -> str:
    """Call Gemini API.

    Args:
        prompt: The prompt to send.
        model: Model name to use.

    Returns:
        Response text or mock response if API unavailable.
    """
    api_key = os.environ.get("GEMINI_API_KEY")

    if not api_key:
        logger.warning("GEMINI_API_KEY not found, using mock response")
        return mock_response(prompt)

    if not _gemini_available or genai is None:
        logger.warning("google-generativeai not installed, using mock response")
        return mock_response(prompt)

    try:
        client = genai.Client(api_key=api_key)
        response = client.models.generate_content(model=model, contents=prompt)
        return response.text or ""
    except Exception as e:
        logger.error(f"Gemini API error: {e}")
        return f"Error: {e}"


def openai_complete(prompt: str, model: str = "gpt-4o") -> str:
    """Call OpenAI API.

    Args:
        prompt: The prompt to send.
        model: Model name to use.

    Returns:
        Response text or mock response if API unavailable.
    """
    api_key = os.environ.get("OPENAI_API_KEY")

    if not api_key:
        logger.warning("OPENAI_API_KEY not found, using mock response")
        return mock_response(prompt)

    if not _openai_available or OpenAI is None:
        logger.warning("openai not installed, using mock response")
        return mock_response(prompt)

    try:
        client = OpenAI(api_key=api_key)
        response = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7,
        )
        return response.choices[0].message.content or ""
    except Exception as e:
        logger.error(f"OpenAI API error: {e}")
        return f"Error: {e}"


def llm_complete(
    prompt: str, provider: Optional[str] = None, model: Optional[str] = None
) -> str:
    """Unified LLM interface.

    Priority:
    1. Explicit provider argument
    2. LLM_PROVIDER env var
    3. Default to Gemini if GEMINI_API_KEY exists
    4. Fall back to OpenAI
    5. Mock response

    Args:
        prompt: The prompt to send.
        provider: Optional provider override ('gemini' or 'openai').
        model: Optional model name override.

    Returns:
        Response text.
    """
    if not provider:
        provider = os.environ.get("LLM_PROVIDER")

    if not provider:
        # Auto-detect based on available keys
        if os.environ.get("GEMINI_API_KEY"):
            provider = "gemini"
        elif os.environ.get("OPENAI_API_KEY"):
            provider = "openai"
        else:
            provider = "gemini"  # Will fall back to mock

    if provider == "openai":
        return openai_complete(prompt, model=model or "gpt-4o")
    else:
        return gemini_complete(prompt, model=model or "gemini-2.0-flash")


def mock_response(prompt: str) -> str:
    """Fallback mock response when no API is available."""
    if "json" in prompt.lower():
        return json.dumps(
            {
                "steps": [
                    {"query": "topic overview", "rationale": "General information"},
                    {"query": "topic news", "rationale": "Recent updates"},
                    {"query": "topic analysis", "rationale": "Deep dive"},
                ]
            }
        )

    return """# Deep Research Report (Mock Mode)

⚠️ **API Key Missing - Using Mock Response**

This is a placeholder because no API key was found.

## To Enable Real LLM Responses:

```bash
# Option 1: Gemini (Recommended)
export GEMINI_API_KEY="your-key-here"

# Option 2: OpenAI
export OPENAI_API_KEY="your-key-here"

# Then run:
source scripts/load_env.sh
```

## Mock Summary

The research would normally contain:
- Executive Summary
- Key Findings
- Detailed Analysis
- Conclusions

---"""
