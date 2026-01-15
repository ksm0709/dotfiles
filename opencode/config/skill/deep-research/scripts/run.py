#!/usr/bin/env python3
"""
Deep Research - Standalone LLM-based research pipeline

이 스크립트는 독립적으로 실행 가능하며, 심층 리서치 파이프라인을 제공합니다.
Plan → Execute → Report 단계로 구성됩니다.

Usage:
    # 환경 변수 로드 후 실행
    source scripts/load_env.sh
    python run.py "research topic" --depth 3

Example:
    python run.py "AI trends in 2026"
"""

import argparse
import json
import logging
import os
import sys
import time
import hashlib
from datetime import datetime
from typing import List, Dict, Callable, Any

# Add scripts directory to path for local imports
scripts_dir = os.path.dirname(os.path.abspath(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Import local modules (after sys.path modification)
from search_engine import SearchEngine  # noqa: E402
from scraper import WebScraper  # noqa: E402
from llm_client import llm_complete  # noqa: E402


class ResearchArchive:
    """Manages local storage of research data."""

    def __init__(self, base_dir: str = None):
        """Initialize archive manager.
        
        Args:
            base_dir: Base directory for research cache. 
                      Defaults to ~/.cache/opencode/research
        """
        if base_dir is None:
            # Use user cache directory for global skill
            cache_home = os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache"))
            self.base_dir = os.path.join(cache_home, "opencode", "research")
        else:
            self.base_dir = os.path.expanduser(base_dir)

        os.makedirs(self.base_dir, exist_ok=True)

    def create_session(self, topic: str) -> str:
        """Create a new research session."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        topic_hash = hashlib.md5(topic.encode()).hexdigest()[:8]
        safe_topic = "".join([c if c.isalnum() else "_" for c in topic])[:30]

        session_id = f"{timestamp}_{safe_topic}_{topic_hash}"
        session_path = os.path.join(self.base_dir, session_id)

        os.makedirs(session_path, exist_ok=True)
        os.makedirs(os.path.join(session_path, "sources"), exist_ok=True)

        # Save metadata
        meta = {"topic": topic, "created_at": timestamp, "status": "initialized"}
        self._save_json(os.path.join(session_path, "metadata.json"), meta)

        logger.info(f"Created session: {session_id}")
        return session_id

    def save_content(
        self, session_id: str, url: str, content: str, title: str = ""
    ) -> str:
        """Save scraped content."""
        session_path = os.path.join(self.base_dir, session_id)

        url_hash = hashlib.md5(url.encode()).hexdigest()
        filename = f"{url_hash}.md"
        filepath = os.path.join(session_path, "sources", filename)

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(f"# {title}\nSource: {url}\n\n{content}")

        return filename

    def _save_json(self, path: str, data: Dict):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)


class DeepResearch:
    """Orchestrates the deep research process."""

    def __init__(
        self, llm_callback: Callable[[str], str], base_dir: str = None
    ):
        """Initialize the researcher."""
        self.llm = llm_callback
        self.engine = SearchEngine()
        self.scraper = WebScraper()
        self.archive = ResearchArchive(base_dir)

    def create_plan(self, topic: str) -> Dict[str, Any]:
        """Generate a research plan."""
        session_id = self.archive.create_session(topic)

        prompt = (
            f"Topic: {topic}\n"
            "Create a research plan. Return ONLY raw JSON:\n"
            '{"steps": [{"query": "search query", "rationale": "reason"}, ...]}\n'
            "Limit to 3-5 steps."
        )

        logger.info(f"Generating plan for: {topic}")
        response = self.llm(prompt)

        try:
            clean_resp = response.strip()
            if clean_resp.startswith("```"):
                clean_resp = clean_resp.split("\n", 1)[1].rsplit("\n", 1)[0]
            if clean_resp.startswith("json"):
                clean_resp = clean_resp[4:]

            plan_data = json.loads(clean_resp)
            if "steps" not in plan_data:
                raise ValueError("Missing 'steps' key")
        except Exception as e:
            logger.warning(f"Failed to parse plan: {e}, using fallback")
            plan_data = {
                "steps": [
                    {"query": topic, "rationale": "General overview"},
                    {"query": f"{topic} news", "rationale": "Recent updates"},
                    {"query": f"{topic} analysis", "rationale": "Deep analysis"},
                ]
            }

        return {"session_id": session_id, "topic": topic, "plan": plan_data}

    def execute_plan(
        self, session_data: Dict[str, Any], max_urls_per_step: int = 3
    ) -> List[Dict]:
        """Execute the research plan."""
        session_id = session_data["session_id"]
        steps = session_data["plan"]["steps"]
        results = []

        print(f"\n🔬 Executing research for session: {session_id}")

        for i, step in enumerate(steps, 1):
            query = step.get("query", "")
            rationale = step.get("rationale", "")

            print(f"\n--- Step {i}/{len(steps)}: {query} ---")
            print(f"Rationale: {rationale}")

            # Search
            search_results = self.engine.search(query, max_results=max_urls_per_step)
            logger.info(f"Found {len(search_results)} links")

            step_content = []

            # Scrape
            for res in search_results:
                url = res["href"]
                title = res["title"]

                print(f"  > Fetching: {title[:50]}...")
                content = self.scraper.fetch_content(url)

                if content:
                    self.archive.save_content(session_id, url, content, title)
                    step_content.append(f"Source: {title} ({url})\n{content[:2000]}...")
                    time.sleep(1)  # Polite delay

            results.append(
                {
                    "step": i,
                    "query": query,
                    "found_sources": len(step_content),
                    "snippets": step_content,
                }
            )

        print("\n✅ Execution complete.")
        return results

    def generate_report(self, session_data: Dict, results: List[Dict]) -> str:
        """Generate final report."""
        context = f"# Research Topic: {session_data['topic']}\n\n"

        for step in results:
            context += f"## Findings for: {step['query']}\n"
            for snippet in step["snippets"]:
                context += snippet + "\n\n---\n\n"

        prompt = (
            "Write a comprehensive research report based on the gathered data.\n"
            "Structure: Executive Summary, Key Findings, Details, Conclusion.\n"
            "Cite sources where possible.\n\n"
            f"{context[:15000]}"
        )

        logger.info("Generating report...")
        report = self.llm(prompt)

        # Save report
        report_path = (
            f"{self.archive.base_dir}/{session_data['session_id']}/final_report.md"
        )
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(report)

        return report


def check_api_keys() -> Dict[str, bool]:
    """Check available API keys."""
    gemini_key = os.environ.get("GEMINI_API_KEY", "")
    openai_key = os.environ.get("OPENAI_API_KEY", "")

    status = {
        "gemini": bool(gemini_key),
        "openai": bool(openai_key),
        "any": bool(gemini_key or openai_key),
    }

    if not status["any"]:
        print("⚠️  No API keys found!")
        print("   Run: source scripts/load_env.sh")
        print("   Or: export GEMINI_API_KEY='your-key'")
        print("   Falling back to mock responses...\n")
    elif status["gemini"]:
        preview = "*" * max(0, len(gemini_key) - 8) + gemini_key[-8:]
        print(f"✅ Using Gemini API (key: {preview})")
    elif status["openai"]:
        preview = "*" * max(0, len(openai_key) - 8) + openai_key[-8:]
        print(f"✅ Using OpenAI API (key: {preview})")

    return status


def main():
    parser = argparse.ArgumentParser(description="Deep Research Tool")
    parser.add_argument("topic", help="Research topic")
    parser.add_argument("--depth", type=int, default=3, help="URLs per step")
    parser.add_argument("--output-dir", type=str, default=None, 
                        help="Custom output directory for research cache")

    args = parser.parse_args()

    # Check API keys
    check_api_keys()

    print(f"🔬 Starting deep research: {args.topic}")

    # Run research
    researcher = DeepResearch(llm_callback=llm_complete, base_dir=args.output_dir)

    session = researcher.create_plan(args.topic)
    print(f"\n📋 Plan:\n{json.dumps(session['plan'], indent=2, ensure_ascii=False)}")

    results = researcher.execute_plan(session, max_urls_per_step=args.depth)

    print("\n📝 Generating report...")
    report = researcher.generate_report(session, results)

    print("\n" + "=" * 60)
    print("📊 FINAL REPORT")
    print("=" * 60)
    print(report)
    print("=" * 60)

    report_path = (
        f"{researcher.archive.base_dir}/{session['session_id']}/final_report.md"
    )
    print(f"\n💾 Saved to: {report_path}")


if __name__ == "__main__":
    main()
