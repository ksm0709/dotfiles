#!/usr/bin/env python3
"""
Quick Search - Standalone DuckDuckGo Web Search

이 스크립트는 독립적으로 실행 가능하며, 외부 프로젝트 의존성이 없습니다.

Usage:
    python run.py "search query" [max_results]

Example:
    python run.py "Python best practices" 10
"""

import argparse
import sys
from typing import Any, Dict, List, Optional, Type

_DDGS_CLASS: Optional[Type[Any]] = None
_DDGS_LOAD_ATTEMPTED = False


def _load_ddgs_class(emit_error: bool = False) -> Optional[Type[Any]]:
    global _DDGS_CLASS, _DDGS_LOAD_ATTEMPTED

    if not _DDGS_LOAD_ATTEMPTED:
        _DDGS_LOAD_ATTEMPTED = True
        try:
            from ddgs import DDGS  # type: ignore

            _DDGS_CLASS = DDGS
        except ImportError:
            try:
                from duckduckgo_search import DDGS  # type: ignore

                _DDGS_CLASS = DDGS
            except ImportError:
                _DDGS_CLASS = None

    if _DDGS_CLASS is None and emit_error:
        print("❌ Error: missing DuckDuckGo search dependency")
        print("   Install with: pip install ddgs")
        print("   Or fallback package: pip install duckduckgo-search>=3.0.0")

    return _DDGS_CLASS


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("max_results must be a positive integer") from exc

    if parsed <= 0:
        raise argparse.ArgumentTypeError("max_results must be a positive integer")

    return parsed


def search(query: str, max_results: int = 5) -> List[Dict[str, str]]:
    """Perform a quick web search using DuckDuckGo.

    Args:
        query: Search query string.
        max_results: Maximum number of results to return.

    Returns:
        List of search result dictionaries with keys: title, url, snippet
    """
    print(f"🔍 Searching for: {query}")
    results = []
    ddgs_class = _load_ddgs_class(emit_error=True)
    if ddgs_class is None:
        return results

    try:
        with ddgs_class() as ddgs:
            # backend='html' 사용 (더 안정적)
            search_results = list(
                ddgs.text(query, max_results=max_results, backend="html")
            )
            if not search_results:
                print("No results with backend='html'. Trying default...")
                search_results = list(ddgs.text(query, max_results=max_results))

            for r in search_results:
                result = {
                    "title": r.get("title", ""),
                    "url": r.get("href", ""),
                    "snippet": r.get("body", ""),
                }
                results.append(result)
                print(f"Title: {result['title']}")
                print(f"URL: {result['url']}")
                print(f"Snippet: {result['snippet']}")
                print("-" * 40)

    except Exception as e:
        print(f"❌ Error during search: {e}")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Quick Search - Standalone DuckDuckGo Web Search"
    )
    parser.add_argument("query", help="Search query")
    parser.add_argument(
        "max_results",
        nargs="?",
        type=_positive_int,
        default=5,
        help="Maximum number of results (positive integer)",
    )

    args = parser.parse_args()
    dependency_available = _load_ddgs_class(emit_error=False) is not None

    results = search(args.query, args.max_results)
    print(f"\n✅ Found {len(results)} results")
    return 0 if dependency_available else 1


if __name__ == "__main__":
    sys.exit(main())
