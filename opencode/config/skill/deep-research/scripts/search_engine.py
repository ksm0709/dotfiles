#!/usr/bin/env python3
"""
Search Engine Module - DuckDuckGo based web search

Standalone module for deep-research skill.
"""

from typing import List, Dict, Optional
import logging

try:
    from duckduckgo_search import DDGS
except ImportError:
    DDGS = None

logger = logging.getLogger(__name__)


class SearchEngine:
    """Web search engine wrapper using DuckDuckGo."""

    def __init__(self, max_results: int = 5):
        """Initialize the search engine.

        Args:
            max_results: Default maximum results per query.
        """
        if DDGS is None:
            raise ImportError(
                "duckduckgo-search not installed. Run: pip install duckduckgo-search"
            )
        self.max_results = max_results
        self._ddgs = DDGS()

    def search(
        self, query: str, max_results: Optional[int] = None
    ) -> List[Dict[str, str]]:
        """Perform a web search.

        Args:
            query: Search query string.
            max_results: Optional limit on results.

        Returns:
            List of dicts with 'title', 'href', 'body' keys.
        """
        limit = max_results or self.max_results
        results = []

        try:
            logger.info(f"Searching: '{query}' (limit={limit})")
            search_gen = self._ddgs.text(query, max_results=limit)

            for r in search_gen:
                results.append(
                    {
                        "title": r.get("title", ""),
                        "href": r.get("href", ""),
                        "body": r.get("body", ""),
                    }
                )
        except Exception as e:
            logger.error(f"Search failed for '{query}': {e}")
            return []

        return results
