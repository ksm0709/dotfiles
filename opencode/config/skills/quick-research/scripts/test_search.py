#!/usr/bin/env python3
"""
Test suite for Research Skill (Quick Search)

Run: python test_search.py
"""

import sys
import argparse
import unittest
from unittest.mock import patch, MagicMock


class TestSearchFunction(unittest.TestCase):
    """Test cases for the search function."""

    def _mock_ddgs_class_with_results(self, search_results):
        mock_ddgs = MagicMock()
        mock_instance = MagicMock()
        mock_ddgs.return_value.__enter__.return_value = mock_instance
        mock_instance.text.return_value = search_results
        return mock_ddgs

    def test_positive_int_valid(self):
        from run import _positive_int

        self.assertEqual(_positive_int("1"), 1)
        self.assertEqual(_positive_int("5"), 5)

    def test_positive_int_invalid(self):
        from run import _positive_int

        with self.assertRaises(argparse.ArgumentTypeError):
            _positive_int("0")

        with self.assertRaises(argparse.ArgumentTypeError):
            _positive_int("-3")

        with self.assertRaises(argparse.ArgumentTypeError):
            _positive_int("abc")

    def test_search_handles_missing_dependency(self):
        from run import search

        with patch("run._load_ddgs_class", return_value=None) as mock_loader:
            results = search("test query", max_results=1)

        mock_loader.assert_called_once_with(emit_error=True)
        self.assertEqual(results, [])

    def test_search_returns_list(self):
        """Test that search returns a list."""
        from run import search

        mock_ddgs = self._mock_ddgs_class_with_results(
            [{"title": "Test", "href": "https://test.com", "body": "Test body"}]
        )

        with patch("run._load_ddgs_class", return_value=mock_ddgs):
            results = search("test query", max_results=1)

        self.assertIsInstance(results, list)
        self.assertEqual(len(results), 1)

    def test_search_result_structure(self):
        """Test that search results have correct structure."""
        from run import search

        mock_ddgs = self._mock_ddgs_class_with_results(
            [{"title": "Test Title", "href": "https://test.com", "body": "Snippet"}]
        )

        with patch("run._load_ddgs_class", return_value=mock_ddgs):
            results = search("test", max_results=1)

        self.assertIn("title", results[0])
        self.assertIn("url", results[0])
        self.assertIn("snippet", results[0])
        self.assertEqual(results[0]["title"], "Test Title")
        self.assertEqual(results[0]["url"], "https://test.com")

    def test_search_handles_empty_results(self):
        """Test that search handles empty results gracefully."""
        from run import search

        mock_ddgs = self._mock_ddgs_class_with_results([])

        with patch("run._load_ddgs_class", return_value=mock_ddgs):
            results = search("nonexistent query xyz123", max_results=5)

        self.assertIsInstance(results, list)
        self.assertEqual(len(results), 0)

    def test_search_handles_exception(self):
        """Test that search handles exceptions gracefully."""
        from run import search

        mock_ddgs = MagicMock()
        mock_ddgs.return_value.__enter__.side_effect = Exception("Network error")

        with patch("run._load_ddgs_class", return_value=mock_ddgs):
            results = search("test", max_results=1)

        self.assertIsInstance(results, list)
        self.assertEqual(len(results), 0)


class TestIntegration(unittest.TestCase):
    """Integration tests (requires network)."""

    @unittest.skipIf("--skip-network" in sys.argv, "Skipping network tests")
    def test_real_search(self):
        """Test actual search (requires network)."""
        from run import search

        results = search("Python programming", max_results=3)

        self.assertIsInstance(results, list)
        # DuckDuckGo should return some results
        self.assertGreater(len(results), 0)

        # Check result structure
        for r in results:
            self.assertIn("title", r)
            self.assertIn("url", r)
            self.assertIn("snippet", r)


def run_tests():
    """Run all tests."""
    print("=" * 60)
    print("Research Skill - Test Suite")
    print("=" * 60)

    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestSearchFunction)

    # Add integration tests if not skipping
    if "--skip-network" not in sys.argv:
        suite.addTests(loader.loadTestsFromTestCase(TestIntegration))

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(run_tests())
