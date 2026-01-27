"""
Context API Client

Provides HTTP client for communicating with Context API Server.
Uses standard library (urllib) to avoid extra dependencies.
"""

import json
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, cast

from .utils.logging import get_client_logger

logger = get_client_logger()


class ContextAPIClient:
    """
    Client for Context API Server.
    """

    def __init__(self, port: int, api_key: str):
        """
        Args:
            port: Server port
            api_key: API Key for authentication
        """
        self.base_url = f"http://127.0.0.1:{port}"
        self.api_key = api_key
        self.headers = {
            "Content-Type": "application/json",
            "X-API-Key": api_key,
        }

    def _request(
        self,
        method: str,
        endpoint: str,
        data: Dict[str, Any] | None = None,
        timeout: int = 2,
        raw: bool = False,
    ) -> Any:
        """
        Make HTTP request.

        Args:
            method: HTTP method (GET, POST)
            endpoint: API endpoint (e.g., "/start")
            data: Request body data
            timeout: Request timeout
            raw: If True, return raw response body instead of JSON

        Returns:
            Any: Response JSON or raw string
        """
        url = f"{self.base_url}{endpoint}"

        try:
            if data is not None:
                json_data = json.dumps(data).encode("utf-8")
                req = urllib.request.Request(
                    url, data=json_data, headers=self.headers, method=method
                )
            else:
                req = urllib.request.Request(url, headers=self.headers, method=method)

            with urllib.request.urlopen(req, timeout=timeout) as response:
                response_body = response.read().decode("utf-8")
                if raw:
                    return response_body
                return json.loads(response_body)

        except urllib.error.HTTPError as e:
            error_body = e.read().decode("utf-8")
            logger.error(f"API Error {e.code}: {error_body}")
            try:
                return cast(Dict[str, Any], json.loads(error_body))
            except Exception:
                raise Exception(f"HTTP {e.code}: {e.reason}")
        except urllib.error.URLError as e:
            logger.error(f"Connection failed: {e.reason}")
            raise ConnectionError(f"Failed to connect to server: {e.reason}")
        except Exception as e:
            logger.error(f"Request failed: {e}")
            raise

    def health_check(self) -> bool:
        """Check if server is healthy."""
        try:
            # Use shorter timeout for health check
            res = cast(Dict[str, Any], self._request("GET", "/health", timeout=1))
            return res.get("status") == "ok"
        except Exception:
            return False

    def init(self, session_id: str) -> Dict[str, Any]:
        """Initialize session."""
        return cast(
            Dict[str, Any], self._request("POST", "/init", {"session_id": session_id})
        )

    def start(self, session_id: str, task: str) -> Dict[str, Any]:
        """Start task."""
        return cast(
            Dict[str, Any],
            self._request("POST", "/start", {"session_id": session_id, "task": task}),
        )

    def checkpoint(self, session_id: str, summary: str = "") -> Dict[str, Any]:
        """Perform checkpoint."""
        return cast(
            Dict[str, Any],
            self._request(
                "POST", "/checkpoint", {"session_id": session_id, "summary": summary}
            ),
        )

    def end(self, session_id: str, result: str = "") -> Dict[str, Any]:
        """End task."""
        return cast(
            Dict[str, Any],
            self._request("POST", "/end", {"session_id": session_id, "result": result}),
        )

    def status(self, session_id: Optional[str] = None) -> Dict[str, Any]:
        """Get status."""
        endpoint = "/status"
        if session_id:
            endpoint += f"?session_id={session_id}"
        return cast(Dict[str, Any], self._request("GET", endpoint))
