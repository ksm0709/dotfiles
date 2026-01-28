"""
Problem Tracker Service

Task 2.3: ProblemTracker 서비스 구현
- 오류 감지 시 추적 시작
- 해결 시도 기록
- 문제 해결 완료 처리
- 활성 문제 상태 관리
"""

import re
import uuid
from typing import Dict, Optional

from opencode_memory.models.semantic import ProblemResolution


class ProblemTracker:
    """오류-해결 추적 서비스

    세션별 활성 문제를 추적하고, 해결 시도와 성공을 기록합니다.
    """

    # 에러 유형 분류 패턴
    ERROR_PATTERNS = [
        (r"ModuleNotFoundError", "ModuleNotFoundError"),
        (r"ImportError", "ImportError"),
        (r"SyntaxError", "SyntaxError"),
        (r"TypeError", "TypeError"),
        (r"ValueError", "ValueError"),
        (r"AttributeError", "AttributeError"),
        (r"KeyError", "KeyError"),
        (r"IndexError", "IndexError"),
        (r"FileNotFoundError", "FileNotFoundError"),
        (r"PermissionError", "PermissionError"),
        (r"ConnectionError", "ConnectionError"),
        (r"TimeoutError", "TimeoutError"),
        (r"RuntimeError", "RuntimeError"),
        (r"exit code [1-9]", "ExitCodeError"),
        (r"FAILED.*test", "TestFailure"),
        (r"AssertionError", "AssertionError"),
        (r"Exception", "Exception"),
        (r"Error:", "GenericError"),
    ]

    def __init__(self):
        """초기화"""
        # session_id -> ProblemResolution
        self._active_problems: Dict[str, ProblemResolution] = {}
        # problem_id -> session_id (역방향 매핑)
        self._problem_sessions: Dict[str, str] = {}

    def start_tracking(self, session_id: str, error: str) -> str:
        """오류 추적 시작

        Args:
            session_id: 세션 ID
            error: 오류 메시지

        Returns:
            problem_id: 생성된 문제 ID
        """
        problem_id = str(uuid.uuid4())
        error_type = self._classify_error(error)

        problem = ProblemResolution(
            id=problem_id,
            error_type=error_type,
            error_message=error[:500],  # 최대 500자
            attempted_solutions=[],
            successful_solution=None,
            path_to_solution=[],
            status="active",
        )

        self._active_problems[session_id] = problem
        self._problem_sessions[problem_id] = session_id

        return problem_id

    def _classify_error(self, error: str) -> str:
        """오류 유형 분류

        Args:
            error: 오류 메시지

        Returns:
            에러 유형 문자열
        """
        for pattern, error_type in self.ERROR_PATTERNS:
            if re.search(pattern, error, re.IGNORECASE):
                return error_type

        return "UnknownError"

    def has_active_problem(self, session_id: str) -> bool:
        """세션에 활성 문제가 있는지 확인

        Args:
            session_id: 세션 ID

        Returns:
            활성 문제 존재 여부
        """
        return session_id in self._active_problems

    def get_active_problem(self, session_id: str) -> Optional[ProblemResolution]:
        """세션의 활성 문제 조회

        Args:
            session_id: 세션 ID

        Returns:
            ProblemResolution 또는 None
        """
        return self._active_problems.get(session_id)

    def get_problem_by_id(self, problem_id: str) -> Optional[ProblemResolution]:
        """문제 ID로 조회

        Args:
            problem_id: 문제 ID

        Returns:
            ProblemResolution 또는 None
        """
        session_id = self._problem_sessions.get(problem_id)
        if session_id:
            problem = self._active_problems.get(session_id)
            if problem and problem.id == problem_id:
                return problem
        return None

    def add_attempt(self, problem_id: str, solution: str) -> bool:
        """해결 시도 기록

        Args:
            problem_id: 문제 ID
            solution: 시도한 해결책

        Returns:
            성공 여부
        """
        session_id = self._problem_sessions.get(problem_id)
        if not session_id:
            return False

        problem = self._active_problems.get(session_id)
        if not problem or problem.id != problem_id:
            return False

        if problem.status != "active":
            return False

        # 중복 방지
        if solution not in problem.attempted_solutions:
            problem.attempted_solutions.append(solution)
            problem.path_to_solution.append(solution)

        return True

    def resolve(self, problem_id: str, solution: str) -> Optional[ProblemResolution]:
        """문제 해결 완료

        Args:
            problem_id: 문제 ID
            solution: 성공한 해결책

        Returns:
            완료된 ProblemResolution 또는 None
        """
        session_id = self._problem_sessions.get(problem_id)
        if not session_id:
            return None

        problem = self._active_problems.get(session_id)
        if not problem or problem.id != problem_id:
            return None

        # 문제 상태 업데이트
        problem.status = "resolved"
        problem.successful_solution = solution

        # solution이 attempted_solutions에 없으면 추가
        if solution not in problem.attempted_solutions:
            problem.attempted_solutions.append(solution)
            problem.path_to_solution.append(solution)

        # 활성 목록에서 제거
        del self._active_problems[session_id]
        del self._problem_sessions[problem_id]

        return problem

    def abandon(self, problem_id: str) -> Optional[ProblemResolution]:
        """문제 포기 처리

        Args:
            problem_id: 문제 ID

        Returns:
            포기된 ProblemResolution 또는 None
        """
        session_id = self._problem_sessions.get(problem_id)
        if not session_id:
            return None

        problem = self._active_problems.get(session_id)
        if not problem or problem.id != problem_id:
            return None

        # 문제 상태 업데이트
        problem.status = "abandoned"

        # 활성 목록에서 제거
        del self._active_problems[session_id]
        del self._problem_sessions[problem_id]

        return problem

    def get_session_for_problem(self, problem_id: str) -> Optional[str]:
        """문제 ID로 세션 ID 조회

        Args:
            problem_id: 문제 ID

        Returns:
            세션 ID 또는 None
        """
        return self._problem_sessions.get(problem_id)


# 싱글톤 인스턴스 (서버 전역에서 사용)
_global_tracker: Optional[ProblemTracker] = None


def get_problem_tracker() -> ProblemTracker:
    """전역 ProblemTracker 인스턴스 반환"""
    global _global_tracker
    if _global_tracker is None:
        _global_tracker = ProblemTracker()
    return _global_tracker


def reset_problem_tracker():
    """테스트용: 전역 인스턴스 리셋"""
    global _global_tracker
    _global_tracker = None
