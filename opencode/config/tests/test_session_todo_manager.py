"""
OpenSpec 시나리오 기반 SimpleTodoManager 테스트 코드

OpenSpec에 명시된 모든 시나리오를 테스트 케이스로 구현
TDD 원칙에 따라 구현 전 실패하는 테스트 작성
"""

import pytest
import tempfile
import shutil
import json
import os
import sys
from pathlib import Path
from unittest.mock import patch, MagicMock
from datetime import datetime

# 테스트 대상 모듈 임포트
sys.path.insert(0, str(Path(__file__).parent.parent / "tools"))
from simple_todo_new import SimpleTodoManager


class TestSessionIDPersistence:
    """OpenSpec Requirement: Session ID Persistence 테스트"""
    
    def setup_method(self):
        """각 테스트 전 임시 프로젝트 구조 설정"""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir)
        self.opencode_dir = self.project_root / ".opencode"
        self.sessions_dir = self.opencode_dir / "sessions"
        self.opencode_dir.mkdir()
        self.sessions_dir.mkdir()
        
        # 프로젝트 루트를 임시 디렉토리로 설정
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)
    
    def teardown_method(self):
        """각 테스트 후 정리"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.temp_dir)
    
    def test_subagent_multiple_calls_session_persistence(self):
        """
        OpenSpec Scenario: Subagent Multiple Calls
        
        Given a subagent calls SimpleTodoManager multiple times in the same session
        When the first call creates a session with ID "abc123"
        Then subsequent calls in the same session shall use ID "abc123"
        And the session data shall be stored in the same directory
        """
        # Arrange
        session_id = "abc123"
        agent_name = "senior-sw-engineer"
        
        # Act - 첫 번째 호출
        todos1 = SimpleTodoManager(agent_name, session_id)
        task1_id = todos1.add_todo("첫 번째 작업", "high")
        
        # Act - 두 번째 호출 (동일한 세션)
        todos2 = SimpleTodoManager(agent_name, session_id)
        task2_id = todos2.add_todo("두 번째 작업", "medium")
        
        # Assert - 동일한 디렉토리 사용
        assert todos1.todo_dir == todos2.todo_dir
        assert todos1.session_id == todos2.session_id == session_id
        
        # Assert - 두 작업 모두 동일한 파일에 저장
        all_todos1 = todos1.load_todos()
        all_todos2 = todos2.load_todos()
        assert len(all_todos1) == len(all_todos2) == 2
        
        # Assert - 작업 내용 확인
        task_contents = [todo["content"] for todo in all_todos1]
        assert "첫 번째 작업" in task_contents
        assert "두 번째 작업" in task_contents
    
    def test_session_id_extraction_from_context(self):
        """
        OpenSpec Scenario: Session ID Extraction
        
        Given a TypeScript wrapper is called as an OpenCode Custom Tool
        When the context object contains sessionID "xyz789"
        Then the wrapper shall extract and pass "xyz789" to the Python core
        And the Python core shall use "xyz789" as the session identifier
        """
        # Arrange
        session_id = "xyz789"
        agent_name = "py-code-reviewer"
        
        # Act - Python Core에서 세션 ID 사용
        todos = SimpleTodoManager(agent_name, session_id)
        
        # Assert - 세션 ID가 올바르게 설정됨
        assert todos.session_id == session_id
        
        # Assert - 세션별 디렉토리 생성
        expected_dir = self.sessions_dir / f"{agent_name}_{session_id}"
        assert todos.todo_dir == str(expected_dir)
        assert expected_dir.exists()


class TestProjectRelativePathResolution:
    """OpenSpec Requirement: Project-Relative Path Resolution 테스트"""
    
    def setup_method(self):
        """각 테스트 전 다중 디렉토리 구조 설정"""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir) / "deep" / "project" / "structure"
        self.project_root.mkdir(parents=True)
        
        # .opencode 디렉토리 생성
        self.opencode_dir = self.project_root / ".opencode"
        self.sessions_dir = self.opencode_dir / "sessions"
        self.opencode_dir.mkdir()
        self.sessions_dir.mkdir()
        
        # 하위 디렉토리에서 작업 시작
        self.work_dir = self.project_root / "subdir"
        self.work_dir.mkdir()
        self.original_cwd = os.getcwd()
        os.chdir(self.work_dir)
    
    def teardown_method(self):
        """각 테스트 후 정리"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.temp_dir)
    
    def test_project_root_detection(self):
        """
        OpenSpec Scenario: Project Root Detection
        
        Given SimpleTodoManager is initialized in any project directory
        When the tool needs to create session directories
        Then it shall detect the project root by looking for ".opencode" directory
        And shall create session directories under "{project_root}/.opencode/sessions/"
        """
        # Arrange
        session_id = "test-session"
        agent_name = "qa-agent"
        
        # Act - 하위 디렉토리에서 SimpleTodoManager 초기화
        todos = SimpleTodoManager(agent_name, session_id)
        
        # Assert - 프로젝트 루트가 올바르게 감지됨
        expected_base_dir = self.sessions_dir
        actual_base_dir = Path(todos.todo_dir).parent
        
        assert actual_base_dir == expected_base_dir
        
        # Assert - 세션 디렉토리가 프로젝트 루트 기반으로 생성됨
        expected_session_dir = expected_base_dir / f"{agent_name}_{session_id}"
        assert expected_session_dir.exists()
    
    def test_path_security_validation(self):
        """
        OpenSpec Scenario: Path Security
        
        Given a malicious agent provides "../evil" as session ID
        When the tool constructs the session directory path
        Then it shall validate and sanitize the path components
        And shall restrict directory creation to the project root boundary
        """
        # Arrange
        malicious_session_id = "../evil"
        agent_name = "malicious-agent"
        
        # Act - 악의적인 세션 ID로 SimpleTodoManager 초기화
        todos = SimpleTodoManager(agent_name, malicious_session_id)
        
        # Assert - 경로가 정제되어 프로젝트 루트 내에 존재
        todo_dir_path = Path(todos.todo_dir)
        
        # 프로젝트 루트 외부로 탈출하지 않음
        try:
            todo_dir_path.relative_to(self.sessions_dir)
            path_is_safe = True
        except ValueError:
            path_is_safe = False
        
        assert path_is_safe, "Path should be contained within project sessions directory"
        
        # Assert - 세션 ID에서 위험한 문자가 정제됨
        assert ".." not in str(todos.todo_dir)


class TestHybridArchitectureIntegration:
    """OpenSpec Requirement: Hybrid Architecture Integration 테스트"""
    
    def setup_method(self):
        """각 테스트 전 CLI 환경 설정"""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir)
        self.opencode_dir = self.project_root / ".opencode"
        self.sessions_dir = self.opencode_dir / "sessions"
        self.opencode_dir.mkdir()
        self.sessions_dir.mkdir()
        
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)
    
    def teardown_method(self):
        """각 테스트 후 정리"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.temp_dir)
    
    @patch('sys.argv')
    def test_python_core_cli_interface(self, mock_argv):
        """
        OpenSpec Scenario: Python Core CLI Interface
        
        Given the Python core is called with CLI arguments
        When it receives "--agent senior-sw-engineer --session abc123 --action add"
        Then it shall parse the arguments and execute the requested operation
        And shall return results in machine-readable format
        """
        # Arrange
        mock_argv.extend([
            "simple-todo.py",
            "--agent", "senior-sw-engineer",
            "--session", "abc123",
            "--action", "add",
            "--content", "API 엔드포인트 구현",
            "--priority", "high"
        ])
        
        # Act - CLI 인터페이스 테스트 (실제 구현에서는 main() 함수 호출)
        # 여기서는 CLI 파싱 로직만 테스트
        import argparse
        
        parser = argparse.ArgumentParser(description="Session-aware Simple Todo Manager")
        parser.add_argument("--agent", required=True, help="Agent name")
        parser.add_argument("--session", required=True, help="OpenCode Session ID")
        parser.add_argument("--action", required=True, choices=["add", "update", "list", "info"], help="Action")
        parser.add_argument("--content", help="Todo content")
        parser.add_argument("--priority", default="medium", choices=["high", "medium", "low"], help="Priority")
        
        args = parser.parse_args()
        
        # Assert - CLI 인자가 올바르게 파싱됨
        assert args.agent == "senior-sw-engineer"
        assert args.session == "abc123"
        assert args.action == "add"
        assert args.content == "API 엔드포인트 구현"
        assert args.priority == "high"
        
        # Assert - 파싱된 인자로 SimpleTodoManager 생성 가능
        todos = SimpleTodoManager(args.agent, args.session)
        assert todos.agent_name == args.agent
        assert todos.session_id == args.session


class TestBackwardCompatibility:
    """OpenSpec Requirement: Backward Compatibility 테스트"""
    
    def setup_method(self):
        """각 테스트 전 호환성 환경 설정"""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir)
        self.opencode_dir = self.project_root / ".opencode"
        self.sessions_dir = self.opencode_dir / "sessions"
        self.opencode_dir.mkdir()
        self.sessions_dir.mkdir()
        
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)
    
    def teardown_method(self):
        """각 테스트 후 정리"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.temp_dir)
    
    def test_direct_python_usage_compatibility(self):
        """
        OpenSpec Scenario: Direct Python Usage
        
        Given existing code uses `SimpleTodoManager("agent", "session-id")`
        When the code is executed after the enhancement
        Then it shall continue to work without modification
        And shall produce the same results as before
        """
        # Arrange - 기존 방식대로 직접 사용
        agent_name = "senior-sw-engineer"
        session_id = "legacy-session"
        
        # Act - 기존 인터페이스 그대로 사용
        todos = SimpleTodoManager(agent_name, session_id)
        task_id = todos.add_todo("레거시 호환성 테스트", "high")
        todos.update_status(task_id, "in_progress")
        
        # Assert - 기존 기능 그대로 동작
        assert todos.agent_name == agent_name
        assert todos.session_id == session_id
        
        all_todos = todos.load_todos()
        assert len(all_todos) == 1
        assert all_todos[0]["content"] == "레거시 호환성 테스트"
        assert all_todos[0]["status"] == "in_progress"
        assert all_todos[0]["priority"] == "high"


class TestStorageLocationMigration:
    """OpenSpec REMOVED Requirements 테스트 - 기존 /tmp 경로 사용 안 함"""
    
    def setup_method(self):
        """각 테스트 전 마이그레이션 환경 설정"""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir)
        self.opencode_dir = self.project_root / ".opencode"
        self.sessions_dir = self.opencode_dir / "sessions"
        self.opencode_dir.mkdir()
        self.sessions_dir.mkdir()
        
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)
    
    def teardown_method(self):
        """각 테스트 후 정리"""
        os.chdir(self.original_cwd)
        shutil.rmtree(self.temp_dir)
    
    def test_no_tmp_directory_usage(self):
        """
        OpenSpec Scenario: Storage Location Migration (Reverse)
        
        Given the new system is implemented
        When SimpleTodoManager creates session directories
        Then session data shall NOT be stored in /tmp/agent_sessions/
        And shall be stored in project-relative locations
        """
        # Arrange
        session_id = "no-tmp-test"
        agent_name = "test-agent"
        
        # Act - 새로운 SimpleTodoManager 사용
        todos = SimpleTodoManager(agent_name, session_id)
        
        # Assert - /tmp 경로를 사용하지 않음
        assert "/tmp/" not in todos.todo_dir
        assert "agent_sessions" not in todos.todo_dir
        
        # Assert - 프로젝트 상대 경로를 사용함
        assert str(self.project_root) in todos.todo_dir
        assert ".opencode/sessions" in todos.todo_dir


# 테스트 실행을 위한 엔트리 포인트
if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])