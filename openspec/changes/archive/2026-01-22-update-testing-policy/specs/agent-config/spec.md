## ADDED Requirements
### Requirement: Isolated Testing Environment
모든 테스트 코드는 프로젝트 소스 코드가 있는 디렉토리가 아닌, 격리된 시스템 임시 디렉토리에서 파일 시스템 조작을 수행해야 합니다.

#### Scenario: File system test execution
- **WHEN** an agent writes a test that involves file creation or modification
- **THEN** the test MUST create a temporary directory (e.g., using `tempfile.TemporaryDirectory`)
- **AND** all file operations MUST be performed within that temporary directory
- **AND** the temporary directory MUST be cleaned up after the test finishes
