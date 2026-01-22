## MODIFIED Requirements
### Requirement: Isolated Testing Environment
모든 테스트 코드는 프로젝트 소스 코드가 있는 디렉토리가 아닌, 격리된 시스템 임시 디렉토리에서 파일 시스템 조작을 수행해야 합니다.

#### Scenario: File system test execution
- **WHEN** an agent writes a test that involves file creation or modification
- **THEN** the test MUST create a temporary directory (e.g., using `tempfile.TemporaryDirectory`)
- **AND** all file operations MUST be performed within that temporary directory
- **AND** the temporary directory MUST be cleaned up after the test finishes

## ADDED Requirements
### Requirement: Global Todo Tool Access
모든 에이전트는 작업 관리 및 투명성 확보를 위해 `todowrite` 및 `todoread` 도구에 대한 접근 권한을 가져야 합니다.

#### Scenario: Agent tool availability
- **WHEN** an agent is initialized
- **THEN** it SHOULD have `todowrite` and `todoread` tools enabled in its configuration
- **AND** these tools MUST be listed as standard global tools in the main configuration
