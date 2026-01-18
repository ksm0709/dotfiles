# Environment Management Capability Spec

## ADDED Requirements

### Requirement: Multi-source Environment Variable Loading
The system must load API keys from multiple sources in a defined priority order.

#### Scenario: API key loading from multiple sources
When the deep-research skill starts, it should:
1. Check for directly set environment variables
2. Load from ~/.local/bin/env script if available
3. Extract export statements from ~/.bashrc
4. Read from ~/.config/opencode/.env file
5. Use mock mode as final fallback

#### Scenario: Priority-based key selection
When multiple API keys are available, the system should:
1. Prefer GEMINI_API_KEY over OPENAI_API_KEY
2. Respect explicit LLM_PROVIDER setting
3. Fall back to available keys automatically
4. Log the selected provider with masked key preview

### Requirement: Environment Variable Validation
The system must validate loaded API keys before use.

#### Scenario: API key format validation
When API keys are loaded, the system should:
1. Check for non-empty string values
2. Validate minimum key length requirements
3. Verify key format patterns (e.g., starts with proper prefix)
4. Report invalid keys with specific error messages

#### Scenario: API connectivity testing
When API keys are validated, the system should:
1. Make minimal test API calls to verify connectivity
2. Handle network timeouts gracefully
3. Report authentication failures clearly
4. Cache validation results to avoid repeated calls

## MODIFIED Requirements

### Requirement: Error Handling and User Feedback
Enhance error messages to provide actionable guidance.

#### Scenario: Missing API key guidance
When no API keys are found, the system should:
1. Display clear warning message
2. Show exact commands to set up API keys
3. Provide links to documentation
4. Continue in mock mode with explicit notification

#### Scenario: Invalid API key handling
When API keys are invalid, the system should:
1. Mask the key in error messages for security
2. Suggest key regeneration steps
3. Check for common formatting issues
4. Offer to try alternative providers

### Requirement: Dependency Verification
Add automatic dependency checking before execution.

#### Scenario: Python package verification
Before starting research, the system should:
1. Check for required packages (duckduckgo-search, beautifulsoup4, etc.)
2. Verify package versions meet minimum requirements
3. Suggest pip install commands for missing packages
4. Continue with limited functionality if possible

#### Scenario: Optional provider handling
When provider-specific packages are missing, the system should:
1. Detect missing google-generativeai or openai packages
2. Automatically disable unavailable providers
3. Fall back to available providers or mock mode
4. Inform user about disabled capabilities