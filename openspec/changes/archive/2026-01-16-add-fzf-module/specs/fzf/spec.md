## ADDED Requirements

### Requirement: fzf Installation
The system SHALL provide a way to install `fzf` fuzzy finder.

#### Scenario: Successful installation
- **WHEN** `./setup.sh fzf` is executed
- **THEN** `fzf` binary should be installed and accessible in the PATH

### Requirement: Shell Integration
The system SHALL configure `fzf` shell integrations (key bindings and fuzzy completion).

#### Scenario: Key bindings available
- **WHEN** the shell is restarted after setup
- **THEN** `fzf` key bindings (like CTRL-R) should be active
