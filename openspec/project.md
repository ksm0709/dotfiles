# Project Context

## Purpose
Automate the installation and configuration of personal development environments and tools (dotfiles). The project provides a modular way to set up various tools like LazyVim, Tmux, WezTerm, and custom CLI tools.

## Tech Stack
- **Shell**: Bash (primary for setup scripts)
- **Editor**: LazyVim (Lua-based configuration)
- **Terminal/Multiplexer**: Tmux, WezTerm
- **Custom Tools**: Python/Node.js based CLI tools (gemini-cli, opencode, etc.)
- **OS**: Linux (Ubuntu/Debian preferred based on `apt-get` usage)

## Project Conventions

### Code Style
- **Bash**: Use color-coded output (RED, GREEN, ORANGE, BLUE).
- **Modularity**: Each tool/environment must reside in its own directory with a `setup.sh` script.
- **Idempotency**: Setup scripts should ideally be idempotent (safe to run multiple times).

### Architecture Patterns
- **Modular Setup**: The root `setup.sh` acts as an orchestrator, delegating tasks to sub-directory `setup.sh` scripts.
- **Configuration Management**: Symlinking or copying configuration files to their respective home directory locations.

### Testing Strategy
- **Manual Verification**: Currently relies on manual verification after setup.
- **Spec-Driven Testing**: Moving towards OpenSpec-based verification for new features.

### Git Workflow
- **Feature-based Changes**: Use OpenSpec proposals for adding new tools or significant configuration changes.
- **Commit Messages**: Concise, verb-led messages.

## Domain Context
- Dotfiles management for a developer workflow.
- Integration with AI tools (gemini-cli, opencode).

## Important Constraints
- Must be compatible with Linux environments (specifically Ubuntu/Debian).
- Should handle sudo privileges where necessary.

## External Dependencies
- `git`, `curl`, `wget` (base requirements)
- Tool-specific dependencies (e.g., `npm`, `python3`, `cargo` depending on the module)
