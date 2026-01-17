# Proposal: Cleanup Vibe-Kanban Scripts

## Purpose
The `vibe-kanban` directory contains several experimental and redundant scripts created during the development of the installation process. This proposal aims to clean up these files to maintain a clean codebase and clarify the official installation method.

## Scope
- Identify and remove redundant/experimental scripts.
- Ensure the core installation scripts are preserved and functional.
- Document the current installation context for future tasks.

## ADDED Requirements
- None

## MODIFIED Requirements
- None

## REMOVED Requirements
- Remove `vk-direct.sh` (experimental)
- Remove `vk-final.sh` (experimental)
- Remove `vk-binary.sh` (experimental)
- Remove `vibe-kanban-service.sh` (experimental)

## Scenarios
#### Scenario: Clean directory
- Given the `vibe-kanban` directory
- When the cleanup is performed
- Then `setup.sh`, `install.sh`, `install-service.sh`, `vibe-kanban-wrapper.sh`, `vk-runner.sh`, and `config-template.sh` should remain.
