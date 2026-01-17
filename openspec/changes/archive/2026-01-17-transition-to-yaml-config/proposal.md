# Proposal: Transition Vibe-Kanban to YAML Configuration

## Purpose
As previously requested, the configuration for `vibe-kanban` should be moved from a Shell script (`.sh`) to a YAML (`.yaml`) format. This provides a more standard configuration structure and prepares for potential future complexity.

## Scope
- Replace `.sh` configuration with `.yaml`.
- Update installation scripts to deploy the YAML template.
- Update execution scripts to parse YAML using Python.
- Ensure compatibility with the existing systemd service.

## ADDED Requirements
- New configuration file: `~/.config/vibe-kanban.yaml`.
- YAML parsing logic in `vibe-kanban-wrapper.sh` and `vk-runner.sh` using Python3.

## MODIFIED Requirements
- `install.sh`: Should now create `vibe-kanban.yaml` instead of `vibe-kanban.sh`.
- `config-template.sh`: Rename to `config-template.yaml` and update content to YAML format.
- `vibe-kanban-wrapper.sh` & `vk-runner.sh`: Remove `source` command and replace with YAML parsing logic.

## REMOVED Requirements
- Support for `~/.config/vibe-kanban.sh` (deprecated).

## Scenarios
#### Scenario: Successful Configuration Loading
- Given a valid `~/.config/vibe-kanban.yaml` with `host: 127.0.0.1` and `port: 8080`
- When `vibe-kanban` or the service is started
- Then the application should run on `127.0.0.1:8080`.

#### Scenario: Default Values
- Given a missing or empty `~/.config/vibe-kanban.yaml`
- When `vibe-kanban` or the service is started
- Then the application should fall back to default values (`0.0.0.0:54545`).
