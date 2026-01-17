# Design: YAML Configuration for Vibe-Kanban

## Architecture
The configuration will move from a sourceable shell script to a structured YAML file. Since the primary execution environment is Bash, we need a reliable way to bridge YAML to environment variables.

## Implementation Details

### 1. YAML Structure
```yaml
# vibe-kanban configuration
host: 0.0.0.0
port: 54545
```

### 2. Parsing Strategy
We will use a Python3 one-liner to parse the YAML and output shell-compatible export commands. This is more robust than `sed` or `awk` for YAML.

Example parsing logic:
```bash
eval $(python3 -c "
import yaml, sys
try:
    with open('$CONFIG_FILE', 'r') as f:
        cfg = yaml.safe_load(f)
        if cfg:
            for k, v in cfg.items():
                print(f'export VK_{k.upper()}=\"{v}\"')
except Exception:
    pass
")
```

### 3. Systemd Service Integration
The systemd service currently uses `EnvironmentFile`. Since systemd cannot parse YAML directly, we will:
1. Remove `EnvironmentFile` from the `.service` file.
2. Rely on `vk-runner.sh` to parse the YAML and export the variables before executing the binary.
3. This keeps the service definition simple and the logic centralized in the runner.

## Trade-offs
- **Dependency**: Adds a runtime dependency on `python3` and `PyYAML`. (Verified: both are present in the environment).
- **Performance**: Negligible overhead for parsing a small YAML file on startup.
