"""
Configuration Loader

Loads YAML configuration files and environment variables.
"""

import os
import re
from pathlib import Path
from typing import Any, Dict, cast

import yaml

# Default configuration (used when config.yaml doesn't exist)
DEFAULT_CONFIG = {
    "tier": "hybrid",
    "embeddings": {
        "provider": "synthetic",
        "fallback": "synthetic",
    },
    "llm": {
        "enabled": False,
        "provider": "gemini",
        "model": "gemini-1.5-flash",
        "temperature": 0.3,
        "max_tokens": 1024,
        "enable_reflection": True,
        "enable_boundary_detection": True,
    },
    "database": {
        "backend": "sqlite",
        "path": "data/memory/context.sqlite",
    },
    "context": {
        "search": {
            "min_score": 0.7,
            "default_limit": 10,
            "keyword_boost": 2.5,
        },
        "decay": {
            "enabled": True,
            "lambda": 0.02,
            "reinforce_on_query": True,
        },
    },
    "working": {
        "auto_checkpoint_interval": 15,
        "max_items": 100,
        "persist": True,
        "persist_path": "data/memory/working_memory.json",
        "summary": {
            "priority_weights": {
                "error_fix": 10,
                "decision": 8,
                "change": 5,
                "read": 1,
            }
        },
    },
    "logging": {
        "level": "INFO",
    },
}


def get_project_root(start_path: Path | None = None) -> Path:
    """
    Return project root directory.

    Finds directory containing AGENTS.md, .opencode, or .git.
    """
    if start_path is None:
        start_path = Path.cwd()

    current = start_path.resolve()

    # Traverse up to find project root
    for _ in range(10):  # Max 10 levels
        if (current / "AGENTS.md").exists():
            return current
        if (current / ".opencode").exists():
            return current
        if (current / ".git").exists():
            return current
        if (current / "shared" / "context" / "config.yaml").exists():
            return current
        if current.parent == current:
            break
        current = current.parent

    # Return current directory if not found
    return start_path


def _expand_env_vars(value: str, project_root: Path) -> str:
    """Expand environment variables and ${PROJECT_ROOT}."""
    # Replace ${PROJECT_ROOT}
    value = value.replace("${PROJECT_ROOT}", str(project_root))

    # Replace ${VAR_NAME} format environment variables
    pattern = r"\$\{([^}]+)\}"

    def replace_env(match: Any) -> str:
        var_name = match.group(1)
        return os.environ.get(var_name, match.group(0))

    return re.sub(pattern, replace_env, value)


def _process_config(config: dict, project_root: Path) -> dict:
    """Recursively expand environment variables in config values."""
    result: Dict[str, Any] = {}
    for key, value in config.items():
        if isinstance(value, dict):
            result[key] = _process_config(value, project_root)
        elif isinstance(value, list):
            result[key] = [
                _expand_env_vars(v, project_root) if isinstance(v, str) else v
                for v in value
            ]
        elif isinstance(value, str):
            result[key] = _expand_env_vars(value, project_root)
        else:
            result[key] = value
    return result


def _deep_merge(base: dict, override: dict) -> dict:
    """Deep merge two dictionaries."""
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(
    config_path: Path | str | None = None,
    project_root: Path | None = None,
) -> dict[str, Any]:
    """
    Load configuration file.

    Args:
        config_path: Config file path. If None, loads from default location
        project_root: Project root. If None, auto-detected

    Returns:
        dict: Configuration dictionary
    """
    if project_root is None:
        project_root = get_project_root()

    # Determine config file path
    if config_path is None:
        # Priority: .opencode/shared/context/config.yaml > default config
        candidates = [
            project_root / ".opencode" / "shared" / "context" / "config.yaml",
            project_root / "shared" / "context" / "config.yaml",
            project_root / "config.yaml",
        ]
        config_path = None
        for candidate in candidates:
            if candidate.exists():
                config_path = candidate
                break
    else:
        config_path = Path(config_path)

    # Copy default config
    config = _deep_merge({}, DEFAULT_CONFIG)

    # Set default paths relative to project root
    config["database"]["path"] = str(
        project_root / "data" / "memory" / "context.sqlite"
    )
    config["working"]["persist_path"] = str(
        project_root / "data" / "memory" / "working_memory.json"
    )

    # Load config file if exists
    if config_path and config_path.exists():
        with open(config_path, "r", encoding="utf-8") as f:
            file_config = yaml.safe_load(f) or {}
        config = _deep_merge(config, file_config)

    # Expand environment variables
    config = _process_config(config, project_root)

    # Validate and fix min_score from config file
    min_score = config.get("context", {}).get("search", {}).get("min_score")
    if min_score is not None:
        if not isinstance(min_score, (int, float)) or not (0.0 <= min_score <= 1.0):
            # Invalid min_score, reset to default
            default_score = cast(Dict[str, Any], DEFAULT_CONFIG["context"])["search"][
                "min_score"
            ]
            config["context"]["search"]["min_score"] = default_score

    # Override with environment variables
    if os.environ.get("CONTEXT_TIER"):
        config["tier"] = os.environ["CONTEXT_TIER"]
    if os.environ.get("CONTEXT_EMBEDDINGS_PROVIDER"):
        config["embeddings"]["provider"] = os.environ["CONTEXT_EMBEDDINGS_PROVIDER"]

    # LLM Environment Variables
    if os.environ.get("LLM_ENABLED"):
        config["llm"]["enabled"] = os.environ["LLM_ENABLED"].lower() == "true"
    if os.environ.get("LLM_PROVIDER"):
        config["llm"]["provider"] = os.environ["LLM_PROVIDER"]
    if os.environ.get("LLM_MODEL"):
        config["llm"]["model"] = os.environ["LLM_MODEL"]
    if os.environ.get("LLM_TEMPERATURE"):
        try:
            config["llm"]["temperature"] = float(os.environ["LLM_TEMPERATURE"])
        except ValueError:
            pass
    if os.environ.get("LLM_MAX_TOKENS"):
        try:
            config["llm"]["max_tokens"] = int(os.environ["LLM_MAX_TOKENS"])
        except ValueError:
            pass
    if os.environ.get("LLM_ENABLE_REFLECTION"):
        config["llm"]["enable_reflection"] = (
            os.environ["LLM_ENABLE_REFLECTION"].lower() == "true"
        )
    if os.environ.get("LLM_ENABLE_BOUNDARY_DETECTION"):
        config["llm"]["enable_boundary_detection"] = (
            os.environ["LLM_ENABLE_BOUNDARY_DETECTION"].lower() == "true"
        )

    if os.environ.get("CONTEXT_LOG_LEVEL"):
        config["logging"]["level"] = os.environ["CONTEXT_LOG_LEVEL"]
    if os.environ.get("CONTEXT_SEARCH_MIN_SCORE"):
        try:
            score = float(os.environ["CONTEXT_SEARCH_MIN_SCORE"])
            if 0.0 <= score <= 1.0:
                config["context"]["search"]["min_score"] = score
            else:
                # Out of range, use default
                pass
        except ValueError:
            # Invalid format, use default
            pass

    return config


def validate_config(config: dict) -> tuple[bool, list[str]]:
    """
    Validate configuration.

    Returns:
        tuple: (valid, errors)
    """
    errors = []

    # Validate tier
    valid_tiers = {"hybrid", "fast", "smart", "deep"}
    if config.get("tier") not in valid_tiers:
        errors.append(
            f"Invalid tier: {config.get('tier')}. Must be one of {valid_tiers}"
        )

    # Validate embeddings provider
    valid_providers = {"synthetic", "ollama", "openai", "gemini"}
    provider = config.get("embeddings", {}).get("provider")
    if provider not in valid_providers:
        errors.append(
            f"Invalid embeddings provider: {provider}. Must be one of {valid_providers}"
        )

    # Validate database path
    db_path = config.get("database", {}).get("path")
    if db_path:
        db_dir = Path(db_path).parent
        if not db_dir.exists():
            try:
                db_dir.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                errors.append(f"Cannot create database directory: {e}")

    # Validate min_score range
    min_score = config.get("context", {}).get("search", {}).get("min_score")
    if min_score is not None:
        if not isinstance(min_score, (int, float)):
            errors.append(f"min_score must be a number, got {type(min_score).__name__}")
        elif not (0.0 <= min_score <= 1.0):
            errors.append(f"min_score must be between 0.0 and 1.0, got {min_score}")

    return len(errors) == 0, errors
