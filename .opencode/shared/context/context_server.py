#!/usr/bin/env python3
"""
Context Server - CLI interface for Context Manager Plugin

This script serves as the backend for the TypeScript plugin.
It provides a CLI interface to the ContextManager functionality.
"""

import argparse
import json
import sys
from typing import Optional


def get_manager():
    """Get or create ContextManager instance."""
    from opencode_memory import ContextManager, load_config
    config = load_config()
    return ContextManager(config)


def cmd_init(session_id: str) -> dict:
    """Initialize session."""
    cm = get_manager()
    return cm.init(session_id)


def cmd_start(task: str) -> dict:
    """Start task and retrieve related memories."""
    cm = get_manager()
    return cm.start(task)


def cmd_checkpoint(summary: str = "") -> dict:
    """Perform checkpoint."""
    cm = get_manager()
    return cm.checkpoint(summary)


def cmd_auto_checkpoint() -> dict:
    """Auto checkpoint (on session idle)."""
    cm = get_manager()
    return cm.auto_checkpoint()


def cmd_end(result: str = "") -> dict:
    """End task."""
    cm = get_manager()
    return cm.end(result)


def cmd_status() -> dict:
    """Get current status."""
    cm = get_manager()
    return cm.status()


def cmd_get_compaction_context() -> str:
    """Get context for compaction."""
    cm = get_manager()
    return cm.get_compaction_context()


def output_json(data):
    """Output data as JSON."""
    print(json.dumps(data, indent=2, ensure_ascii=False, default=str))


def output_text(text: str):
    """Output text directly."""
    print(text)


def main():
    parser = argparse.ArgumentParser(
        prog="context_server",
        description="Context Manager backend for plugin"
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Commands")
    
    # init
    p_init = subparsers.add_parser("init", help="Initialize session")
    p_init.add_argument("--session", required=True, help="Session ID")
    
    # start
    p_start = subparsers.add_parser("start", help="Start task")
    p_start.add_argument("--task", required=True, help="Task description")
    
    # checkpoint
    p_checkpoint = subparsers.add_parser("checkpoint", help="Perform checkpoint")
    p_checkpoint.add_argument("--summary", default="", help="Summary (optional)")
    
    # auto-checkpoint
    subparsers.add_parser("auto-checkpoint", help="Auto checkpoint on idle")
    
    # end
    p_end = subparsers.add_parser("end", help="End task")
    p_end.add_argument("--result", default="", help="Result summary (optional)")
    
    # status
    subparsers.add_parser("status", help="Get current status")
    
    # get-compaction-context
    subparsers.add_parser("get-compaction-context", help="Get context for compaction")
    
    args = parser.parse_args()
    
    if args.command is None:
        parser.print_help()
        sys.exit(1)
    
    try:
        if args.command == "init":
            result = cmd_init(args.session)
            output_json(result)
        elif args.command == "start":
            result = cmd_start(args.task)
            output_json(result)
        elif args.command == "checkpoint":
            result = cmd_checkpoint(args.summary)
            output_json(result)
        elif args.command == "auto-checkpoint":
            result = cmd_auto_checkpoint()
            output_json(result)
        elif args.command == "end":
            result = cmd_end(args.result)
            output_json(result)
        elif args.command == "status":
            result = cmd_status()
            output_json(result)
        elif args.command == "get-compaction-context":
            result = cmd_get_compaction_context()
            output_text(result)
        else:
            parser.print_help()
            sys.exit(1)
            
    except Exception as e:
        error_output = {"error": str(e), "type": type(e).__name__}
        print(json.dumps(error_output), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
