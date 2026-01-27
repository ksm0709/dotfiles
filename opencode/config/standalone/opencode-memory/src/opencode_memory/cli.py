#!/usr/bin/env python3
"""
OpenCode Memory CLI

Command-line interface for the installed opencode-memory package.
"""

import argparse
import json
import sys


def cmd_status(args):
    """Print current status."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    status = {
        "working": cm.working.get_summary(),
        "context": cm.context.get_summary(),
    }

    print(json.dumps(status, indent=2, ensure_ascii=False, default=str))


def cmd_init(args):
    """Initialize session."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)
    result = cm.init(args.session_id)

    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_start(args):
    """Start task."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)
    result = cm.start(args.task)

    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_checkpoint(args):
    """Perform checkpoint."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)
    result = cm.checkpoint(args.summary or "")

    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_end(args):
    """End task."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)
    result = cm.end()

    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_query(args):
    """Search memories."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)
    results = cm.context.query(args.query, limit=args.limit)

    print(json.dumps(results, indent=2, ensure_ascii=False))


def cmd_add(args):
    """Add memory."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    tags = args.tags.split(",") if args.tags else None
    result = cm.context.add(args.content, tags=tags)

    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_version(args):
    """Print version."""
    from opencode_memory import __version__

    print(f"opencode-memory {__version__}")


def main():
    parser = argparse.ArgumentParser(
        prog="opencode-memory",
        description="2-layer memory system for OpenCode AI agents",
    )

    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # status
    subparsers.add_parser("status", help="Show current status")

    # init
    p_init = subparsers.add_parser("init", help="Initialize session")
    p_init.add_argument("session_id", help="Session ID")

    # start
    p_start = subparsers.add_parser("start", help="Start task")
    p_start.add_argument("task", help="Task description")

    # checkpoint
    p_checkpoint = subparsers.add_parser("checkpoint", help="Create checkpoint")
    p_checkpoint.add_argument("--summary", "-s", help="External summary")

    # end
    subparsers.add_parser("end", help="End task")

    # query
    p_query = subparsers.add_parser("query", help="Search memories")
    p_query.add_argument("query", help="Search query")
    p_query.add_argument("--limit", "-l", type=int, default=5, help="Result limit")

    # add
    p_add = subparsers.add_parser("add", help="Add memory")
    p_add.add_argument("content", help="Memory content")
    p_add.add_argument("--tags", "-t", help="Comma-separated tags")

    # version
    subparsers.add_parser("version", help="Show version")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(0)

    commands = {
        "status": cmd_status,
        "init": cmd_init,
        "start": cmd_start,
        "checkpoint": cmd_checkpoint,
        "end": cmd_end,
        "query": cmd_query,
        "add": cmd_add,
        "version": cmd_version,
    }

    if args.command in commands:
        try:
            commands[args.command](args)
        except Exception as e:
            print(json.dumps({"error": str(e)}, indent=2), file=sys.stderr)
            sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
