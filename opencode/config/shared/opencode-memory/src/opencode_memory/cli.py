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
    task = args.task or args.task_positional
    if not task:
        print(json.dumps({"error": "Task description required"}, indent=2), file=sys.stderr)
        sys.exit(1)

    result = cm.start(task)

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
    result_summary = args.result or args.result_positional or ""
    result = cm.end(result_summary)

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


def cmd_record_intent(args):
    """Record intent."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        # Ensure session_id is set in ContextManager if provided in payload
        if "session_id" in payload:
            cm.session_id = payload["session_id"]

        result = cm.record_intent(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_record_decision(args):
    """Record decision."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        if "session_id" in payload:
            cm.session_id = payload["session_id"]

        result = cm.record_decision(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_record_learning(args):
    """Record learning."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        if "session_id" in payload:
            cm.session_id = payload["session_id"]

        result = cm.record_learning(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_record_semantic(args):
    """Record semantic memory."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        if "session_id" in payload:
            cm.session_id = payload["session_id"]

        result = cm.record_semantic(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


# ═══════════════════════════════════════════════════════════════
# Phase 2: Problem Tracking Commands (Task 2.4, 2.5)
# ═══════════════════════════════════════════════════════════════


def cmd_problem_start(args):
    """Start problem tracking."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        result = cm.problem_start(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_problem_attempt(args):
    """Record problem attempt."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        result = cm.problem_attempt(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_problem_resolve(args):
    """Resolve problem."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        result = cm.problem_resolve(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


# ═══════════════════════════════════════════════════════════════
# Phase 3: Episode Management Commands (Task 3.4)
# ═══════════════════════════════════════════════════════════════


def cmd_episode_start(args):
    """Start episode."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        result = cm.episode_start(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_episode_complete(args):
    """Complete episode."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    try:
        payload = json.loads(args.payload)
        result = cm.episode_complete(payload)
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except json.JSONDecodeError:
        print(json.dumps({"error": "Invalid JSON payload"}, indent=2), file=sys.stderr)
        sys.exit(1)


def cmd_episode_active(args):
    """Get active episode."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    result = cm.episode_active(args.session_id)
    print(json.dumps(result, indent=2, ensure_ascii=False))


def cmd_episode_get(args):
    """Get episode details."""
    from opencode_memory import ContextManager, load_config

    config = load_config()
    cm = ContextManager(config)

    result = cm.episode_get(args.episode_id)
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
    p_start.add_argument("--task", help="Task description")
    p_start.add_argument("task_positional", nargs="?", help="Task description (positional)")

    # checkpoint
    p_checkpoint = subparsers.add_parser("checkpoint", help="Create checkpoint")
    p_checkpoint.add_argument("--summary", "-s", help="External summary")

    # end
    p_end = subparsers.add_parser("end", help="End task")
    p_end.add_argument("--result", help="Task result summary")
    p_end.add_argument("result_positional", nargs="?", help="Task result summary (positional)")

    # query
    p_query = subparsers.add_parser("query", help="Search memories")
    p_query.add_argument("query", help="Search query")
    p_query.add_argument("--limit", "-l", type=int, default=5, help="Result limit")

    # add
    p_add = subparsers.add_parser("add", help="Add memory")
    p_add.add_argument("content", help="Memory content")
    p_add.add_argument("--tags", "-t", help="Comma-separated tags")

    # record-intent
    p_intent = subparsers.add_parser("record-intent", help="Record intent")
    p_intent.add_argument("payload", help="JSON payload")

    # record-decision
    p_decision = subparsers.add_parser("record-decision", help="Record decision")
    p_decision.add_argument("payload", help="JSON payload")

    # record-learning
    p_learning = subparsers.add_parser("record-learning", help="Record learning")
    p_learning.add_argument("payload", help="JSON payload")

    # record-semantic
    p_semantic = subparsers.add_parser("record-semantic", help="Record semantic memory")
    p_semantic.add_argument("payload", help="JSON payload")

    # problem-start (Phase 2)
    p_problem_start = subparsers.add_parser("problem-start", help="Start problem tracking")
    p_problem_start.add_argument("payload", help="JSON payload")

    # problem-attempt (Phase 2)
    p_problem_attempt = subparsers.add_parser("problem-attempt", help="Record problem attempt")
    p_problem_attempt.add_argument("payload", help="JSON payload")

    # problem-resolve (Phase 2)
    p_problem_resolve = subparsers.add_parser("problem-resolve", help="Resolve problem")
    p_problem_resolve.add_argument("payload", help="JSON payload")

    # episode-start (Phase 3)
    p_episode_start = subparsers.add_parser("episode-start", help="Start episode")
    p_episode_start.add_argument("payload", help="JSON payload")

    # episode-complete (Phase 3)
    p_episode_complete = subparsers.add_parser("episode-complete", help="Complete episode")
    p_episode_complete.add_argument("payload", help="JSON payload")

    # episode-active (Phase 3)
    p_episode_active = subparsers.add_parser("episode-active", help="Get active episode")
    p_episode_active.add_argument("session_id", help="Session ID")

    # episode-get (Phase 3)
    p_episode_get = subparsers.add_parser("episode-get", help="Get episode details")
    p_episode_get.add_argument("episode_id", help="Episode ID")

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
        "record-intent": cmd_record_intent,
        "record-decision": cmd_record_decision,
        "record-learning": cmd_record_learning,
        "record-semantic": cmd_record_semantic,
        "problem-start": cmd_problem_start,
        "problem-attempt": cmd_problem_attempt,
        "problem-resolve": cmd_problem_resolve,
        "episode-start": cmd_episode_start,
        "episode-complete": cmd_episode_complete,
        "episode-active": cmd_episode_active,
        "episode-get": cmd_episode_get,
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
