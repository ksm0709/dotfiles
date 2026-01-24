# Session Todo Manager Integration

## Purpose
Integrate OpenCode Context API with SimpleTodoManager to ensure session ID persistence across multiple subagent calls within the same session.

## Background
Subagents currently lose session context when calling SimpleTodoManager multiple times within the same OpenCode session. Each call creates a new session ID instead of maintaining the existing one.

## Current State
- SimpleTodoManager located at `~/.config/opencode/tools/simple-todo.py`
- Subagents call SimpleTodoManager directly
- Session ID generated randomly on each instantiation
- No session persistence across multiple calls

## Desired State
- Session ID extracted from OpenCode Context API
- Session persistence guaranteed across multiple calls
- Hybrid TypeScript/Python architecture for optimal integration
- Configuration stored in project directory (not hardcoded paths)

## Scope
- Create TypeScript wrapper for OpenCode Context API integration
- Enhance Python SimpleTodoManager with session awareness
- Ensure backward compatibility with existing subagent usage
- Store session data in project-relative paths

## Constraints
- Must avoid hardcoded absolute paths (e.g., `/home/taeho/`)
- Must maintain existing Python SimpleTodoManager functionality
- Must integrate with OpenCode Context API for session ID extraction
- Must be backward compatible with current subagent implementations