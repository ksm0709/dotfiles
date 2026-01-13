# ByteRover Setup

A minimal installation and setup directory for ByteRover, a context engineering platform for AI coding agents.

## Quick Start

```bash
# Install dependencies and byterover-cli
./setup.sh
```

## Scripts

- **`setup.sh`** - Installs byterover-cli and system dependencies

## Requirements

- Node.js 20 or higher
- npm
- Linux: libsecret-1-dev (for credential storage)

## Essential Commands

Once in the ByteRover REPL (run `brv`), use these slash commands:

```bash
/login      # Authenticate with your ByteRover account
/init       # Initialize team/space/agent configuration
/curate     # Add context from files or directories
/query      # Retrieve and analyze context
/push       # Push context to remote storage
/pull       # Pull context from remote storage
```

## File Structure

ByteRover creates a `.brv/` directory in your workspace:
- `.brv/config.json` - Configuration settings
- `.brv/context-tree/` - Local context storage

## Documentation

For full documentation, visit: https://docs.byterover.dev/
