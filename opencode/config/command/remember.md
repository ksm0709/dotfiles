---
description: Alias for /note - Remember information
agent: general
---
# Remember Command

User invoked: /remember $ARGUMENTS

This command is an alias for `/note`. It saves information to the Zettelkasten memory system.

## Workflow

1.  **Delegate**:
    - Treat this exactly as a `/note` command.
    - Refer to the logic in `command/note.md` or simply proceed with note creation.

2.  **Process**:
    - Infer title from "$ARGUMENTS" if not provided.
    - Save to `$HOME/.config/opencode/notes`.
    - Use Zettelkasten formatting.
