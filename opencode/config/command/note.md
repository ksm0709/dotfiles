---
description: Create a new memory/note
agent: general
---
# Create Note Command

User invoked: /note $ARGUMENTS

This command allows the user to quickly save information to the Zettelkasten memory system.

## Workflow

1.  **Parse Arguments**:
    - Input: "$ARGUMENTS"
    - Identify if a specific title is provided (e.g. quoted string at start) or if it should be inferred.
    - Extract the content body.

2.  **Locate Memory Store**:
    - Target directory: `$HOME/.config/opencode/notes`

3.  **Process Content**:
    - **Search**: Check for existing related notes to link to or update.
    - **Format**: Ensure content is in Markdown.
    - **Metadata**: Add relevant tags (e.g., `#user-input`, `#memory`).

4.  **Write File**:
    - Filename: `kebab-case-title.md` (e.g., `project-setup.md`).
    - Content:
      ```markdown
      # [Title]
      
      [Content]
      
      Tags: #tag
      Links: [[RelatedNote]]
      ```

5.  **Output**:
    - Confirm the note creation/update to the user.
