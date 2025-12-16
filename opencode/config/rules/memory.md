# AI Agent Memory Management (Zettelkasten)

This rule governs how the AI agent persists information using a Zettelkasten-like system.

## Storage Configuration
- **Location**: `$HOME/.config/opencode/notes`
- **Format**: Markdown (`.md`)
- **Compatibility**: Obsidian-friendly (Wikilinks `[[Link]]`, Tags `#tag`)

## Operational Guidelines

### 1. Automatic Management
- **Proactive Recording**: Automatically create or update notes whenever you encounter important project information (e.g., architecture, conventions, complex logic) or successfully complete a non-trivial task.
- **User Triggered**: Immediately add a note when the user explicitly requests to "take a note" or "remember" something.
- **Self-Correction**: Automatically edit or delete notes if you find information to be outdated or incorrect during your work.

### 2. Creation (Write)
- **Atomicity**: Create single-purpose notes. One idea per file.
- **Naming**: Use descriptive, unique filenames (kebab-case preferred, e.g., `project-setup-guidelines.md` or timestamped `YYYYMMDD-topic.md`).
- **Links**: Actively link to existing related notes using `[[Filename]]`.
- **Tags**: Use `#hashtags` for categorization.

### 3. Retrieval (Read)
- Before executing complex tasks, check the notes directory.
- Use `grep` or `glob` to find relevant keywords or tags.
- Follow links to traverse the knowledge graph.

### 4. Maintenance (Refactor)
- Update notes when information changes.
- Merge duplicate concepts into a single atomic note.

### 5. Commands
- **`/note [title] <content>`**: Create a new note. If the title is omitted, infer a descriptive one from the content.
- **`/remember <content>`**: Alias for `/note`.
