## [2026-02-19 21:04:55] Task 2: SKILL.md
- Created SKILL.md for command-hooks following the conversational pattern from command-creator.
- Included interactive workflow using the question tool for step-by-step configuration.
- Defined 7 template variables: {id}, {agent}, {tool}, {cmd}, {stdout}, {stderr}, {exitCode}.
- Linked to reference files: hook-types.md, examples.md, troubleshooting.md.
- Ensured non-blocking nature and configuration methods (JSONC/Markdown) are highlighted.
## Hook Types Reference Created
- Created authoritative schema reference for command hooks v0.3.0.
- Documented Tool Hook, Session Hook, Markdown Frontmatter, Toast Config, Template Variables, Global Config, and Precedence Rules.
- Ensured all 7 template variables are included.
- Verified file structure and content against requirements.
## Command Hooks Examples Creation
- Created a comprehensive examples file for the command-hooks plugin.
- Ensured correct schema fields: 'tool', 'session', 'callingAgent', 'toolArgs'.
- Demonstrated both JSONC and YAML frontmatter formats.
- Included practical use cases like validation gates, QA, and context injection.
- Technical documentation for plugins should maintain language consistency with existing files (English in this case).
- Using OPENCODE_HOOKS_DEBUG=true is the primary way to debug hook matching logic.
- Sequential execution in run arrays means the last command's output is what gets captured for templates.
