# Troubleshooting Guide (v0.3.0)

This guide helps you diagnose and resolve common issues with the `opencode-command-hooks` plugin.

## Table of Contents
- [Section 1: Installation Issues](#section-1-installation-issues)
- [Section 2: Hook Not Triggering](#section-2-hook-not-triggering)
- [Section 3: Configuration Errors](#section-3-configuration-errors)
- [Section 4: Output Issues](#section-4-output-issues)
- [Section 5: Debugging Techniques](#section-5-debugging-techniques)
- [Section 6: Best Practices](#section-6-best-practices)

---

## Section 1: Installation Issues

If the plugin fails to load or initialize, check the following:

- **Plugin Registration**: Ensure that `"opencode-command-hooks"` is included in the `plugin` array of your `opencode.json` file.
- **Missing Dependencies**: If you are running from source, ensure all dependencies are installed by running `bun install`. Check your `package.json` for any missing peer dependencies.
- **Version Compatibility**: Verify that your plugin version (v0.3.0) is compatible with your current OpenCode version. Incompatible versions may lead to silent failures or missing features.

---

## Section 2: Hook Not Triggering

This is the most common issue. If your hook isn't firing, verify these matching rules:

- **Wrong Phase**: Ensure you are using the correct phase. `before` hooks fire *before* the tool executes, while `after` hooks fire *after* the tool completes.
- **Tool Name Mismatch**: Tool names must match exactly. For example, use `"write"` instead of `"file_write"`. Check the agent's logs to see the exact tool name being called.
- **`toolArgs` Matching**: Argument filters require an exact string match. Substring or regex matching is not supported.
- **`toolArgs` Array Behavior**: If a `toolArgs` filter value is an array, the actual argument must match **any one** of the elements in that array.
- **Session Event Names**: `session.start` is an alias for `session.created`. Both are valid, but ensure you aren't using unsupported event names.
- **Config File Scope**: Global JSONC hooks apply to all agents. Markdown frontmatter hooks apply **only** to the agent defined in that file.
- **`callingAgent` Mismatch**: The `callingAgent` field must match the agent's internal name (e.g., `engineer`), not its display name.
- **Wildcards**: Use `"*"` to match all tools or agents. If the `tool` field is omitted in a global hook, it defaults to matching all tools.

---

## Section 3: Configuration Errors

Malformed configuration will prevent hooks from loading correctly:

- **Duplicate IDs**: Defining the same `id` multiple times within the **same source** (e.g., two hooks with the same ID in `command-hooks.jsonc`) will cause an error. However, a markdown hook can safely override a global hook by using the same ID.
- **YAML Frontmatter Syntax**: Ensure your markdown frontmatter is valid YAML. Common pitfalls include incorrect indentation or missing quotes for strings containing special characters.
- **JSONC Parse Errors**: While JSONC allows comments and trailing commas, ensure the overall structure is valid. Check for missing braces or brackets.
- **Missing Required Fields**: 
  - **Global Hooks**: Require `id`, `when`, and `run`.
  - **Markdown Hooks**: Require `run` (IDs and triggers are often implicit).
- **Invalid Enum Values**: `phase` must be exactly `"before"` or `"after"`. `event` must be one of `"session.created"`, `"session.idle"`, `"session.end"`, or `"session.start"`.

---

## Section 4: Output Issues

If your hook runs but the output isn't what you expect:

- **`inject` Not Showing**: Verify that the `inject` template string is defined. Also, ensure the command actually produces output on `stdout`.
- **Toast Not Appearing**: The `toast.message` field is required for a notification to show.
- **Output Truncation**: By default, command output is limited to 30,000 characters. If your output is cut off, increase the `truncationLimit` in your global JSONC config.
- **Empty Template Variables**: If a variable like `{tool}` is empty, it may not be available for that hook type (e.g., `{tool}` is not available in session hooks).
- **Empty `{stdout}`/`{stderr}`**: This occurs if the command produced no output or if the command failed before it could write to the output streams.
- **Array `run` Behavior**: When `run` is an array of commands, the `inject` and `toast` templates only use the output from the **last** command in the sequence.

---

## Section 5: Debugging Techniques

Use these methods to gain visibility into hook execution:

- **Enable Debug Logging**: Set the environment variable `OPENCODE_HOOKS_DEBUG=true` before starting OpenCode. This enables verbose logging of hook matching and execution.
- **Check OpenCode Logs**: Look for log lines indicating whether a hook was matched or if there were errors loading the configuration.
- **Verify Config Loading**: The plugin logs a message when it successfully loads the global or markdown configuration.
- **Manual Testing**: Run your commands directly in the terminal first to ensure they work as expected before adding them to a hook.
- **Simple Verification**: Use a simple command like `echo "hook fired"` to verify that the hook trigger is working before implementing complex logic.
- **Config File Location**: The plugin searches for `.opencode/command-hooks.jsonc` upward from the current working directory, up to 20 levels. Ensure your file is within this range.
- **Markdown Delimiters**: For markdown hooks, ensure the frontmatter is placed at the **very top** of the file between `---` delimiters.

---

## Section 6: Best Practices

Follow these guidelines for a smooth experience:

- **Performance**: Keep hook commands fast. Since hooks run during tool execution, slow commands can add noticeable latency to the agent's workflow.
- **Non-Destructive `before` Hooks**: Avoid modifying files that the agent is currently working on in a `before` hook, as this can lead to race conditions or unexpected agent behavior.
- **Test Outside Hooks**: Always verify your shell scripts independently to ensure they handle edge cases and exit codes correctly.
- **User vs. Agent Awareness**: Use `toast` for information you need to see, and `inject` for information the agent needs to know.
- **Colocation**: Prefer markdown frontmatter for hooks that are specific to a single agent. This keeps the logic close to the agent's instructions.
- **Global Concerns**: Use the global JSONC file for project-wide validation, monitoring, or environment setup.
- **Meaningful IDs**: Use descriptive IDs like `"lint-after-write"` instead of generic names like `"hook1"` to make debugging easier.
- **Sequential Execution**: Commands in a `run` array execute one after another. Ensure the order is correct if one command depends on the result of a previous one.
- **Non-Blocking**: Remember that hook execution is non-blocking; the agent will continue its task while the hook runs in the background.
