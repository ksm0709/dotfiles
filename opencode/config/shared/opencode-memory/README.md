# OpenCode Memory

A **2-layer memory system** for AI agents. Use with OpenCode plugin to provide agents with long-term memory and working context.

## Features

- **2-layer Architecture**: Working Memory (session-based) + Context Memory (long-term semantic search)
- **Semantic Search**: Ollama/OpenAI/Gemini embedding support
- **Auto Checkpoint**: Compress working memory and store in long-term memory
- **Race Condition Handling**: Safe multi-agent concurrent access
- **Noise Filtering**: Automatic filtering of meaningless content
- **File Persistence**: State persistence across subprocess calls

## Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/ksm0709/opencode-memory.git
cd opencode-memory

# Run install script
./scripts/install.sh --target /path/to/your/project

# Or install directly with pip
pip install -e ".[all]"
```

### Python Usage

```python
from opencode_memory import ContextManager

# Initialize
cm = ContextManager()
cm.init("session-123")
cm.start("API server refactoring")

# Add working memory items
cm.working.add_item("decision", "Decided to switch from REST to GraphQL", "high")
cm.working.add_item("change", "Completed schema design document", "medium", {"file": "schema.md"})

# Checkpoint (auto-summarize and save to long-term memory)
result = cm.checkpoint()
print(result["summary"])  # Compressed work summary

# Search memories
memories = cm.context.query("API design", limit=5)
for m in memories:
    print(f"[{m['score']:.2f}] {m['content'][:100]}")

# End task
cm.end()
```

### CLI Usage

```bash
# Check status
opencode-memory status

# Search memories
opencode-memory query "API design"

# Add memory
opencode-memory add "REST API v2 endpoint design completed" --tags "design,API"
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ContextManager                           │
│  (2-layer integration manager)                              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────┐    ┌─────────────────────────────┐ │
│  │   Working Memory    │    │     Context Memory          │ │
│  │   (short-term/      │    │    (long-term/              │ │
│  │    session-based)   │    │     semantic search)        │ │
│  │                     │    │                             │ │
│  │  - Accumulate work  │───▶│  - OpenMemory based         │ │
│  │    memories         │    │  - Vector embedding search  │ │
│  │  - Generate         │    │  - Auto noise filtering     │ │
│  │    compressed       │    │                             │ │
│  │    summaries        │    │                             │ │
│  │  - File persistence │    │                             │ │
│  └─────────────────────┘    └─────────────────────────────┘ │
│                                                             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   Embedding     │
                    │   Provider      │
                    ├─────────────────┤
                    │ - Ollama (default)│
                    │ - OpenAI        │
                    │ - Gemini        │
                    │ - Synthetic     │
                    └─────────────────┘
```

## Configuration

### config.yaml

```yaml
# .opencode/shared/context/config.yaml

tier: hybrid  # hybrid | fast | smart | deep

embeddings:
  provider: ollama  # ollama | openai | gemini | synthetic
  model: nomic-embed-text
  url: http://localhost:11434
  fallback: synthetic

database:
  backend: sqlite
  path: ${PROJECT_ROOT}/data/memory/context.sqlite

context:
  search:
    min_score: 0.3
    default_limit: 10
  decay:
    enabled: true
    lambda: 0.02

working:
  auto_checkpoint_interval: 3
  max_items: 50
  persist: true
  persist_path: ${PROJECT_ROOT}/data/memory/working_memory.json
```

### Environment Variables

```bash
export CONTEXT_TIER=hybrid
export CONTEXT_EMBEDDINGS_PROVIDER=ollama
export CONTEXT_LOG_LEVEL=INFO
```

## OpenCode Plugin Integration

### Plugin Registration

```json
// .opencode/config.json
{
  "plugins": [
    ".opencode/plugin/context-manager.ts"
  ]
}
```

### Memory Skill Usage

```
# In agent prompt
/memory add "API v2 refactoring complete: 30% performance improvement"
/memory query "recent analysis results"
/memory stats
```

## API Reference

### ContextManager

| Method | Description |
|--------|-------------|
| `init(session_id)` | Initialize session |
| `start(task)` | Start task + retrieve related memories |
| `checkpoint(summary?)` | Create checkpoint |
| `end()` | End task + final save |
| `get_compaction_context()` | Get context for compaction |

### ContextMemory

| Method | Description |
|--------|-------------|
| `add(content, tags?, metadata?)` | Add memory |
| `query(question, limit?, auto_cleanup?)` | Search memories |
| `delete(memory_id)` | Delete memory |
| `query_and_update(summary)` | Atomic search + save |

### WorkingMemory

| Method | Description |
|--------|-------------|
| `add_item(type, content, importance, metadata?)` | Add work item |
| `summarize_since_checkpoint()` | Generate compressed summary |
| `clear_since_checkpoint()` | Clear after checkpoint |
| `get_items()` | Get current items list |

## Development

```bash
# Setup development environment
python -m venv venv
source venv/bin/activate
pip install -e ".[dev]"

# Run tests
pytest tests/ -v

# Lint
ruff check src/
ruff format src/
```

## License

MIT License

## Troubleshooting

### ModuleNotFoundError: No module named 'dotenv'

`openmemory-py` requires `python-dotenv`:

```bash
pip install python-dotenv
```

### NameError: name 'BaseMessage' is not defined

`openmemory-py` requires `langchain-core`:

```bash
pip install langchain-core
```

### Install All OpenMemory Dependencies

To install all dependencies at once:

```bash
pip install openmemory-py python-dotenv langchain-core
# Or install full package
pip install opencode-memory[all]
```

### Ollama Connection Failed

Check if Ollama service is running:

```bash
# Start Ollama
ollama serve

# Download embedding model
ollama pull nomic-embed-text
```

### Using Synthetic Fallback

To use synthetic embeddings without Ollama, modify `config.yaml`:

```yaml
embeddings:
  provider: synthetic
```
