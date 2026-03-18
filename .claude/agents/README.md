# Claude Code Agents

This directory contains custom Claude Code agents for the Lentigram project. These agents provide specialized knowledge and can be invoked via the Task tool.

## What Are Agents?

Agents are specialized prompt files that give Claude Code deep knowledge about specific aspects of your codebase. They travel with the repository and can be shared with your team.

## Available Agents

### developer.md
**Core development patterns and architecture**

Contains:
- Architectural principles (Model as Source of Truth)
- Design patterns (Action/Command/MVVM)
- Performance best practices (scanLine vs pixel, threading)
- Qt coding standards for Lentigram
- Common pitfalls and how to avoid them
- depth-lib specific knowledge (blur pyramids, depth conventions)
- Code review checklist

**When to use**: Working on features, refactoring, reviewing code, making architectural decisions, setting up new actions/commands/viewmodels.

---

### build-script-expert.md
**Cross-platform build system specialist**

Contains:
- CMake patterns for Linux/Windows/macOS
- Dependency compilation scripts (install-deps.sh)
- MSYS2/MinGW64 path handling on Windows
- x86_64 architecture enforcement on macOS
- Compiler toolchain selection per platform
- Build failure debugging

**When to use**: Modifying install-deps.sh, adding new dependencies, fixing build issues, ensuring cross-platform compatibility.

---

### debugger.md
**Error investigation and root cause analysis**

Contains:
- Debugging strategies for test failures
- Crash investigation techniques
- Performance issue diagnosis
- Unexpected behavior analysis
- Log analysis and tracing
- Memory leak detection

**When to use**: Test failures, crashes, segfaults, unexpected behavior, performance regressions. Invoke proactively when errors occur.

---

### code-reviewer.md
**Code quality and best practices analysis**

Contains:
- Readability improvement suggestions
- Performance optimization opportunities
- Security vulnerability detection
- Best practices verification
- Code smell identification
- Refactoring recommendations

**When to use**: After implementing features, before submitting PRs, improving code quality, cleaning up technical debt.

---

### tech-docs-writer.md
**Technical documentation creation and maintenance**

Contains:
- Documentation structure and style guidelines
- README patterns and best practices
- API documentation standards
- Installation guide templates
- Architecture overview formats
- Consistent cross-platform documentation

**When to use**: Creating README files, documenting APIs, writing installation guides, updating architecture docs, ensuring documentation consistency.

---

### testing.md
**Testing strategies and best practices**

Contains:
- Dynamic test name patterns (no hardcoded paths)
- Google Test best practices (ASSERT vs EXPECT, modern API)
- Test output directory management for fixtures and non-fixtures
- Image processing test patterns (format preservation, pixel comparison)
- Resource finding with ResourceFinder
- Parameterized test patterns
- Test naming conventions
- Anti-patterns to avoid

**When to use**: Writing new tests, updating existing tests, fixing test failures, organizing test output, ensuring test maintainability.

---

## How to Use Agents

### Via Task Tool (Recommended)
Claude Code can invoke agents using the Task tool with the appropriate `subagent_type`:

```
"Use the developer agent to review this code for architectural best practices"
"Use the build-script-expert to fix the Windows build issue"
"Use the debugger agent to investigate this test failure"
"Use the code-reviewer to analyze this file for improvements"
"Use the tech-docs-writer to create installation documentation"
"Use the testing agent to review test patterns and best practices"
```

### Manual Reference
You can also reference agents directly in prompts:
```
"Check .claude/agents/developer.md for our threading patterns"
"Following the guidelines in .claude/agents/build-script-expert.md, update install-deps.sh"
```

---

## Agent Hierarchy

```
CLAUDE.md (project root)
    ↓ (project overview, build commands)
    ↓
.claude/agents/ (this directory)
    ↓ (specialized deep knowledge)
    ↓
    ├── developer.md        → Architecture, patterns, best practices
    ├── build-script-expert.md → Cross-platform builds, dependencies
    ├── debugger.md         → Error investigation, root cause analysis
    ├── code-reviewer.md    → Code quality, refactoring suggestions
    ├── tech-docs-writer.md → Documentation standards, templates
    └── testing.md          → Testing strategies, test best practices
    ↓
~/.claude/.../MEMORY.md (auto memory)
    ↓ (session-specific temporary notes)
```

**Think of it as**:
- `CLAUDE.md` = "What is this project?"
- `.claude/agents/*.md` = "How do we build/debug/document/code this?"
- `MEMORY.md` = "What am I working on right now?"

---

## When to Use Which Agent

| Scenario | Agent to Use |
|----------|--------------|
| Implementing new feature | developer.md |
| Writing new tests | testing.md |
| Reviewing code quality | code-reviewer.md |
| Test failure or crash | debugger.md |
| Modifying build scripts | build-script-expert.md |
| Creating documentation | tech-docs-writer.md |
| Threading/performance issue | developer.md |
| Cross-platform build failure | build-script-expert.md |
| Architectural decision | developer.md |
| Pre-PR code review | code-reviewer.md |
| API documentation | tech-docs-writer.md |
| Test output organization | testing.md |

---

## Benefits

- **Version controlled**: Agents are checked into git with your code
- **Team shared**: All developers benefit from accumulated knowledge
- **Specialized**: Each agent focuses on a specific domain
- **Portable**: Agents travel with the repository
- **Discoverable**: Easy to find in `.claude/agents/`
- **Executable**: Can be used via Task tool for autonomous work
- **Maintainable**: Update agents as patterns evolve

---

## Adding New Agents

To add a new agent:

1. Create a new `.md` file in this directory
2. Follow the structure of existing agents
3. Document the agent's purpose in this README
4. Update MEMORY.md if needed to reference the new agent

Potential new agents:
- `ui-guidelines.md` - UI/UX conventions
- `api-design.md` - API design principles
- `security.md` - Security best practices
- `performance.md` - Performance profiling and optimization

---

For more information about Claude Code agents, see: https://github.com/anthropics/claude-code
