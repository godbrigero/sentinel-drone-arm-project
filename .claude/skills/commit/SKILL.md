---
name: commit
description: "Review changes, reuse still-valid checks from the current conversation, run missing verification, and create a local Git commit. Use when the user requests a commit or invokes /commit; project-specific to Sentinel/Airtruck."
---

# Commit

Use this adapter for Claude Code's `/commit` command. Optional scope or message guidance from the user:

$ARGUMENTS

Read and follow the repository's [shared commit workflow](../../../.agents/skills/commit/SKILL.md) in full before acting. That file is the source of truth for verification, documentation review, staging, committing, and final verification. Treat its `$commit` invocation as `/commit` here and its references to the current thread as this conversation. Keep the workflow in this conversation so previous verification evidence remains available.

Read the repository's `AGENTS.md` and applicable nested instructions, even when Claude Code has not loaded them automatically. Resolve links in the shared workflow relative to that file, including [airtruck-preflight](../../../.agents/skills/airtruck-preflight/SKILL.md) and its review contract. Read those files directly; they do not need separate Claude Code skill registrations. Run project commands from the repository root.

Creating or editing this adapter is not a request to commit. A commit request authorizes review, in-scope corrections, staging, and a verified local commit; it does not authorize pushing, amending, resetting, bypassing hooks, or changing Git identity. Preserve unrelated and concurrent work as required by the shared workflow.
