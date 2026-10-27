---
name: airtruck-preflight
description: "Audit the Sentinel/Airtruck project before a commit or on request: run verification, review documentation and repository layout, inspect all pending changes, and explain the current architecture and changes with visual maps and code examples. Use for this project's preflight or commit-readiness review, not unrelated repositories."
---

# Airtruck preflight

Produce an evidence-based review, not just a green test count. This skill combines a deterministic checker with a semantic code/documentation review. Run it before a requested commit, but it does not itself authorize staging, committing, installing hooks, pushing, hardware operations, or CAD changes.

## Establish scope

Read the checkout's `AGENTS.md` and any nested instructions. Read root `information.json` if present and model-specific information records; this project currently uses `models/*/information.json`. Treat these as engineering records, not a place for review notes. Preserve unrelated and concurrent edits.

Confirm the project from `pyproject.toml` and the repository root. Default review scope is **everything currently changed relative to HEAD**, distinguishing staged, unstaged, deleted, renamed, and untracked files. Inspect unchanged modules and documents needed to understand the whole system. If the user asks about changes since a specific revision, inspect that range too; don't substitute the current diff for the requested range.

## Run deterministic checks

Run the bundled script from the project root:

```sh
python3 .agents/skills/airtruck-preflight/scripts/preflight.py --project .
```

It writes evidence and a standalone HTML review into a new system temporary directory, prints its location, and returns nonzero for failures. It inventories source/config/assets, checks layout and local documentation links, parses Python/JSON/TOML/XML, maps imports, records all pending Git paths, checks whitespace and the lockfile, runs the full headless pytest suite, mypy and basedpyright, and flags suspicious source imports and diagnostic debris. It does not modify source, stage changes, commit, delete user files, or launch a GUI. Dependency tooling may update the existing `.venv`; it does not update dependency declarations or the lockfile.

Read `report.json` and individual check logs even if the command fails. Missing tools, timeouts, skipped checks, and unexpected file changes are **unverified**, never passes. Do not repeatedly rerun a failing check without diagnosing it. Keep `MUJOCO_LOG.TXT` warnings visible: inspect new logs, identify their cause, and report them rather than ignoring their contents. Root rules and directory expectations are defaults for this project; reconcile legitimate new folders with current instructions rather than automatically deleting them.

## Review what automation cannot prove

Read [references/review.md](references/review.md). Inspect **every changed file** (for large generated/binary assets use provenance, hashes, inputs, and consumers instead of dumping them), and all project Markdown documentation outside dependency/cache metadata. Explain findings with file/line evidence and impact. Review staged and unstaged versions separately when they differ; do not imply that unstaged fixes are in the proposed commit.

Check documentation meaning, runnable examples, source-of-truth boundaries, actual package paths and ownership, type coverage, stale scripts, duplicate configuration, generated debris, untracked files a commit could omit, simulation limitations, and whether tests prove the intended behavior. Trace actual calls from system construction through model attachment, backend selection, command dispatch and shared-world stepping to the Gymnasium and LeRobot consumers. Distinguish dependency edges from runtime call order. The script's graph is an import graph, not proof of runtime behavior.

Review mode is read-only. If the user also requested fixes, implement authorized fixes, update documentation, then rerun affected checks and refresh the final snapshot. Do not silently fix unrelated issues, delete assets, rebuild CAD, or enable real motors. Any Onshape change needs the project's separate CAD verification workflow.

## Explain and publish the result

Read [references/review.md](references/review.md) for the review-note schema. Write `review.json` beside the report, covering every document, meaningful changes grouped by purpose, risks, a runtime flowchart, and short code examples copied from or checked against the current API. Compare current behavior with HEAD only where verified; label inference and unverified behavior.

Render without repeating tests:

```sh
python3 .agents/skills/airtruck-preflight/scripts/preflight.py --render-report /tmp/airtruck-preflight-.../report.json --review-notes /tmp/airtruck-preflight-.../review.json
```

Rendering checks that the repository snapshot has not changed. If it has, inspect the changes and refresh evidence before claiming readiness. Nonzero after rendering can mean accurately reported findings; don't force a green report.

When T3 `html_preview` and `html_render` are available, preview the generated HTML at desktop and narrow widths, inspect the screenshot/console, correct presentation failures, and publish with `html_render`. Discover these tools rather than assuming a particular prefix. Otherwise provide the standalone HTML link and a compact Mermaid/text flowchart in the response; don't claim you rendered a visual you did not inspect. No CDN or network-dependent chart libraries are needed.

The final review must clearly separate blockers, concerns, passed checks, and manual/unverified items. Link the report and key files, state the exact test/type-check results and review scope, and explain how to invoke this skill again. Avoid declaring the entire project physically validated: configured contact transfer, reference inertias, and motor approximations are not hardware validation. Do not write reports or screenshots into the repository root; temporary artifacts are the requested review deliverable.
