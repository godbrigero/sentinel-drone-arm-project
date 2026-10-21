---
name: commit
description: "Review changes, reuse still-valid checks from the current thread, run missing verification, and create a local Git commit. Use when the user requests a commit or invokes $commit; project-specific to Sentinel/Airtruck."
---

# Commit

Carry the user's commit request through verification, staging, and a verified local commit. An invocation requesting a commit authorizes those actions; do not ask for routine confirmation again. Creating or editing this skill is not an instruction to commit. Do not push, amend, reset, bypass hooks, or change Git identity unless separately authorized.

## Establish the proposed commit

Read the repository's current `AGENTS.md` and nested instructions. Inspect branch, HEAD, Git status, staged and unstaged diffs, and untracked files. Follow the user's specified scope; otherwise commit the work requested in this thread, preserving unrelated or concurrent changes. Inspect existing staged changes rather than assuming they belong. Ask only if scope cannot be resolved from the thread and files. Do not sweep unrelated edits into a commit or silently unstage them. If there is nothing to commit, report that and stop.

## Verify, reusing evidence

For Sentinel/Airtruck, read and apply [airtruck-preflight](../airtruck-preflight/SKILL.md) and its review contract. This commit request supplies the staging/commit authorization that the audit skill alone does not. This skill belongs to this repository; use its checked-in instructions and preflight resources.

Build a brief verification ledger from the thread: check/review, command or scope, result, and the files/revision/environment it covered. Reuse successful checks already performed in this thread only when their evidence is available and their relevant inputs still match the proposed commit. A statement that tests passed, without identifiable coverage or a trustworthy snapshot, is insufficient. Missing, failed, skipped, or stale checks remain unverified.

- If code, test inputs, dependencies, configuration, or relevant tooling changed, rerun the affected checks. Run the full suite when impact cannot be bounded confidently or project instructions require it.
- A documentation-only edit does not invalidate unchanged code test results; recheck documentation and links. Review all project documentation before committing as required by Airtruck's AGENTS.md. A complete review already in this thread may be reused after checking that neither the documents nor the behavior they describe changed.
- If nothing relevant changed, reuse the existing preflight report and semantic review instead of rerunning the whole checker or regenerating identical visuals. If only some evidence is stale, run the affected commands and record the newer results alongside the previous evidence. Do not alter an old report to imply it covered a later snapshot.
- Inspect each changed file, repository structure, configuration consumers, generated debris, and outstanding findings. Validate that the proposed staged contents include the fixes that were tested. Testing a working tree with unstaged fixes does not validate a different index snapshot.
- For Airtruck, `requirements.txt` is optional; its absence is not an error. Preserve `pyproject.toml`, `uv.lock`, and the ignored persistent `.venv`.

Apply necessary in-scope corrections and update relevant existing docs. Refresh invalidated evidence after edits. Resolve blockers before committing; report a genuine blocker if it cannot be fixed within scope. Do not silently downgrade failures or claim physical/hardware/CAD verification from passing software tests. Explain reviewed warnings and limitations proportionately.

If a new Airtruck audit is needed, retain its architecture/change map, flowchart, and checked code examples as temporary review artifacts per that skill. Link an existing current report instead of duplicating it.

## Commit and verify

Stage explicit reviewed paths or hunks, including intended deletions. Preserve unrelated changes and partial staging. Freshly inspect `git diff --cached`, run `git diff --cached --check`, and confirm the index matches the reviewed and tested content. Check HEAD and status again for concurrent changes; reconcile new changes before proceeding.

Write a concise commit message describing the final behavior and purpose. Create a normal local commit, allowing repository hooks to run. If a hook fails or changes files, inspect the result, fix within scope, rerun affected checks, and review the index before retrying. Never use `--no-verify` merely to finish.

After success, freshly read the resulting commit (hash, message, and file summary) and Git status. Confirm the intended files were committed and identify remaining changes without discarding them. Report the short hash and message, checks run versus reused, any material limitations, and remaining changes. Do not claim a clean working tree unless verified.
