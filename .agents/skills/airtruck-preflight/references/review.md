# Airtruck review contract

## Project-specific semantic review

- `airtruck/raw_api/`: device contracts and adapters; `RealSO101` must not gain simulation stepping or implicit hardware connections. Validate calibration/transport boundaries.
- `airtruck/testing_util/`: shared world controls, scene loading, explicit pytest opt-in, contacts, lifecycle and tests. Ensure exactly one physics step per world tick, disjoint actuator ownership, no writable MuJoCo buffers in public views, bounded waits, and cleanup on partial failures.
- `models/so101/runtime/servo-model.json`: controller/build inputs. `source/cad-manifest.json`: CAD rebuild/verification inputs. `information.json`: compact engineering references with unknowns explicit. Check moved/removed inputs against all consumers; don't duplicate computed values into new JSON without a clear need.
- `worlds/`: environment plus system instances; paths relative to manifest, namespace isolation, initial joints, explicit contact policy. Collision-disabled demonstrations and convex-hull contact tests have different claims.
- `scripts/`: launchers/builders/verifiers. Verify executable bits, paths, dependency entrypoints, and no stale package imports. A successful import is not a successful GUI run; run GUI checks only when relevant and available.
- `typings/`: local MuJoCo stubs, not runtime PYTHONPATH. Check scope of mypy versus all source, including otherwise untested entrypoints.
- `.venv/`, `uv.lock`, `pyproject.toml`, `requirements.txt`: preserve the user-requested persistent environment. Treat `requirements.txt` as an optional compatibility export; its absence is not an error. Check dependency consistency when it is present; don't regenerate a lockfile merely to make preflight pass.
- CAD meshes and inertias: unchanged hashes/provenance or the full authorized CAD verification workflow. No claims that hull contacts reproduce concave surfaces exactly or that reference mass is measured mass.
- Documentation: read all project `.md` files outside caches and tool metadata. Validate descriptions and examples, not only links. Check all changed/deleted paths, examples, dependencies, test commands, GUI assumptions, limitations, and root hygiene. User-requested commits require this review before the commit.

The inventory excludes `.git`, `.venv*`, caches and tool/account metadata directories. Report these exclusions. Project skills under `.agents/skills/` are included in the review and snapshot. Symlinks are listed, not followed. A root `information.json` is not required when model-specific records are present. New directories are review candidates, not automatically forbidden.

## Review notes supplied to the renderer

```json
{
  "summary": "What changed, why it matters, and the readiness verdict.",
  "reviewed_documents": ["AGENTS.md", "docs/README.md"],
  "changes": [
    {"title": "One world owns physics", "detail": "Verified before/after behavior and why it changed", "paths": ["airtruck/testing_util/mujoco_world.py"]}
  ],
  "findings": [
    {"severity": "warning", "path": "airtruck/main.py:1", "message": "Concrete finding with effect and next action"}
  ],
  "flow": [
    {"label": "Typed test", "detail": "@simulation selects a world"},
    {"label": "World", "detail": "Controllers update, then physics steps once"}
  ],
  "examples": [
    {"title": "Current public API", "code": "from airtruck.testing_util import simulation, WorldControl"}
  ],
  "limitations": ["GUI not exercised in this headless preflight"]
}
```

Include the complete document list from the report, not just the examples above. Allowed severities: `error`, `warning`, `info`. Missing manual coverage remains unverified. Do not insert secrets or private external data into the report. The renderer escapes all supplied text; code examples are displayed, never executed. Keep code examples short and verify imports/type signatures against the actual checkout.

Use observations from both index and working tree to describe pending changes. Newly added files appear separately until Git stages/recognizes renames; do not count a deleted config plus a byte-identical untracked replacement as lost data. Use `git show HEAD:path` and content hashes to verify moves. Branch history is context, not proof that a change was made in the current task.
