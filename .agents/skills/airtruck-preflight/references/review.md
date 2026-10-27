# Airtruck review contract

## Project-specific semantic review

- `airtruck/systems/`: system-owned controls/state, model bindings and optional hardware. SO-101 leader/follower classes inherit the native LeRobot roles; validate calibration, backend transitions, normalized units, input-only leader behavior and cleanup after partial failures. Hardware activation must be explicit; ordinary simulation must not open ports.
- `airtruck/simulation/`: `MujocoDriver` owns the shared world and clock; system updates install controls before one native physics advance. The separate Gymnasium adapter owns packing; standard wrappers own flattening and episode limits. Check namespace isolation, disjoint actuator ownership, copied observations, deterministic reset and cleanup. Public native model/data access is an explicit diagnostic escape hatch.
- `airtruck/examples/` and `airtruck/main.py`: exercise the same object API. Check dataset observation/action alignment, native LeRobot training/checkpoint consumers and hardware-free rollout metadata. Review PX4 direct imports/native SDK installation, NED frame conversion, heartbeat warmup, ACK plus measured-status confirmation and shared flight commands. Mixed-system fixtures and transport doubles do not establish a real Sentinel model, PX4 SITL validation or physical flight performance.
- `models/so101/source/cad-manifest.json`: CAD rebuild/verification inputs. `so101.xml`: native actuators, action-unit metadata, collision pieces and exclusions. `information.json`: compact engineering references with unknowns explicit. LeRobot owns device calibration and training configurations. Check moved/removed inputs against all consumers; do not restore retired runtime JSON or duplicate computed values without a clear need.
- `worlds/`: base templates and standalone native reference scenes. Check relative asset paths, namespace isolation, initial joints and native contact policy. Reference scenes already contain robots and must not be passed as object-assembly templates. Collision-disabled diagnostics and decomposed convex contacts have different claims.
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
    {"title": "One world owns physics", "detail": "Verified before/after behavior and why it changed", "paths": ["airtruck/simulation/driver.py"]}
  ],
  "findings": [
    {"severity": "warning", "path": "airtruck/main.py:1", "message": "Concrete finding with effect and next action"}
  ],
  "flow": [
    {"label": "Systems", "detail": "Objects own controls, observations and hardware flags"},
    {"label": "Driver", "detail": "Validate every command, dispatch all systems, then advance one shared world"}
  ],
  "examples": [
    {"title": "Current public API", "code": "from airtruck.simulation.driver import MujocoDriver"}
  ],
  "limitations": ["GUI not exercised in this headless preflight"]
}
```

Include the complete document list from the report, not just the examples above. Allowed severities: `error`, `warning`, `info`. Missing manual coverage remains unverified. Do not insert secrets or private external data into the report. The renderer escapes all supplied text; code examples are displayed, never executed. Keep code examples short and verify imports/type signatures against the actual checkout.

Use observations from both index and working tree to describe pending changes. Newly added files appear separately until Git stages/recognizes renames; do not count a deleted config plus a byte-identical untracked replacement as lost data. Use `git show HEAD:path` and content hashes to verify moves. Branch history is context, not proof that a change was made in the current task.
