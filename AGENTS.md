# Project working requirements

- Double-check work before reporting completion. Verify the saved result with a fresh read, not just a successful write response.
- For Onshape CAD imports and edits, verify physical orientation, units, assembly relationships, geometry integrity, and relevant mass properties. Do not assume the source file's axes match Onshape's up direction.
- Inspect rendered images from the resulting Onshape model. For orientation changes, inspect an isometric view and at least one orthographic view, and confirm numerically that the landing gear is below the body and upper equipment is above it.
- Provide the user an image of the verified final CAD and a direct link to the resulting Onshape element.
- Continue monitoring imports and regeneration until they finish, then inspect the result. Diagnose and correct failures within the authorized scope; do not treat queued, uploaded, or partially completed work as finished.
- Preserve the official Sentinel geometry. Do not recreate or simplify drone geometry unless explicitly requested.
- Distinguish reference aircraft weight from assigned CAD mass, center of mass, and inertia. Clearly report missing or unverified properties.

## Project information record

- Read `information.json` before project work. Keep it compact: component models, numeric geometry, vectors, dimensions, masses and units. Use null for unknowns; distinguish manufacturer references, CAD values and physical measurements. Do not add narrative, checklists, history or artifact inventories to this file.
- Update affected fields whenever new verified information becomes available. Keep source references alongside the relevant model data. Do not create a separate state directory.
- Re-read affected information records before edits and preserve unrelated or concurrent changes. Validate JSON and freshly read the saved result. Do not invent missing values or treat this record as permission to modify CAD or hardware.

## Workspace hygiene

- Keep only `information.json`, `AGENTS.md`, `.gitignore`, an optional compatibility export `requirements.txt`, `pyrightconfig.json`, `pyproject.toml`, and `uv.lock` as root files. Store model-specific configuration under its model directory.
- Onshape is the CAD source of truth. Do not retain local STEP/ZIP exports, images, HTML previews, temporary scripts, virtual environments or caches after verification unless the user requests them. Use temporary storage during work and remove it afterward. Preserve version-control metadata if present.

- The user-requested `.venv/` is the persistent uv project environment; keep it ignored by Git. Manage dependencies in `pyproject.toml` and lock them in `uv.lock`. These are the dependency sources of truth for this uv project. `requirements.txt` is optional: generate it from the lockfile only when a compatibility workflow needs it. Do not require it or flag its absence as a preflight error.

## Documentation maintenance

- Keep documentation clean, accurate, and up to date. After every major change, review all files under `docs/` and other project documentation for relevance, including paths, imports, commands, examples, behavior, and limitations. Update or remove stale or duplicated material rather than only appending new notes.
- Record implemented changes in the relevant existing documentation so it describes the current project. Keep narrative and change notes out of `information.json`.
- Whenever the user requests a commit, review all project documentation before committing anything. Complete needed documentation corrections and verify documented paths and commands before creating the commit.
- Inspect generated diagnostics such as `MUJOCO_LOG.TXT` when they appear, fix their cause where applicable, and remove stale logs after verification. Do not commit generated logs or suppress useful warnings merely to keep the workspace clean.
