#!/usr/bin/env python3
"""Read-only Airtruck preflight with temporary evidence and a visual review."""
from __future__ import annotations
import argparse
import ast
from collections import Counter
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import tomllib
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT_FILES = {'information.json', 'AGENTS.md', '.gitignore', 'requirements.txt',
              'pyrightconfig.json', 'pyproject.toml', 'uv.lock'}
ROOT_DIRS = {'.agents', 'airtruck', 'models', 'worlds', 'scripts', 'docs', 'typings', 'global-info-images'}
OMIT = {'.git', '.aws', '.codex', '__pycache__', '.mypy_cache', '.pytest_cache', '.ruff_cache', '.cache'}


def inventory(root: Path) -> tuple[list[dict], list[str]]:
    files, omitted = [], []
    for directory, dirs, names in os.walk(root, followlinks=False):
        for name in list(dirs):
            path = Path(directory, name)
            relative = path.relative_to(root).as_posix()
            if name in OMIT or name.startswith('.venv'):
                omitted.append(relative)
                dirs.remove(name)
            elif path.is_symlink():
                files.append({'path': relative, 'size': 0, 'symlink': True})
                dirs.remove(name)
        for name in names:
            path = Path(directory, name)
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                files.append({'path': relative, 'size': 0, 'symlink': True})
            else:
                files.append({'path': relative, 'size': path.stat().st_size, 'symlink': False})
    return sorted(files, key=lambda f: f['path']), sorted(omitted)


def fingerprint(root: Path, files: list[dict]) -> str:
    digest = hashlib.sha256()
    for item in files:
        p = root/item['path']
        digest.update(item['path'].encode())
        digest.update(str(p.lstat().st_mode).encode())
        if item['symlink']:
            digest.update(os.readlink(p).encode())
        else:
            with p.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(block)
    return digest.hexdigest()


def command(root: Path, output: Path, name: str, argv: list[str], timeout: int = 180) -> dict:
    started = time.monotonic()
    env = os.environ.copy()
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    log = output/(name+'.log')
    try:
        with log.open('w') as stream:
            proc = subprocess.Popen(argv, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
                                    env=env, start_new_session=True)
            try:
                code = proc.wait(timeout=timeout)
                status = 'pass' if code == 0 else 'fail'
            except subprocess.TimeoutExpired:
                import signal
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                code, status = None, 'timeout'
    except OSError as error:
        log.write_text(str(error))
        code, status = None, 'unavailable'
    return {'name': name, 'argv': argv, 'status': status, 'returncode': code,
            'seconds': round(time.monotonic()-started, 2), 'log': str(log),
            'output_tail': log.read_text(errors='replace')[-5000:]}


def git(root: Path, *args: str) -> str:
    return subprocess.check_output(['git', *args], cwd=root).decode('utf-8', 'replace')


def index_fingerprint(root: Path) -> str:
    return hashlib.sha256(git(root, 'diff', '--cached', '--binary').encode()).hexdigest()


def changes(root: Path) -> list[dict]:
    entries = git(root, 'status', '--porcelain=v1', '-z', '--untracked-files=all').split('\0')
    result = []
    i = 0
    while i < len(entries):
        entry = entries[i]
        i += 1
        if not entry:
            continue
        item = {'index': entry[0], 'working_tree': entry[1], 'path': entry[3:]}
        if 'R' in entry[:2] or 'C' in entry[:2]:
            item['old_path'] = entries[i]
            i += 1
        result.append(item)
    return result


def inspect(root: Path, files: list[dict]) -> tuple[list[dict], list[str], list[dict]]:
    findings, documents, edges = [], [], []
    paths = {f['path'] for f in files}
    modules = {p[:-3].replace('/', '.').removesuffix('.__init__') for p in paths if p.endswith('.py')}

    def finding(severity: str, path: str, message: str) -> None:
        findings.append(dict(severity=severity, path=path, message=message))

    for required in ('AGENTS.md', 'pyproject.toml', 'uv.lock', 'pyrightconfig.json'):
        if not (root/required).is_file():
            finding('error', required, 'Required project file is missing; reconcile project instructions and consumers.')
    for child in root.iterdir():
        if child.is_file() and child.name not in ROOT_FILES:
            finding('error', child.name, 'Unexpected root file; inspect and relocate or remove only with appropriate authorization.')
        if child.is_dir() and child.name not in ROOT_DIRS | OMIT and not child.name.startswith('.venv'):
            finding('warning', child.name+'/', 'Unclassified top-level directory; review ownership and documentation.')
    for item in files:
        relative = item['path']
        path = root/relative
        if item['symlink']:
            finding('warning', relative, 'Symlink inventoried but not followed; inspect its target separately.')
            continue
        if path.name == 'MUJOCO_LOG.TXT':
            finding('error', relative, 'Generated MuJoCo diagnostic log: inspect warnings before cleanup.')
        if path.suffix.lower() in {'.step', '.stp', '.zip', '.html'}:
            finding('warning', relative, 'Possible generated/export artifact; confirm it was explicitly requested.')
        if path.suffix not in {'.py', '.json', '.toml', '.xml', '.md'}:
            continue
        try:
            text = path.read_text()
            if path.suffix == '.json':
                def unique(pairs: list[tuple[str, object]]) -> dict:
                    result = {}
                    for key, value in pairs:
                        if key in result:
                            raise ValueError(f'Duplicate JSON key: {key}')
                        result[key] = value
                    return result
                json.loads(text, object_pairs_hook=unique,
                           parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f'Nonfinite JSON: {value}')))
            elif path.suffix == '.toml':
                tomllib.loads(text)
            elif path.suffix == '.xml':
                ET.fromstring(text)
            elif path.suffix == '.py':
                tree = ast.parse(text, filename=relative)
                for node in ast.walk(tree):
                    imports = []
                    if isinstance(node, ast.Import):
                        imports = [x.name for x in node.names]
                    elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                        imports = [node.module]
                    for target in imports:
                        if target.startswith('airtruck.'):
                            edges.append({'source': relative, 'target': target})
                            if target not in modules and not any(m.startswith(target+'.') for m in modules):
                                finding('error', f'{relative}:{node.lineno}', f'Unresolved project import {target}')
                        elif target not in sys.stdlib_module_names and '.' not in target and target not in modules and any(m.endswith('.'+target) for m in modules):
                            finding('warning', f'{relative}:{node.lineno}', f'Possible bare package import {target!r}; verify module execution from the project root.')
            else:
                documents.append(relative)
                for block in re.findall(r'```python\n(.*?)```', text, re.S):
                    ast.parse(block)
                for target in re.findall(r'\]\(([^)\n]+)\)', text):
                    target = target.strip('<>')
                    if '://' in target or target.startswith(('#', 'mailto:')):
                        continue
                    target = unquote(target.split('#')[0])
                    if target and not (path.parent/target).exists():
                        finding('error', relative, f'Broken local documentation link: {target}')
                if re.search(r'airtruck[./](testing|simulation)\b|models/so101/config/', text):
                    finding('warning', relative, 'Retired package/config path mentioned; check whether this is intentional migration guidance.')
        except (ValueError, SyntaxError, ET.ParseError, UnicodeError) as error:
            finding('error', relative, f'Parse/example validation failed: {error}')
    for p in root.glob('worlds/*/world.json'):
        try:
            data = json.loads(p.read_text())
            targets = [data['scene']]
            for system in data.get('systems', []):
                targets.extend([system['model'], system['controller_config']])
            for target in targets:
                if not (p.parent/target).is_file():
                    finding('error', str(p.relative_to(root)), f'Missing world input: {target}')
        except (ValueError, KeyError, TypeError) as error:
            finding('error', str(p.relative_to(root)), f'Invalid world manifest: {error}')
    return findings, sorted(documents), edges


def render(report: dict, review: dict | None) -> str:
    esc = lambda value: html.escape(str(value))
    notes = review or {}
    missing = sorted(set(report['documents'])-set(notes.get('reviewed_documents', [])))
    reviewed_paths = {f['path'] for f in notes.get('findings', [])}
    findings = [f for f in report['findings'] if f['path'] not in reviewed_paths] + notes.get('findings', [])
    failed = any(c['status'] != 'pass' for c in report['checks']) or any(f['severity'] == 'error' for f in findings)
    verdict = 'Blocked by findings/checks' if failed else ('Manual review incomplete' if missing or not review else ('Review concerns before commit' if any(f['severity']=='warning' for f in findings) else 'Reviewed; checks passed'))
    boxes = ''.join(f'<div class="metric"><b>{esc(value)}</b><span>{esc(label)}</span></div>' for value,label in [(verdict,'Readiness'),(len(report['files']),'Inventoried files'),(len(report['changes']),'Pending paths'),(len(missing),'Docs awaiting review')])
    check_rows = ''.join(f'<details><summary><strong class="{esc(c["status"])}">{esc(c["status"].upper())}</strong> {esc(c["name"])} · {c["seconds"]}s</summary><code>{esc(" ".join(c["argv"]))}</code><pre>{esc(c["output_tail"])}</pre></details>' for c in report['checks'])
    finding_rows = ''.join(f'<li><strong>{esc(f["severity"].upper())}</strong> <code>{esc(f["path"])}</code><p>{esc(f["message"])}</p></li>' for f in findings) or '<li>No automated or reviewed findings.</li>'
    summary = esc(notes.get('summary', 'Automated evidence is ready. Semantic review of code, docs and change intent is still required.'))
    change_rows = ''.join(f'<tr><td><code>{esc(c["index"] or " ")}</code></td><td><code>{esc(c["working_tree"] or " ")}</code></td><td>{esc(c.get("old_path", ""))}{" → " if c.get("old_path") else ""}{esc(c["path"])}</td></tr>' for c in report['changes'])
    explanation = ''.join(f'<details><summary>{esc(c["title"])}</summary><p>{esc(c["detail"])}</p><code>{esc(", ".join(c.get("paths", [])))}</code></details>' for c in notes.get('changes', []))
    flow = notes.get('flow', [])
    flow_html = '<div class="flow">' + ''.join(f'<div class="node"><strong>{esc(n["label"])}</strong><p>{esc(n["detail"])}</p></div>' + ('<span class="arrow" aria-label="then">↓</span>' if i<len(flow)-1 else '') for i,n in enumerate(flow)) + '</div>' if flow else '<p>Runtime flow awaits code review; imports alone cannot establish execution order.</p>'
    counts = Counter(p['path'].split('/')[0] if '/' in p['path'] else '(root files)' for p in report['files'])
    tree = ''.join(f'<li><code>{esc(name)}</code> <b>{count}</b> files</li>' for name,count in sorted(counts.items()))
    packages = Counter((e['source'].split('/')[1] if '/' in e['source'] else e['source'], '.'.join(e['target'].split('.')[:2])) for e in report['imports'])
    imports = ''.join(f'<div class="edge"><code>{esc(a)}</code><span> → {count} imports → </span><code>{esc(b)}</code></div>' for (a,b),count in sorted(packages.items()))
    examples = ''.join(f'<h3>{esc(e["title"])}</h3><pre>{esc(e["code"])}</pre>' for e in notes.get('examples', []))
    limitations = '<ul>'+''.join(f'<li>{esc(x)}</li>' for x in notes.get('limitations', []))+'</ul>'
    docs = ''.join(f'<li>{"Pending" if p in missing else "Reviewed"}: <code>{esc(p)}</code></li>' for p in report['documents'])
    files = ''.join(f'<tr><td>{esc(p["path"])}</td><td>{p["size"]:,}</td></tr>' for p in report['files'])
    return f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Airtruck preflight</title>
<style>
:root{{font-family:system-ui,sans-serif;color:var(--foreground,#1f2937);font-size:14px}}body{{margin:0;background:transparent}}h2{{font-size:19px;margin-top:26px}}h3{{font-size:15px}}p{{line-height:1.55}}code,pre{{font-family:ui-monospace,monospace;font-size:12px}}code{{overflow-wrap:anywhere}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:var(--code-background,#f1f5f9);padding:16px;border-radius:8px}}.metrics{{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}}.metric,.node,article{{padding:16px;background:var(--card,#f8fafc);border:1px solid var(--border,#ddd);border-radius:10px}}.metric b,.metric span{{display:block}}.metric span{{margin-top:6px;color:var(--muted-foreground,#64748b)}}.flow{{max-width:560px;margin:auto}}.arrow{{display:block;text-align:center;font-size:24px;color:var(--accent,#2563eb)}}.node p{{margin-bottom:0}}details{{border-bottom:1px solid var(--border,#ddd);padding:10px 0}}summary{{cursor:pointer;line-height:1.5}}table{{border-collapse:collapse;width:100%;font-size:12px;table-layout:fixed}}th,td{{text-align:left;padding:8px;border-bottom:1px solid var(--border,#ddd);overflow-wrap:anywhere}}th:first-child,td:first-child{{width:32px}}.inventory td:first-child{{width:auto}}.pass{{color:var(--success,#15803d)}}.fail,.timeout,.unavailable{{color:var(--destructive,#b91c1c)}}.edge{{padding:8px 0;overflow-wrap:anywhere}}article{{margin:10px 0}}li{{margin-bottom:9px}}.meta{{color:var(--muted-foreground,#64748b);font-size:12px;overflow-wrap:anywhere}}button{{padding:8px 12px;border:1px solid var(--border,#ddd);border-radius:6px;background:var(--card,#fff);color:inherit;cursor:pointer}}@media(max-width:420px){{.metrics{{grid-template-columns:1fr}}}}
</style><body><p class="meta">{esc(report['timestamp'])} · {esc(report['project'])} · HEAD {esc(report['head'][:12])}</p><div class="metrics">{boxes}</div><p>{summary}</p>
<h2>Findings</h2><ul>{finding_rows}</ul><h2>Checks and evidence</h2>{check_rows}
<h2>What changed</h2>{explanation}<details><summary>All pending paths · index / working tree (? = untracked)</summary><table><tr><th>Index</th><th>Work</th><th>Path</th></tr>{change_rows}</table></details>
<h2>Runtime flow · reviewed</h2>{flow_html}<h2>Repository map · observed</h2><ul>{tree}</ul><details><summary>Package dependency map (imports, not runtime order)</summary>{imports}</details>
<h2>Current API examples</h2>{examples}<h2>Documentation coverage</h2><details><summary>{len(report["documents"])-len(missing)} of {len(report["documents"])} documents reviewed</summary><ul>{docs}</ul></details><h2>Limits of this review</h2>{limitations}<p class="meta">Excluded from content inspection: {esc(', '.join(report['excluded']))}. Headless tests do not certify real hardware or CAD edits. This report never stages or commits files.</p>
<details><summary>Complete file inventory</summary><table class="inventory"><tr><th>Path</th><th>Bytes</th></tr>{files}</table></details></body></html>'''


def validate_notes(notes: dict) -> None:
    for key in ('reviewed_documents', 'changes', 'findings', 'flow', 'examples', 'limitations'):
        if not isinstance(notes.get(key, []), list):
            raise ValueError(f'{key} must be an array')
    for finding in notes.get('findings', []):
        if finding.get('severity') not in {'error', 'warning', 'info'}:
            raise ValueError('Unknown finding severity')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path.cwd())
    parser.add_argument('--render-report', type=Path)
    parser.add_argument('--review-notes', type=Path)
    args = parser.parse_args()
    if args.render_report:
        report = json.loads(args.render_report.read_text())
        root = Path(report['project'])
        files, _ = inventory(root)
        if index_fingerprint(root) != report['index_fingerprint'] or fingerprint(root, files) != report['fingerprint'] or changes(root) != report['changes'] or git(root, 'rev-parse', 'HEAD').strip() != report['head']:
            raise ValueError('Repository changed since the checks. Refresh evidence before rendering.')
        output = args.render_report.parent
    else:
        root = args.project.resolve()
        if tomllib.loads((root/'pyproject.toml').read_text()).get('project', {}).get('name') != 'airtruck':
            raise ValueError('This checker targets the Airtruck project')
        if Path(git(root, 'rev-parse', '--show-toplevel').strip()).resolve() != root:
            raise ValueError('--project must identify the repository root')
        output = Path(tempfile.mkdtemp(prefix='airtruck-preflight-'))
        files, excluded = inventory(root)
        before = fingerprint(root, files)
        pending = changes(root)
        before_index = index_fingerprint(root)
        findings, documents, imports = inspect(root, files)
        checks = [
            command(root, output, 'working-whitespace', ['git', 'diff', '--check']),
            command(root, output, 'staged-whitespace', ['git', 'diff', '--cached', '--check']),
            command(root, output, 'lockfile', ['uv', 'lock', '--check']),
            command(root, output, 'pytest', ['uv', 'run', '--locked', 'pytest', '-q']),
            command(root, output, 'mypy', ['uv', 'run', '--locked', 'mypy', '--no-incremental', '--cache-dir', str(output/'mypy-cache')]),
        ]
        if (root/'requirements.txt').exists():
            checks.append(command(root, output, 'requirements-export', ['uv', 'export', '--locked', '--no-hashes', '--output-file', str(output/'requirements.expected.txt')]))
            if checks[-1]['status'] == 'pass':
                normalize = lambda text: '\n'.join(line.rstrip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#'))
                if normalize((root/'requirements.txt').read_text()) != normalize((output/'requirements.expected.txt').read_text()):
                    findings.append(dict(severity='error', path='requirements.txt', message='Compatibility export differs from the locked dependency export.'))
        for p in sorted(root.glob('scripts/**/*.command')):
            checks.append(command(root, output, 'shell-'+p.stem, ['zsh', '-n', str(p)]))
            if not os.access(p, os.X_OK):
                findings.append(dict(severity='error', path=str(p.relative_to(root)), message='Launcher is not executable'))
        after_files, _ = inventory(root)
        after = fingerprint(root, after_files)
        if before != after or pending != changes(root) or before_index != index_fingerprint(root):
            findings.append(dict(severity='error', path='working tree', message='Files or index changed during checks; inspect and rerun for a stable snapshot.'))
        if (root/'MUJOCO_LOG.TXT').exists() and not any(f['path']=='MUJOCO_LOG.TXT' for f in findings):
            findings.append(dict(severity='error', path='MUJOCO_LOG.TXT', message='New MuJoCo diagnostic log generated during checks; inspect it.'))
        report = dict(project=str(root), timestamp=datetime.now(timezone.utc).isoformat(),
                      head=git(root, 'rev-parse', 'HEAD').strip(), fingerprint=after, index_fingerprint=index_fingerprint(root),
                      files=after_files, excluded=excluded, changes=changes(root),
                      findings=findings, documents=documents, imports=imports, checks=checks)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    notes = json.loads(args.review_notes.read_text()) if args.review_notes else None
    if notes is not None:
        validate_notes(notes)
    (output/'report.html').write_text(render(report, notes))
    print(json.dumps({'report': str(output/'report.json'), 'visual': str(output/'report.html'),
                      'checks': {c['name']: c['status'] for c in report['checks']},
                      'findings': report['findings'], 'semantic_review': 'supplied' if notes else 'pending'}, indent=2))
    all_findings = report['findings'] + (notes or {}).get('findings', [])
    return int(any(c['status'] != 'pass' for c in report['checks']) or any(f['severity']=='error' for f in all_findings))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'Preflight could not complete: {error}', file=sys.stderr)
        raise SystemExit(2)
