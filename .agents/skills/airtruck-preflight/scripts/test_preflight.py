"""Behavior checks for the auditor, using isolated synthetic repositories."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

# Load the sibling by path under both direct unittest execution and pytest's
# importlib mode, without adding this tooling directory to runtime sys.path.
spec = importlib.util.spec_from_file_location(
    "airtruck_preflight_tests_subject", Path(__file__).with_name("preflight.py")
)
assert spec is not None and spec.loader is not None
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        for name in ('AGENTS.md', 'requirements.txt', 'uv.lock'):
            (self.root/name).write_text('')
        (self.root/'pyproject.toml').write_text('[project]\nname="airtruck"\n')
        (self.root/'pyrightconfig.json').write_text('{}')

    def inspect(self):
        files, _ = preflight.inventory(self.root)
        return preflight.inspect(self.root, files)[0]

    def test_missing_optional_export_is_allowed(self):
        (self.root/'requirements.txt').unlink()
        self.assertFalse(any(f['path']=='requirements.txt' for f in self.inspect()))

    def test_standard_library_import_is_not_confused_with_local_module(self):
        folder = self.root/'airtruck/testing_util'
        folder.mkdir(parents=True)
        (folder/'types.py').write_text('from types import SimpleNamespace\n')
        (self.root/'airtruck/constants.py').write_text('')
        (self.root/'airtruck/main.py').write_text('from constants import *\n')
        findings = self.inspect()
        self.assertEqual([f['path'] for f in findings], ['airtruck/main.py:1'])

    def test_bad_json_and_broken_documentation_link(self):
        (self.root/'pyrightconfig.json').write_text('{"x":1,"x":2}')
        (self.root/'AGENTS.md').write_text('[broken](docs/missing.md)')
        self.assertEqual(len([f for f in self.inspect() if f['severity']=='error']), 2)

    def test_symlinks_are_not_followed(self):
        (self.root/'outside').symlink_to('/tmp', target_is_directory=True)
        files, _ = preflight.inventory(self.root)
        self.assertEqual(len([f for f in files if f['symlink']]), 1)
        self.assertFalse(any(f['path'].startswith('outside/') for f in files))

    def test_index_only_changes_affect_snapshot(self):
        def git(*args):
            subprocess.run(['git', *args], cwd=self.root, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        git('init')
        git('add', '.')
        before = preflight.index_fingerprint(self.root)
        (self.root/'AGENTS.md').write_text('changed')
        git('add', 'AGENTS.md')
        self.assertNotEqual(before, preflight.index_fingerprint(self.root))

    def test_html_escapes_untrusted_notes_and_keeps_review_pending(self):
        report = dict(timestamp='now', project='repo', head='a'*40, documents=['AGENTS.md'],
                      findings=[], checks=[], files=[], changes=[], imports=[], excluded=[])
        page = preflight.render(report, {'summary': '<script>alert(1)</script>'})
        self.assertNotIn('<script>alert(1)</script>', page)
        self.assertIn('&lt;script&gt;', page)
        self.assertIn('Manual review incomplete', page)


if __name__ == '__main__':
    unittest.main()
