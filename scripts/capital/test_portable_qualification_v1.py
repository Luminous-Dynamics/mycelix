from __future__ import annotations

import copy
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from portable_qualification_v1 import PortableQualificationError, SPEC_VERSION, build_bundle, verify_bundle


def cmd(*args, cwd):
    return subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=True).stdout.strip()


def canon(v):
    return json.dumps(v, sort_keys=True, separators=(",", ":")).encode()


def h(v):
    return hashlib.sha256(canon(v)).hexdigest()


class PortableTests(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory()
        self.r = Path(self.t.name)
        cmd("git", "init", "-q", cwd=self.r)
        cmd("git", "config", "user.email", "t@example.invalid", cwd=self.r)
        cmd("git", "config", "user.name", "t", cwd=self.r)
        case = {"profile": {"p": 1}, "events": [{"e": 1}]}
        receipt = {"ok": True}
        files = {
            "scripts/core.py": 'import argparse,json\nfrom pathlib import Path\np=argparse.ArgumentParser();p.add_argument("case",type=Path);p.add_argument("--receipt-out",type=Path);a=p.parse_args();a.receipt_out.write_text(json.dumps({"ok":True},sort_keys=True,indent=2)+"\\n")\n',
            "scripts/tests.py": 'import unittest\nclass T(unittest.TestCase):\n def test_ok(self): self.assertTrue(True)\nif __name__=="__main__": unittest.main()\n',
            "evidence/PROFILE.md": "profile\n",
            "evidence/case.json": json.dumps(case, sort_keys=True, indent=2) + "\n",
            "evidence/receipt.json": json.dumps(receipt, sort_keys=True, indent=2) + "\n",
            ".github/workflows/q.yml": "name: q\n",
        }
        for name, data in files.items():
            p = self.r / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(data)
        cmd("git", "add", ".", cwd=self.r); cmd("git", "commit", "-qm", "subject", cwd=self.r)
        self.subject = cmd("git", "rev-parse", "HEAD", cwd=self.r)
        port = self.r / "portable"; port.mkdir()
        (port / "NONCLAIMS.md").write_text("portable receipt is evidence, not self-authenticating PASS\n")
        self.spec = {
            "spec_version": SPEC_VERSION,
            "qualified_subject_sha": self.subject,
            "qualification_profile": "test",
            "source_files": sorted(files),
            "case_path": "evidence/case.json",
            "receipt_path": "evidence/receipt.json",
            "qualification_command": ["python3", "-m", "unittest", "-v", "scripts/tests.py"],
            "receipt_command": ["python3", "scripts/core.py", "{case}", "--receipt-out", "{receipt}"],
            "semantic_expectations": {"case_canonical_sha256": h(case), "profile_sha256": h(case["profile"]), "event_history_sha256": h(case["events"]), "event_chain_tip_sha256": h(case["events"][-1])},
            "hosted_run": {"repository": "x/y", "workflow_path": ".github/workflows/q.yml", "run_id": 1, "run_attempt": 1, "head_sha": self.subject, "conclusion": "success"},
            "pass_scope": "test only",
            "nonclaims_path": "portable/NONCLAIMS.md",
        }
        self.sp = port / "spec.json"; self.sp.write_text(json.dumps(self.spec, sort_keys=True, indent=2) + "\n")
        cmd("git", "add", "portable", cwd=self.r); cmd("git", "commit", "-qm", "child", cwd=self.r)
        self.b = self.r / "bundle"

    def tearDown(self):
        self.t.cleanup()

    def build(self):
        build_bundle(self.sp, self.r, self.b)

    def test_build_and_replay(self):
        self.build(); verify_bundle(self.sp, self.r, self.b)

    def test_extra_member_fails(self):
        self.build(); (self.b / "authority.json").write_text("{}")
        with self.assertRaisesRegex(PortableQualificationError, "member mismatch"):
            verify_bundle(self.sp, self.r, self.b, False)

    def test_receipt_tamper_fails(self):
        self.build(); (self.b / "receipt.json").write_text("{}\n")
        with self.assertRaisesRegex(PortableQualificationError, "receipt mismatch"):
            verify_bundle(self.sp, self.r, self.b, False)

    def test_source_drift_fails(self):
        self.build(); (self.r / "scripts/core.py").write_text("print('changed')\n")
        with self.assertRaisesRegex(PortableQualificationError, "source drift"):
            verify_bundle(self.sp, self.r, self.b, False)

    def test_semantic_mutation_fails(self):
        bad = copy.deepcopy(self.spec); bad["semantic_expectations"]["profile_sha256"] = "0" * 64
        p = self.r / "portable/bad.json"; p.write_text(json.dumps(bad))
        with self.assertRaisesRegex(PortableQualificationError, "semantic commitments mismatch"):
            build_bundle(p, self.r, self.b)

    def test_hosted_success_cannot_replace_replay(self):
        self.build(); (self.r / "scripts/tests.py").write_text("raise SystemExit(1)\n")
        with self.assertRaisesRegex(PortableQualificationError, "source drift"):
            verify_bundle(self.sp, self.r, self.b)


if __name__ == "__main__":
    unittest.main()
