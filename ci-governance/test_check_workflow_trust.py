import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("checker", ROOT / "check_workflow_trust.py")
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)
POLICY = ROOT / "trust-policy-v0.1.json"
FIXTURES = ROOT / "fixtures"


class TrustPolicyTests(unittest.TestCase):
    def report(self, name):
        return checker.check(FIXTURES / name, POLICY)

    def test_positive_fixture(self):
        self.assertEqual(self.report("pass.yml")["status"], "PASS")

    def test_missing_default_permissions_fails(self):
        report = self.report("fail-missing-default-permissions.yml")
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any("workflow permissions" in x for x in report["findings"]))

    def test_write_permission_fails(self):
        report = self.report("fail-write-permission.yml")
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any("forbidden write permission" in x for x in report["findings"]))

    def test_mutable_action_fails(self):
        report = self.report("fail-mutable-action.yml")
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any("not pinned" in x for x in report["findings"]))

    def test_checkout_credentials_fails(self):
        report = self.report("fail-persisted-credentials.yml")
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any("persist-credentials" in x for x in report["findings"]))

    def test_yaml_anchor_fails_closed(self):
        with self.assertRaises(checker.PolicyError):
            checker.parse_workflow((FIXTURES / "fail-anchor.yml").read_text())

    def test_extra_read_permission_fails_exact_contract(self):
        text = (FIXTURES / "pass.yml").read_text().replace(
            "      pull-requests: read\n",
            "      pull-requests: read\n      packages: read\n",
            1,
        )
        tmp = ROOT / "fixtures" / "_tmp-extra-read.yml"
        try:
            tmp.write_text(text)
            report = checker.check(tmp, POLICY)
            self.assertEqual(report["status"], "FAIL")
            self.assertTrue(any("permissions must equal" in x for x in report["findings"]))
        finally:
            tmp.unlink(missing_ok=True)

    def test_duplicate_workflow_permissions_fail_closed(self):
        text = (FIXTURES / "pass.yml").read_text().replace(
            "jobs:\n",
            "permissions:\n  contents: read\njobs:\n",
            1,
        )
        with self.assertRaises(checker.PolicyError):
            checker.parse_workflow(text)


if __name__ == "__main__":
    unittest.main()
