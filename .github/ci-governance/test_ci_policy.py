#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("check_ci_policy", HERE / "check_ci_policy.py")
mod = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = mod
SPEC.loader.exec_module(mod)

POLICY = {
    "top_level_permissions": {"contents": "read"},
    "critical_jobs": {
        "changes": {
            "contents": "read",
            "pull_requests": "read",
            "require_checkout_no_persist": True,
            "immutable_actions": [
                "actions/checkout@11d5960a326750d5838078e36cf38b85af677262",
                "dorny/paths-filter@0e4a8c6effa4802afeda77dc8d303f8176d7dfad",
            ],
        },
        "ci_pass": {"forbid_external_actions": True},
    },
}

GOOD = """\
name: fixture
permissions:
  contents: read

jobs:
  changes:
    permissions:
      contents: read
      pull-requests: read
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@11d5960a326750d5838078e36cf38b85af677262
        with:
          persist-credentials: false
      - uses: dorny/paths-filter@0e4a8c6effa4802afeda77dc8d303f8176d7dfad
  build:
    runs-on: ubuntu-latest
    steps:
      - run: echo build
  ci-pass:
    runs-on: ubuntu-latest
    steps:
      - run: echo pass
"""


class PolicyTests(unittest.TestCase):
    def assert_rejected(self, text: str, needle: str) -> None:
        with self.assertRaises(mod.PolicyError) as ctx:
            mod.validate(text, POLICY)
        self.assertIn(needle, str(ctx.exception))

    def test_positive_fixture(self):
        report = mod.validate(GOOD, POLICY)
        self.assertEqual(report[0], "CI_GOV_001E_POLICY_PASS")

    def test_missing_top_level_permissions(self):
        self.assert_rejected(
            GOOD.replace("permissions:\n  contents: read\n\n", "", 1),
            "top-level permissions",
        )

    def test_top_level_write_rejected(self):
        self.assert_rejected(
            GOOD.replace("permissions:\n  contents: read", "permissions:\n  contents: write", 1),
            "top-level permissions mismatch",
        )

    def test_job_write_rejected(self):
        bad = GOOD.replace(
            "  build:\n    runs-on:",
            "  build:\n    permissions:\n      contents: write\n    runs-on:",
        )
        self.assert_rejected(bad, "forbidden write permissions")

    def test_changes_pull_request_write_rejected(self):
        self.assert_rejected(
            GOOD.replace("pull-requests: read", "pull-requests: write", 1),
            "forbidden write permissions",
        )

    def test_mutable_checkout_rejected(self):
        self.assert_rejected(
            GOOD.replace(
                "actions/checkout@11d5960a326750d5838078e36cf38b85af677262",
                "actions/checkout@v4",
                1,
            ),
            "mutable action refs",
        )

    def test_mutable_paths_filter_rejected(self):
        self.assert_rejected(
            GOOD.replace(
                "dorny/paths-filter@0e4a8c6effa4802afeda77dc8d303f8176d7dfad",
                "dorny/paths-filter@v3",
                1,
            ),
            "mutable action refs",
        )

    def test_checkout_persist_credentials_required(self):
        self.assert_rejected(
            GOOD.replace("          persist-credentials: false\n", "", 1),
            "persist-credentials: false",
        )

    def test_unexpected_changes_action_rejected(self):
        bad = GOOD.replace(
            "      - uses: dorny/paths-filter@0e4a8c6effa4802afeda77dc8d303f8176d7dfad",
            "      - uses: actions/cache@0123456789abcdef0123456789abcdef01234567\n"
            "      - uses: dorny/paths-filter@0e4a8c6effa4802afeda77dc8d303f8176d7dfad",
        )
        self.assert_rejected(bad, "action sequence mismatch")

    def test_ci_pass_must_not_execute_external_actions(self):
        bad = GOOD.replace(
            "  ci-pass:\n    runs-on: ubuntu-latest\n    steps:\n",
            "  ci-pass:\n    runs-on: ubuntu-latest\n    steps:\n"
            "      - uses: actions/checkout@11d5960a326750d5838078e36cf38b85af677262\n",
        )
        self.assert_rejected(bad, "external actions forbidden")


if __name__ == "__main__":
    unittest.main()
