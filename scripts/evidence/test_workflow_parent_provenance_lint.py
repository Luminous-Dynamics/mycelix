#!/usr/bin/env python3
from __future__ import annotations

import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))
import workflow_parent_provenance_lint as lint


def wf(steps: str, extra_job: str = "") -> str:
    return f"""name: test
on: pull_request
jobs:
  check:
    runs-on: ubuntu-latest
    steps:
{steps}{extra_job}
"""


def checkout(depth: str | None = None) -> str:
    d = "" if depth is None else f"\n        fetch-depth: {depth}"
    return f"""      - uses: actions/checkout@deadbeef
        with:
          persist-credentials: false{d}
"""


def run(body: str, name: str = "probe") -> str:
    indented = "\n".join("          " + line for line in body.splitlines())
    return f"""      - name: {name}
        run: |
{indented}
"""


class ProvenanceLintTests(unittest.TestCase):
    def report(self, text: str):
        return lint.scan_text(text, "x.yml")

    def one(self, text: str):
        r = self.report(text)
        self.assertEqual(len(r["findings"]), 1, r)
        return r["findings"][0]

    def test_head_parent_omitted_depth_fails(self):
        f = self.one(wf(checkout() + run('test "$(git rev-parse HEAD^)" = "0123456789012345678901234567890123456789"')))
        self.assertFalse(f["passed"]); self.assertEqual(f["finding_code"], "ANCESTRY_DEPTH_INSUFFICIENT")

    def test_head_parent_depth_one_fails(self):
        f = self.one(wf(checkout("1") + run('git rev-parse HEAD^')))
        self.assertFalse(f["passed"])

    def test_head_parent_depth_two_passes(self):
        f = self.one(wf(checkout("2") + run('git rev-parse HEAD^')))
        self.assertTrue(f["passed"]); self.assertEqual(f["required_checkout_depth"], 2)

    def test_head_tilde_two_depth_two_fails(self):
        f = self.one(wf(checkout("2") + run('git rev-parse HEAD~2')))
        self.assertFalse(f["passed"]); self.assertEqual(f["required_checkout_depth"], 3)

    def test_head_tilde_two_depth_three_passes(self):
        self.assertTrue(self.one(wf(checkout("3") + run('git rev-parse HEAD~2')))["passed"])

    def test_full_history_passes(self):
        self.assertTrue(self.one(wf(checkout("0") + run('git merge-base HEAD origin/main')))["passed"])

    def test_deepen_one_after_depth_one_passes_head_parent(self):
        text = wf(checkout("1") + run('git fetch --deepen=1 origin HEAD\ngit rev-parse HEAD^'))
        r = self.report(text); self.assertEqual(len(r["findings"]), 1); self.assertTrue(r["findings"][0]["passed"])

    def test_exact_parent_object_fetch_does_not_unshallow_head(self):
        sha = "0123456789012345678901234567890123456789"
        text = wf(checkout("1") + run(f'git fetch origin {sha}\ntest "$(git rev-parse HEAD^)" = "{sha}"'))
        self.assertFalse(self.one(text)["passed"])

    def test_exact_object_fetch_can_satisfy_exact_cat_file(self):
        sha = "0123456789012345678901234567890123456789"
        text = wf(checkout("1") + run(f'git fetch origin {sha}\ngit cat-file -e {sha}^{{commit}}'))
        self.assertTrue(self.one(text)["passed"])

    def test_unrelated_object_fetch_fails_exact_object(self):
        a = "0123456789012345678901234567890123456789"
        b = "abcdefabcdefabcdefabcdefabcdefabcdefabcd"
        text = wf(checkout("1") + run(f'git fetch origin {a}\ngit cat-file -e {b}^{{commit}}'))
        self.assertFalse(self.one(text)["passed"])

    def test_ancestry_before_deepen_fails_even_if_later_deepened(self):
        text = wf(checkout("1") + run('git rev-parse HEAD^\ngit fetch --deepen=1 origin HEAD'))
        self.assertFalse(self.one(text)["passed"])

    def test_deep_checkout_in_other_job_does_not_help(self):
        extra = """  other:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@deadbeef
        with:
          fetch-depth: 2
      - run: git rev-parse HEAD^
"""
        text = wf(checkout("1") + run('git rev-parse HEAD^'), extra)
        r = self.report(text); self.assertEqual(len(r["findings"]), 2)
        by_job = {x["job_id"]: x for x in r["findings"]}
        self.assertFalse(by_job["check"]["passed"]); self.assertTrue(by_job["other"]["passed"])

    def test_dynamic_depth_fails_closed(self):
        f = self.one(wf(checkout("${{ inputs.depth }}") + run('git rev-parse HEAD^')))
        self.assertFalse(f["passed"]); self.assertEqual(f["finding_code"], "DYNAMIC_FETCH_DEPTH_UNPROVEN")

    def test_no_checkout_fails(self):
        f = self.one(wf(run('git rev-parse HEAD^')))
        self.assertFalse(f["passed"]); self.assertEqual(f["finding_code"], "PARENT_OBJECT_AVAILABILITY_UNPROVEN")

    def test_harmless_workflow_passes_with_no_findings(self):
        r = self.report(wf(checkout("1") + run('python3 -m unittest -v tests')))
        self.assertEqual(r["result"], "PASS"); self.assertEqual(r["findings"], [])

    def test_comments_and_echoed_documentation_ignored(self):
        text = wf(checkout("1") + run('# git rev-parse HEAD^\necho "git rev-parse HEAD^"\nprintf "%s\\n" "HEAD^"'))
        r = self.report(text); self.assertEqual(r["findings"], [])

    def test_multiline_shell_order_is_deterministic(self):
        text = wf(checkout("1") + run('git fetch --deepen=2 origin HEAD\ngit rev-parse HEAD~2\ngit rev-parse HEAD^'))
        r1 = self.report(text); r2 = self.report(text)
        self.assertEqual(r1, r2); self.assertTrue(all(x["passed"] for x in r1["findings"]))

    def test_unshallow_passes_unknown_ancestry_operation(self):
        text = wf(checkout("1") + run('git fetch --unshallow origin\ngit merge-base HEAD origin/main'))
        self.assertTrue(self.one(text)["passed"])


if __name__ == "__main__":
    unittest.main()
