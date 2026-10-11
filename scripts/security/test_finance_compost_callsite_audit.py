#!/usr/bin/env python3
"""Unit tests for finance_compost_callsite_audit.py."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from finance_compost_callsite_audit import (
    MIRRORED_PAIRS,
    audit_repository,
    compost_callsite_issues,
)


SAFE = r'''
fn deliver() {
    let _ = call(
        CallTargetCell::Local,
        ZomeName::from("treasury"),
        FunctionName::from("receive_compost"),
        None,
        ReceiveCompostPayload {
            commons_pool_id: "global-fee-pool".into(),
            amount: 42,
            source_member_did: sender.clone(),
            effect_id: Some(stable_effect_id.to_string()),
        },
    );
}
'''

UNKEYED = r'''
fn deliver() {
    let _ = call(
        CallTargetCell::Local,
        ZomeName::from("treasury"),
        FunctionName::from("receive_compost"),
        None,
        ReceiveCompostPayload {
            commons_pool_id: "global-fee-pool".into(),
            amount: 42,
            source_member_did: sender.clone(),
        },
    );
}
'''

COMMENT_ONLY = r'''
// call(CallTargetCell::Local, ZomeName::from("treasury"),
//      FunctionName::from("receive_compost"), None, ReceiveCompostPayload {});
fn ordinary() {}
'''


class CompostCallsiteAuditTests(unittest.TestCase):
    def test_keyed_call_passes(self) -> None:
        issues, count = compost_callsite_issues("safe.rs", SAFE)
        self.assertEqual(count, 1)
        self.assertEqual(issues, [])

    def test_unkeyed_call_fails(self) -> None:
        issues, count = compost_callsite_issues("unsafe.rs", UNKEYED)
        self.assertEqual(count, 1)
        self.assertEqual(len(issues), 1)
        self.assertIn("effect_id: Some", issues[0])

    def test_comment_only_reference_is_ignored(self) -> None:
        issues, count = compost_callsite_issues("comment.rs", COMMENT_ONLY)
        self.assertEqual(count, 0)
        self.assertEqual(issues, [])

    def test_string_with_parentheses_does_not_break_balancing(self) -> None:
        source = SAFE.replace('"global-fee-pool"', '"pool(name(with(parens)))"')
        issues, count = compost_callsite_issues("nested.rs", source)
        self.assertEqual(count, 1)
        self.assertEqual(issues, [])

    def test_marker_inside_raw_string_is_ignored(self) -> None:
        source = 'const EXAMPLE: &str = r#"FunctionName::from("receive_compost")"#;\\n'
        issues, count = compost_callsite_issues("literal.rs", source)
        self.assertEqual(count, 0)
        self.assertEqual(issues, [])

    def test_rust_lifetime_does_not_break_call_scanning(self) -> None:
        source = "fn lifetime_example<'a>() { let _: Option<&'a str> = None; }\\n" + SAFE
        issues, count = compost_callsite_issues("lifetime.rs", source)
        self.assertEqual(count, 1)
        self.assertEqual(issues, [])

    def test_repository_audit_detects_mirror_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for left_rel, right_rel in MIRRORED_PAIRS:
                left = root / left_rel
                right = root / right_rel
                left.parent.mkdir(parents=True, exist_ok=True)
                right.parent.mkdir(parents=True, exist_ok=True)
                left.write_text("same\n", encoding="utf-8")
                right.write_text("same\n", encoding="utf-8")
            (root / "fixture.rs").write_text(SAFE, encoding="utf-8")
            issues, total, mirrored = audit_repository(root)
            self.assertEqual(issues, [])
            self.assertEqual(total, 1)
            self.assertEqual(len(mirrored), len(MIRRORED_PAIRS))

            changed = root / MIRRORED_PAIRS[0][1]
            changed.write_text("different\n", encoding="utf-8")
            issues, _, _ = audit_repository(root)
            self.assertTrue(any("not byte-identical" in issue for issue in issues))


if __name__ == "__main__":
    unittest.main()
