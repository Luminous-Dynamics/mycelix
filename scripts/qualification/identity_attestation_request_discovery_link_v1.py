#!/usr/bin/env python3
"""Materialize append-only SubjectToRequest discovery-link integrity.

Qualification-only: edits the CI checkout after the request create/delete
candidate. The exact RegisterDeleteLink before-image must occur once.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / "mycelix-workspace/mycelix-identity/zomes/trust_credential/integrity/src/lib.rs"


def replace_once(text: str, before: str, after: str, label: str) -> str:
    count = text.count(before)
    if count != 1:
        raise SystemExit(
            f"ERROR: {label} before-image count is {count}, expected exactly 1; "
            "refuse stale/ambiguous integrity transformation"
        )
    return text.replace(before, after, 1)


text = LIB.read_text()

before = '''        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
'''
after = '''        FlatOp::RegisterDeleteLink {
            original_action,
            action,
            link_type,
            ..
        } => {
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the link creator can delete their links".into(),
                ));
            }

            // SubjectToRequest is a canonical discovery edge. Request state is
            // expressed by the typed lifecycle, so deleting this link must not
            // hide a still-existing request from ordinary subject discovery.
            if matches!(link_type, LinkTypes::SubjectToRequest) {
                return Ok(ValidateCallbackResult::Invalid(
                    "SubjectToRequest discovery links are append-only".into(),
                ));
            }

            Ok(ValidateCallbackResult::Valid)
        }
'''
text = replace_once(text, before, after, "SubjectToRequest append-only link rule")

start = text.index("FlatOp::RegisterDeleteLink {")
end = text.index("FlatOp::StoreRecord", start)
body = text[start:end]
required = [
    "link_type,",
    "Only the link creator can delete their links",
    "matches!(link_type, LinkTypes::SubjectToRequest)",
    "SubjectToRequest discovery links are append-only",
]
for needle in required:
    if needle not in body:
        raise SystemExit(f"ERROR: request discovery-link delete contract missing: {needle!r}")

LIB.write_text(text)
print("Materialized append-only SubjectToRequest discovery-link integrity.")
