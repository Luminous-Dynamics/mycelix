#!/usr/bin/env python3
"""Fail-closed static guard for selected Mycelix Finance balance mutations.

Scope ceiling: static source-pattern presence/order only. This is not a proof of
Holochain transaction atomicity, distributed serializability, runtime authority,
or SweetConductor correctness. Those require the corresponding runtime tests.
"""

from __future__ import annotations

import hashlib
import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Contract:
    path: str
    function: str
    ordered_markers: tuple[str, ...]


CONTRACTS = (
    Contract(
        "mycelix-finance/zomes/currency-mint/coordinator/src/helpers.rs",
        "mutate_balance",
        (
            "for attempt in 0..MAX_BALANCE_RETRIES",
            "follow_update_chain(original_hash.clone())?",
            "f(&mut bal);",
            "update_entry(record.action_address().clone(), &bal)?;",
            "let verify_record = follow_update_chain(original_hash.clone())?;",
            "actual.balance == expected.balance",
            "actual.exchange_count == expected.exchange_count",
            "failed after {} retries due to concurrent modifications",
        ),
    ),
    Contract(
        "mycelix-finance/zomes/tend/coordinator/src/lib.rs",
        "update_hearth_balance",
        (
            "for attempt in 0..MAX_BALANCE_RETRIES",
            "follow_update_chain(original_hash.clone())?",
            "update_entry(record.action_address().clone(), &bal)?;",
            "let verify_record = follow_update_chain(original_hash.clone())?;",
            "actual.balance == expected_balance",
            "actual.exchange_count == expected_count",
            "failed after {} retries due to concurrent modifications",
        ),
    ),
    Contract(
        "mycelix-finance/zomes/tend/coordinator/src/lib.rs",
        "update_balance_after_exchange",
        (
            "for attempt in 0..MAX_BALANCE_RETRIES",
            "follow_update_chain(original_hash.clone())?",
            "update_entry(record.action_address().clone(), &balance)?;",
            "let verify_record = follow_update_chain(original_hash.clone())?;",
            "actual.balance == expected_balance",
            "actual.exchange_count == expected_count",
            "failed after {} retries due to concurrent modifications",
        ),
    ),
    Contract(
        "mycelix-finance/zomes/payments/coordinator/src/lib.rs",
        "credit_sap",
        (
            "for attempt in 0..MAX_SAP_RETRIES",
            "let (record, bal) = get_sap_balance_inner(&input.member_did)?;",
            "update_entry(",
            "record.action_address().clone(),",
            "find_sap_balance_record(&input.member_did)?",
            "actual.balance == expected_balance",
            "failed after {} retries due to concurrent modifications",
        ),
    ),
    Contract(
        "mycelix-finance/zomes/payments/coordinator/src/lib.rs",
        "debit_sap",
        (
            "for attempt in 0..MAX_SAP_RETRIES",
            "let (record, bal) = get_sap_balance_inner(&input.member_did)?;",
            "if input.amount > effective",
            "update_entry(",
            "record.action_address().clone(),",
            "find_sap_balance_record(&input.member_did)?",
            "actual.balance == expected_balance",
            "failed after {} retries due to concurrent modifications",
        ),
    ),
)


class AuditFailure(RuntimeError):
    pass


def extract_function(source: str, name: str) -> str:
    """Extract a top-level Rust function body using its unindented closing brace."""
    match = re.search(r"\bfn\s+" + re.escape(name) + r"\s*\(", source)
    if match is None:
        raise AuditFailure(f"missing Rust function: {name}")
    end = source.find("\n}\n", match.start())
    if end < 0:
        raise AuditFailure(f"could not locate closing brace for Rust function: {name}")
    body = source[match.start(): end + 2]
    if "{" not in body:
        raise AuditFailure(f"function body missing for Rust function: {name}")
    return body


def assert_ordered_markers(label: str, function_body: str, markers: tuple[str, ...]) -> None:
    cursor = 0
    for marker in markers:
        found = function_body.find(marker, cursor)
        if found < 0:
            raise AuditFailure(f"{label}: missing or out-of-order source invariant {marker!r}")
        cursor = found + len(marker)


def negative_control() -> None:
    """Prove the checker rejects a deliberately incomplete mutation routine."""
    markers = (
        "for attempt in 0..MAX_RETRIES",
        "update_entry(record.action_address().clone(), &balance)?;",
        "read_back_from_original_anchor()?",
        "actual.balance == expected_balance",
    )
    valid = """fn example() {
    for attempt in 0..MAX_RETRIES {
        update_entry(record.action_address().clone(), &balance)?;
        read_back_from_original_anchor()?;
        if actual.balance == expected_balance { return; }
    }
}
"""
    invalid = valid.replace("update_entry(record.action_address().clone(), &balance)?;\n", "")
    assert_ordered_markers("self-test-valid", extract_function(valid, "example"), markers)
    try:
        assert_ordered_markers("self-test-invalid", extract_function(invalid, "example"), markers)
    except AuditFailure:
        return
    raise AuditFailure("negative control failed: missing update operation was accepted")


def main() -> int:
    try:
        negative_control()
        digests: dict[str, str] = {}
        for contract in CONTRACTS:
            path = ROOT / contract.path
            if not path.is_file():
                raise AuditFailure(f"required source file is missing: {contract.path}")
            source_bytes = path.read_bytes()
            source = source_bytes.decode("utf-8")
            body = extract_function(source, contract.function)
            label = f"{contract.path}::{contract.function}"
            assert_ordered_markers(label, body, contract.ordered_markers)
            digests[contract.path] = hashlib.sha256(source_bytes).hexdigest()

        print("FINANCE_UPDATE_PREDECESSOR_STATIC_AUDIT_PASS")
        print(f"checked_functions={len(CONTRACTS)}")
        print(f"unique_source_files={len(digests)}")
        for path, digest in sorted(digests.items()):
            print(f"source_sha256[{path}]={digest}")
        print("negative_control=PASS")
        print("claim_ceiling=StaticSourceInvariantPresenceOnly")
        return 0
    except (AuditFailure, OSError, UnicodeDecodeError) as error:
        print(f"FINANCE_UPDATE_PREDECESSOR_STATIC_AUDIT_FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
