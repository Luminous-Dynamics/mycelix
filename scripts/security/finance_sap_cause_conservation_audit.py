#!/usr/bin/env python3
"""Fail-closed static contract for SAP cause-bound debit-capacity wiring.

This audit proves only the current AC-153 claim ceiling:
exact predecessor + explicit cause binding + deterministic demurrage accounting +
bounded causative-debit coverage on the covered SAP/PaymentChannel paths.

It deliberately does not claim exact monetary conservation; AC-153B owns that
stronger theorem.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
TREES = [ROOT / "mycelix-finance", ROOT / "mycelix-workspace/mycelix-finance"]

@dataclass(frozen=True)
class FunctionSpan:
    name: str
    signature_start: int
    body_start: int
    body_end: int

def _mask_range(chars: list[str], start: int, end: int) -> None:
    for i in range(start, min(end, len(chars))):
        if chars[i] != "\n":
            chars[i] = " "

def _raw_string_end(source: str, start: int) -> int | None:
    i = start
    if source.startswith("br", i) or source.startswith("rb", i):
        i += 2
    elif source.startswith("r", i):
        i += 1
    else:
        return None
    hashes = 0
    while i < len(source) and source[i] == "#":
        hashes += 1
        i += 1
    if i >= len(source) or source[i] != '"':
        return None
    terminator = '"' + ("#" * hashes)
    close = source.find(terminator, i + 1)
    return len(source) if close < 0 else close + len(terminator)

def mask_rust_noncode(source: str) -> str:
    """Mask comments/literals while preserving newlines and code offsets."""
    chars = list(source)
    i = 0
    n = len(source)
    while i < n:
        if source.startswith("//", i):
            j = source.find("\n", i + 2)
            j = n if j < 0 else j
            _mask_range(chars, i, j)
            i = j
            continue

        if source.startswith("/*", i):
            depth = 1
            j = i + 2
            while j < n and depth:
                if source.startswith("/*", j):
                    depth += 1
                    j += 2
                elif source.startswith("*/", j):
                    depth -= 1
                    j += 2
                else:
                    j += 1
            _mask_range(chars, i, j)
            i = j
            continue

        raw_end = None
        if source.startswith("r", i) or source.startswith("br", i) or source.startswith("rb", i):
            raw_end = _raw_string_end(source, i)
        if raw_end is not None:
            _mask_range(chars, i, raw_end)
            i = raw_end
            continue

        if source[i] == '"' or (
            source[i] in "bBcC" and i + 1 < n and source[i + 1] == '"'
        ):
            start = i
            if source[i] != '"':
                i += 1
            i += 1
            while i < n:
                if source[i] == "\\":
                    i += 2
                    continue
                if source[i] == '"':
                    i += 1
                    break
                i += 1
            _mask_range(chars, start, i)
            continue

        # Mask char literals, but leave lifetime syntax such as 'a alone.
        if source[i] == "'":
            j = i + 1
            if j < n and source[j] == "\\":
                j += 2
            elif j < n and source[j] not in "\n\r":
                j += 1
            if j < n and source[j] == "'":
                _mask_range(chars, i, j + 1)
                i = j + 1
                continue

        i += 1
    return "".join(chars)

FN_RE = re.compile(
    r"\b(?:(?:pub)(?:\s*\([^)]*\))?\s+)?(?:async\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)\s*\("
)

def extract_functions(source: str) -> dict[str, FunctionSpan]:
    """Find Rust function bodies using masked source and balanced braces."""
    masked = mask_rust_noncode(source)
    out: dict[str, FunctionSpan] = {}
    for match in FN_RE.finditer(masked):
        open_brace = masked.find("{", match.end())
        if open_brace < 0:
            continue
        depth = 0
        for i in range(open_brace, len(masked)):
            if masked[i] == "{":
                depth += 1
            elif masked[i] == "}":
                depth -= 1
                if depth == 0:
                    out[match.group(1)] = FunctionSpan(
                        match.group(1), match.start(), open_brace, i + 1
                    )
                    break
    return out

def _body(masked_source: str, fn: FunctionSpan) -> str:
    return masked_source[fn.body_start:fn.body_end]

def _line(source: str, offset: int) -> int:
    return source[:offset].count("\n") + 1

def _check(
    assertions: list[dict],
    source: str,
    masked: str,
    functions: dict[str, FunctionSpan],
    fn_name: str,
    assertion_id: str,
    predicate_class: str,
    pattern: str,
) -> None:
    fn = functions.get(fn_name)
    result = {
        "id": assertion_id,
        "predicate_class": predicate_class,
        "function": fn_name,
    }
    if fn is None:
        result.update(status="FAIL", reason="function_not_found")
    elif re.search(pattern, _body(masked, fn), re.MULTILINE):
        result.update(status="PASS", line=_line(source, fn.signature_start))
    else:
        result.update(
            status="FAIL",
            line=_line(source, fn.signature_start),
            reason="required_executable_predicate_not_found",
        )
    assertions.append(result)

def audit_tree(tree: Path) -> tuple[bool, dict]:
    integrity = tree / "zomes/payments/integrity/src/lib.rs"
    coord = tree / "zomes/payments/coordinator/src/lib.rs"
    errors: list[str] = []
    assertions: list[dict] = []

    if not integrity.exists() or not coord.exists():
        return False, {"tree": str(tree), "errors": ["missing Payments sources"]}

    isrc = integrity.read_text(encoding="utf-8")
    csrc = coord.read_text(encoding="utf-8")
    imasked = mask_rust_noncode(isrc)
    cmasked = mask_rust_noncode(csrc)
    ifns = extract_functions(isrc)
    cfns = extract_functions(csrc)

    checks = [
        (cfns, cmasked, csrc, "credit_sap", "sap.positive.amount",
         r"input\.amount\s*==\s*0", "reject_zero_credit"),
        (cfns, cmasked, csrc, "credit_sap", "sap.positive.cause",
         r"input\.justified_by\.is_none\s*\(\)", "require_explicit_cause"),
        (cfns, cmasked, csrc, "credit_sap", "sap.root.no_autocreate",
         r"find_sap_balance_record\s*\(\s*&input\.member_did", "require_existing_root"),
        (ifns, imasked, isrc, "validate_update_sap_balance", "sap.predecessor",
         r"must_get_valid_record\s*\(\s*action\.original_action_address", "exact_predecessor"),
        (ifns, imasked, isrc, "validate_update_sap_balance", "sap.cause.prev_action",
         r"action\.prev_action\s*!=\s*cause_hash", "immediate_cause_binding"),
        (ifns, imasked, isrc, "validate_update_sap_balance", "sap.demurrage",
         r"compute_demurrage_with_exemption", "deterministic_demurrage"),
        (ifns, imasked, isrc, "validate_update_sap_balance", "sap.debit.capacity",
         r"if\s+debited\s*<\s*credited_amount", "bounded_debit_coverage"),
        (cfns, cmasked, csrc, "find_sap_balance_record", "sap.strict.lookup",
         r"follow_update_chain_strict\s*\(", "strict_root_resolution"),
        (cfns, cmasked, csrc, "transfer_sap", "sap.transfer.cause",
         r"justified_by\s*:\s*Some\s*\(\s*debit_record\.action_address\(\)\.clone\(\)\s*\)", "transfer_cause_wiring"),
        (cfns, cmasked, csrc, "send_payment", "sap.payment.cause",
         r"justified_by\s*:\s*Some\s*\(\s*debit_record\.action_address\(\)\.clone\(\)\s*\)", "payment_cause_wiring"),
        (ifns, imasked, isrc, "validate_update_payment_channel", "channel.conservation",
         r"old_total\s*!=\s*new_total", "aggregate_conservation"),
        (ifns, imasked, isrc, "validate_update_payment_channel", "channel.participant",
         r"author_did\s*!=\s*channel\.party_a\s*&&\s*author_did\s*!=\s*channel\.party_b", "participant_authorization"),
    ]
    for fns, masked, src, fn, pc, pat, aid in checks:
        _check(assertions, src, masked, fns, fn, aid, pc, pat)

    find_fn = cfns.get("find_sap_balance_record")
    if find_fn:
        body = _body(cmasked, find_fn)
        bad = bool(re.search(r"\bfollow_update_chain\s*\(", body))
        assertions.append({
            "id": "strict.no_generic_lookup",
            "predicate_class": "sap.strict.lookup",
            "function": "find_sap_balance_record",
            "line": _line(csrc, find_fn.signature_start),
            "status": "FAIL" if bad else "PASS",
            **({"reason": "generic_non_strict_traversal_in_authoritative_lookup"} if bad else {}),
        })

    exact = False
    fn = ifns.get("validate_update_sap_balance")
    if fn:
        exact = bool(re.search(r"\bdebited\s*==\s*credited_amount\b", _body(imasked, fn)))

    for a in assertions:
        if a["status"] != "PASS":
            errors.append(f'{tree}: {a["id"]}: {a.get("reason", "failed")}')

    report = {
        "schema": "mycelix.finance.ac-153.sap-cause-capacity-audit.v2",
        "claim_ceiling": "cause_bound_debit_capacity",
        "exact_conservation_proven": exact,
        "tree": str(tree),
        "assertions": assertions,
        "errors": errors,
    }
    return not errors, report

def audit_all() -> tuple[bool, dict]:
    reports = []
    ok = True
    for tree in TREES:
        tree_ok, report = audit_tree(tree)
        ok = ok and tree_ok
        reports.append(report)
    return ok, {
        "schema": "mycelix.finance.ac-153.sap-cause-capacity-audit.v2",
        "claim_ceiling": "cause_bound_debit_capacity",
        "exact_conservation_proven": all(
            r.get("exact_conservation_proven", False) for r in reports
        ),
        "trees": reports,
    }

def main() -> int:
    ok, report = audit_all()
    print(json.dumps(report, sort_keys=True, indent=2))
    print("SAP_CAUSE_CAPACITY_AUDIT=" + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
