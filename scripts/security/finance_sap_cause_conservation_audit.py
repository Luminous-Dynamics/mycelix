#!/usr/bin/env python3
"""Fail-closed static contract for SAP balance cause/conservation wiring."""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
TREES = [ROOT / "mycelix-finance", ROOT / "mycelix-workspace/mycelix-finance"]

errors = []

for tree in TREES:
    integrity = tree / "zomes/payments/integrity/src/lib.rs"
    coord = tree / "zomes/payments/coordinator/src/lib.rs"

    if not integrity.exists() or not coord.exists():
        errors.append(f"{tree}: missing Payments integrity/coordinator source")
        continue

    isrc = integrity.read_text(encoding="utf-8")
    csrc = coord.read_text(encoding="utf-8")

    required_integrity = (
        "validate_create_sap_balance",
        "action.prev_action",
        "justified_by",
        "compute_demurrage_with_exemption",
        "causative_debit_amount",
        "payment_channel_balances_conserved",
    )
    for token in required_integrity:
        if token not in isrc:
            errors.append(f"{integrity}: missing SAP cause requirement {token}")

    m = re.search(
        r"pub fn credit_sap\([^)]*\)[\s\S]*?(?=\n#\[hdk_extern\]|\nfn )",
        csrc,
    )
    if not m:
        errors.append(f"{coord}: cannot locate credit_sap function")
    else:
        body = m.group(0)
        for token in (
            "input.justified_by.is_none()",
            "SAP credit requires an explicit validated cause",
            "find_sap_balance_record(&input.member_did)?",
        ):
            if token not in body:
                errors.append(f"{coord}: credit_sap missing fail-closed requirement {token}")
        if "initialize_sap_balance(input.member_did" in body:
            errors.append(f"{coord}: credit_sap still auto-creates recipient balance roots")

    if not re.search(
        r"pub fn transfer_sap\([\s\S]*?credit_sap\(CreditSapInput\{?[\s\S]{0,1200}?justified_by\s*:\s*Some\(",
        csrc,
    ):
        errors.append(f"{coord}: transfer_sap missing debit-cause binding")

    if not re.search(
        r"pub fn send_payment\([\s\S]*?credit_sap\(CreditSapInput\{?[\s\S]{0,1600}?justified_by\s*:\s*Some\(",
        csrc,
    ):
        errors.append(f"{coord}: send_payment missing debit-cause binding")

print(f"Audited SAP cause wiring in {len(TREES)} Finance trees.")
if errors:
    print("SAP_CAUSE_CONSERVATION_AUDIT=FAIL")
    for error in errors:
        print("ERROR:", error)
    sys.exit(1)
print("SAP_CAUSE_CONSERVATION_AUDIT=PASS")
