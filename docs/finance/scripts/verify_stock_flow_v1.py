#!/usr/bin/env python3
"""Verify FIN-SYS stock-flow synthetic corpus v1."""

from __future__ import annotations

import hashlib
import json
import sys
from collections import defaultdict
from decimal import Decimal, InvalidOperation
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PUBLIC = ROOT / "fixtures" / "stock_flow_v1" / "public.json"
ORACLE = ROOT / "fixtures" / "stock_flow_v1" / "oracle.json"

PUBLIC_SCHEMA = "mycelix.fin-sys.stock-flow-public.v1"
ORACLE_SCHEMA = "mycelix.fin-sys.stock-flow-oracle.v1"
PROFILE = "closed-synthetic-financial-accounts-v1"

EXPECTED = {
    "ReconciledUnderProfile",
    "StockFlowIdentityViolation",
    "CounterpartViolation",
    "FlowClassificationViolation",
    "ConsolidationViolation",
    "InsufficientData",
    "UnsupportedProfile",
}

INVALID_FLOW_CLASS = {
    ("Borrowing", "CurrentIncome"),
    ("Revaluation", "OperatingCashFlow"),
    ("Revaluation", "CashFlow"),
    ("Writeoff", "Repayment"),
    ("PrincipalRepayment", "InterestExpense"),
    ("SecondaryEquityPurchase", "IssuerCapitalFormation"),
}


class Error(RuntimeError):
    pass


def dec(value, where):
    if not isinstance(value, str):
        raise Error(f"{where}: numeric value must be decimal string")
    try:
        parsed = Decimal(value)
    except InvalidOperation as exc:
        raise Error(f"{where}: invalid decimal {value!r}") from exc
    if not parsed.is_finite():
        raise Error(f"{where}: non-finite decimal forbidden")
    return parsed


def load(path):
    raw = path.read_bytes()
    return raw, json.loads(raw)


def index(doc, path):
    out = {}
    fixtures = doc.get("fixtures")
    if not isinstance(fixtures, list) or not fixtures:
        raise Error(f"{path}: fixtures must be a non-empty array")
    for fixture in fixtures:
        if not isinstance(fixture, dict):
            raise Error(f"{path}: fixture must be an object")
        fixture_id = fixture.get("fixture_id")
        if not isinstance(fixture_id, str) or not fixture_id:
            raise Error(f"{path}: invalid fixture_id")
        if fixture_id in out:
            raise Error(f"{path}: duplicate fixture_id {fixture_id}")
        out[fixture_id] = fixture
    return out


def classify(fixture):
    fixture_id = fixture["fixture_id"]

    # Semantic flow-class violations can occur even when all stock equations
    # balance, so inspect them separately from arithmetic reconciliation.
    for event in fixture.get("events", []):
        pair = (event.get("event_type"), event.get("claimed_flow_class"))
        dec(event.get("amount"), f"{fixture_id}.event.{event.get('event_id')}.amount")
        if pair in INVALID_FLOW_CLASS:
            return "FlowClassificationViolation"

    # Position-level stock-flow identity.
    fields = ["opening", "transactions", "revaluations", "other_changes", "closing"]
    for index_, position in enumerate(fixture.get("positions", [])):
        values = [
            dec(position.get(field), f"{fixture_id}.positions[{index_}].{field}")
            for field in fields
        ]
        opening, transactions, revaluations, other_changes, closing = values
        if opening + transactions + revaluations + other_changes != closing:
            return "StockFlowIdentityViolation"

    # Closed-universe counterpart identity: aggregate asset and liability
    # positions/changes for each financial instrument must match.
    if fixture.get("universe") == "ClosedSynthetic":
        totals = defaultdict(lambda: {
            "Asset": [Decimal(0)] * 5,
            "Liability": [Decimal(0)] * 5,
        })
        for index_, position in enumerate(fixture.get("positions", [])):
            side = position.get("side")
            if side not in {"Asset", "Liability"}:
                raise Error(f"{fixture_id}.positions[{index_}]: invalid side")
            instrument = position.get("instrument_id")
            if not isinstance(instrument, str) or not instrument:
                raise Error(f"{fixture_id}.positions[{index_}]: invalid instrument_id")
            values = [
                dec(position.get(field), f"{fixture_id}.positions[{index_}].{field}")
                for field in fields
            ]
            totals[instrument][side] = [
                prior + value
                for prior, value in zip(totals[instrument][side], values)
            ]
        for sides in totals.values():
            if sides["Asset"] != sides["Liability"]:
                return "CounterpartViolation"

    return "ReconciledUnderProfile"


def main():
    public_raw, public = load(PUBLIC)
    oracle_raw, oracle = load(ORACLE)

    if public.get("schema") != PUBLIC_SCHEMA or oracle.get("schema") != ORACLE_SCHEMA:
        raise Error("schema mismatch")
    if public.get("profile_id") != PROFILE or oracle.get("profile_id") != PROFILE:
        raise Error("profile mismatch")
    if oracle.get("visibility") != "evaluator_only":
        raise Error("oracle must be evaluator_only")

    public_fixtures = index(public, PUBLIC)
    oracle_fixtures = index(oracle, ORACLE)
    if set(public_fixtures) != set(oracle_fixtures):
        raise Error("public/oracle fixture-id mismatch")

    errors = []
    for fixture_id in sorted(public_fixtures):
        expected = oracle_fixtures[fixture_id].get("expected")
        if expected not in EXPECTED:
            raise Error(f"{fixture_id}: invalid expected disposition")
        observed = classify(public_fixtures[fixture_id])
        if observed != expected:
            errors.append(f"{fixture_id}: expected {expected}, observed {observed}")

    if errors:
        raise Error("; ".join(errors))

    print("FIN-SYS stock-flow oracle v1: CONFORMANT")
    print(f"fixtures={len(public_fixtures)}")
    print("public_sha256=" + hashlib.sha256(public_raw).hexdigest())
    print("oracle_sha256=" + hashlib.sha256(oracle_raw).hexdigest())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, json.JSONDecodeError, Error) as exc:
        print(f"FIN-SYS stock-flow oracle v1: NOT CONFORMANT: {exc}", file=sys.stderr)
        raise SystemExit(1)
