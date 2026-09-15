#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PROFILE_VERSION = "mycelix-capital-to-commons-fixed-preferred-v1"
RECEIPT_VERSION = "mycelix-commons-transition-receipt-v1"
PPM = 1_000_000
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:/-]{0,127}$")

PROFILE_KEYS = {
    "profile_version", "project_id", "unit", "max_amount_units",
    "initial_principal_units", "preferred_return_cap_units",
    "max_qualified_new_capital_units", "max_recoverable_lifecycle_units",
    "required_reserve_units", "max_events",
}
EVENT_KEYS = {
    "seq", "event_id", "project_id", "profile_sha256", "prev_event_sha256",
    "kind", "amount_units", "authority_ref", "evidence_ref", "counterparty_ref",
}
EVENT_KINDS = {
    "InitialCapitalContribution", "QualifiedNewCapital",
    "ApprovedRecoverableLifecycleCost", "InvestorDistribution",
    "PrincipalRedemption", "GrantOrSubsidy", "ReserveFunding",
    "ReserveDraw", "Impairment", "RefinanceReplaceClaim",
    "OperatorChange", "ControlChange", "TransitionCheckpoint",
}
ZERO_AMOUNT_EVENTS = {
    "RefinanceReplaceClaim", "OperatorChange", "ControlChange",
    "TransitionCheckpoint",
}
NONCLAIMS = (
    "financial satisfaction is not legal title transfer",
    "financial satisfaction is not democratic legitimacy",
    "financial satisfaction is not handback acceptance",
    "receipt validity is not accounting-standard compliance",
    "receipt validity is not tax or securities-law compliance",
    "receipt validity is not infrastructure safety or performance",
)

class TransitionError(ValueError):
    pass

@dataclass(frozen=True)
class QualifiedTransition:
    _receipt: dict[str, Any]

    def receipt(self) -> dict[str, Any]:
        return json.loads(json.dumps(self._receipt))


def _exact_keys(obj: dict[str, Any], expected: set[str], context: str) -> None:
    actual = set(obj)
    unknown = actual - expected
    missing = expected - actual
    if unknown or missing:
        raise TransitionError(
            f"{context}: key mismatch; missing={sorted(missing)} unknown={sorted(unknown)}"
        )


def _bounded_id(value: Any, context: str) -> str:
    if not isinstance(value, str) or not _ID.fullmatch(value):
        raise TransitionError(f"{context}: invalid bounded identifier")
    return value


def _bounded_ref(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value or len(value.encode("utf-8")) > 512:
        raise TransitionError(f"{context}: expected non-empty <=512-byte UTF-8 string")
    if value != value.strip():
        raise TransitionError(f"{context}: leading/trailing whitespace forbidden")
    return value


def _amount(value: Any, maximum: int, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TransitionError(f"{context}: expected integer amount")
    if value < 0 or value > maximum:
        raise TransitionError(f"{context}: amount outside profile bounds")
    return value


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def sha256_hex(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def validate_profile(profile: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(profile, dict):
        raise TransitionError("profile: expected object")
    _exact_keys(profile, PROFILE_KEYS, "profile")
    if profile["profile_version"] != PROFILE_VERSION:
        raise TransitionError("profile.profile_version: unsupported profile")
    _bounded_id(profile["project_id"], "profile.project_id")
    _bounded_id(profile["unit"], "profile.unit")

    max_amount = profile["max_amount_units"]
    if isinstance(max_amount, bool) or not isinstance(max_amount, int):
        raise TransitionError("profile.max_amount_units: expected integer")
    if max_amount < 1 or max_amount > 10**24:
        raise TransitionError("profile.max_amount_units: outside hard bounds")

    for field in (
        "initial_principal_units", "preferred_return_cap_units",
        "max_qualified_new_capital_units", "max_recoverable_lifecycle_units",
        "required_reserve_units",
    ):
        _amount(profile[field], max_amount, f"profile.{field}")

    max_events = profile["max_events"]
    if isinstance(max_events, bool) or not isinstance(max_events, int):
        raise TransitionError("profile.max_events: expected integer")
    if max_events < 1 or max_events > 100_000:
        raise TransitionError("profile.max_events: outside hard bounds")

    max_envelope = (
        profile["initial_principal_units"]
        + profile["preferred_return_cap_units"]
        + profile["max_qualified_new_capital_units"]
        + profile["max_recoverable_lifecycle_units"]
    )
    if max_envelope > max_amount:
        raise TransitionError(
            "profile: maximum possible return envelope exceeds max_amount_units"
        )
    return profile


def validate_events(events: Any, profile: dict[str, Any]) -> list[dict[str, Any]]:
    if not isinstance(events, list):
        raise TransitionError("events: expected array")
    if not events:
        raise TransitionError("events: history must be non-empty")
    if len(events) > profile["max_events"]:
        raise TransitionError("events: exceeds profile.max_events")

    seen_ids: set[str] = set()
    normalized: list[dict[str, Any]] = []
    for i, event in enumerate(events):
        if not isinstance(event, dict):
            raise TransitionError(f"events[{i}]: expected object")
        _exact_keys(event, EVENT_KEYS, f"events[{i}]")
        if event["seq"] != i:
            raise TransitionError(f"events[{i}].seq: expected exact contiguous sequence {i}")
        event_id = _bounded_id(event["event_id"], f"events[{i}].event_id")
        if event_id in seen_ids:
            raise TransitionError(f"events[{i}].event_id: duplicate")
        seen_ids.add(event_id)

        if event["project_id"] != profile["project_id"]:
            raise TransitionError(f"events[{i}].project_id: project substitution")
        expected_profile_sha = sha256_hex(profile)
        if event["profile_sha256"] != expected_profile_sha:
            raise TransitionError(f"events[{i}].profile_sha256: profile substitution")
        prev = event["prev_event_sha256"]
        if i == 0:
            if prev is not None:
                raise TransitionError("events[0].prev_event_sha256: must be null")
        else:
            expected_prev = sha256_hex(normalized[-1])
            if prev != expected_prev:
                raise TransitionError(f"events[{i}].prev_event_sha256: broken event chain")

        kind = event["kind"]
        if kind not in EVENT_KINDS:
            raise TransitionError(f"events[{i}].kind: unsupported kind")
        amount = _amount(
            event["amount_units"], profile["max_amount_units"],
            f"events[{i}].amount_units",
        )
        if kind in ZERO_AMOUNT_EVENTS and amount != 0:
            raise TransitionError(f"events[{i}]: {kind} must carry amount_units=0")
        _bounded_ref(event["authority_ref"], f"events[{i}].authority_ref")
        _bounded_ref(event["evidence_ref"], f"events[{i}].evidence_ref")
        cp = event["counterparty_ref"]
        if cp is not None:
            _bounded_ref(cp, f"events[{i}].counterparty_ref")
        normalized.append(event)

    first = normalized[0]
    if first["kind"] != "InitialCapitalContribution":
        raise TransitionError("events[0]: must be InitialCapitalContribution")
    if first["amount_units"] != profile["initial_principal_units"]:
        raise TransitionError(
            "events[0]: initial contribution must equal frozen profile principal"
        )
    if any(e["kind"] == "InitialCapitalContribution" for e in normalized[1:]):
        raise TransitionError("events: InitialCapitalContribution may occur exactly once")
    return normalized


def qualify(profile: dict[str, Any], events: Any) -> QualifiedTransition:
    validate_profile(profile)
    history = validate_events(events, profile)

    initial = profile["initial_principal_units"]
    preferred = profile["preferred_return_cap_units"]
    remaining_claim = initial + preferred
    qualified_new_capital = 0
    lifecycle_additions = 0
    counted_distributions = 0
    grants = 0
    reserve_balance = 0
    impairments = 0
    retired_claim_units = 0

    for i, event in enumerate(history):
        kind = event["kind"]
        amount = event["amount_units"]
        if i == 0:
            continue

        if kind == "QualifiedNewCapital":
            if qualified_new_capital + amount > profile["max_qualified_new_capital_units"]:
                raise TransitionError(f"events[{i}]: qualified new capital cap exceeded")
            qualified_new_capital += amount
            remaining_claim += amount
        elif kind == "ApprovedRecoverableLifecycleCost":
            if lifecycle_additions + amount > profile["max_recoverable_lifecycle_units"]:
                raise TransitionError(f"events[{i}]: recoverable lifecycle cap exceeded")
            lifecycle_additions += amount
            remaining_claim += amount
        elif kind in {"InvestorDistribution", "PrincipalRedemption"}:
            if reserve_balance < profile["required_reserve_units"]:
                raise TransitionError(
                    f"events[{i}]: investor distribution forbidden while required reserve is underfunded"
                )
            if amount > remaining_claim:
                raise TransitionError(f"events[{i}]: distribution would underflow claim")
            remaining_claim -= amount
            counted_distributions += amount
            retired_claim_units += amount
        elif kind == "GrantOrSubsidy":
            grants += amount
        elif kind == "ReserveFunding":
            reserve_balance += amount
            if reserve_balance > profile["max_amount_units"]:
                raise TransitionError(f"events[{i}]: reserve exceeds authority bounds")
        elif kind == "ReserveDraw":
            if amount > reserve_balance:
                raise TransitionError(f"events[{i}]: reserve draw underflow")
            reserve_balance -= amount
        elif kind == "Impairment":
            if amount > remaining_claim:
                raise TransitionError(f"events[{i}]: impairment would underflow claim")
            remaining_claim -= amount
            impairments += amount
            retired_claim_units += amount
        elif kind in ZERO_AMOUNT_EVENTS:
            pass
        else:  # pragma: no cover
            raise AssertionError(kind)

        if remaining_claim > profile["max_amount_units"]:
            raise TransitionError(f"events[{i}]: claim exceeds authority bounds")
        if counted_distributions > profile["max_amount_units"]:
            raise TransitionError(f"events[{i}]: distributions exceed authority bounds")
        if grants > profile["max_amount_units"]:
            raise TransitionError(f"events[{i}]: grants exceed authority bounds")

    created_entitlement = initial + preferred + qualified_new_capital + lifecycle_additions
    if retired_claim_units > created_entitlement:
        raise TransitionError("internal: retired claim exceeds created entitlement")
    retired_ppm = (
        PPM if created_entitlement == 0
        else (retired_claim_units * PPM) // created_entitlement
    )
    reserve_compliant = reserve_balance >= profile["required_reserve_units"]
    financial_state = (
        "RETURN_ENVELOPE_SATISFIED" if remaining_claim == 0 else "CLAIM_ACTIVE"
    )

    receipt = {
        "receipt_version": RECEIPT_VERSION,
        "profile_version": profile["profile_version"],
        "project_id": profile["project_id"],
        "unit": profile["unit"],
        "profile_sha256": sha256_hex(profile),
        "event_history_sha256": sha256_hex(history),
        "event_chain_tip_sha256": sha256_hex(history[-1]),
        "event_count": len(history),
        "initial_principal_units": initial,
        "preferred_return_cap_units": preferred,
        "qualified_new_capital_units": qualified_new_capital,
        "recoverable_lifecycle_units": lifecycle_additions,
        "created_entitlement_units": created_entitlement,
        "counted_investor_distributions_units": counted_distributions,
        "impairments_units": impairments,
        "grant_or_subsidy_units": grants,
        "reserve_balance_units": reserve_balance,
        "required_reserve_units": profile["required_reserve_units"],
        "reserve_compliant": reserve_compliant,
        "retired_claim_units": retired_claim_units,
        "retired_claim_ppm": retired_ppm,
        "remaining_claim_units": remaining_claim,
        "financial_state": financial_state,
        "legal_transition_complete": False,
        "handback_accepted": False,
        "nonclaims": list(NONCLAIMS),
    }
    return QualifiedTransition(receipt)


def load_case(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != {"profile", "events"}:
        raise TransitionError("case: expected exact keys ['events', 'profile']")
    return data["profile"], data["events"]


def _cli() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("case", type=Path)
    parser.add_argument("--receipt-out", type=Path)
    args = parser.parse_args()
    profile, events = load_case(args.case)
    receipt = qualify(profile, events).receipt()
    rendered = json.dumps(receipt, sort_keys=True, indent=2, ensure_ascii=False) + "\n"
    if args.receipt_out:
        args.receipt_out.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0

if __name__ == "__main__":
    raise SystemExit(_cli())
