#!/usr/bin/env python3
"""Independent FIN-ECO-002A canonical-vector verifier.

Uses only Python stdlib and checked-in JSON fixtures. It does not call Rust or
reuse production canonicalization code. All proof obligations use explicit
fail-closed checks; Python optimization flags cannot disable them.
"""

from __future__ import annotations

import hashlib
import json
import struct
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parent.parent
VECTORS = ROOT / "test-vectors"


class VerificationError(RuntimeError):
    """Raised when any canonical-vector proof obligation fails."""


def require(condition: bool, *detail: object) -> None:
    if not condition:
        message = "verification requirement failed"
        if detail:
            message += ": " + " | ".join(str(item) for item in detail)
        raise VerificationError(message)


def load(name: str) -> dict:
    with (VECTORS / name).open(encoding="utf-8") as fh:
        return json.load(fh)


def u8(value: int) -> bytes:
    require(0 <= value <= 0xFF, "u8 out of range", value)
    return bytes([value])


def u16(value: int) -> bytes:
    require(0 <= value <= 0xFFFF, "u16 out of range", value)
    return struct.pack(">H", value)


def u32(value: int) -> bytes:
    require(0 <= value <= 0xFFFFFFFF, "u32 out of range", value)
    return struct.pack(">I", value)


def u64(value: int) -> bytes:
    require(0 <= value <= 0xFFFFFFFFFFFFFFFF, "u64 out of range", value)
    return struct.pack(">Q", value)


def ref(value: str) -> bytes:
    raw = value.encode("utf-8")
    return u32(len(raw)) + raw


def digest(hex_value: str) -> bytes:
    try:
        raw = bytes.fromhex(hex_value)
    except ValueError as exc:
        raise VerificationError(f"invalid digest hex: {hex_value!r}") from exc
    require(len(raw) == 32, "digest must be exactly 32 bytes", len(raw))
    return raw


def ordered_refs(values: Iterable[str]) -> bytes:
    unique = sorted(set(values), key=lambda value: value.encode("utf-8"))
    return u32(len(unique)) + b"".join(ref(value) for value in unique)


def ordered_digests(values: Iterable[bytes]) -> bytes:
    unique = sorted(set(values))
    for value in unique:
        require(len(value) == 32, "ordered digest must be 32 bytes", len(value))
    return u32(len(unique)) + b"".join(unique)


def check(expected: dict, canonical: bytes, label: str) -> bytes:
    require(
        len(canonical) == expected["canonical_length"],
        label,
        "canonical length",
        len(canonical),
        expected["canonical_length"],
    )
    require(
        canonical.hex() == expected["canonical_hex"],
        label,
        "canonical bytes mismatch",
    )
    actual = hashlib.sha256(canonical).hexdigest()
    require(
        actual == expected["commitment_hex"],
        label,
        "commitment mismatch",
        actual,
        expected["commitment_hex"],
    )
    return bytes.fromhex(actual)


def context_bytes(context: dict) -> bytes:
    return (
        b"MYCELIX_FINANCE_SETTLEMENT_EVALUATION_CONTEXT_V1\0"
        + u16(1)
        + u8({"DeterministicSupplied": 0, "HistoricalReplay": 1}[context["class"]])
        + u64(context["evaluation_time_unix_ms"])
        + ref(context["temporal_profile_id"])
        + u64(context["temporal_profile_revision"])
        + digest(context["temporal_context_digest_hex"])
    )


def evidence_bytes(evidence: dict, subject: str) -> bytes:
    return (
        b"MYCELIX_FINANCE_SETTLEMENT_EVIDENCE_V1\0"
        + u16(1)
        + ref(evidence["evidence_id"])
        + ref(subject)
        + ref(evidence["operation_id"])
        + u64(evidence["operation_revision"])
        + ref(evidence["observation_id"])
        + ref(evidence["kind"])
        + ref(evidence["source"])
        + digest(evidence["payload_digest_hex"])
    )


def profile_bytes(profile: dict, required_kinds: Iterable[str] | None = None) -> bytes:
    kinds = (
        profile["required_evidence_kinds"]
        if required_kinds is None
        else list(required_kinds)
    )
    return (
        b"MYCELIX_FINANCE_SETTLEMENT_FINALITY_PROFILE_V1\0"
        + u16(1)
        + ref(profile["id"])
        + u64(profile["revision"])
        + ref(profile["rail"])
        + ref(profile["network"])
        + ordered_refs(kinds)
        + u16(profile["min_distinct_sources"])
        + u64(profile["max_observation_age_ms"])
        + u8({"MayReverse": 0, "DeclaredTerminalByProfile": 1}[profile["reversal_model"]])
    )


def verify_settlement() -> dict:
    vector = load("settlement-v1.json")
    profile = vector["profile"]
    context = vector["evaluation_context"]
    subject = vector["subject"]
    evidence = vector["evidence"]
    observation = vector["observation"]

    profile_commitment = check(profile, profile_bytes(profile), "settlement.profile")
    context_commitment = check(
        context, context_bytes(context), "settlement.evaluation_context"
    )
    evidence_commitment = check(
        evidence,
        evidence_bytes(evidence, subject["id"]),
        "settlement.evidence",
    )

    observation_canonical = (
        b"MYCELIX_FINANCE_SETTLEMENT_OBSERVATION_V1\0"
        + u16(1)
        + ref(observation["observation_id"])
        + ref(subject["id"])
        + digest(subject["financial_effect_commitment_hex"])
        + ref(subject["attempt"])
        + ref(profile["rail"])
        + ref(profile["network"])
        + ref(observation["operation_id"])
        + u64(observation["revision"])
        + ref(subject["asset"])
        + u64(subject["atomic_units"])
        + u8(
            {
                "Applied": 0,
                "Pending": 1,
                "Rejected": 2,
                "Unknown": 3,
                "Reversed": 4,
            }[observation["state"]]
        )
        + u64(observation["observed_at_unix_ms"])
        + ordered_digests([evidence_commitment])
    )
    observation_commitment = check(
        observation, observation_canonical, "settlement.observation"
    )

    frontier_canonical = (
        b"MYCELIX_FINANCE_SETTLEMENT_FRONTIER_V1\0"
        + u16(1)
        + ref(subject["id"])
        + digest(subject["financial_effect_commitment_hex"])
        + ref(subject["attempt"])
        + ref(profile["rail"])
        + ref(profile["network"])
        + profile_commitment
        + ref(subject["asset"])
        + u64(subject["atomic_units"])
        + context_commitment
        + ordered_digests([observation_commitment])
    )
    check(vector["frontier"], frontier_canonical, "settlement.frontier")
    print("independent_settlement_v1_oracle=PASS")
    return vector


def verify_invalidation(settlement: dict) -> None:
    vector = load("invalidation-receipt-v1.json")
    profile = settlement["profile"]
    subject = settlement["subject"]
    context = vector["evaluation_context"]
    evidence = vector["invalidating_evidence"]
    observation = vector["invalidating_observation"]
    receipt = vector["receipt"]

    require(
        vector["profile_commitment_hex"] == profile["commitment_hex"],
        "invalidation profile commitment does not match settlement fixture",
    )

    context_commitment = check(
        context, context_bytes(context), "invalidation.evaluation_context"
    )
    evidence_commitment = check(
        evidence,
        evidence_bytes(evidence, evidence["subject"]),
        "invalidation.evidence",
    )

    observation_canonical = (
        b"MYCELIX_FINANCE_SETTLEMENT_OBSERVATION_V1\0"
        + u16(1)
        + ref(observation["observation_id"])
        + ref(subject["id"])
        + digest(subject["financial_effect_commitment_hex"])
        + ref(subject["attempt"])
        + ref(profile["rail"])
        + ref(profile["network"])
        + ref(observation["operation_id"])
        + u64(observation["operation_revision"])
        + ref(subject["asset"])
        + u64(subject["atomic_units"])
        + u8(
            {
                "Applied": 0,
                "Pending": 1,
                "Rejected": 2,
                "Unknown": 3,
                "Reversed": 4,
            }[observation["state"]]
        )
        + u64(observation["observed_at_unix_ms"])
        + ordered_digests([evidence_commitment])
    )
    observation_commitment = check(
        observation, observation_canonical, "invalidation.observation"
    )

    receipt_canonical = (
        b"MYCELIX_FINANCE_SETTLEMENT_INVALIDATION_RECEIPT_V1\0"
        + u16(1)
        + ref(receipt["subject"])
        + digest(receipt["financial_effect_commitment_hex"])
        + ref(receipt["prior_profile_id"])
        + u64(receipt["prior_profile_revision"])
        + digest(vector["profile_commitment_hex"])
        + ref(receipt["operation_id"])
        + u64(receipt["prior_revision"])
        + ref(receipt["invalidating_observation_id"])
        + observation_commitment
        + context_commitment
        + u8({"DeterministicSupplied": 0, "HistoricalReplay": 1}[context["class"]])
        + u64(receipt["invalidated_at_unix_ms"])
        + ordered_refs(receipt["evidence_ids"])
    )
    check(receipt, receipt_canonical, "invalidation.receipt")
    print("independent_invalidation_receipt_v1_oracle=PASS")


def verify_reference_ordering() -> None:
    vector = load("reference-ordering-v1.json")
    physical = vector["physical_input_references"]
    expected_order = vector["expected_unique_order"]
    actual_order = sorted(set(physical), key=lambda value: value.encode("utf-8"))
    require(
        actual_order == expected_order,
        "reference ordering mismatch",
        actual_order,
        expected_order,
    )

    profile = vector["profile"]
    canonical = profile_bytes(profile, actual_order)
    check(profile, canonical, "ordering.profile")

    # Negative control: sorting complete length-prefixed encodings is not v1.
    encoded_values = {ref(value) for value in set(physical)}
    wrong_order = sorted(encoded_values)
    correct_order = [ref(value) for value in actual_order]
    require(
        wrong_order != correct_order,
        "negative control failed: wrong and correct reference orders are equal",
    )
    require(
        b"".join(wrong_order) != b"".join(correct_order),
        "negative control failed: wrong and correct byte streams are equal",
    )

    print("independent_reference_ordering_v1_oracle=PASS")


def main() -> None:
    settlement = verify_settlement()
    verify_invalidation(settlement)
    verify_reference_ordering()
    print("independent_fin_eco_002a_canonical_oracles=PASS")


if __name__ == "__main__":
    try:
        main()
    except (VerificationError, KeyError, TypeError, ValueError, struct.error) as exc:
        raise SystemExit(f"independent_fin_eco_002a_canonical_oracles=FAIL: {exc}") from exc
