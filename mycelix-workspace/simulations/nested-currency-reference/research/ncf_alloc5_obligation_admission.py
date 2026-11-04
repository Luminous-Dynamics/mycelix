#!/usr/bin/env python3
"""NCF-ALLOC-5: obligation-level deduplication and principal-scoped admission.

Research reference only. The verifier_receipt_ok input represents an independent
verification step that this model does NOT implement cryptographically.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import hashlib, json
from pathlib import Path
from typing import Optional

class Decision(str, Enum):
    ACCEPTED = "accepted"
    IDEMPOTENT_REPLAY = "idempotent_replay"
    REJECTED = "rejected"

@dataclass(frozen=True)
class ObligationEvidence:
    obligation_id: str
    principal_id: str
    instrument_id: str
    shard_id: str
    epoch: int
    sequence: int
    amount_units: int
    obligation_payload_digest: str
    verifier_version: str
    verifier_receipt_digest: str
    verifier_receipt_ok: bool

@dataclass(frozen=True)
class Receipt:
    obligation_id: str
    principal_id: str
    instrument_id: str
    amount_units: int
    obligation_payload_digest: str
    original_epoch: int
    receipt_digest: str

@dataclass(frozen=True)
class ApplyResult:
    decision: Decision
    reason: str
    receipt: Optional[Receipt] = None

class ObligationBook:
    """Sequential oracle for globally unique obligation IDs and principal caps."""
    def __init__(self, principal_caps: dict[tuple[str, str], int]):
        if any(not p.strip() or not i.strip() or c < 0 for (p, i), c in principal_caps.items()):
            raise ValueError("principal caps require non-empty IDs and nonnegative values")
        self._caps = dict(principal_caps)
        self._accepted: dict[str, tuple[tuple[str, str, int, str], Receipt]] = {}
        self._canonical_id_by_digest: dict[str, str] = {}
        self._used_by_principal_instrument: dict[tuple[str, str], int] = {}

    @staticmethod
    def _validate(e: ObligationEvidence) -> Optional[str]:
        if not all(x.strip() for x in (e.obligation_id, e.principal_id, e.instrument_id, e.shard_id, e.verifier_version)):
            return "empty_identity_or_verifier_version"
        if e.epoch < 0 or e.sequence < 0 or e.amount_units <= 0:
            return "invalid_epoch_sequence_or_amount"
        if len(e.obligation_payload_digest) != 64 or any(c not in "0123456789abcdef" for c in e.obligation_payload_digest):
            return "invalid_obligation_payload_digest"
        if len(e.verifier_receipt_digest) != 64 or any(c not in "0123456789abcdef" for c in e.verifier_receipt_digest):
            return "invalid_verifier_receipt_digest"
        if not e.verifier_receipt_ok:
            return "verifier_did_not_accept_evidence"
        return None

    def apply(self, e: ObligationEvidence) -> ApplyResult:
        reason = self._validate(e)
        if reason:
            return ApplyResult(Decision.REJECTED, reason)
        payload = (e.principal_id, e.instrument_id, e.amount_units, e.obligation_payload_digest)
        prior = self._accepted.get(e.obligation_id)
        if prior:
            prior_payload, prior_receipt = prior
            if payload == prior_payload:
                return ApplyResult(Decision.IDEMPOTENT_REPLAY, "same_obligation_payload_already_accepted", prior_receipt)
            return ApplyResult(Decision.REJECTED, "obligation_id_conflicts_with_accepted_payload", prior_receipt)
        canonical_id = self._canonical_id_by_digest.get(e.obligation_payload_digest)
        if canonical_id is not None:
            canonical_payload, canonical_receipt = self._accepted[canonical_id]
            if payload != canonical_payload:
                return ApplyResult(Decision.REJECTED, "canonical_obligation_digest_conflicts_with_payload", canonical_receipt)
            self._accepted[e.obligation_id] = (payload, canonical_receipt)
            return ApplyResult(Decision.IDEMPOTENT_REPLAY, "canonical_obligation_already_accepted_under_another_id", canonical_receipt)

        cap_key = (e.principal_id, e.instrument_id)
        if cap_key not in self._caps:
            return ApplyResult(Decision.REJECTED, "missing_principal_instrument_cap")
        used = self._used_by_principal_instrument.get(cap_key, 0)
        next_used = used + e.amount_units
        if next_used > self._caps[cap_key]:
            return ApplyResult(Decision.REJECTED, "principal_instrument_cap_exceeded")
        canonical = "|".join((e.obligation_id, e.principal_id, e.instrument_id, str(e.amount_units),
                               e.obligation_payload_digest, str(e.epoch)))
        receipt = Receipt(e.obligation_id, e.principal_id, e.instrument_id, e.amount_units,
                          e.obligation_payload_digest, e.epoch, hashlib.sha256(canonical.encode()).hexdigest())
        self._accepted[e.obligation_id] = (payload, receipt)
        self._canonical_id_by_digest[e.obligation_payload_digest] = e.obligation_id
        self._used_by_principal_instrument[cap_key] = next_used
        return ApplyResult(Decision.ACCEPTED, "accepted_verified_obligation_once", receipt)

    def outstanding_for(self, principal_id: str, instrument_id: str) -> int:
        return self._used_by_principal_instrument.get((principal_id, instrument_id), 0)

    def accepted_count(self) -> int:
        return len(self._canonical_id_by_digest)

    def receipt_for(self, obligation_id: str) -> Optional[Receipt]:
        entry = self._accepted.get(obligation_id)
        return entry[1] if entry else None

def evidence(obligation_id: str, principal_id: str = "principal-A", instrument_id: str = "instrument-1",
             shard_id: str = "shard-0", epoch: int = 1, sequence: int = 1, amount_units: int = 100,
             verifier_version: str = "queue-verifier-v1", verifier_receipt_ok: bool = True,
             obligation_payload: str | None = None) -> ObligationEvidence:
    payload = obligation_payload if obligation_payload is not None else f"service|{obligation_id}|{principal_id}|{instrument_id}|{amount_units}"
    obligation_digest = hashlib.sha256(payload.encode()).hexdigest()
    digest = hashlib.sha256(f"receipt|{obligation_id}|{principal_id}|{instrument_id}|{amount_units}|{obligation_digest}".encode()).hexdigest()
    return ObligationEvidence(obligation_id, principal_id, instrument_id, shard_id, epoch, sequence,
                              amount_units, obligation_digest, verifier_version, digest, verifier_receipt_ok)

def run_alias_scenario(alias_count: int = 32) -> dict:
    if alias_count < 1: raise ValueError("alias_count must be positive")
    book = ObligationBook({("principal-A", "instrument-1"): 10_000})
    outcomes = []
    for i in range(alias_count):
        row = evidence("service-obligation-0001", shard_id=f"shard-alias-{i:03d}", epoch=1+i//8, sequence=i+1, amount_units=4_000)
        outcomes.append(book.apply(row).decision.value)
    return {"alias_count": alias_count, "actual_unique_obligation_units": 4_000,
            "book_outstanding_units": book.outstanding_for("principal-A", "instrument-1"),
            "accepted_unique_obligations": book.accepted_count(), "outcome_counts": {v: outcomes.count(v) for v in sorted(set(outcomes))},
            "original_receipt_preserved": book.receipt_for("service-obligation-0001") is not None}

def run_canonical_digest_alias_scenario(alias_count: int = 32) -> dict:
    if alias_count < 1: raise ValueError("alias_count must be positive")
    book = ObligationBook({("principal-A", "instrument-1"): 10_000})
    decisions = []
    original_receipt = None
    for i in range(alias_count):
        row = evidence(f"request-alias-{i:03}", shard_id=f"shard-alias-{i:03}", epoch=i // 4,
                       sequence=i + 1, amount_units=4_000, obligation_payload="canonical-eligible-work-0001")
        applied = book.apply(row)
        decisions.append(applied.decision.value)
        if i == 0: original_receipt = applied.receipt
    return {"alias_count": alias_count, "canonical_work_units": 4_000,
            "accepted_unique_obligations": book.accepted_count(),
            "principal_instrument_exposure_units": book.outstanding_for("principal-A", "instrument-1"),
            "outcome_counts": {v: decisions.count(v) for v in sorted(set(decisions))},
            "all_aliases_return_original_receipt": all(book.receipt_for(f"request-alias-{i:03}") == original_receipt for i in range(alias_count)),
            "claim_ceiling": "Assumes a trusted verifier derives canonical work digest; model does not authenticate real work."}

def main() -> None:
    profile = {"alias_counts": [1, 4, 8, 16, 32], "same_underlying_obligation_units": 4_000,
               "principal_cap_units": 10_000, "model": "sequential admission/dedup oracle; verifier receipt is assumed externally validated"}
    rows = [run_alias_scenario(n) for n in profile["alias_counts"]]
    digest_aliases = [run_canonical_digest_alias_scenario(n) for n in profile["alias_counts"]]
    result = {"schema": "ncf-alloc5-obligation-admission-v1", "status": "INDEPENDENT_PYTHON_REFERENCE_MODEL_NOT_CRYPTOGRAPHIC_NOT_RUST_NOT_CI_NOT_PRODUCTION",
              "profile": profile, "alias_scenarios": rows, "canonical_digest_alias_scenarios": digest_aliases,
              "claim_ceiling": "Sequential policy oracle only. It does not authenticate identities or work, model concurrent writes, or establish Holochain/distributed correctness."}
    result["source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    print(json.dumps(result, indent=2, sort_keys=True))

if __name__ == "__main__": main()
