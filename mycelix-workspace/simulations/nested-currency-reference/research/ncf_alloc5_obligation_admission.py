#!/usr/bin/env python3
"""NCF-ALLOC-5: obligation-level deduplication and principal-scoped admission.

Research reference only. The verifier_receipt_ok input represents an independent
verification step that this model does NOT implement cryptographically.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import hashlib, json
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
    verifier_version: str
    verifier_receipt_digest: str
    verifier_receipt_ok: bool

@dataclass(frozen=True)
class Receipt:
    obligation_id: str
    principal_id: str
    instrument_id: str
    amount_units: int
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
        self._accepted: dict[str, tuple[tuple[str, str, int], Receipt]] = {}
        self._used_by_principal_instrument: dict[tuple[str, str], int] = {}

    @staticmethod
    def _validate(e: ObligationEvidence) -> Optional[str]:
        if not all(x.strip() for x in (e.obligation_id, e.principal_id, e.instrument_id, e.shard_id, e.verifier_version)):
            return "empty_identity_or_verifier_version"
        if e.epoch < 0 or e.sequence < 0 or e.amount_units <= 0:
            return "invalid_epoch_sequence_or_amount"
        if len(e.verifier_receipt_digest) != 64 or any(c not in "0123456789abcdef" for c in e.verifier_receipt_digest):
            return "invalid_verifier_receipt_digest"
        if not e.verifier_receipt_ok:
            return "verifier_did_not_accept_evidence"
        return None

    def apply(self, e: ObligationEvidence) -> ApplyResult:
        reason = self._validate(e)
        if reason:
            return ApplyResult(Decision.REJECTED, reason)
        # The caller provides verifier_receipt_ok; this model tests downstream policy
        # assuming an independently verified receipt exists. It is not signature proof.
        payload = (e.principal_id, e.instrument_id, e.amount_units)
        prior = self._accepted.get(e.obligation_id)
        if prior:
            prior_payload, prior_receipt = prior
            if payload == prior_payload:
                return ApplyResult(Decision.IDEMPOTENT_REPLAY, "same_obligation_payload_already_accepted", prior_receipt)
            return ApplyResult(Decision.REJECTED, "obligation_id_conflicts_with_accepted_payload", prior_receipt)
        cap_key = (e.principal_id, e.instrument_id)
        if cap_key not in self._caps:
            return ApplyResult(Decision.REJECTED, "missing_principal_instrument_cap")
        used = self._used_by_principal_instrument.get(cap_key, 0)
        next_used = used + e.amount_units
        if next_used > self._caps[cap_key]:
            return ApplyResult(Decision.REJECTED, "principal_instrument_cap_exceeded")
        # Receipt digest is deterministic for the reference oracle and not a signature.
        canonical = "|".join((e.obligation_id, e.principal_id, e.instrument_id, str(e.amount_units), str(e.epoch)))
        receipt = Receipt(e.obligation_id, e.principal_id, e.instrument_id, e.amount_units,
                          e.epoch, hashlib.sha256(canonical.encode()).hexdigest())
        self._accepted[e.obligation_id] = (payload, receipt)
        self._used_by_principal_instrument[cap_key] = next_used
        return ApplyResult(Decision.ACCEPTED, "accepted_verified_obligation_once", receipt)

    def outstanding_for(self, principal_id: str, instrument_id: str) -> int:
        return self._used_by_principal_instrument.get((principal_id, instrument_id), 0)

    def accepted_count(self) -> int:
        return len(self._accepted)

    def receipt_for(self, obligation_id: str) -> Optional[Receipt]:
        entry = self._accepted.get(obligation_id)
        return entry[1] if entry else None

def evidence(obligation_id: str, principal_id: str = "principal-A", instrument_id: str = "instrument-1",
             shard_id: str = "shard-0", epoch: int = 1, sequence: int = 1, amount_units: int = 100,
             verifier_version: str = "queue-verifier-v1", verifier_receipt_ok: bool = True) -> ObligationEvidence:
    digest = hashlib.sha256(f"receipt|{obligation_id}|{principal_id}|{instrument_id}|{amount_units}".encode()).hexdigest()
    return ObligationEvidence(obligation_id, principal_id, instrument_id, shard_id, epoch, sequence,
                              amount_units, verifier_version, digest, verifier_receipt_ok)

def run_alias_scenario(alias_count: int = 32) -> dict:
    if alias_count < 1: raise ValueError("alias_count must be positive")
    book = ObligationBook({("principal-A", "instrument-1"): 10_000})
    # One 4,000-unit underlying obligation is replayed through many shard aliases.
    outcomes = []
    for i in range(alias_count):
        row = evidence("service-obligation-0001", shard_id=f"shard-alias-{i:03d}", epoch=1+i//8, sequence=i+1, amount_units=4_000)
        result = book.apply(row)
        outcomes.append(result.decision.value)
    return {"alias_count": alias_count, "actual_unique_obligation_units": 4_000,
            "book_outstanding_units": book.outstanding_for("principal-A", "instrument-1"),
            "accepted_unique_obligations": book.accepted_count(), "outcome_counts": {v: outcomes.count(v) for v in sorted(set(outcomes))},
            "original_receipt_preserved": book.receipt_for("service-obligation-0001") is not None}

def main() -> None:
    profile = {"alias_counts": [1, 4, 8, 16, 32], "same_underlying_obligation_units": 4_000,
               "principal_cap_units": 10_000, "model": "sequential admission/dedup oracle; verifier receipt is assumed externally validated"}
    rows = [run_alias_scenario(n) for n in profile["alias_counts"]]
    result = {"schema": "ncf-alloc5-obligation-admission-v1", "status": "INDEPENDENT_PYTHON_REFERENCE_MODEL_NOT_CRYPTOGRAPHIC_NOT_RUST_NOT_CI_NOT_PRODUCTION",
              "profile": profile, "alias_scenarios": rows,
              "claim_ceiling": "This proves only the Python model's sequential deduplication and principal-cap logic under an assumed valid verifier receipt. It does not authenticate identities or work, model concurrent writes, or establish Holochain/distributed correctness."}
    print(json.dumps(result, indent=2, sort_keys=True))

if __name__ == "__main__": main()
