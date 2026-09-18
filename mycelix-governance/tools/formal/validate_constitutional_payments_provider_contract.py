#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "mycelix-governance/specs/constitutional-payments-provider-contract.v1.json"
SOURCE = ROOT / "mycelix-governance/crates/constitutional-payments-provider/src/lib.rs"

PARENTS = {
    "f0c_semantic_head": "36bb486794f3e2720a2e02b3045c270b147cbacc",
    "f0p0_semantic_head": "0442bc487dc5ea3303144b3f7be8979c673d6018",
}
OUTCOMES = {"KnownSuccess", "KnownNoEffect", "UnknownOutcome"}


def fail(msg: str) -> None:
    raise ValueError(msg)


def validate_profile(p: dict) -> None:
    if p["schema"] != "mycelix.constitutional-payments-provider-contract.v1": fail("schema drift")
    if p["profile_id"] != "mycelix-constitutional-payments-provider-v1": fail("profile drift")
    if p["revision"] != 1 or p["authority_class"] != "InertReferenceContract": fail("authority drift")
    if p["parents"] != PARENTS: fail("parent drift")
    ident = p["provider_operation_identity"]
    if set(ident["inputs"]) != {"provider_profile_id","provider_profile_commitment","execution_id","request_commitment"}: fail("operation key input census drift")
    for k in ("wall_clock_participates","attempt_id_participates","payment_id_participates","receipt_id_participates"):
        if ident[k] is not False: fail(f"nonsemantic identity input admitted: {k}")
    if not ident["same_f0_effect_same_profile_same_key"]: fail("retry-stable key weakened")
    if not ident["changed_f0_effect_changes_key"]: fail("effect binding weakened")
    if not ident["changed_provider_profile_commitment_changes_key"]: fail("provider profile binding weakened")
    if set(p["outcomes"]) != OUTCOMES: fail("outcome census drift")
    obs = p["observation_binding"]
    for k in ("provider_operation_key","execution_id","request_commitment","observation_id"):
        if obs[k] is not True: fail(f"observation binding weakened: {k}")
    if obs["observed_at_is_metadata_only"] is not True: fail("observation time promoted to authority")
    state = p["state_rules"]
    for k in ("unknown_may_resolve_to_success","unknown_may_resolve_to_no_effect","success_and_no_effect_conflict_halts","conflicting_success_evidence_halts","same_observation_id_same_payload_is_idempotent","same_observation_id_changed_payload_halts","unknown_after_terminal_is_historical_only"):
        if state[k] is not True: fail(f"state rule weakened: {k}")
    reg = p["registry_rules"]
    for k in ("same_exact_intent_returns_existing","one_provider_operation_per_execution_id","same_execution_changed_provider_profile_conflicts","operation_key_collision_conflicts"):
        if reg[k] is not True: fail(f"registry rule weakened: {k}")
    if reg["last_write_wins"] is not False: fail("last-write-wins enabled")
    if any(p["activation"].values()): fail("F1A activation/capability inflated")
    if len(p["required_successor"]) != 9: fail("successor gate census drift")
    if len(p["non_claims"]) != 11: fail("non-claim census drift")


def validate_source(s: str) -> None:
    required = [
        'const OPERATION_KEY_DOMAIN:',
        'push_str(&mut h, &self.provider_profile_id);',
        'push_str(&mut h, &self.provider_profile_commitment);',
        'push_str(&mut h, &self.execution_id);',
        'push_str(&mut h, &self.request_commitment);',
        'if self.provider_operation_key != self.compute_operation_key()?',
        'if self.provider_operation_key != intent.provider_operation_key',
        'if self.execution_id != intent.execution_id',
        'if self.request_commitment != intent.request_commitment',
        'ProviderOutcomeEvidence::KnownSuccess',
        'ProviderOutcomeEvidence::KnownNoEffect',
        'ProviderOutcomeEvidence::UnknownOutcome',
        'ProviderIntegrityFault::SuccessContradictsNoEffect',
        'ProviderIntegrityFault::NoEffectContradictsSuccess',
        'ProviderIntegrityFault::ConflictingSuccessEvidence',
        'RegistryIntegrityFault::ExecutionIdentityReuse',
        'RegistryIntegrityFault::OperationKeyCollision',
    ]
    for needle in required:
        if needle not in s: fail(f"source invariant missing: {needle}")
    op_start = s.index('fn compute_operation_key(&self)')
    op_end = s.index('fn compute_intent_commitment(&self)', op_start)
    operation_fn = s[op_start:op_end]
    for forbidden in ("committed_at_unix_ms", "observed_at_unix_ms", "payment_id", "provider_receipt_id"):
        if forbidden in operation_fn: fail(f"nonsemantic field entered provider operation key: {forbidden}")
    if "capability" in operation_fn.lower(): fail("capability authority entered operation-key derivation")


def rejected(name: str, fn) -> None:
    try:
        fn()
    except (ValueError, KeyError, TypeError):
        return
    raise AssertionError(f"mutation survived: {name}")


def self_test(p: dict) -> None:
    def mutate(name, f):
        def run():
            q = copy.deepcopy(p); f(q); validate_profile(q)
        rejected(name, run)
    mutate("wall-clock-key", lambda q: q["provider_operation_identity"].__setitem__("wall_clock_participates", True))
    mutate("payment-id-key", lambda q: q["provider_operation_identity"].__setitem__("payment_id_participates", True))
    mutate("drop-request-binding", lambda q: q["observation_binding"].__setitem__("request_commitment", False))
    mutate("allow-last-write-wins", lambda q: q["registry_rules"].__setitem__("last_write_wins", True))
    mutate("drop-execution-uniqueness", lambda q: q["registry_rules"].__setitem__("one_provider_operation_per_execution_id", False))
    mutate("permit-capability-mint", lambda q: q["activation"].__setitem__("capability_minting_path", True))
    mutate("wire-payments", lambda q: q["activation"].__setitem__("payments_zome_wired", True))
    mutate("enable-replay", lambda q: q["activation"].__setitem__("automatic_replay_enabled", True))
    mutate("drop-conflict-halt", lambda q: q["state_rules"].__setitem__("success_and_no_effect_conflict_halts", False))


def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--self-test", action="store_true"); args = ap.parse_args()
    p = json.loads(PROFILE.read_text()); s = SOURCE.read_text()
    validate_profile(p); validate_source(s)
    if args.self_test: self_test(p)
    canonical = json.dumps(p, sort_keys=True, separators=(",", ":")).encode()
    print(json.dumps({"validated":True,"self_test":args.self_test,"profile_id":p["profile_id"],"canonical_profile_sha256":hashlib.sha256(canonical).hexdigest()}, sort_keys=True))
    return 0

if __name__ == "__main__": raise SystemExit(main())
