# Operational Policy Provider Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Existing current-freshness ABI is preserved

The public function remains:

`resolve_operational_policy_candidates(AuthoritySubjectRef) → OperationalPolicyCandidateBundle`.

The response still contains the legacy `VerifiedAuthorityCoveragePolicy` and `VerifiedCoverageTrustContextPolicy` expected by the current-freshness coordinator.

Those receipts are now compatibility projections produced locally after #172 qualification. No downstream caller needs to trust a raw provider-generated verified receipt.

## 2. Policy discovery is not authority

`authority_operational_policy_candidate_provider` may return only:

- raw `AuthorityCoveragePolicy` semantics + nominated record ref; and
- raw `CoverageTrustContextPolicy` semantics + nominated record ref.

It cannot return verified policy receipts, record-proof receipts, adoption-proof receipts, currentness receipts or qualified policy objects.

`policy_discovery_grants_authority = false`.

## 3. Target and cross-policy semantics are checked before proof qualification

The coordinator locally validates both policy candidates.

The coverage policy must cover the exact target namespace/kind.

The context policy must bind the exact locally recomputed coverage-policy digest/profile.

#116 later performs the complete root/institution/rulebook/witness-policy/currentness join; these early checks do not replace it.

## 4. Policy record proof is separate

For each exact policy identity, the nominated record ref is sent to `authority_operational_policy_record_proof_verifier`.

It may return only `VerifiedPolicyRecordProof`, authenticating the exact digest/profile at the exact record location.

It cannot claim institutional adoption or generation currentness.

## 5. Institutional adoption proof is separate

The exact policy digest/profile plus the policy's exact `authority_ref` and `policy_proof_ref` are independently sent to `authority_operational_policy_adoption_proof_verifier`.

It may return only `VerifiedPolicyAdoptionProof`.

It cannot authenticate a record location or claim generation currentness.

Thus:

`policy discovery ≠ record authenticity ≠ institutional adoption ≠ currentness`.

## 6. #172 qualification is mandatory and local

After both proof-verifier calls for one policy, the coordinator samples host time and invokes the appropriate #172 pure qualifier locally.

Only a non-deserializable qualified policy verification may project the legacy verified receipt.

#172 also forbids projection when the legacy receipt would outlive either independent proof horizon.

## 7. Currentness remains #115/#116

This runtime intentionally does not inspect authority-state transition history and does not produce `QualifiedControlPlaneSubjectFreshness`.

The current-freshness coordinator still requires #115 current `Active` proofs for the exact policy identities and #116 reconstructs the exact target-specific policy context.

`generation_currentness_decided_here = false`.

## 8. Fail closed

Candidate discovery failure, malformed candidate, target mismatch, coverage/context identity mismatch, record-verifier outage, adoption-verifier outage, stale/wrong proof, lossy legacy projection or failed #172 qualification denies.

## 9. Deliberately unprovisioned

The coordinator is a Rust workspace member only. `mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_operational_policy_provider` in this tranche.

Its candidate/record/adoption adapters are not production-qualified here.

No external effects are enabled and `operational = false` remains explicit.
