# Operational Authority Policy Verifier v0.1 — Normative Invariants

Status: **pure qualification candidate; not provisioned**

## 1. Policy semantics are not proof of adoption or currentness

A valid `AuthorityCoveragePolicy` or `CoverageTrustContextPolicy` is candidate semantics only.

Live use requires three different facts:

`exact semantic policy bytes ≠ authentic immutable policy record ≠ institutional adoption ≠ generation-bound currentness`.

This crate proves only authentic record + exact institutional adoption. #115 remains the authority for generation-bound currentness and #116 remains the exact policy-context composer.

## 2. Record authenticity and institutional adoption are independent

`VerifiedPolicyRecordProof` binds the exact policy digest/profile and exact nominated record reference under one record verifier.

It contains no institutional authority/proof fields.

`VerifiedPolicyAdoptionProof` binds the same exact policy digest/profile plus the policy's exact institutional authority ref and adoption proof ref under a separate authority verifier.

It contains no policy record reference or record-proof fields.

Neither proof domain can stand in for the other.

## 3. Stable policy identity is recomputed locally

Coverage policy identity is recomputed under `POLICY_IDENTITY_PROFILE`.

Context policy identity is recomputed under `CONTEXT_POLICY_PROFILE`.

Verifier receipts must bind that exact digest/profile. A provider cannot nominate one policy and verify another.

## 4. Semantic activity and generation currentness remain distinct

The policy must be semantically active at qualification time, but semantic expiry is not current authority.

A superseded/revoked policy may remain within its semantic time interval and still fail #115 currentness.

This layer does not inspect authority-state history and does not produce `QualifiedControlPlaneSubjectFreshness`.

## 5. Compatibility projection cannot widen verifier validity

The existing `VerifiedAuthorityCoveragePolicy` and `VerifiedCoverageTrustContextPolicy` ABIs have verification timestamps but no independent proof-expiry field.

Therefore compatibility projection is forbidden unless:

`policy.valid_until_ms <= min(record_proof.valid_until_ms, adoption_proof.valid_until_ms)`.

If either independent proof horizon is tighter, projection denies rather than silently allowing #94/#96/#116 to reuse the policy after its proof authority expired.

A future richer ABI may preserve a tighter proof lease without this liveness cost.

## 6. Positive results are non-deserializable

`QualifiedCoveragePolicyVerification` and `QualifiedContextPolicyVerification` derive `Serialize` but not `Deserialize`.

Legacy deserializable policy receipts are emitted only as compatibility projections after local qualification.

## 7. Stable identity vs dynamic evidence

Stable qualification commits the exact semantic policy digest/profile.

Dynamic evidence additionally commits record location/proof, record-verifier identity, adoption-verifier identity, verification references, timestamps and horizons.

Re-verification can refresh evidence without minting a new semantic policy authority domain.

## 8. Containment

This crate contains no HDK calls, DHT reads, policy discovery, current-state lookup, persistence, challenge generation, source/witness selection, lifecycle mutation, external effects, reputation, Phi or model-score authority.

Before runtime provisioning, candidate discovery, record proof verification and adoption proof verification must remain separate roles, and #115 currentness must still be required.
