# Authority-State Witness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Existing caller ABI is preserved

The public `verify_witness_evidence` request/response remains the contract already consumed by the current-freshness runtime:

`VerifiedCoverageChallenge + VerifiedAuthoritySourceHead → witnesses + trust bindings`.

This runtime does not reinterpret receipt transport as source authority. `source_head_authenticated_here = false` is explicit: the caller is expected to obtain that source head through the independent #130/#131 authentication path.

## 2. Context discovery is digest-constrained candidate lookup

The context candidate provider receives only the exact context-policy digest/profile already committed by the verified challenge.

It returns raw `CoverageTrustContextPolicy` candidate semantics only.

The coordinator validates and recomputes the candidate identity and requires exact equality with the challenge digest/profile before using it.

Therefore context lookup may locate bytes but cannot choose another context policy.

`context_discovery_grants_authority = false`.

Currentness of the semantic policy remains the #115/#116/#96 responsibility; this runtime does not create policy currentness.

## 3. DirectSource does not invoke witness authority

If the exact context contains no witness-trust policy/verifier, this runtime returns an empty witness/trust set without probing witness providers.

Code presence does not create a witness requirement.

A semantic mismatch with the actual coverage mode still denies later in #94/#96.

## 4. Witness discovery is not witness truth

`authority_state_witness_candidate_provider` receives an exact discovery tuple derived from the challenge and source-head input and may return `AuthorityHeadWitnessObservation` candidates only.

It cannot return `VerifiedAuthorityHeadWitness`, `VerifiedWitnessTrustBinding`, observation-proof receipts or trust classifications.

`witness_discovery_grants_authority = false`.

## 5. Observation proof verification is separate

For every candidate observation, the coordinator computes the exact observation identity locally and calls `authority_state_witness_observation_proof_verifier`.

That verifier may return only `VerifiedWitnessObservationProof`.

It authenticates exact observation bytes, observer identity and observation proof; it does not classify trust domains.

## 6. Trust classification verification is separate and non-steerable

The trust-classification request contains only:

- exact observer ID;
- exact witness-trust policy identity; and
- exact designated trust-verifier ref.

It intentionally contains no requested `trust_domain_id` or `trust_domain_ref`.

The classifier must return the independently verified observer→domain result under that policy/verifier.

It cannot authenticate the observation or construct #96's trust binding.

## 7. #168 qualification is mandatory and local

Only after observation-proof verification and trust-classification verification does the coordinator sample host time and run `qualify_witness_evidence` locally.

#168 independently binds the context, challenge, source head, observation, observer, classified domain and proof references and applies the no-validity-widening projection rule.

Neither verifier nor discovery provider can directly create either compatibility receipt.

## 8. Trust binding is constructed locally

`VerifiedWitnessTrustBinding` exists only as the projection of a non-deserializable `QualifiedWitnessEvidence` obtained locally.

`trust_binding_constructed_locally = true`.

The classifier cannot choose a different context/challenge/observation association because #168 constructs and digests that association itself.

## 9. Quorum remains downstream

This runtime qualifies witnesses independently. It does not decide whether the returned set satisfies minimum witness count, trust-domain diversity or concentration rules.

#94/#96 remain the quorum/closed-set authority boundary.

`quorum_decided_here = false`.

## 10. Fail closed

Context lookup failure, candidate decode failure, observation-proof verifier failure, trust-classification verifier failure, stale proof, identity mismatch, wrong domain, wrong verifier, wrong challenge/source binding or lossy legacy projection denies.

No DHT absence/latest-record heuristic creates positive witness authority.

## 11. Deliberately unprovisioned

This coordinator is a Rust workspace member for native/Clippy/WASM qualification only.

`mycelix-governance/dna/dna.yaml` MUST NOT contain `authority_state_witness_verifier` in this tranche.

The context/candidate/proof/classification providers are not production-qualified by this tranche.

No external effects are enabled and `operational = false` remains explicit.
