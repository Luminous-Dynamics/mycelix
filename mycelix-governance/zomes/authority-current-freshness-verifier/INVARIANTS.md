# Authority Current Freshness Verifier Runtime v0.3 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`. The coordinator locally rechecks the statement digest and rejects legacy constitutional authority.

## 2. Root discovery and root authority remain separate

The root-manifest provider returns candidate semantics only. The runtime builds #148's adoption claim locally, accepts only `VerifiedBootstrapRootAdoptionProof` from the proof verifier, rechecks constitutional currentness, runs #148 locally and only then projects #111 adoption/root authority.

## 3. Source-head authentication is independent

Each probe is independently verified by #114. The source-head verifier independently reconstructs probe validity and #130 authenticates the exact challenged source response. Source authentication still does not establish institutional source trust or complete current authority by itself.

## 4. Transition discovery is not transition truth

`authority_state_transition_candidate_provider` may return candidate `AuthorityStateTransition` bytes only.

It cannot return `VerifiedAuthorityStateTransition`, record-proof validity, institutional-authorization validity, or a qualified transition.

`transition_discovery_grants_authority = false`.

A malicious or incomplete candidate set must fail later #159/#91 qualification rather than becoming current by observation.

## 5. Transition record proof and institutional authority proof are separate

For every candidate transition, the coordinator computes the exact transition identity digest itself and calls:

- `authority_state_transition_record_proof_verifier::verify_transition_record_proof`; and
- `authority_state_transition_authority_proof_verifier::verify_transition_authority_proof`.

The first may return only `VerifiedTransitionRecordProof`.

The second may return only `VerifiedTransitionAuthorityProof`.

Neither verifier may return a `VerifiedAuthorityStateTransition` or stand in for the other proof domain.

## 6. Transition authoritative source comes from the source head

The `authoritative_source_ref` consumed by #159 is taken only from:

`source_head.attestation.authoritative_source_ref`.

Candidate discovery and both transition proof verifiers cannot choose or rewrite it.

Thus:

`transition location ≠ record authenticity ≠ institutional authorization ≠ authoritative source identity`.

## 7. Transition qualification is local

After both transition proof calls, the coordinator samples host time and invokes #159 `qualify_authority_state_transition` locally.

Only its non-deserializable positive result may project the compatibility `VerifiedAuthorityStateTransition` consumed by #91.

Proof-verifier timestamps cannot be judged using a clock captured before verification work.

## 8. Complete lineage remains #91's authority boundary

Per-transition verification does not make a candidate set complete/current.

#91 still requires one contiguous generation chain, exact parent-transition digest linkage and endpoint equality with the independently authenticated/covered source head.

Provider order, highest generation, latest timestamp, DHT arrival order and absence of a locally observed later record have no authority meaning.

A valid transition prefix is not current authority.

## 9. Positive operational authority is reconstructed locally

The semantic positive path remains:

`binding constitution → #148 adoption → #111 root → #115 policy currentness → #116 operational policy context → #117 operational currentness`.

No provider-supplied serialized `Qualified*` value is accepted as authority.

## 10. Host DNA is independent deployment evidence

Only after #117 succeeds does the coordinator derive the local DNA through `dna_info()?.hash.to_string()`, re-project the binding constitution, confirm the root epoch remains identical and run #154 locally.

The constitutional DNA and host-local DNA are separate facts and must be exactly equal.

## 11. Semantic and deployment identities remain separate

The wire receipt preserves #117 semantic authority/evidence identity and separately exposes #154 deployment authority/evidence identity. Live consumers must use the deployment identity.

## 12. Leases only narrow

#148, #111, #115/#116/#117, #159 and #154 each independently narrow their evidence/authority lifetimes. No composer may lengthen any upstream horizon.

## 13. No latest-record heuristic

Highest generation, newest timestamp, arrival order, absence of a later DHT record, author identity, reputation, Phi or model output cannot establish current authority.

## 14. Fail closed

Missing constitution/adoption/probe/source/witness/transition-discovery/record-proof/authority-proof verifier, host `dna_info` failure, DNA mismatch, malformed/stale proof, changed constitutional head, failed pure qualification or expired lease denies.

## 15. Deliberately unprovisioned

The current-freshness verifier and new transition verifier roles remain absent from `dna.yaml`. No external effect is enabled and `operational = false` remains explicit.

## 16. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- #148/#154/#159 pure tests;
- candidate discovery returning only raw transition semantics;
- separate record-proof and institutional-authority proof verifier responses;
- source identity derived only from the authenticated source head;
- proof-verifier outage/substitution/stale-window denial;
- #91 truncated-prefix, fork, parent-digest and endpoint mismatch denial;
- direct/witness coverage paths;
- constitutional race and host-DNA mismatch denial; and
- adversarial multi-agent transition omission/insertion/reordering/replay tests.
