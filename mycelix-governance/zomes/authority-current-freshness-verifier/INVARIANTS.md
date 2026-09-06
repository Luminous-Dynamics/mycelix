# Authority Current Freshness Verifier Runtime v0.1 — Normative Invariants

Status: **implemented composition candidate; deliberately unprovisioned in the binding governance DNA**

This runtime answers one question:

> Does one exact operational authority subject have a current, covered, generation-bound freshness state under the exact current constitution-rooted policy plane?

It does not own policy, constitution, root adoption, state transitions, source truth, witness truth, probe entropy, lifecycle claims, or external effects.

## 1. Positive authority is reconstructed locally

Cross-zome roles may return evidence-shaped data or role-specific verification receipts only.

The coordinator reconstructs positive authority through #111 → #115 → #116 → #117. No provider may return a deserialized `Qualified*` value and have it treated as authority by existence.

## 2. Bootstrap manifest discovery is not bootstrap authority

`authority_state_bootstrap_root_manifest_provider` may return one semantic `AuthorityStateBootstrapRootManifest` candidate only.

It MUST NOT return `VerifiedCurrentConstitutionReceipt`, `VerifiedBootstrapRootAdoption`, or `QualifiedAuthorityStateBootstrapRoot`.

`root_manifest_provider_grants_authority = false`.

## 3. Current constitution is independently discovered and verified

`authority_current_constitution_verifier::verify_current_constitution` receives `()`.

It receives **no selector derived from the candidate manifest**: no network ID, institution ID, constitution ID, desired version, statement digest, rulebook digest, or positive root/adoption receipt.

The verifier must derive the applicable local constitutional domain and current head from its own trusted runtime/constitution context and return `VerifiedCurrentConstitutionReceipt`.

Therefore the manifest provider cannot steer either which constitutional domain or which constitutional version is treated as current.

The returned current statement is later compared against the candidate manifest only inside local #111 qualification.

## 4. Root adoption is independently verified

`authority_bootstrap_root_adoption_verifier` receives the exact candidate root manifest plus the current constitutional statement digest/profile returned by the independent constitution verifier.

It MUST NOT receive the surrounding `VerifiedCurrentConstitutionReceipt` as positive authority.

It returns only `VerifiedBootstrapRootAdoption` for that exact tuple. #111 then independently recomputes and cross-binds the manifest, current statement, rulebook and adoption evidence.

The root-adoption verifier does not decide constitutional currentness; the current-constitution verifier does not decide root adoption.

## 5. No omnibus bootstrap oracle

The old `authority_state_bootstrap_root_provider::resolve_bootstrap_root_candidates` bundle is forbidden.

No single role may supply all three of root manifest, current-constitution verification and root-adoption verification.

A positive bootstrap root requires the candidate role, both independent verifiers and local #111 qualification.

## 6. Discovery and operational verification are different roles

`authority_state_evidence_plan_provider` is discovery-only. Evidence-plan selection grants no authority.

## 7. Probe verification is independent

Every planned probe action is sent to `authority_state_challenge::verify_issued_authority_state_probe`. Probe authorship remains provenance only.

## 8. Source-head verification independently reconstructs probe validity

The source-head verifier receives the exact `probe_action: ActionHash`, not this coordinator's serialized positive `VerifiedCoverageChallenge`, and independently invokes #114 before source authentication.

## 9. Witness/trust verification is independent

The exact verified challenge + exact verified source head are sent to `authority_state_witness_verifier::verify_witness_evidence`.

Its output is bounded to at most 64 witnesses and 64 trust bindings. #96 still owns exact trust-policy/verifier binding and independence checks.

## 10. Transition verification is independent

The exact target subject + exact verified source head are sent to `authority_state_transition_verifier::verify_transition_lineage`.

Its output is bounded to 1–256 transitions. #91 still requires a contiguous, fork-free, parent-digest-linked chain ending at the independently covered head.

A valid transition prefix is not current authority.

## 11. Qualification time follows evidence production

For the root, causal ordering is:

`manifest candidate → zero-input current-constitution verification → root-adoption verification → root qualification time → #111`.

Control-plane/context/operational qualification retains the evidence-before-clock ordering from #133.

A timestamp captured before asynchronous verifier work MUST NOT be reused for later qualification.

## 12. Operational policy semantics and policy currentness are separate

The policy provider returns semantic policy receipts. Currentness of those exact policy objects comes only from #115 proofs rooted in #111.

Missing, extra, revoked, superseded, stale, wrong-root, or wrong-namespace policy evidence denies.

## 13. Covered namespace and registry namespace remain distinct

`coverage_policy.namespace` controls which operational subjects may be covered. `root.control_plane_namespace` controls where policy objects' own authority state lives.

## 14. No latest-record heuristics

The runtime MUST NOT infer currentness from highest observed generation, newest timestamp, DHT arrival order, absence of a later record, cached local state, first/last link order, reputation, Phi, or model output.

## 15. Stable authority and evidence provenance remain separate

The runtime preserves #117's stable `authority_digest/profile` and fresh `evidence_digest/profile` distinction.

## 16. Fail closed on every role boundary

Missing zome/function, non-OK call response, decode failure, malformed evidence, failed pure qualification, stale lease, mismatched subject, impossible evidence fan-in, or unavailable independent verifier MUST deny.

`operational = false` remains declarative.

## 17. Historical/live separation

Only current projection paths may contribute to this runtime. Historical/as-of evidence cannot become a live freshness receipt.

## 18. Deliberately unprovisioned

`authority_current_freshness_verifier` remains absent from `mycelix-governance/dna/dna.yaml`.

The manifest/current-constitution/root-adoption/policy/source/witness/transition roles are not production-qualified here.

## 19. No external-effect authority

This coordinator cannot create grants, mutate authority transitions, sign governance actions, create lifecycle claims, execute actions, or authorize an external effect.

## 20. Required qualification before provisioning

At minimum:

- rustfmt, native check, warnings-denied Clippy and WASM build;
- proof that current-constitution verification is zero-input and receives no manifest-derived selector;
- proof that the adoption verifier does not receive `VerifiedCurrentConstitutionReceipt`;
- proof that the old omnibus bootstrap provider cannot reappear;
- root verifier ordering audit;
- source-head request ABI proof: probe action only;
- qualification-time ordering audit at all layers;
- old-constitution replay and wrong-adoption denial;
- hidden-later-revocation and truncated-transition denial;
- source/policy/verifier substitution and outage denial;
- direct-source and witness-quorum paths;
- proof that no latest-DHT heuristic exists; and
- multi-agent Sweettests showing stale/partitioned evidence cannot produce live execution authority.

No external effects are enabled.
