# Authority Current Freshness Verifier Runtime v0.5 — Normative Invariants

Status: **implemented provenance-closed composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`. The coordinator locally rechecks the statement digest, rejects legacy constitutional authority, fences root adoption with before/after reads, and rechecks the same constitutional head immediately before deployment qualification.

## 2. Root discovery and root authority remain separate

The root-manifest provider returns candidate semantics only. The runtime builds #148's adoption claim locally, accepts only `VerifiedBootstrapRootAdoptionProof` from the proof verifier, rechecks constitutional currentness, runs #148 locally and only then runs #111.

## 3. Dynamic evidence lease is not authority identity

`EvidenceLease` and `QualifiedEvidenceLeaseManifest` constrain reuse and record provenance only. They MUST NOT establish institutional permission, semantic currentness, source trust/completeness, quorum, execution authority, or permission for an external effect.

Refreshing a proof may change lease/provenance evidence while the same semantic #117 authority identity remains unchanged.

## 4. Leased source-head evidence is mandatory

The coordinator consumes only `authority_state_source_head_verifier::verify_source_head_leased` for currentness composition.

The source-head envelope is cross-bound to the independently reverified local challenge. Its exact source-head identity and verification reference become provenance contributors, while #94/#96/#91 still decide institutional source trust, coverage and complete current authority.

The historical unleased endpoint is not used here.

## 5. Leased witness evidence is mandatory

The coordinator consumes only `authority_state_witness_verifier::verify_witness_evidence_leased`.

Witness observation and trust-binding identities are recomputed locally. Parallel vector position is insufficient: every trust binding must name the exact recomputed witness-observation digest and share the qualification verification time before either may enter provenance.

Direct-source paths may legitimately contribute zero witness/trust-binding entries.

## 6. Transition proof domains remain separate and their leases are explicit

Transition discovery returns candidate `AuthorityStateTransition` bytes only. Exact record proof and institutional authority proof remain separate verifiers. The authoritative source reference comes only from the independently authenticated source head.

After #159 local qualification, each `VerifiedAuthorityStateTransition.lease_until_ms` becomes its own `AuthorityStateTransition` provenance contribution. #91 still decides contiguous complete lineage and exact endpoint equality.

## 7. Operational policy proof leases remain separate from generation currentness

The coordinator consumes only `authority_operational_policy_leased_provider::resolve_operational_policy_candidates_leased`.

The exact coverage-policy and context-policy identities and verification references are recorded as separate provenance roles, but both use the shared leased-policy bundle horizon because that is the exact outer contract transported by the leased policy composer.

Policy record/adoption verification is not generation currentness. #115 still proves the exact policies are current `Active` authority-state subjects before #116 may use them.

## 8. Control-plane provenance is hierarchical

Every control-plane probe contributes its detailed source/witness/trust/transition evidence. The resulting #115 `QualifiedControlPlaneSubjectFreshness` contributes an additional `ControlPlaneFreshness` entry whose lease is the complete probe-subtree lease intersected with #115 semantic freshness.

This preserves the exact subtree minimum even when a legacy lower-level ABI does not expose every internal challenge/context horizon separately.

## 9. Global evidence lease is closed before #117

Before operational currentness can qualify, the coordinator intersects:

- #111 bootstrap-root lease;
- operational policy proof bundle lease;
- the complete control-plane lease; and
- operational source/witness/transition evidence lease.

After #117 succeeds, its semantic freshness lease is intersected as well. This final pre-deployment lease is `composition_lease`.

No upstream verifier horizon may disappear from the only reusable positive result.

## 10. Provenance contributor set is constructed locally

Providers never submit `EvidenceLeaseContribution` or `QualifiedEvidenceLeaseManifest` objects.

The coordinator derives every contribution role, evidence digest/profile, verification reference and lease from already-qualified local objects and leased adapter outputs.

`provenance_contributor_set_constructed_locally = true`.

## 11. Required role/cardinality closure is exact

A canonical manifest is insufficient unless it covers the exact authority theorem that ran.

Before manifest qualification the coordinator requires exactly:

- one `BootstrapRoot`;
- one `OperationalCoveragePolicy`;
- one `OperationalContextPolicy`;
- one `SourceHead` per control-plane probe plus one operational source head;
- one `WitnessObservation` per actually qualified witness;
- one `WitnessTrustBinding` per actually qualified trust binding;
- one `AuthorityStateTransition` per actually qualified transition;
- one `ControlPlaneFreshness` per #115 result; and
- one final `OperationalFreshness`.

The expected counts are derived from the already-qualified runtime objects, not from the contribution list itself. Missing or extra roles deny before manifest qualification.

## 12. Canonical provenance manifest must reproduce the global lease

The coordinator locally calls `qualify_evidence_lease_manifest` only after role/cardinality closure.

The resulting canonical manifest MUST have an aggregate lease exactly equal to the separately computed `composition_lease`.

This double-entry check prevents the provenance list and the runtime lease accumulator from drifting apart silently.

The final `OperationalFreshness` contribution carries the complete pre-deployment composition lease, while detailed lower-level contributions retain forensic attribution.

## 13. Manifest identity is audit evidence, not semantic authority

The wire receipt exposes:

- manifest digest/profile;
- contributor count; and
- the manifest aggregate lease.

Changing contributor identity/reference/role/horizon or adding/removing a contributor changes manifest identity subject to the hash assumption. Input ordering has no meaning.

The manifest digest MUST NOT be substituted for #117 semantic authority identity or #154 deployment authority identity.

## 14. Deployment is capped by the global evidence lease

After host `dna_info()` and the final binding-constitution recheck, the canonical manifest aggregate must still be live.

The five-second deployment window is only an additional requested cap:

`host_context.valid_until = min(manifest.aggregate.valid_until, final_now + 5 seconds)`.

#154 then independently intersects host context with #117 semantic freshness. Deployment/wire freshness may never outlive the resulting capped evidence horizon.

## 15. Host DNA remains independent deployment evidence

The coordinator derives local DNA only through `dna_info()?.hash.to_string()` and compares it with the binding constitutional plane. Host DNA is not read from a provider-controlled receipt.

Semantic authority identity and deployment authority identity remain separate.

## 16. No latest-record heuristic

Highest generation, newest timestamp, arrival order, absence of a later DHT record, author identity, reputation, Phi or model output cannot establish current authority.

Discovery is location only; proof verification, institutional authorization, completeness, currentness and deployment remain separate facts.

## 17. Fail closed

Missing/failed constitution, adoption, policy, probe, source, witness, transition, host-DNA or proof boundary; malformed/expired lease; changed constitutional head; provenance role/cardinality mismatch; duplicate provenance contribution; provenance/global-lease mismatch; failed pure qualification; or deployment horizon widening MUST deny.

## 18. Deliberately unprovisioned

The current-freshness verifier and new leased/proof roles remain absent from `dna.yaml`. No external effect is enabled and `operational = false` remains explicit.

## 19. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- #148/#154/#159/#181/#192 pure tests;
- leased source/witness/policy endpoint integration;
- transition proof-domain separation;
- truncated/forked/reordered/omitted lineage denial;
- witness/trust-binding cross-pair substitution denial;
- provenance omission/extra-role/duplicate-contribution denial;
- canonical manifest order-independence;
- manifest aggregate equals independently computed global lease;
- constitutional race and host-DNA mismatch denial; and
- adversarial multi-agent tests proving stale or incomplete evidence cannot produce deployment-bound live authority.
