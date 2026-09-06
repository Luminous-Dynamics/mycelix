# Authority Current Freshness Verifier Runtime v0.9 — Normative Invariants

Status: **implemented constitution/provenance-bound deployment composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional authority is reused

Current constitutional truth comes only from `constitution_transition::get_verified_current_constitution(())`. The coordinator locally rechecks the statement digest, rejects legacy constitutional authority, fences root adoption with before/after reads, and rechecks the same constitutional head immediately before deployment qualification.

## 2. Root discovery and root authority remain separate

The root-manifest provider returns candidate semantics only. The runtime builds #148's adoption claim locally, accepts only `VerifiedBootstrapRootAdoptionProof` from the proof verifier, rechecks constitutional currentness, runs #148 locally and only then runs #111.

## 3. Root establishment provenance is explicit

`resolve_root()` constructs and returns exactly three root-establishment provenance contributions after positive local qualification:

1. `CurrentConstitution` — the exact current-constitution receipt entering #111;
2. `BootstrapRootAdoption` — #148's exact adoption evidence; and
3. `BootstrapRoot` — #111's exact qualified root.

The exact root lease is constructed at this same boundary and must be no wider than constitution/adoption evidence. Later composition consumes the returned lease/contributions directly rather than reconstructing them.

The before/after constitutional reads used only for TOCTOU fencing are process-order checks, not fabricated extra lease contributors.

`current_constitution_provenance_explicit = true`.

`root_adoption_provenance_explicit = true`.

`root_provenance_lease_constructed_at_root_boundary = true`.

## 4. Dynamic evidence lease is not authority identity

`EvidenceLease` and `QualifiedEvidenceLeaseManifest` constrain reuse and record currentness provenance only. They do not establish institutional permission, semantic currentness, source trust/completeness, quorum, execution authority, or external-effect permission.

The closed role vocabulary is protocol-versioned. This runtime consumes evidence-lease protocol v0.2 with explicit `CurrentConstitution` and `BootstrapRootAdoption` roles.

## 5. Leased source-head evidence is mandatory

The coordinator consumes only `authority_state_source_head_verifier::verify_source_head_leased` for currentness composition.

The source-head envelope is cross-bound to the independently verified local challenge. #94/#96/#91 still decide institutional source trust, coverage and complete current authority.

## 6. Leased witness evidence is mandatory

The coordinator consumes only `authority_state_witness_verifier::verify_witness_evidence_leased`.

Witness observation and trust-binding identities are recomputed locally. Parallel-vector position alone is insufficient: each trust binding must reference the exact recomputed observation digest and share its qualification verification time.

Direct-source paths may contribute zero witness/trust-binding entries.

## 7. Transition proof domains remain separate

Transition discovery returns candidate bytes only. Record proof and institutional authority proof remain independent verifier roles. The authoritative source reference comes only from the independently authenticated source head.

After local #159 qualification, every transition contributes its exact identity/reference/lease. #91 still decides contiguous complete lineage and exact endpoint equality.

## 8. Operational policy proof leases remain separate from generation currentness

The coordinator consumes only `authority_operational_policy_leased_provider::resolve_operational_policy_candidates_leased`.

Coverage/context policy identities are separate provenance roles. Their shared outer lease is the exact contract exported by the leased policy composer. Policy record/adoption validity is not current generation state: #115 must still prove those policy subjects are currently `Active` before #116 uses them.

## 9. Control-plane provenance is hierarchical

Each control-plane probe contributes detailed source/witness/trust/transition evidence. Each resulting #115 object additionally contributes a `ControlPlaneFreshness` entry whose lease is the full probe-subtree lease intersected with #115 semantic freshness.

This preserves exact subtree minima where legacy lower-level ABIs do not expose every internal horizon separately.

## 10. Global currentness-evidence lease is closed before deployment

Before #117, the coordinator intersects:

- exact returned root lease;
- operational policy proof bundle lease;
- complete control-plane lease; and
- operational source/witness/transition evidence lease.

After #117 succeeds, its semantic freshness lease is intersected too. This is `composition_lease`.

No upstream currentness-verifier horizon may disappear before the only reusable positive result leaves the coordinator.

## 11. Provenance contributor set is constructed locally

Providers never submit `EvidenceLeaseContribution` or `QualifiedEvidenceLeaseManifest` values.

The coordinator derives every role, evidence identity, verification reference and lease from already-qualified local objects and leased adapter outputs.

`provenance_contributor_set_constructed_locally = true`.

## 12. Required role/cardinality closure is exact

Before manifest qualification the coordinator requires exactly:

- one `CurrentConstitution`;
- one `BootstrapRootAdoption`;
- one `BootstrapRoot`;
- one `OperationalCoveragePolicy`;
- one `OperationalContextPolicy`;
- one `SourceHead` per control-plane probe plus one operational source head;
- one `WitnessObservation` per qualified witness;
- one `WitnessTrustBinding` per qualified trust binding;
- one `AuthorityStateTransition` per locally qualified transition;
- one `ControlPlaneFreshness` per #115 result; and
- one `OperationalFreshness`.

Expected counts come from qualified runtime objects, not the contribution list. Missing or extra roles deny.

## 13. Canonical currentness provenance must reproduce the global lease

The coordinator calls `qualify_evidence_lease_manifest` only after exact role/cardinality closure.

Its aggregate lease must equal the independently accumulated `composition_lease` exactly. This double-entry check prevents provenance and runtime lease arithmetic from silently drifting apart.

The canonical manifest itself must be qualified and live before the runtime performs its final binding-constitution re-observation.

## 14. Composition provenance is audit evidence, not semantic authority

Changing currentness contributor identity/reference/role/horizon changes manifest identity. Input order does not.

The manifest digest is never substituted for #117 semantic authority or stable deployment authority.

## 15. Final constitutional re-observation is a separate deployment phase

The canonical #192 manifest answers which evidence established #117 currentness.

The runtime MUST finish #117, exact #192 role/cardinality closure, canonical manifest qualification, and manifest/global-lease equality **before** performing the final binding-constitution read. That read is therefore the last external authority-plane observation before local host/deployment qualification.

The required temporal order is:

1. qualify #117 and intersect its semantic lease;
2. close and qualify the exact #192 contributor set;
3. require the #192 aggregate to equal `composition_lease`;
4. re-read the binding constitution;
5. require it to equal the constitutional head used for the bootstrap root;
6. construct a fresh `VerifiedCurrentConstitutionReceipt` from that exact final read;
7. call `qualify_binding_constitution_context` locally with that receipt **and the exact non-deserializable #111 `QualifiedAuthorityStateBootstrapRoot` object**;
8. observe local host DNA;
9. choose a fresh `deployment_now` after those observations;
10. revalidate the canonical #192 aggregate at `deployment_now`; and
11. run the final deployment theorem.

The final constitutional read is not retroactively inserted into #192. It belongs to the later deployment-fence evidence phase.

The resulting `QualifiedBindingConstitutionContext` is non-deserializable.

`final_constitution_context_qualified_locally = true`.

## 16. Final constitution context is cross-bound to the exact #111 root

The final constitution qualifier MUST NOT accept a caller-selected `expected_statement_digest` or other primitive substitute for the rooted authority object.

It consumes the exact locally qualified #111 root and requires:

- the final statement digest equals `root.current_constitution_digest()`;
- the root manifest commits the same statement digest/profile;
- the root qualification digest is non-zero;
- the root qualification profile is exactly `ROOT_QUALIFICATION_PROFILE`; and
- the final context identity commits that exact root qualification digest/profile.

The active deployment theorem then independently requires the #117 semantic currentness root qualification digest/profile to equal the root identity committed by the final constitution context.

Thus:

`same constitutional statement != same qualified root`.

A constitution context established under one #111 root cannot be paired with currentness from another root merely because both roots happen to name the same constitutional statement.

## 17. Active deployment path accepts no plain constitutional primitives

The active runtime MUST call only:

`qualify_operational_freshness_for_deployment_with_constitution_and_provenance`.

It MUST NOT directly call either:

- `qualify_operational_freshness_for_deployment`; or
- `qualify_operational_freshness_for_deployment_with_provenance`.

The active theorem receives the non-deserializable final constitution context. Plain constitutional DNA hash or statement digest values are not caller-selectable arguments at this boundary.

Likewise, the final constitution-context qualifier consumes the non-deserializable #111 root rather than a plain expected constitutional digest.

`plain_constitution_primitives_accepted_by_active_deployment_path = false`.

## 18. Final constitution evidence is bound into dynamic deployment evidence

The final deployment theorem delegates stable deployment authority through the existing #201/#154 path, then derives new dynamic deployment evidence that commits the exact `QualifiedBindingConstitutionContext` digest/profile.

That context digest itself commits the exact #111 root qualification identity plus the final constitutional statement/DNA observation and its timing/reference.

Therefore:

`same semantic authority + same qualified root + same constitution statement + same host DNA + refreshed final constitution observation`

means:

`same stable deployment authority + different dynamic deployment evidence`.

`final_constitution_evidence_bound_into_deployment_evidence = true`.

## 19. Final constitutional evidence can only narrow reuse

After the final constitution and host observations, the runtime chooses a fresh `deployment_now`. It revalidates the canonical #192 aggregate at that time and computes:

`host_context.valid_until = min(#192 aggregate validity, final constitution validity, deployment_now + 5 seconds)`.

The final deployment theorem again validates/narrows against the qualified constitution context and host context.

A final constitutional observation may never widen deployment authority or evidence lifetime. An observation that consumes enough time for #192 to expire causes denial rather than reuse.

## 20. Runtime cross-checks both evidence phases

Before serialization, the coordinator requires the final deployment object to echo exactly:

### Currentness composition evidence
- #192 manifest digest/profile/count;
- aggregate verification time; and
- aggregate validity horizon.

### Final deployment-constitution evidence
- binding-constitution context digest/profile;
- exact verification reference;
- exact verification time; and
- exact validity horizon.

Mismatch in either evidence phase denies.

## 21. Wire receipt is transport/audit only

`CurrentOperationalFreshnessAuditReceipt` exports both evidence phases separately along with stable/deployment identities and final narrowed freshness.

Deserializing that receipt is never positive authority by itself. Future lifecycle/signing/effect consumers must call the designated local currentness verifier for the exact target and perform their own domain/lease checks.

`wire_receipt_transport_only = true`.

`caller_supplied_positive_currentness_accepted = false`.

`wire_receipt_grants_execution_authority = false`.

## 22. Host DNA remains independent deployment evidence

Local DNA comes only from `dna_info()?.hash.to_string()` and is not copied from the constitutional receipt. The qualified deployment theorem still requires host/constitution DNA equality.

Semantic authority, stable deployment authority, currentness provenance, qualified-root-bound final constitution evidence and final dynamic deployment evidence remain distinct facts.

## 23. No latest-record heuristic

Highest generation, newest timestamp, arrival order, absence of later DHT data, author identity, reputation, Phi or model output cannot establish current authority.

Discovery, authentication, institutional trust, completeness, currentness, currentness provenance, deployment fencing and effect permission remain separate.

## 24. Fail closed

Any missing/failed constitution, adoption, policy, probe, source, witness, transition, host-DNA or proof boundary; malformed/expired lease; root-provenance containment failure; changed constitutional head; final constitution/root epoch mismatch; final constitution/root qualification mismatch; semantic-currentness/final-context root mismatch; provenance closure mismatch; provenance/global-lease mismatch; canonical provenance expiry during the final constitution/host fence; deployment evidence echo mismatch; or horizon widening denies.

## 25. Deliberately unprovisioned

The current-freshness verifier and new leased/proof roles remain absent from `dna.yaml`. No external effect is enabled and `operational = false` remains explicit.

## 26. Required qualification before provisioning

At minimum:

- native/Clippy/WASM qualification;
- bootstrap/adoption/evidence-lease/deployment-fence/transition pure tests;
- leased source/witness/policy integration;
- transition proof-domain separation;
- truncated/forked/reordered/omitted lineage denial;
- witness/trust substitution denial;
- root provenance and lease containment;
- exact #192 role/cardinality closure and aggregate equality;
- #192 qualification before the final binding-constitution read;
- #192 revalidation after final constitution/host observation;
- final constitution context/root qualification mismatch denial;
- semantic #117 root vs final-context root mismatch denial;
- final constitution context expiry denial;
- host context exceeding final constitution horizon denial;
- canonical provenance echo substitution denial;
- final constitution-context echo substitution denial;
- stable deployment authority unchanged by proof/observation refresh;
- dynamic deployment evidence changes when #192 provenance or final constitutional observation changes;
- constitutional race and host-DNA mismatch denial; and
- adversarial multi-agent tests proving stale/incomplete evidence cannot produce live deployment-bound authority.
