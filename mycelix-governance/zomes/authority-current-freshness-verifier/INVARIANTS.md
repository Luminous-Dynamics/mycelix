# Authority Current Freshness Verifier Runtime v0.11 — Normative Invariants

Status: **implemented leased-currentness/provenance/full-currentness deployment composition candidate; deliberately unprovisioned in the binding governance DNA**

## 1. Binding constitutional currentness has its own verifier role

Current constitutional truth on the active authority path comes only from `constitution_currentness_verifier::get_leased_current_constitution(())`.

The current-freshness coordinator MUST NOT consume `constitution_transition::get_verified_current_constitution`, infer currentness from candidate/DHT visibility, or construct a constitutional freshness window itself.

The designated verifier is genesis-only while amendments are disabled. Amendment currentness without complete transition state plus bounded verifier horizons denies.

`leased_binding_constitution_consumed = true`.

`synthetic_constitution_lease_constructed_locally = false`.

`amendment_currentness_accepted_without_bounded_transition_lease = false`.

`constitution_candidate_discovery_grants_authority = false`.

## 2. One shared currentness contract owns evidence shape

Producer and consumer use `mycelix-governance-constitution-currentness::LeasedVerifiedCurrentConstitution` directly.

The consumer reruns `validate_at(now)` after every direct local verifier call. A valid wire object is evidence-shaped data, not authority merely because it deserializes.

There is no local mirror struct, duplicate currentness hash function, duplicate protocol constant, or consumer-created lease/reference.

## 3. Constitutional semantics and currentness evidence are different identities

`statement_digest` identifies the constitutional semantic epoch.

`currentness_evidence_digest/profile` identifies one exact bounded observation of that epoch.

Refreshing an unchanged DNA genesis preserves statement identity and changes currentness-evidence identity.

The #192 `CurrentConstitution` contributor uses the exact currentness evidence digest/profile/reference/lease; it MUST NOT substitute the statement digest.

## 4. Root establishment remains constitution + adoption + local #111 qualification

The root-manifest provider supplies candidate semantics only. The runtime builds #148's adoption claim locally, calls the separate adoption proof verifier, rechecks leased constitutional currentness, runs #148 locally, then runs #111.

`resolve_root()` returns exactly three root-establishment provenance contributions:

1. `CurrentConstitution` — exact leased currentness evidence;
2. `BootstrapRootAdoption` — exact #148 adoption evidence; and
3. `BootstrapRoot` — exact #111 qualified root.

The root lease is created at this boundary and may not outlive constitution/adoption evidence.

## 5. Dynamic evidence lease is not authority identity

`EvidenceLease` and `QualifiedEvidenceLeaseManifest` constrain reuse and record evidence provenance only. They do not establish institutional permission, semantic currentness, source trust/completeness, quorum, execution authority, or effect permission.

The runtime consumes evidence-lease protocol v0.2 with explicit `CurrentConstitution` and `BootstrapRootAdoption` roles.

## 6. Leased source, witness and policy evidence are mandatory

The coordinator consumes only:

- `authority_state_source_head_verifier::verify_source_head_leased`;
- `authority_state_witness_verifier::verify_witness_evidence_leased`; and
- `authority_operational_policy_leased_provider::resolve_operational_policy_candidates_leased`.

The older unleased endpoints are not accepted on the active currentness path.

## 7. Transition proof domains remain separate

Transition discovery yields candidate bytes only. Immutable-record proof and institutional-authority proof are separate verifier roles. Authoritative source identity comes only from the independently authenticated source head.

After local #159 qualification every transition contributes its exact identity/reference/lease, and #91 still decides contiguous complete lineage and endpoint equality.

This operational transition theorem grants no constitutional amendment-currentness authority.

## 8. Operational policy proof validity remains separate from generation currentness

Policy record/adoption proof validity does not prove current generation state. #115 must still prove the exact policy subjects are currently `Active`, and #116 joins those exact current policies to operational context.

## 9. Global composition lease is monotone

Before #117, the coordinator intersects:

- root evidence lease;
- leased operational policy proof horizon;
- all control-plane source/witness/transition horizons; and
- operational source/witness/transition horizons.

After #117, semantic freshness is intersected too.

No upstream evidence horizon may disappear before the only reusable positive result leaves the coordinator.

## 10. #192 contributor closure is exact

Providers never supply provenance entries.

The coordinator derives every `EvidenceLeaseContribution` locally and requires exact cardinalities for root, policies, sources, witnesses, trust bindings, transitions, control-plane freshness and final operational freshness.

Missing or extra contributors deny.

## 11. Canonical provenance must reproduce the independently accumulated lease

The runtime calls `qualify_evidence_lease_manifest` only after exact role/cardinality closure.

`provenance.aggregate_lease() == composition_lease` is mandatory.

The canonical manifest itself must be live before the final constitutional re-observation and must be revalidated again after the final constitution/host observations.

## 12. Composition provenance is not deployment-fence evidence

#192 explains which evidence established #117 currentness.

The final constitutional currentness re-observation occurs later, after #192 closure, specifically to detect constitutional change during composition.

It is therefore a separate deployment-evidence phase and is not retroactively inserted into #192.

## 13. Final deployment consumes the full shared currentness object

The v0.11 active path MUST NOT project the final `LeasedVerifiedCurrentConstitution` into `VerifiedCurrentConstitutionReceipt` before selecting final deployment evidence identity.

Instead it calls:

`qualify_currentness_binding_constitution_context(&final_constitution, root, constitution_now)`.

That pure sibling:

- reruns the shared currentness validator;
- internally derives the historical receipt only as a compatibility step;
- invokes the reviewed #217 root-bound qualifier;
- cross-checks statement/DNA/reference/timing; and
- constructs a non-deserializable full-currentness context whose digest explicitly commits `currentness_evidence_digest/profile`.

`final_constitution_context_qualified_locally = true`.

`final_constitution_currentness_evidence_explicit = true`.

## 14. Exact #111 root binding remains mandatory

The final context consumes the exact locally qualified #111 root, not a caller-selected expected statement/root digest.

The compatibility qualifier requires the final constitutional epoch to match the rooted statement and commits the exact root qualification digest/profile.

The final deployment theorem also requires #117 semantic currentness to name that same root.

Thus `same constitutional statement != same qualified root`.

## 15. Active deployment path accepts no plain constitutional primitives

The v0.11 runtime calls only:

`qualify_operational_freshness_for_deployment_with_currentness_and_provenance`.

It MUST NOT call the older `qualify_operational_freshness_for_deployment`, `qualify_operational_freshness_for_deployment_with_provenance`, or `qualify_operational_freshness_for_deployment_with_constitution_and_provenance` directly on the active path.

The active theorem receives only non-deserializable positive contexts plus #117 semantic currentness and #192 provenance.

`plain_constitution_primitives_accepted_by_active_deployment_path = false`.

## 16. Stable deployment authority remains unchanged

The v0.11 result delegates `deployment_authority_digest/profile` through the v0.4 deployment sibling to #217/#201/#154.

The explicit currentness-evidence digest/profile does not enter stable deployment authority identity.

Refreshing only final currentness observation evidence while semantic statement + root + DNA remain identical preserves stable deployment authority.

## 17. Explicit final currentness evidence changes dynamic deployment evidence

The final full-currentness context digest commits:

- the reviewed #217 compatibility context digest/profile; and
- the exact shared currentness-evidence digest/profile.

The outer v0.4 deployment-evidence digest commits the #217 deployment evidence plus that full-currentness context.

Therefore refreshed final currentness evidence changes dynamic deployment evidence while stable authority remains unchanged.

`final_constitution_evidence_bound_into_deployment_evidence = true`.

## 18. Final currentness evidence can only narrow reuse

After #192 closure, final currentness re-observation and host DNA observation, the runtime chooses a fresh `deployment_now`.

It computes:

`host_context.valid_until = min(#192 aggregate validity, final currentness validity, deployment_now + 5 seconds)`.

The final deployment theorem revalidates/narrows again. No final currentness observation may widen the composition/global lease.

## 19. Runtime cross-checks three dynamic evidence views

Before serialization the runtime requires exact echo of:

### Composition evidence
- #192 manifest digest/profile/count;
- aggregate verified-at; and
- aggregate valid-until.

### Final constitutional deployment context
- full currentness-context digest/profile;
- exact verification reference;
- exact verified-at; and
- exact valid-until.

### Explicit final currentness evidence
- exact `binding_constitution_currentness_evidence_digest`; and
- exact `binding_constitution_currentness_evidence_profile`.

Mismatch in any view denies.

## 20. v0.11 wire receipt is transport/audit only

`CurrentOperationalFreshnessAuditReceipt` v0.11 exports composition evidence, final full-currentness context, explicit final currentness evidence, stable deployment identity, dynamic deployment evidence and final narrowed freshness.

Deserializing it is never positive authority.

`wire_receipt_transport_only = true`.

`caller_supplied_positive_currentness_accepted = false`.

`wire_receipt_grants_execution_authority = false`.

## 21. Host DNA remains independent deployment evidence

Local DNA comes only from `dna_info()?.hash.to_string()` and is not copied from constitutional currentness evidence.

The deployment theorem independently requires host/constitution DNA equality.

## 22. No latest-record heuristic

Highest generation/version, newest timestamp, DHT arrival order, absence of later data, author identity, reputation, stake, Phi or model output cannot establish current authority.

This applies independently to operational authority-state transitions and constitutional state.

## 23. Fail closed

Any missing/failed constitution-currentness verifier, constitution authority, adoption, policy, probe, source, witness, transition, host-DNA or proof boundary; malformed/expired evidence; unsupported amendment mode; currentness-evidence identity mismatch; root mismatch; provenance closure/aggregate mismatch; final full-currentness context mismatch; explicit final currentness echo mismatch; deployment evidence mismatch; or horizon widening denies.

## 24. Deliberately unprovisioned

The current-freshness verifier, constitution-currentness verifier and leased/proof roles remain absent from binding `dna.yaml`.

No lifecycle/effect runtime is enabled. `external_effects_enabled = false` and `operational = false` remain explicit.

## 25. Required causal order

The active runtime must preserve:

1. local #117 qualification;
2. exact #192 role/cardinality closure;
3. #192 aggregate equality;
4. final direct leased-currentness re-read;
5. equality with the root's constitutional epoch;
6. full-currentness context qualification against the exact #111 root;
7. local host DNA observation;
8. fresh `deployment_now`;
9. #192 revalidation at `deployment_now`;
10. host-context cap by #192 + final currentness + return horizon;
11. v0.4 currentness/provenance deployment qualification;
12. exact provenance/context/currentness echo checks; and
13. transport-only v0.11 audit projection.

The old `current_constitution_receipt` function may still be used inside `resolve_root()` for #111 compatibility but MUST NOT appear between the final shared currentness re-read and final deployment qualification.

## 26. Amendment currentness remains a separate future theorem

Enabling constitutional amendments requires a versioned currentness contract plus an authoritative source/head completeness theorem, bounded rights/tally/threshold verifier evidence, revocation/trust-expiry propagation, canonical provenance, fork ambiguity denial and partition/delayed-propagation adversarial qualification.

Until then, amendments-enabled currentness fails closed.
