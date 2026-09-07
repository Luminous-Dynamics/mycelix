# Operational Deployment Fence v0.4 — Normative Invariants

Status: **pure qualification kernels; explicit final constitutional-currentness evidence; no runtime provisioning and no external effects**

This layer answers four deliberately separate questions:

1. Is one locally qualified #117 operational-currentness proof live in this exact host DNA and constitutional epoch?
2. Is that deployment backed by one exact canonical #192 currentness-evidence provenance manifest?
3. Did the final deployment decision use one exact, live, locally qualified binding-constitution re-observation cross-bound to the exact qualified bootstrap root?
4. Does final dynamic deployment evidence commit the explicit shared constitutional-currentness evidence identity, rather than preserving it only indirectly through a verification-reference string?

None of these theorems grants permission for an external effect.

## 1. Semantic currentness must already exist locally

The positive semantic input is `&QualifiedOperationalSubjectFreshness` from #117.

No deployment theorem accepts transportable `VerifiedAuthorityFreshness`, provider-created `Qualified*` bytes, or an omnibus evidence bundle as a substitute.

## 2. Host DNA and constitutional evidence remain independent

Host DNA comes from the local runtime/cell. Constitutional DNA comes from independently verified constitutional currentness evidence.

Stable deployment authority requires exact equality between those identities, but neither source may substitute for the other.

`HostLocalDnaContext` remains non-deserializable.

## 3. Original #154 compatibility theorem remains unchanged

`qualify_operational_freshness_for_deployment` remains available with its historical primitive constitutional inputs.

It is retained only as the compatibility theorem underlying stronger siblings. The active current-freshness runtime MUST NOT call it directly.

Stable `deployment_authority_digest/profile` continues to commit exact #117 semantic authority, bootstrap root, operational context, constitutional statement and host DNA.

## 4. Provenance-bound deployment remains a sibling theorem

`qualify_operational_freshness_for_deployment_with_provenance` consumes the non-deserializable canonical #192 evidence-lease manifest and delegates stable authority to #154.

Canonical composition provenance changes dynamic deployment evidence only, never stable deployment authority.

## 5. #217 binding-constitution context remains a compatibility theorem

`QualifiedBindingConstitutionContext` remains non-deserializable and is qualified from:

- one live `VerifiedCurrentConstitutionReceipt`; and
- the exact non-deserializable `QualifiedAuthorityStateBootstrapRoot` already used by the currentness stack.

It cross-binds statement/DNA/timing/reference to the exact #111 root and remains the reviewed compatibility theorem beneath the v0.4 sibling.

The active v0.11 runtime MUST NOT project its final shared constitutional-currentness object into this older receipt before choosing the final deployment evidence identity.

## 6. Full currentness context is non-deserializable

`QualifiedCurrentnessBindingConstitutionContext` derives `Serialize` but not `Deserialize`.

It may be produced only by `qualify_currentness_binding_constitution_context` from:

- the canonical shared `LeasedVerifiedCurrentConstitution`; and
- the exact non-deserializable #111 `QualifiedAuthorityStateBootstrapRoot`.

Qualification first calls the shared currentness contract's `validate_at(now_ms)`, then internally derives the older receipt only to run the reviewed #217 compatibility qualifier.

The caller cannot supply a plain expected statement digest, currentness digest, verification reference, lease basis, DNA hash, or compatibility context.

## 7. Explicit currentness evidence survives the final fence

The full currentness context commits:

- the exact #217 compatibility-context digest/profile; and
- the exact shared `currentness_evidence_digest/profile`.

The shared currentness evidence digest already commits exact DNA, statement epoch, lease basis and observation window.

Therefore a refreshed currentness observation under unchanged constitutional semantics/root changes the full currentness-context identity even though stable deployment authority does not change.

## 8. Root equality remains mandatory

The shared currentness object is internally projected to the historical receipt only after shared validation. #217 then requires exact equality with the #111 root:

`current statement/root epoch == rooted.current_constitution_digest + rooted.qualification_digest/profile`.

The later deployment theorem also requires #117 semantic currentness to name that same root.

Thus `same constitutional statement != same qualified root`.

## 9. Composition provenance and final currentness evidence remain causally separate

The #192 manifest describes evidence used to establish #117 currentness.

The final shared constitutional-currentness re-observation happens after exact #192 closure, specifically to fence deployment against a constitutional change during composition.

It is therefore not retroactively inserted into #192.

The causal chain remains:

`composition evidence -> #117 -> #192 closure -> final currentness re-observation -> host DNA -> deployment evidence`.

## 10. Active v0.4 theorem accepts no plain constitutional primitives

`qualify_operational_freshness_for_deployment_with_currentness_and_provenance` accepts only:

- `&QualifiedOperationalSubjectFreshness`;
- `&QualifiedCurrentnessBindingConstitutionContext`;
- `&HostLocalDnaContext`;
- `&QualifiedEvidenceLeaseManifest`; and
- `now_ms`.

It does NOT accept caller-selectable constitutional DNA, statement digest, currentness digest, currentness reference, or lease fields.

## 11. Stable deployment authority remains unchanged

`QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness::deployment_authority_digest/profile` delegates to the #217 result, which delegates through #201 to #154.

The explicit currentness context/digest does not enter that getter or stable authority hash domain.

Therefore refreshing only constitutional currentness evidence while semantic statement + root + DNA remain identical MUST preserve stable deployment authority.

## 12. Explicit currentness changes dynamic deployment evidence

The v0.4 dynamic deployment-evidence digest commits:

- the exact #217 constitution/provenance deployment-evidence digest/profile; and
- the exact full-currentness constitution-context digest/profile.

That context digest explicitly commits the canonical currentness-evidence digest/profile.

Thus the same stable authority with a refreshed final currentness observation produces a different dynamic deployment-evidence identity.

## 13. Currentness evidence can only narrow deployment reuse

The full currentness context must be live.

The host context MUST NOT outlive it. The v0.4 result remains narrowed to the minimum of the #217 deployment horizon and the full currentness context horizon.

No currentness observation may widen deployment reuse or predate the final evidence lease.

## 14. Provenance lease containment remains mandatory

The canonical #192 aggregate must remain live and contain the host/base deployment reuse window as required by the provenance sibling.

Final currentness evidence is an additional later fence, not a replacement for composition-provenance containment.

## 15. Audit output exposes explicit final currentness identity

The v0.11 runtime receipt exports, separately:

- #192 composition-provenance identity/timing;
- full final binding-constitution context identity/timing;
- final `binding_constitution_currentness_evidence_digest/profile`;
- stable deployment authority identity; and
- final dynamic deployment-evidence identity.

The runtime MUST cross-check those fields against the final non-deserializable deployment object before serialization.

## 16. All positive deployment contexts/results are non-deserializable

The following derive `Serialize` but not `Deserialize`:

- `HostLocalDnaContext`;
- `QualifiedDeploymentOperationalFreshness`;
- `QualifiedProvenanceBoundDeploymentOperationalFreshness`;
- `QualifiedBindingConstitutionContext`;
- `QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness`;
- `QualifiedCurrentnessBindingConstitutionContext`; and
- `QualifiedCurrentnessBoundProvenanceDeploymentOperationalFreshness`.

Another runtime/cell must reconstruct currentness, canonical provenance, host DNA and final constitutional currentness locally.

## 17. Pure separation

This crate contains no HDK/Holochain calls, DHT lookup, persistence, discovery, signature service, lifecycle mutation, execution action, reputation, Phi, stake, Guardian override or model-score authority.

The runtime owns host/constitution retrieval. This crate only qualifies relationships among evidence and non-deserializable positive objects already obtained through the appropriate trust boundaries.

## 18. Runtime acceptance target

The active current-freshness runtime must, in order:

1. locally construct #117 currentness;
2. close and qualify the exact #192 composition-provenance manifest and require aggregate equality;
3. re-read the canonical shared leased binding constitution and confirm the rooted epoch did not change;
4. call `qualify_currentness_binding_constitution_context` directly on that full shared object plus the exact #111 root;
5. obtain host DNA locally;
6. choose a fresh deployment qualification time after constitution + host observation;
7. revalidate the canonical #192 aggregate at that later time;
8. cap host reuse by the canonical provenance aggregate, final currentness horizon and deployment return cap;
9. construct non-deserializable `HostLocalDnaContext`;
10. call only `qualify_operational_freshness_for_deployment_with_currentness_and_provenance` on the active path;
11. require the result to echo the exact full currentness-context digest/profile, currentness-evidence digest/profile, verification reference and horizon; and
12. export audit evidence only from that final non-deserializable result.

The final active path MUST NOT call `current_constitution_receipt` between the final shared currentness read and deployment qualification.

No external effect is authorized by deployment qualification alone.
