# Operational Deployment Fence v0.3 — Normative Invariants

Status: **pure qualification kernels; no runtime provisioning and no external effects**

This layer now answers three deliberately separate questions:

1. Is one locally qualified #117 operational-currentness proof live in this exact host DNA and constitutional epoch?
2. Is that deployment backed by one exact canonical currentness-evidence provenance manifest?
3. Did the final deployment decision use one exact, live, locally qualified binding-constitution re-observation cross-bound to the exact qualified bootstrap root rather than caller-supplied constitutional primitives?

None of these theorems grants permission for an external effect.

## 1. Semantic currentness must already exist locally

The positive semantic input is `&QualifiedOperationalSubjectFreshness` from #117.

No deployment theorem accepts transportable `VerifiedAuthorityFreshness`, provider-created `Qualified*` bytes, or an omnibus evidence bundle as a substitute.

## 2. Host DNA and constitutional evidence remain independent

Host DNA comes from the local runtime/cell. Constitutional DNA comes from independently verified constitutional evidence.

Stable deployment authority requires exact equality between those identities, but neither source may substitute for the other.

`HostLocalDnaContext` remains non-deserializable.

## 3. Original #154 compatibility theorem remains unchanged

`qualify_operational_freshness_for_deployment` remains available with its historical primitive constitutional inputs.

It is retained as the compatibility theorem underlying stronger siblings. The active current-freshness runtime MUST NOT call it directly.

Stable `deployment_authority_digest/profile` continues to commit exact #117 semantic authority, bootstrap root, operational context, constitutional statement and host DNA.

## 4. Provenance-bound deployment remains a sibling theorem

`qualify_operational_freshness_for_deployment_with_provenance` consumes the non-deserializable canonical evidence-lease manifest and delegates stable authority to #154.

Canonical composition provenance changes dynamic deployment evidence only, never stable deployment authority.

## 5. Final binding constitution context is non-deserializable

`QualifiedBindingConstitutionContext` derives `Serialize` but not `Deserialize`.

It may be produced only by `qualify_binding_constitution_context` from:

- one exact live `VerifiedCurrentConstitutionReceipt`; and
- the exact non-deserializable `QualifiedAuthorityStateBootstrapRoot` already used by the currentness stack.

No caller-supplied expected statement digest is accepted by this qualifier.

Qualification requires:

- the current-constitution receipt validates at `now_ms`;
- exact statement digest equality with `rooted.current_constitution_digest()`;
- exact statement digest/profile equality with the qualified root manifest;
- exact root qualification profile `ROOT_QUALIFICATION_PROFILE`;
- non-zero root qualification identity;
- canonical `STATEMENT_PROFILE`;
- valid DNA identity;
- non-empty verification reference; and
- a live bounded verification window.

Its context digest commits the exact DNA, statement digest/profile, root qualification digest/profile, verification reference and verification/validity times.

## 6. Final constitution context is bound to the exact qualified bootstrap root

The binding constitution context is not merely “the same constitutional statement.” It commits the exact #111 root qualification identity that was used to build currentness.

The active deployment theorem additionally requires:

`semantic.root_qualification_digest/profile == constitution_context.root_qualification_digest/profile`.

A final constitution context qualified against one root cannot be reused with #117 currentness qualified under another root, even if both roots happen to commit the same constitutional statement.

## 7. Final constitutional re-observation is deployment evidence, not #192 composition provenance

The #192 manifest describes dynamic evidence used to establish #117 currentness.

The final constitutional re-observation happens after exact #192 closure, specifically to fence deployment against a constitutional change during composition. It therefore remains a separate deployment-evidence context rather than being retroactively inserted into the #192 currentness manifest.

This separation preserves causal meaning:

`currentness evidence -> #117 -> #192 closure -> final constitution fence -> deployment evidence`.

## 8. Active theorem accepts no plain constitutional primitives

`qualify_operational_freshness_for_deployment_with_constitution_and_provenance` accepts:

- `&QualifiedOperationalSubjectFreshness`;
- `&QualifiedBindingConstitutionContext`;
- `&HostLocalDnaContext`;
- `&QualifiedEvidenceLeaseManifest`; and
- `now_ms`.

It does NOT accept a plain constitutional DNA hash or statement digest from its caller.

Those primitive values are projected internally from the non-deserializable constitutional context only when invoking the older compatibility theorem.

## 9. Stable deployment authority remains unchanged

`QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness::deployment_authority_digest/profile` delegates through the provenance sibling to the original #154 result.

Therefore refreshing only the final binding-constitution observation while statement + root + DNA remain identical MUST NOT mint a new stable deployment authority identity.

## 10. Final constitutional observation changes dynamic deployment evidence

The final dynamic deployment-evidence digest commits:

- the exact provenance-bound deployment-evidence digest/profile; and
- the exact binding-constitution context digest/profile.

The context digest itself commits the rooted #111 identity, verification reference and exact observation horizon.

Thus the same stable authority with a refreshed final constitutional observation produces new dynamic evidence identity.

## 11. Constitution evidence can only narrow deployment reuse

The qualified binding-constitution context must be live.

The host context MUST NOT outlive it. The final result is narrowed to:

`verified_at = max(provenance-bound deployment, final constitution observation)`

`valid_until = min(provenance-bound deployment, final constitution observation)`.

No constitutional observation may widen deployment reuse.

## 12. Provenance lease containment remains mandatory

The canonical #192 aggregate must remain live and contain the host/base deployment reuse window as required by the provenance sibling.

Final binding-constitution evidence is an additional later fence, not a replacement for composition-provenance containment.

## 13. All positive deployment contexts/results are non-deserializable

The following derive `Serialize` but not `Deserialize`:

- `HostLocalDnaContext`;
- `QualifiedDeploymentOperationalFreshness`;
- `QualifiedProvenanceBoundDeploymentOperationalFreshness`;
- `QualifiedBindingConstitutionContext`; and
- `QualifiedConstitutionBoundProvenanceDeploymentOperationalFreshness`.

Another runtime/cell must reconstruct currentness, canonical provenance, host DNA and final constitutional context locally.

## 14. Pure separation

This crate contains no HDK/Holochain calls, DHT lookup, persistence, discovery, signature service, lifecycle mutation, execution action, reputation, Phi, stake, Guardian override or model-score authority.

The runtime owns host/constitution retrieval. This crate only qualifies relationships among already-provided evidence and non-deserializable positive objects.

## 15. Runtime acceptance target

The active current-freshness runtime must, in order:

1. locally construct #117 currentness;
2. close and qualify the exact #192 composition-provenance manifest and require aggregate equality;
3. re-read the binding constitution and confirm the rooted epoch did not change;
4. construct a fresh `VerifiedCurrentConstitutionReceipt` from that final read;
5. locally qualify `QualifiedBindingConstitutionContext` against the exact non-deserializable #111 root;
6. obtain host DNA locally;
7. choose a fresh deployment qualification time after constitution + host observation;
8. revalidate the canonical #192 aggregate at that later time;
9. cap host reuse by the canonical provenance aggregate, final constitution horizon and deployment return cap;
10. construct non-deserializable `HostLocalDnaContext`;
11. call only `qualify_operational_freshness_for_deployment_with_constitution_and_provenance` on the active path; and
12. export constitution/provenance/deployment audit evidence only from that final non-deserializable result.

No external effect is authorized by deployment qualification alone.
