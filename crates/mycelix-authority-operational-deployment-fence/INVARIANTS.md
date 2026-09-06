# Operational Deployment Fence v0.2 — Normative Invariants

Status: **pure qualification kernels; no runtime provisioning and no external effects**

This layer answers two related but distinct questions:

1. Is one already locally qualified, non-deserializable operational-currentness proof live in this exact host DNA and exact current constitutional epoch?
2. Is that exact deployment qualification backed by one exact canonical current-evidence provenance manifest?

The second theorem adds dynamic evidence provenance only. It MUST NOT alter stable deployment authority identity.

## 1. Semantic currentness must already exist locally

The positive semantic input is `&QualifiedOperationalSubjectFreshness` from #117.

Neither deployment theorem accepts a transportable `VerifiedAuthorityFreshness`, provider-created `Qualified*` bytes, or an omnibus evidence-shaped reconstruction bundle as a substitute.

## 2. Local DNA and constitutional DNA are different evidence

The exact local DNA identity is a host-runtime fact. The constitutional plane's `dna_hash` is independently produced constitutional evidence.

Positive deployment requires exact equality:

`binding_constitution_dna_hash == host_local_dna_hash`.

Neither value may substitute for the other.

## 3. Exact constitutional epoch is stable deployment authority

`deployment_authority_digest/profile` commits:

- exact #117 semantic authority digest/profile;
- exact bootstrap-root qualification digest/profile;
- exact operational-context digest/profile;
- exact current constitutional statement digest/profile; and
- exact host DNA hash.

A DNA migration or constitutional transition changes live deployment authority even if lower semantic authority were otherwise byte-identical.

## 4. Dynamic evidence is separate from stable authority

The original #154 `deployment_evidence_digest/profile` commits stable deployment authority plus #117 fresh evidence and the exact host-DNA observation/window.

Re-observing the same DNA under the same constitution may refresh deployment evidence without minting a new stable deployment authority identity.

## 5. Provenance-bound deployment is a sibling theorem

The original `qualify_operational_freshness_for_deployment` remains unchanged.

`qualify_operational_freshness_for_deployment_with_provenance` first obtains the original non-deserializable #154 result, then consumes `&QualifiedEvidenceLeaseManifest` from #192 in-process.

The provenance manifest is non-deserializable positive audit evidence; callers cannot replace it with arbitrary manifest bytes.

## 6. Provenance changes deployment evidence, not deployment authority

`QualifiedProvenanceBoundDeploymentOperationalFreshness::deployment_authority_digest/profile` MUST delegate exactly to the base #154 result.

The provenance-bound dynamic evidence digest instead commits:

- exact base #154 deployment-evidence digest/profile;
- exact #192 manifest digest/profile;
- exact contributor count; and
- exact manifest aggregate verification/validity horizon.

Thus:

`same semantic authority + same constitution + same DNA + different provenance`

MUST produce:

`same deployment authority identity + different deployment evidence identity`.

## 7. Provenance lease must contain deployment reuse

The canonical manifest aggregate must be live at qualification time.

The already-capped host context and base #154 deployment result MUST NOT outlive the manifest aggregate. The base deployment verification time MUST NOT predate the manifest aggregate verification time.

The provenance-bound projected freshness is narrowed again to the minimum of base deployment and manifest validity.

No evidence lease may be widened.

## 8. Provenance identity is not a permission source

The #192 manifest digest/profile/count MUST NOT enter `deployment_authority_digest` or any semantic authority identity.

Provenance cannot establish source trust, completeness, policy currentness, quorum, institutional permission, execution authority, or external-effect permission.

It only commits which already-qualified dynamic evidence justified the deployment proof being reused.

## 9. Host DNA context is non-deserializable

`HostLocalDnaContext` derives `Serialize` but not `Deserialize`.

A runtime may construct it only in-process after obtaining local DNA from the host/cell context. Runtime CI must continue to bind this construction to `dna_info()?.hash`.

## 10. Positive results are non-deserializable

Both `QualifiedDeploymentOperationalFreshness` and `QualifiedProvenanceBoundDeploymentOperationalFreshness` derive `Serialize` but not `Deserialize`.

Another runtime/cell must reconstruct semantic currentness, host DNA, current constitution and canonical provenance locally.

## 11. Pure separation

This crate contains no HDK/Holochain calls, DHT lookup, persistence, policy discovery, cryptographic proof service, lifecycle mutation, execution action, Phi, reputation, stake, Guardian override or model-score authority.

Host retrieval belongs to the runtime adapter; semantic currentness belongs to #111/#115/#116/#117; provenance-set closure belongs to the current-freshness runtime + #192.

## 12. Runtime acceptance target

The provenance-aware current-freshness coordinator must, in order:

1. locally construct #117 `QualifiedOperationalSubjectFreshness`;
2. construct and locally close the exact required #192 contribution set;
3. locally qualify `QualifiedEvidenceLeaseManifest`;
4. require its aggregate lease to equal the independently accumulated composition lease;
5. obtain `dna_info()?.hash` from the local host;
6. re-project and fence the binding constitution;
7. cap host context by the canonical aggregate plus deployment return cap;
8. construct non-deserializable `HostLocalDnaContext`;
9. run `qualify_operational_freshness_for_deployment_with_provenance`; and
10. export deployment/provenance evidence only from that non-deserializable result.

No external effect is authorized by either deployment qualification alone.
