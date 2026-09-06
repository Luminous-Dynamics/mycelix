# Authority Evidence Closure v0.1 — Normative Invariants

Status: **pure evidence-closure candidate; no runtime provider or DNA provisioning**

This layer closes a dynamic provenance gap without changing stable authority semantics.

#120/#123 already prove the correct semantic and deployment-bound live authority. Their stable authority identities MUST remain unchanged by this tranche.

The missing fact is complete proof provenance: the final dynamic evidence identity must also commit the exact fresh #115 control-plane proof instances used to establish current operational policy authority.

## 1. Stable deployment authority is upstream and unchanged

A positive result first requires #123 `qualify_current_authority_for_local_dna` over the complete evidence-shaped input and exact independently supplied local DNA context.

This layer does not define another stable authority domain.

Its stable deployment identity getters delegate to #123:

- semantic qualification digest/profile;
- deployment qualification digest/profile; and
- exact DNA hash.

A control-plane re-probe under the same root/policy/generations MUST NOT change those identities.

## 2. Control-plane proof instances are requalified, not hashed as raw caller data

Raw `CoveredSubjectEvidence` bytes are not directly committed as proof.

After #123 succeeds, this layer independently re-runs:

- #111 `qualify_bootstrap_root`; and
- #115 `qualify_control_plane_subject_freshness` for every supplied control-plane evidence bundle.

Only the resulting non-deserializable qualified proof identities enter the control-plane evidence-set digest.

Thus malformed or unauthoritative candidate data cannot gain provenance merely by being hashed.

## 3. #123 success closes the semantic set before provenance closure

#120/#116 reconstruct the exact required operational policy-currentness set:

- exactly two policy subjects for DirectSource; or
- exactly three for WitnessQuorum.

Extra, missing, wrong-root, wrong-identity, inactive, revoked or superseded policy state denies before #123 succeeds.

This layer therefore closes provenance over the exact control-plane evidence list already proven semantically sufficient by the complete current-authority chain.

## 4. Canonical control-plane evidence identity

For every requalified #115 proof, the evidence-set item is:

`(exact AuthoritySubjectRef.identity_digest, exact QualifiedControlPlaneSubjectFreshness.qualification_digest)`.

Items are sorted by exact subject identity digest before hashing.

Input order is not authority and MUST NOT change the evidence-set digest.

Duplicate exact subject identities deny.

The digest also commits:

- a dedicated domain;
- `CONTROL_PLANE_EVIDENCE_PROFILE`; and
- exact item count.

## 5. Control-plane proof refresh changes evidence, not authority

A new challenge/source/witness proof for the same current control-plane policy generation may change the #115 qualification digest.

That MUST change:

- `control_plane_evidence_digest`; and
- `complete_evidence_digest`.

It MUST NOT change:

- #120 semantic authority identity; or
- #123 deployment authority identity.

This is the purpose of separating authority from evidence provenance.

## 6. Target proof and local-DNA proof remain committed through #123

#123 `deployment_evidence_digest` already commits:

- the #120 dynamic current-authority evidence digest, including #117 target operational evidence; and
- the local-DNA verification instance.

The complete evidence digest therefore commits:

1. exact stable #123 deployment qualification digest;
2. exact #123 deployment evidence digest; and
3. exact canonical control-plane evidence-set digest/profile.

Changing target proof, local-DNA proof or any control-plane proof changes final evidence provenance.

## 7. Freshness window is conservatively closed over all proof instances

The result's verification time is the maximum of:

- #123 deployment verification time; and
- all requalified control-plane proof verification times.

The result's validity horizon is the minimum of:

- #123 deployment validity; and
- all requalified control-plane freshness leases.

The downstream freshness projection is revalidated after replacing/narrowing dynamic proof metadata.

## 8. Positive output is non-deserializable

`QualifiedCompleteDeploymentAuthority` derives `Serialize` but not `Deserialize`.

Transported output bytes do not recreate live authority. Another runtime must re-run the complete evidence and DNA-bound qualification chain.

## 9. No raw proof ordering authority

The final evidence identity MUST NOT depend on:

- caller ordering of control-plane bundles;
- DHT arrival order;
- witness arrival order after lower-layer qualification;
- newest timestamp selection; or
- arbitrary provider list order.

Exact semantic proof identities are canonicalized before closure.

## 10. Historical/live separation is preserved

This layer consumes only live #123/#115 qualifications.

It has no historical/as-of conversion path. Historical evidence may remain auditable but cannot be closed into a new live deployment proof without current qualification.

## 11. No advisory or execution shortcuts

Phi, reputation, stake, Guardian status, Symthaea output, model confidence, or caller identity cannot:

- supply a missing control-plane proof;
- waive a stale lease;
- change canonical proof ordering;
- preserve evidence identity after a proof-instance change; or
- authorize an external effect.

## 12. Containment

Pure Rust only. No HDK calls, persistence, DHT reads, remote calls, lifecycle claims, signing, or external effects.

A future runtime should treat this layer as the final pure current-authority/evidence boundary before projecting a live verifier receipt.

## 13. Required qualification

Before runtime provisioning:

- rustfmt/tests/warnings-denied Clippy;
- downstream #123/#120/#115 compilation;
- order-independent control-plane evidence tests;
- duplicate-subject denial;
- one control-plane proof refresh changes complete evidence;
- target proof refresh changes complete evidence through #123;
- local-DNA proof refresh changes complete evidence through #123;
- same stable authority + refreshed evidence leaves semantic/deployment authority digests unchanged; and
- positive output remains non-deserializable.

Related: #124, #123, #120, #117, #116, #115, #121.
