# Authority Coordinator Release Contract v0.1 — Normative Invariants

Status: **pure signed-release authentication theorem; signature-verifier provenance and release currentness/withdrawal are not yet implemented; provisioning remains blocked**

## 1. Release semantics are not deployment observation

`CoordinatorReleaseManifest` describes an independently approved software release: exact DNA identity, complete coordinator WASM closure and reproducibility/supply-chain lineage.

It does not describe what a conductor is currently running. Live conductor observation remains the independent native boundary from #264.

## 2. Release scope is DNA/code, not installation-specific agent identity

The signed release commits the exact `DnaHash` and complete approved coordinator set but intentionally does not contain a cell agent public key.

An exact target CellId is selected independently at deployment time. `required_deployment_for_target_agent` may specialize authenticated release code to a target agent, but that function does not prove who selected the target cell.

Release authority != target-cell authority != conductor observation.

## 3. Complete coordinator closure is signed

The manifest digest commits the canonical zome-name-sorted complete approved coordinator name + raw 39-byte `WasmHash` set.

Duplicate names, malformed hashes or an empty/oversized set deny. Input order does not change release identity.

## 4. Supply-chain lineage is part of release identity

The signed manifest commits at minimum:

- DNA bundle digest;
- source-tree digest;
- lockfile digest;
- toolchain digest;
- build-recipe digest;
- SBOM digest;
- source/build references; and
- exact release policy identity.

These fields make release identity richer than a signature over one opaque bundle hash. This crate does not itself prove reproducible builds, SBOM correctness or vulnerability status; those facts must be established by the release/build pipeline that is authorized to produce the manifest.

## 5. Signature verification is a separate trust domain

`VerifiedCoordinatorReleaseSignatureProof` is intentionally deserializable because it is an evidence-shaped boundary object.

A successful local qualification requires exact equality of:

- manifest digest/profile;
- release authority reference; and
- release policy digest/profile.

But deserialization or pure qualification cannot prove the receipt originated from the designated cryptographic verifier. A future live composer must call that verifier independently and never accept caller-supplied signature-proof receipts as positive release authenticity.

## 6. Positive release qualification is non-deserializable

`QualifiedCoordinatorReleaseRequirement` derives `Serialize` but not `Deserialize`.

It can be constructed only by the local qualifier over the exact manifest plus the exact signature-proof receipt.

## 7. Verification leases never widen release semantics

Qualification uses:

`verified_at = max(signature_verified_at, manifest.valid_from)`

`valid_until = min(signature_valid_until, manifest.valid_until)`

A short signature-verifier lease may qualify a long-lived release now, but never silently becomes the release's full semantic lifetime.

## 8. Release currentness / withdrawal is not established here

This v0.1 theorem proves exact signed release authenticity only.

It does **not** prove that the release has not been withdrawn, superseded, revoked or forbidden by a newer release policy. A later independent release-currentness theorem is mandatory before live deployment/effect admission.

`not expired != not withdrawn`.

## 9. Release policy currentness is separate from signed policy identity

The manifest and signature proof exact-bind a `release_policy_digest/profile`. That establishes which policy lineage the release claims.

This crate does not establish that this policy is the currently binding release policy. A later currentness verifier must do that independently.

## 10. Specialization does not create target authority

`required_deployment_for_target_agent` constructs the exact #262 `RequiredCoordinatorDeployment` shape from authenticated DNA/code plus a supplied agent public key.

The resulting requirement remains evidence/data. Target-agent provenance must come independently from the deployment consumer/native target selection, and the live conductor observation must still match it exactly.

## 11. No release-observation circularity

The authenticated expected coordinator set must never be derived from the conductor observation it is intended to verify.

The native observer must never choose or rewrite the signed release manifest.

Expected release != observed deployment != exact match.

## 12. No authority/effect claim

A qualified signed release does not establish current operational authority, current coordinator deployment, coordinator update atomicity, lifecycle admission, executor authority, effect safety or external-effect permission.

## 13. Provisioning remains blocked

Before the coordinator deployment gate can be satisfied, qualification still needs:

1. a real independent release-signature verifier adapter;
2. authenticated/current release policy and withdrawal state;
3. independently selected target CellId;
4. native conductor observation from #264;
5. #262 exact whole-set matching;
6. coordinator-update race/atomicity handling; and
7. later lifecycle/effect authorization.

No currentness authority zome should be provisioned into binding `dna.yaml` merely because this signed-release theorem passes.
