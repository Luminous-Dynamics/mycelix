# Current Freshness — Coordinator Deployment Gate v0.8

Status: **native observer candidate + release semantics/currentness + exact deployment composition + stability fence + non-circular manifest-bound hybrid key-policy theorem implemented; live verifier provenance/atomic effect admission still incomplete**

## Core deployment distinction

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation binds **DNA hash + cell agent public key**, not DNA alone.

Coordinator self-reported code identity is never sufficient.

## Native conductor observation candidate

`mycelix-authority-coordinator-native-attestor` directly queries a loopback Holochain Admin API for an exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values, samples time only after observation/extraction and owns the fixed five-second observation reuse cap.

It does not choose an approved release or perform the match.

## Coordinator release semantics

`mycelix-authority-coordinator-release` defines a DNA/code-scoped candidate release manifest committing the complete coordinator `WasmHash` closure plus DNA bundle/source/lock/toolchain/build-recipe/SBOM identities, release authority and release-policy identity.

The semantic manifest exists before signature authentication and does not choose an installation-specific agent.

## Non-circular hybrid release signing-key policy

`mycelix-authority-coordinator-release-key-policy` gives the candidate manifest's release-policy commitment one exact v0.1 hybrid-key meaning **before** release authentication.

The v0.1 policy fixes:

`Ed25519 + ML-DSA-65`

with exact public-key wire lengths:

- Ed25519: 32 bytes;
- ML-DSA-65: 1952 bytes.

The semantic `CoordinatorReleaseKeyPolicy` commits exact release authority, policy id/generation, complete canonical authorized-key set, exact key bytes, key-specific validity windows and policy lifetime.

Positive key authorization now follows only this direction:

```text
candidate CoordinatorReleaseManifest
        +
semantic CoordinatorReleaseKeyPolicy
        +
independently verified current-policy proof
        ↓
qualify_manifest_key_policy
        ↓
QualifiedCoordinatorReleaseManifestKeyPolicy
        ↓ exact authorized key id + key lifetime
QualifiedCoordinatorReleaseSigningKey
        ↓
future real hybrid signature verification
        ↓
#269 authenticated release
        ↓
#275 current/non-withdrawn release
```

The key-policy crate MUST NOT consume `QualifiedCurrentCoordinatorRelease`, `QualifiedCoordinatorReleaseRequirement`, `VerifiedCoordinatorReleaseSignatureProof`, or any other positive result whose construction already depends on release signature authentication. That would recreate circular authority.

## Exact candidate-manifest binding

The key-policy qualifier validates the candidate #269 manifest and recomputes its canonical `manifest_digest`.

It then requires:

- candidate `release_policy_profile` == exact v0.1 key-policy profile;
- candidate `release_policy_digest` == locally recomputed semantic policy digest;
- candidate `release_authority_ref` == policy release authority; and
- a separate current-policy receipt naming the exact policy digest/profile/generation.

The non-deserializable policy qualification commits the exact candidate manifest digest/profile plus the exact policy/currentness identities.

`QualifiedCoordinatorReleaseSigningKey` also carries that exact candidate manifest digest/profile. A future crypto verifier must recompute the candidate manifest digest and require equality before using the key bytes. A key capability for release manifest A cannot authenticate manifest B.

## Current key-policy evidence is bounded but still provenance-limited

`VerifiedCurrentCoordinatorReleaseKeyPolicyProof` remains deserializable evidence. Pure qualification cannot prove which verifier produced it or whether policy-registry completeness/currentness was established correctly.

Its v0.1 reuse window is capped at 30 seconds. This limits reuse but is not proof that policy cannot change inside the window.

A live path must obtain current-policy evidence directly from an independently qualified verifier and ignore caller-supplied receipt bytes for positive authority.

## Raw trusted-key input is forbidden

The future hybrid verifier must accept a non-deserializable `QualifiedCoordinatorReleaseSigningKey`, not a caller-selected trusted-key path, public-key blob or key pair.

The existing Luminous Ed25519 + ML-DSA-65 implementation motivating the wire profile remains experimental/unaudited; this gate does not upgrade its assurance status.

## Key authorization is not signature verification

The key-policy crate contains no Ed25519/ML-DSA verifier implementation and does not construct #269's signature proof.

The future native signature verifier must:

1. accept the exact candidate `CoordinatorReleaseManifest` plus a manifest-bound `QualifiedCoordinatorReleaseSigningKey`;
2. recompute the #269 manifest digest and require exact equality with the key capability's manifest digest/profile;
3. construct one fixed deterministic release-signature message from that manifest identity plus exact release-authority/policy identity;
4. verify **both** Ed25519 and ML-DSA-65 signatures with the qualified key;
5. sample verification time only after both cryptographic checks succeed;
6. own a bounded proof horizon rather than accept caller-selected proof times; and
7. only then construct `VerifiedCoordinatorReleaseSignatureProof` for local #269 qualification.

## Release currentness / withdrawal follows authentication

`mycelix-authority-coordinator-release-currentness` applies only after #269 authenticated release qualification. It requires the independently verified current registry head plus exact status of that authenticated release at that exact head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, old-head `Active`, wrong policy or alternate v0.1 profiles deny.

Thus:

`candidate release != key authorization != signature authentication != release currentness`.

## Exact target/release/observation composition

After authentication/currentness, `mycelix-authority-coordinator-deployment-composer` keeps target selection, current release and conductor observation as separate inputs, specializes the release to the independently selected target agent, and delegates whole-set equality to #262.

Its positive result is non-deserializable and lease-monotone, but pure composition does not prove live origin of any input.

## Subject-bound pre/post stability

`mycelix-authority-coordinator-stability-fence` requires a strictly later second exact deployment observation inside the first composition's evidence window, reconstructs the exact same release/target requirement, re-runs #262 and binds the result to one admission subject + per-attempt nonce.

This proves no detected code change across the observed interval. It is still not a mutex/transaction/atomicity guarantee: `UpdateCoordinators` can race after the post observation.

## Live provenance remains separate

The pure theorems do not prove that their deserializable evidence inputs came from designated live verifier roles.

In particular:

`semantic key policy != current key-policy provenance != manifest-bound qualified signing key != hybrid signature verification`.

Likewise:

`target selection != current approved release != observed deployment != stability interval != atomic effect admission`.

## Consumer rule

A future lifecycle/effect consumer needs independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. trusted exact target CellId selection;
4. candidate coordinator release semantics;
5. independently current release-key policy;
6. manifest-bound non-deserializable qualified hybrid signing key;
7. live hybrid release-signature verification using that key;
8. #269 authenticated release followed by independently proven #275 current release-registry head/completeness and exact status-at-head;
9. native pre deployment observation for the exact CellId;
10. exact target/current-release/observation composition through #290/#262;
11. native post observation and subject/attempt-bound stability through #298;
12. explicit native policy for coordinator-update races after the final observation;
13. later lifecycle/executor/effect-safety authority; and
14. an effect path that consumes only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The corrected non-circular key-policy theorem still does not satisfy this gate.

Until current key-policy provenance, real hybrid signature verification, release-head/status provenance, real-conductor observation, target-selection provenance, native pre/post orchestration and post-observation race semantics are qualified and bound into final lifecycle/effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret deployment evidence as coordinator-code-attested atomic authority; and
- external effects remain disabled.

## Qualification still required

Remaining work includes: real-conductor native observer qualification; trusted target-cell selection provenance; current release-key-policy verifier/completeness; native hybrid signature verifier using only a manifest-bound qualified key; current registry-head verifier/completeness; exact status-at-head verifier; live provenance-preserving composition; native pre/post admission orchestration; explicit post-observation update-race/atomicity policy; final lifecycle/effect binding; and adversarial wrong-key/wrong-policy/key-rotation/manifest-substitution/forged-signature/wrong-cell/stale/extra/old-head/withdrawn/replayed-attempt/local-endpoint-impersonation/update-race tests.
