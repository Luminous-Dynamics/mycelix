# Current Freshness — Coordinator Deployment Gate v0.9

Status: **native observer candidate + non-circular manifest-bound key authorization + native hybrid release authentication + release currentness + exact deployment composition + stability fence implemented; remaining live provenance/atomic effect admission incomplete**

## Core deployment distinction

The v0.11 current-freshness theorem binds local `DnaHash`, but coordinator WASM can change without changing that hash. Therefore `same DNA != same coordinator implementation`.

Current Holochain 0.6 Admin APIs retrieve/update coordinator definitions by exact `CellId`, so deployment attestation binds **DNA hash + cell agent public key**, not DNA alone.

Coordinator self-reported code identity is never sufficient.

## Native conductor observation candidate

`mycelix-authority-coordinator-native-attestor` directly queries a loopback Holochain Admin API for an exact `CellId`, enumerates the complete installed coordinator set, extracts exact `WasmHash` values, samples time only after observation/extraction and owns the fixed five-second observation reuse cap.

It does not choose an approved release or perform deployment matching.

## Coordinator release semantics

`mycelix-authority-coordinator-release` defines a DNA/code-scoped candidate manifest committing the complete coordinator `WasmHash` closure plus DNA bundle/source/lock/toolchain/build-recipe/SBOM identities, release authority and release-policy identity.

The semantic manifest exists before signature authentication and does not choose an installation-specific agent.

## Non-circular manifest-bound key authorization

`mycelix-authority-coordinator-release-key-policy` gives the candidate manifest's release-policy commitment one exact v0.1 hybrid-key meaning **before** release authentication.

The active key-policy profile fixes:

`Ed25519 + ML-DSA-65`

with exact public-key wires:

- Ed25519: 32 bytes;
- ML-DSA-65: 1952 bytes.

The semantic policy commits exact release authority, policy id/generation, complete canonical authorized-key set, exact key bytes, key-specific validity windows and policy lifetime.

The causal order is strictly:

```text
candidate CoordinatorReleaseManifest
        +
semantic CoordinatorReleaseKeyPolicy
        +
independently verified current-policy proof
        ↓
QualifiedCoordinatorReleaseManifestKeyPolicy
        ↓ exact authorized key id + key lifetime
QualifiedCoordinatorReleaseSigningKey
```

The qualified signing key is non-deserializable and bound to the exact candidate manifest digest/profile, release authority and release-policy identity.

The key-policy crate is forbidden from depending on #269/#275 positive authenticated/current release objects; that would recreate circular authority.

## Current key-policy evidence is bounded but provenance is still open

`VerifiedCurrentCoordinatorReleaseKeyPolicyProof` remains deserializable evidence. Pure #307 qualification proves shape/equality only; it cannot prove verifier origin or policy-registry completeness/currentness.

Its v0.1 reuse window is capped at 30 seconds. This limits reuse but does not make the policy immutable during that window.

A future live path must obtain this current-policy proof directly from an independently qualified policy-currentness verifier and ignore caller-supplied currentness bytes for positive authority.

## Native hybrid release authentication candidate implemented

`mycelix-authority-coordinator-release-hybrid-verifier` is the native cryptographic boundary between #307 key authorization and #269 release authentication.

Its public live input is exactly:

```text
candidate CoordinatorReleaseManifest
+ non-deserializable manifest-bound QualifiedCoordinatorReleaseSigningKey
+ detached CoordinatorReleaseHybridSignature
```

The detached signature contains only protocol/profile, exact signing key id, 64-byte Ed25519 signature and 3309-byte ML-DSA-65 signature. It carries no public key, trusted-key path, verifier reference, signature reference, verification timestamp or proof horizon.

The verifier pins the same concrete RustCrypto wire implementations used by the existing Luminous hybrid prototype:

- `ed25519-dalek = 2.2.0`;
- `ml-dsa = 0.1.1`.

That existing hybrid construction remains experimental/unaudited; this candidate does not upgrade its assurance status.

## Exact signed message

Both algorithms verify the identical deterministic v0.1 message committing:

- fixed message domain;
- verifier protocol version;
- fixed signature-message profile;
- fixed signature-evidence profile;
- canonical #269 manifest digest/profile;
- exact release authority;
- exact release-policy digest/profile; and
- exact qualified signing key id.

Changing the manifest or key id changes the signed bytes.

## Hybrid authentication is fail-closed AND composition

The Ed25519 half uses strict Dalek verification. The ML-DSA-65 half decodes the exact RustCrypto encoded key/signature wires and verifies the same message.

Both must succeed. No classical-only, PQ-only, OR or fallback success mode exists in v0.1.

The verifier also recomputes the candidate manifest digest and rechecks its exact manifest/profile, release-authority and release-policy bindings against #307's qualified key before using the key bytes.

## Positive proof metadata is verifier-owned

After both cryptographic checks succeed, the verifier samples host time. It then rechecks that the manifest-bound key and candidate manifest are still live.

The private #269 proof horizon is:

`valid_until = min(qualified signing key, candidate manifest, verified_at + 5 seconds)`.

The caller cannot select or widen this time window.

The #269 `signature_ref` is also verifier-derived: BLAKE3 over the fixed signature-reference protocol/profile, signing key id and exact Ed25519 + ML-DSA-65 signature bytes.

A caller therefore cannot relabel verified signature bytes with arbitrary positive provenance text.

## Evidence-shaped signature proof stays private

The native verifier does **not** publicly return `VerifiedCoordinatorReleaseSignatureProof`.

Its private helper constructs that evidence-shaped receipt only after both crypto checks and immediately feeds it into local #269 `qualify_coordinator_release`.

The public successful result is only non-deserializable:

`QualifiedCoordinatorReleaseRequirement`.

Thus the live authentication boundary is:

```text
manifest-bound qualified key
+ detached hybrid signature bytes
        ↓
strict Ed25519 AND ML-DSA-65 verification
        ↓
post-crypto host clock
        ↓
private verifier-owned #269 proof
        ↓
local #269 qualification
        ↓
non-deserializable authenticated release
```

This significantly reduces positive receipt replay/injection surface.

## Release currentness / withdrawal follows authentication

`mycelix-authority-coordinator-release-currentness` applies only after #269 authentication. It requires the independently verified current registry head plus exact status of that authenticated release at that exact head.

Only `Active` qualifies. `Withdrawn`, `Superseded`, old-head `Active`, wrong policy or alternate v0.1 profiles deny.

Therefore:

`candidate release != key authorization != cryptographic authentication != release currentness`.

## Exact target/release/observation composition

After authentication/currentness, `mycelix-authority-coordinator-deployment-composer` keeps target selection, current release and conductor observation separate, specializes the release to the independently selected target agent, and delegates exact whole-set equality to #262.

Missing, substituted, duplicate or unexpected installed coordinator code denies.

## Subject-bound pre/post stability

`mycelix-authority-coordinator-stability-fence` requires a strictly later second exact deployment observation inside the first composition's evidence window, reconstructs the exact same release/target requirement, re-runs #262 and binds the result to one admission subject + per-attempt nonce.

This proves no detected coordinator-code change across the observed interval. It is not a mutex, transaction or atomicity guarantee: `UpdateCoordinators` can still race after the post observation.

## Remaining provenance boundaries

The new native crypto verifier establishes actual signature verification, but several independent live origins remain unresolved:

- current key-policy proof provenance/completeness;
- current release-registry head provenance/completeness;
- exact release status-at-head provenance;
- target CellId selection provenance;
- real-conductor qualification of the native observer;
- native ownership of the admission subject/attempt nonce and pre/post bracketing; and
- post-observation update-race/effect atomicity semantics.

In particular:

`real crypto != current key-policy provenance != release-registry currentness`.

And:

`exact deployment stability != atomic effect admission`.

## Consumer rule

A future lifecycle/effect consumer needs independently:

1. fresh v0.11 operational currentness from a direct local verifier call;
2. exact stable/dynamic deployment evidence + lease;
3. trusted exact target CellId selection;
4. candidate coordinator release semantics;
5. independently current release-key policy from a qualified live verifier;
6. manifest-bound non-deserializable qualified hybrid signing key;
7. native strict hybrid authentication returning only local #269 authenticated release;
8. independently proven #275 current release-registry head/completeness and exact status-at-head;
9. native pre deployment observation for the exact CellId;
10. exact target/current-release/observation composition through #290/#262;
11. native post observation and subject/attempt-bound stability through #298;
12. explicit native policy for coordinator-update races after the final observation;
13. later lifecycle/executor/effect-safety authority; and
14. an effect path that consumes only in-process positive qualifications, never caller-supplied serialized positive receipts.

## Provisioning state

The native hybrid verifier candidate still does not satisfy the whole deployment gate.

Until current key-policy verifier provenance/completeness, release-head/status provenance, real-conductor observer qualification, target-selection provenance, native pre/post orchestration and post-observation race semantics are qualified and bound into final lifecycle/effect admission:

- `authority_current_freshness_verifier` remains absent from binding `dna.yaml`;
- `constitution_currentness_verifier` remains absent from binding `dna.yaml`;
- no effect-capable consumer may interpret deployment evidence as coordinator-code-attested atomic authority; and
- external effects remain disabled.

## Qualification still required

Remaining work includes: real-conductor native observer qualification; trusted target-cell selection provenance; current release-key-policy verifier/completeness; current registry-head verifier/completeness; exact status-at-head verifier; live provenance-preserving composition; native pre/post admission orchestration; explicit post-observation update-race/atomicity policy; final lifecycle/effect binding; and adversarial wrong-key/wrong-policy/key-rotation/manifest-substitution/forged-signature/wrong-cell/stale/extra/old-head/withdrawn/replayed-attempt/local-endpoint-impersonation/update-race tests.
