# Authority Coordinator Release Key Policy v0.1 — Normative Invariants

Status: **pure candidate-manifest key-authorization theorem; live policy-currentness provenance and hybrid signature verification remain unimplemented**

## 1. Key authorization precedes release authentication

This theorem MUST NOT consume `QualifiedCoordinatorReleaseRequirement`, `QualifiedCurrentCoordinatorRelease`, `VerifiedCoordinatorReleaseSignatureProof`, or any other positive result whose construction already depends on release signature verification.

The causal order is:

`candidate CoordinatorReleaseManifest + independently current key policy -> manifest-bound qualified signing key -> hybrid signature verification -> #269 authenticated release -> #275 current release`.

Any dependency from key authorization back to #269/#275 positive release authentication/currentness is circular and forbidden.

## 2. Release-policy bytes have one exact v0.1 meaning

The hybrid release path uses `POLICY_PROFILE = mycelix-authority-coordinator-release-key-policy-v1-blake3-framed`.

A candidate release manifest must commit this exact profile and the exact locally recomputed digest of `CoordinatorReleaseKeyPolicy` before a policy can qualify for that manifest.

An arbitrary nonempty release-policy profile is not accepted.

## 3. Release authority and key policy are exact-bound

The policy's `release_authority_ref` must equal the candidate manifest's exact release authority.

A valid/current policy for another release authority cannot authorize a key for this manifest.

## 4. Candidate manifest identity is explicit

`qualify_manifest_key_policy` validates the candidate `CoordinatorReleaseManifest`, recomputes its canonical #269 `manifest_digest`, and binds that exact digest/profile into the non-deserializable policy qualification.

This does NOT authenticate the candidate manifest. It establishes the exact semantic subject for later signature verification.

## 5. The hybrid scheme and wire representation are fixed

v0.1 authorizes exactly the hybrid AND-composition profile:

`Ed25519 + ML-DSA-65`

with exact public-key lengths:

- Ed25519: 32 bytes;
- ML-DSA-65: 1952 bytes.

A future algorithm or wire change requires a versioned successor profile and must not silently broaden v0.1.

This mirrors the existing Luminous hybrid prototype's wire contract. That implementation remains experimental/unaudited; this theorem does not upgrade its assurance status.

## 6. The complete authorized key set is committed canonically

Each authorized key commits exact key id, exact Ed25519 public key bytes, exact ML-DSA-65 public key bytes, exact key validity window, and fixed key/scheme profile.

Policy identity is independent of input key ordering. Duplicate key IDs deny.

## 7. Semantic policy and current-policy proof are separate facts

`CoordinatorReleaseKeyPolicy` is semantic policy data.

`VerifiedCurrentCoordinatorReleaseKeyPolicyProof` is an evidence-shaped currentness receipt for the exact policy digest/profile/generation.

Neither may replace the other.

## 8. Current-policy verifier provenance remains separate

The currentness receipt is deliberately deserializable. Pure qualification proves shape/equality only; it does not prove that the receipt originated from the designated current-policy verifier or that policy-registry completeness/currentness was established correctly.

A live path must obtain it directly from an independently qualified verifier and reject caller-supplied currentness bytes as positive authority.

## 9. Current-policy reuse is bounded

The v0.1 currentness receipt may span at most `MAX_CURRENTNESS_PROOF_REUSE_MS = 30_000` milliseconds from `verified_at_ms` to `valid_until_ms`.

This is a reuse cap, not proof that the policy cannot change inside the window. A live verifier may issue a shorter horizon.

## 10. Positive manifest/policy qualification is non-deserializable

`QualifiedCoordinatorReleaseManifestKeyPolicy` derives `Serialize` but not `Deserialize`.

Its identity commits the exact candidate manifest digest/profile, release authority, release-policy digest/profile, exact semantic policy, currentness-proof identity and final lease.

## 11. Raw trusted-key input is forbidden

A future cryptographic verifier must not accept an arbitrary public-key file/blob and call it trusted.

A key ID must be selected through `QualifiedCoordinatorReleaseManifestKeyPolicy::qualify_key`, which checks exact membership and key-specific lifetime.

## 12. Positive signing key is non-deserializable and manifest-bound

`QualifiedCoordinatorReleaseSigningKey` derives `Serialize` but not `Deserialize`.

It carries the exact candidate `manifest_digest` and `manifest_profile` in addition to the exact hybrid public-key identity and release-policy binding.

The future hybrid verifier MUST recompute the supplied candidate manifest digest and require exact equality with this capability before using its key bytes. A key capability qualified for manifest A must not authenticate manifest B.

## 13. Lease composition is monotone

Manifest/policy qualification:

`verified_at = max(policy-currentness proof, policy valid_from, candidate manifest valid_from)`

`valid_until = min(policy-currentness proof, policy valid_until, candidate manifest valid_until)`

Qualified key:

`verified_at = max(manifest/policy qualification, key valid_from)`

`valid_until = min(manifest/policy qualification, key valid_until)`

No stage may widen an upstream evidence or semantic lifetime.

## 14. This is authorization, not signature verification

This crate does not parse or verify Ed25519/ML-DSA signatures and does not construct #269's `VerifiedCoordinatorReleaseSignatureProof`.

It proves only which exact hybrid public key is currently eligible to authenticate one exact candidate release manifest.

## 15. No release-currentness/deployment/effect authority

A qualified signing key does not establish signature validity, #269 release authenticity, #275 release status, installed coordinator deployment, stability, lifecycle authority, effect safety or external-effect permission.

## 16. Provisioning remains blocked

Before effect-capable provisioning, the stack still requires live key-policy currentness provenance, real hybrid signature verification using only a manifest-bound qualified signing key, live release-head/status provenance, conductor/target provenance, native pre/post orchestration and final effect admission.
