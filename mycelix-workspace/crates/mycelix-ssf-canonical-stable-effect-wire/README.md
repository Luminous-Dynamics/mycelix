# SSF Canonical Stable Effect Wire v0.1

Complete canonical byte encoding of `ActuatorStableEffectIdentityV1` and every nested execution/provider descriptor it contains.

This is the first complete composite encoder in the replay-evidence wire series. It deliberately covers the semantics that must remain unchanged across an explicitly authorized replay while excluding fresh attempt-specific pre-invocation qualification evidence.

## Fixed v0.1 field order

The composite writes:

- stable-effect encoding version;
- durable capability claim record;
- actuator stable identity, policy, generation;
- exact execution-surface identity;
- logical session + authenticated transcript;
- transport-authority generation;
- actuator policy + health generations;
- exact operation carrier;
- execution-generation time basis + lifetime;
- source-owned provider identity, policy, generation, time basis, lifetime;
- actuator recovery mode, pre-invocation time basis, actuator lifetime;
- immutable operation handle;
- payload commitment;
- encoding/schema commitment;
- payload length;
- top-level stable-effect recovery mode.

The recovery mode is intentionally encoded twice because the source semantic type carries it both inside the actuator descriptor and as a top-level stable-effect field. Canonical encoding follows semantic equality; it does not silently normalize duplicated fields.

Changing an authenticated session transcript, transport generation, actuator policy/health generation, provider generation, operation material commitment, payload length, recovery mode, or other encoded field changes the canonical bytes.

## Claim boundary

This crate is complete only for `ActuatorStableEffectIdentityV1`. It is **not** a complete encoding of `ActuatorEffectSubjectV1`, `ActuatorInvocationAttemptManifestV1`, canonical completed-effect evidence, history, or `CanonicalReplayEvidenceDecisionSubjectV1`.

No hashing, signing, verification, evidence qualification, replay authorization, or effect authorization is implemented here.
