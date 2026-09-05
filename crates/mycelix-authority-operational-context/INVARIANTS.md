# Operational Authority Policy Context v0.1 — Normative Invariants

Status: **pure qualification candidate; no runtime provider or DNA provisioning**

This layer answers one question:

> Are these exact semantic operational coverage/trust policies currently authorized for this exact operational authority subject under the current constitution-rooted control plane?

It does not issue probes, discover policy, query DHT state, project transition history, verify signatures, persist state, grant capabilities, execute governance actions, or authorize external effects.

## 1. Control-plane registry namespace and covered namespace are different concepts

An operational `AuthorityCoveragePolicy.namespace` describes the namespace of authority subjects that policy may cover.

The policy object's own Active/Revoked/Superseded state is registered under the bootstrap root's `control_plane_namespace`.

Therefore:

`covered subject namespace != policy-state registry namespace`

is valid and expected.

This layer MUST NOT derive an `AuthorityCoveragePolicy` freshness subject from `coverage_policy.namespace`.

The exact current policy subjects are instead:

- `AuthorityCoveragePolicy` in `root.control_plane_namespace`, with `subject_id = coverage_policy.policy_id` and exact coverage-policy digest/profile;
- `CoverageTrustContextPolicy` in `root.control_plane_namespace`, with `subject_id = context_policy.context_policy_id` and exact context-policy digest/profile; and
- for witness-quorum mode only, `WitnessTrustPolicy` in `root.control_plane_namespace`, with an exact digest-derived subject ID and exact trust-policy digest/profile.

## 2. This layer is operational-plane only

The target subject may be only one of the operational authority classes:

- `AuthorityGrant`;
- `SigningPolicy`;
- `ThresholdAuthorization`;
- `ExecutorDesignation`;
- `EffectSafetyPolicy`; or
- `Delegation`.

The three control-plane policy classes MUST be qualified through the bootstrap/control-plane path and MUST NOT recursively enter this operational context as targets.

## 3. Exact semantic policy verification is mandatory

The coverage and context policy receipts remain independent semantic-verification boundaries.

A positive context requires:

- both policy structures validate;
- both policies are semantically active at `now_ms`;
- each receipt's verified authority/proof exactly echoes the semantic policy;
- verification time is non-zero, not future, and not before policy activation;
- policy record and verification refs are non-empty;
- context policy institution equals the current bootstrap-root institution; and
- context policy rulebook equals the current bootstrap-root rulebook.

A deserialized policy receipt is not current authority merely because it exists.

## 4. Context must bind the exact coverage policy

The context policy's `coverage_policy` profiled digest MUST equal the exact digest/profile recomputed from the supplied operational `AuthorityCoveragePolicy`.

Same policy IDs, source refs, institution refs, or human-readable names are insufficient.

## 5. Target coverage is checked explicitly

The operational coverage policy MUST cover the exact target subject namespace and target subject kind.

A current coverage policy for one namespace or authority class cannot be replayed onto another.

The target subject's exact identity digest is included in the final operational-context qualification identity.

## 6. Witness trust semantics come from the current context policy

There is no second `VerifiedWitnessTrustPolicyAuthority` semantic receipt in this layer.

For witness-quorum mode, the current context policy already commits:

- the exact witness-trust policy digest/profile; and
- the exact witness-trust verifier ref.

This layer separately requires current control-plane freshness for that exact `WitnessTrustPolicy` subject.

Later #96 witness bindings still have to prove that each observer/trust-domain classification came from the exact verifier and policy named by the context.

Direct-source mode MUST have neither witness-trust policy nor witness-trust verifier.

Witness-quorum mode MUST have both.

## 7. Probe authorship is provenance, not institutional authority

No probe issuer DID, issuer grant, challenge capability, or challenge-authority receipt appears in this qualification.

A read-only probe can collect randomness-bound evidence without already possessing the policy currentness it is intended to discover.

This preserves the #109 bootstrap-cycle correction:

`probe -> current policy freshness -> probe`

must never return.

Probe provenance remains verified later as randomness provenance; it does not enter the operational policy authority identity.

## 8. Exact policy-currentness set is reconstructed locally

The control-plane freshness provider does not choose which policies must be current.

This layer deterministically reconstructs the exact required policy subjects from:

- current bootstrap root;
- exact coverage policy;
- exact context policy; and
- exact witness mode.

It then converts only non-deserializable #115 `QualifiedControlPlaneSubjectFreshness` proofs to freshness receipts and re-runs #74 `qualify_current_freshness` over that exact closed set.

Missing, unexpected, duplicate/conflicting, stale, revoked, or superseded policy state denies.

## 9. All policy freshness must belong to one exact bootstrap-root epoch

Every supplied #115 proof MUST bind the exact current:

- bootstrap root qualification digest; and
- bootstrap root qualification profile.

Mixing otherwise-valid policy freshness across constitutional/root epochs is forbidden.

## 10. Current lease is the conservative minimum

The operational context cannot outlive any authority fact used to construct it.

Its `valid_until_ms` is the minimum of:

- current control-plane freshness lease;
- bootstrap-root current lease;
- semantic context-policy expiry; and
- semantic coverage-policy expiry.

Its `verified_at_ms` is the maximum of the contributing current/semantic verification times.

## 11. Stable identity excludes verifier refresh metadata

The qualification digest commits:

- exact bootstrap-root qualification identity;
- exact target subject identity;
- exact coverage-policy semantic identity;
- exact context-policy semantic identity;
- optional exact witness-trust policy digest/profile; and
- exact closed current-policy freshness digest.

Dynamic verification timestamps and proof-ref refreshes do not change this semantic identity unless they reveal a different current generation or policy object.

## 12. Non-deserializable positive authority object

`QualifiedOperationalPolicyContext` derives `Serialize` but not `Deserialize`.

Runtime adapters may transport candidate policy receipts and freshness evidence, but must call `qualify_operational_policy_context` to construct this positive object.

## 13. Historical state cannot enter this boundary

This layer accepts only #115 current control-plane freshness proofs.

It has no historical/as-of projection API and cannot convert historical policy validity into live operational authority.

## 14. No DHT/currentness heuristics

This crate contains no Holochain calls and must not infer current policy from:

- highest observed generation;
- latest timestamp;
- DHT arrival order;
- absence of a later record;
- local cache contents;
- author reputation; or
- model output.

Current policy evidence comes only through the covered bootstrap/control-plane path.

## 15. No execution/advisory authority

This layer cannot execute actions, issue lifecycle claims, sign threshold decisions, create grants, mutate governance state, or authorize external effects.

Phi, consciousness, reputation, stake, Guardian status and Symthaea output are not inputs to policy currentness.

## 16. Required qualification before runtime consumption

At minimum:

- rustfmt;
- native tests;
- warnings-denied Clippy;
- downstream compile of #115/#96/#74 dependencies;
- regression audit that policy currentness uses `root.control_plane_namespace`;
- regression audit that target coverage uses `coverage.namespace`;
- regression audit that issuer-grant/challenge-issuer authority is absent;
- direct-source exact two-policy-set tests;
- witness-quorum exact three-policy-set tests;
- wrong-root / stale-root denial;
- wrong target namespace/kind denial;
- missing/extra/revoked policy freshness denial; and
- constitutional/root rotation invalidating the old context.

No Holochain provider should be provisioned until the pure kernel and its upstream stack have executable green evidence.
