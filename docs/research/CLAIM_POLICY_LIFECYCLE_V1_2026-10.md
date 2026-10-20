# Claim Policy Lifecycle V1: Authenticated Distribution, Freshness and Anti-Rollback

**Status:** reference state-machine candidate; signature verification and durable persistence are intentionally outside this module.  
**Stack:** follow-on to [claim-ceiling policy gate PR #5264](https://github.com/Luminous-Dynamics/mycelix/pull/5264) and [CLI hardening PR #5272](https://github.com/Luminous-Dynamics/mycelix/pull/5272).  
**Implementation:** `policy_lifecycle.rs`, release descriptor schema, and frozen lifecycle corpus.  
**Security rule:** only a separately trusted distribution adapter may supply verified metadata status, trusted time, the revocation floor, policy identity/audience expectations, and installed high-water mark.

## 1. Paradigm shift

A pinned hash proves the received bytes match an expected digest. It does not prove who approved that digest, whether the policy is current, whether a newer policy was revoked, or whether a consumer is allowed to act on the resulting decision.

The lifecycle check therefore treats policy as a versioned, scoped, revocable artifact with an independently verified distribution chain. It must prevent:

- publisher/key confusion, invalid or revoked publisher keys, and missing signature thresholds;
- metadata expiry and freeze/staleness conditions;
- rollback to an older version after a newer version was accepted;
- same-version equivocation where a version is reused with a different profile digest or revocation epoch;
- cross-policy and cross-audience substitution;
- profile-byte or schema-digest substitution;
- acceptance while trusted time or required revocation state is unavailable.

This implementation is a **post-verification state machine**. It does not implement or pretend to implement cryptographic signature validation. The upstream trust adapter is responsible for authenticating metadata and passing its result through a separate trusted context—not as a field in the untrusted policy request.

## 2. Reuse established secure-distribution machinery

Do not invent a bespoke signature-and-update protocol unless a concrete interoperability requirement demonstrates that existing machinery is inadequate.

The recommended production direction is to adapt the [The Update Framework (TUF) specification](https://theupdateframework.github.io/specification/v1.0.36/) for policy-artifact distribution. TUF already defines a metadata verification workflow intended to address rollback, freeze, mix-and-match, key-compromise and wrong-target attacks. Bind each policy profile as a verified target; place the policy identity, audience, schema digest, release version and revocation epoch in the relevant authenticated target metadata/custom metadata; use TUF's root/delegated trust, snapshot consistency, timestamp expiry and version checks for distribution authenticity and freshness.

The Mycelix lifecycle state machine remains responsible for application semantics TUF does not know about: which policy identifier and audience are expected, whether the profile schema is the accepted one, whether the target bytes match the descriptor, whether this release is older than the locally accepted policy head, and whether its revocation epoch meets the locally required floor.

This creates a composition rather than a competing update framework:

1. TUF-compatible adapter verifies root/delegation, signatures/threshold, metadata versions, expiry, snapshot consistency and target hash.
2. The adapter obtains the exact policy target bytes and trusted time/status from its trusted boundary.
3. The Mycelix lifecycle evaluator checks domain/audience, profile/schema digest, validity interval, revocation floor and monotonic policy version.
4. The existing claim-ceiling policy evaluator checks the profile contents against the candidate claim ceiling.
5. Canonical evidence admission and operational authorization remain separate decisions.

TUF verification should be reused rather than reimplemented inside this crate. The trusted adapter's exact identity, configuration, root metadata and result should be bound into a future receipt before production integration.

## 3. Release descriptor and trusted context

The release descriptor schema is strict and rejects unknown properties. Its fields are:

- `policy_id` and `audience`: exact, non-wildcard intended identity/scope;
- `version`: strictly monotonic policy release version;
- `revocation_epoch`: policy revocation generation;
- `target_profile_sha256`: expected digest of the exact policy profile target bytes;
- `policy_profile_schema_sha256`: digest of the supported profile contract/schema;
- `issued_at_unix_seconds` and `expires_at_unix_seconds`: explicit validity interval.

The target descriptor's authenticity comes from the authenticated distribution metadata that carries it; the schema only validates its structure. A digest or valid JSON schema does not authenticate the publisher.

The trusted context must be supplied out of band and must include the expected policy identity/audience, expected schema digest, minimum revocation epoch, an authoritative time value, the distribution verifier's status, and the previously accepted high-water mark. The head is explicitly scoped by both policy ID and audience; reusing a high-water mark from a different scope is rejected. This context must never be loaded from the candidate release payload.

The receipt binds the exact release descriptor bytes, the release-descriptor schema digest, the exact profile-target bytes, and a SHA-256 of the deterministic serialization of the trusted evaluation context (policy/audience expectations, schema pin, time, revocation floor, verifier status, and installed high-water mark). The receipt itself is checked against a strict Draft 2020-12 schema. The context digest is an audit binding—not proof that the context provider was trustworthy; that authority still comes from the integration boundary.

The reference module emits a proposed next high-water mark but does not persist it. Production state storage must perform an atomic compare-and-update so two concurrent evaluators cannot both advance state from the same prior head, and so rejected/failed checks never mutate the trusted head. If local anti-rollback storage itself can be rolled back, the system needs stronger state anchoring or a remotely attested/consensus-backed high-water mark.

## 4. State-machine rules

- **Accept new release:** all distribution, identity, audience, schema, target digest, validity and revocation checks pass; and the version is newer than the installed head or no head exists.
- **Already current:** the version, profile digest and revocation epoch exactly match the installed head, and all freshness and authority checks pass. This is idempotent and does not advance state.
- **Reject:** any check fails. A rejected receipt never proposes a next head.

Stable violation codes cover invalid signatures, unknown/revoked publisher keys, expired/stale metadata, rollback, mix-and-match, unmet signature threshold, identity/audience mismatch, schema/profile digest mismatch, unavailable trusted time, invalid/future/expired release intervals, stale revocation epoch, version rollback and version equivocation.

Missing trusted time is rejection, not “best effort”. An expired TUF timestamp/metadata chain is not rescued by an otherwise valid profile hash. An exact old profile digest is not currentness evidence. A passing lifecycle result is still not a claim that profile semantics are correct.

## 5. Frozen conformance corpus

The machine-readable corpus at `fixtures/claim-policy-lifecycle-golden-v1.json` contains 19 deterministic scenarios. It includes:

- valid new-release acceptance and idempotent exact replay;
- lower-version rollback and same-version digest equivocation;
- invalid signature, unknown publisher key, revoked publisher key, unmet signature threshold;
- expired/stale metadata, distribution rollback and mix-and-match;
- missing trusted time, future-issued and expired release windows;
- wrong audience, profile-byte tampering, schema-digest mismatch, and stale revocation epoch.

The Rust reference evaluator executes the corpus and checks the expected disposition, stable violation codes, and the invariant that rejected candidates cannot produce a proposed next state. These are reference state-machine tests. They do **not** prove a TUF adapter, signature verifier, trusted clock, secure storage layer, or production integration works correctly.

## 6. Benchmark plan — targets are not measurements

First record reproducible baselines with exact source SHA, compiler/toolchain, host, input corpus digest and seed. Measure the lifecycle evaluator separately from distribution-chain signature verification, I/O and durable state update; publish combined end-to-end results too.

| Metric | Proposed target / acceptance |
|---|---|
| Frozen lifecycle negative controls | 100% reject with the expected stable code set |
| Distribution verifier failures | 100% reject; no verified-current success result from invalid metadata |
| State-machine corpus | 100% agreement with the frozen expected decisions |
| Version monotonicity | 0 accepted rollbacks or same-version equivocations in the conformance and generated corpora |
| Missing trusted time/revocation status | 100% fail closed |
| Generated attack corpus | 100,000 seeded cases, zero observed false accepts; retain seeds and minimized counterexamples |
| Fuzzing | Initial target: 24 CPU-hours per release candidate; retain corpus, crashes, OOM and timeout records |
| Lifecycle decision latency | p95 below 1 ms for the pure evaluator at 10,000 prevalidated scopes in a documented reference environment (hypothesis; measure before adoption) |
| Whole chain | Report p50/p95/p99/max latency, throughput, peak memory, distribution verification cost and atomic state-update cost separately |

These targets should be revised only from measured evidence and a documented rationale. Zero failures in a generated corpus is not a proof of no false accepts. No current performance superiority over Cedar or OPA is claimed.

## 7. Industry comparison

- **NIST ABAC, SP 800-162:** authorization uses attributes of the subject, object, operation and environment against policy. Mycelix's policy identity, audience, claim atoms and evidence provenance should retain this explicit contextual framing while separating policy fit from authorization. [NIST publication](https://www.nist.gov/publications/guide-attribute-based-access-control-abac-definition-and-considerations)
- **Cedar:** policy validation checks policy structure/type usage against an application schema before runtime authorization. Mycelix should similarly validate contracts before evaluation, then add its domain-specific evidence, lifecycle and revocation constraints. Cedar's documentation explicitly distinguishes schema validity from the question of whether policy intent is correct. [Cedar validation](https://docs.cedarpolicy.com/policies/validation.html)
- **Open Policy Agent (OPA):** provides policy testing/coverage and query benchmarks, and supports signed policy bundles with verification configured through trusted keys. Match this with reproducible coverage and latency reports instead of claiming superiority without comparable workloads. [OPA policy testing](https://www.openpolicyagent.org/docs/policy-testing) · [OPA policy performance](https://www.openpolicyagent.org/docs/policy-performance) · [OPA bundles](https://www.openpolicyagent.org/docs/management-bundles)
- **TUF:** explicitly targets rollback, freeze, mix-and-match, key-compromise and wrong-target attacks in secure update systems. Reuse those update-security controls for profile distribution instead of creating a parallel homegrown update protocol. [TUF v1.0.36 specification](https://theupdateframework.github.io/specification/v1.0.36/)

Mycelix's potential differentiator is a typed epistemic policy contract that composes with well-established secure distribution rather than attempting to replace it, and never infers factual truth or operational authority from a policy-profile pass.

## 8. Production integration gates

Before promotion out of reference status:

1. Select and pin the TUF implementation, trust-root bootstrap, root-rotation procedure and metadata-verification configuration.
2. Define the authoritative revocation source/epoch and maximum freshness window; document offline behavior and failure semantics.
3. Implement atomic high-water-state updates and test crash/concurrency/rollback behavior.
4. Bind release descriptor, exact profile bytes, TUF metadata/root snapshot, verifier build/source SHA, trusted-time evidence, revocation epoch and receipt into an independent qualification record.
5. Add end-to-end negative tests in the canonical evidence-admission and action-authorization consumers, proving that lifecycle or profile fit alone never admits evidence or executes an operation.
6. Require exact-head hosted conformance and independent-verifier evidence. Queued, missing or skipped checks remain unqualified.
