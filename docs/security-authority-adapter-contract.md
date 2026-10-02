# Security Authority Adapter Contract

This document defines the integration seam between the bridge security kernel and the existing generation-bound institutional authority work.

It deliberately does not duplicate authority identity or invent a second revocation protocol.

## Authority facts remain separate

The adapter must keep three propositions distinct:

1. **Immutable authority identity** — the exact semantic authority object.
2. **Historical validity** — whether that object was valid for a historical event.
3. **Current freshness** — whether that exact object remains usable for current execution.

A current authorization decision must never be reconstructed from a bare grant ID, a historical validity result, or a model/reputation signal.

## Existing canonical identity

Institutional grants use the canonical profile:

`mycelix-authority-grant-v1-blake3-framed-semantic`

The adapter must consume that identity rather than introduce another grant hash.

The canonical identity includes authority-bearing grant semantics, including:

- institutional-core protocol version;
- grant ID and holder principal;
- institution and jurisdiction;
- complete roles and capabilities;
- exact rulebook identity;
- complete authority-source set;
- issued/expiry times;
- delegation parent;
- grant proof lineage.

Therefore a changed proof lineage, delegation parent, capability set, or rulebook produces a different authority identity.

## Current freshness

The freshness layer supplies an exact current snapshot for each required authority subject.

The adapter must preserve:

- exact immutable subject identity;
- non-zero generation;
- current status (`Active`, `Revoked`, or `Superseded`);
- effective timestamp;
- immutable status-record/content reference;
- authoritative source reference;
- verification reference;
- verification timestamp;
- bounded freshness lease.

The stable authority identity and the dynamic verification/lease metadata must not be conflated.

## Capability-to-authority binding

For a signed capability:

`SignedCapability.issuer_public_key`

must first be bound to the expected issuer key and the Ed25519 signature must verify over the canonical capability bytes.

That proves possession/integrity of the signing key and signed capability bytes.

It does **not** prove institutional authorization.

The adapter must then establish:

`issuer key -> authorized institutional authority -> canonical AuthorityGrant identity -> current freshness -> policy authorization`

Only after this chain is established may the bridge kernel receive trusted verification evidence.

## Fail-closed rules

The adapter must return no positive current-authority evidence when:

- the issuer key is not authorized for the required institutional authority;
- the canonical authority identity does not match the required grant;
- a required subject is missing;
- unexpected authority subjects are present;
- two conflicting current snapshots exist for one subject;
- the current generation is revoked or superseded;
- the freshness lease is expired;
- verification time is invalid/future;
- delegation lineage is ambiguous or invalid;
- proof lineage does not match the canonical authority object.

Ambiguity is not an Allow.

## Race closure

Authorization is intentionally two-stage:

1. obtain and verify current authority;
2. mint a short-lived permit;
3. revalidate current revocation/authority evidence at enforcement time;
4. enforce only if the permit remains valid.

This closes the authorization-to-enforcement revocation race represented by the bridge kernel.

A later generation must invalidate the old authority domain for new execution even if the historical event remains cryptographically reconstructable.

## Holochain boundary

Holochain integrity validation should validate deterministic relationships and signed evidence, not select mutable "latest" authority state.

Current authority status belongs in the runtime/application authority provider. If the authoritative current snapshot cannot be resolved deterministically, the security kernel must fail closed rather than select a record by timestamp, DHT order, or local preference.

## Evidence hand-off

`VerificationEvidence` remains opaque and non-serializable outside the bridge crate.

The integration must therefore create trusted evidence only inside the bridge's verifier boundary. Public callers must not receive a constructor that accepts arbitrary booleans such as `verified: true`.

The resulting evidence must also be bound to the exact capability and authority-freshness state it verifies. The bridge kernel derives a stable capability commitment from the canonical capability semantics and records that commitment in the opaque evidence and any issued permit. The adapter must also supply an opaque authority-freshness commitment that changes when the authoritative generation changes. The permit carries that commitment through enforcement, so evidence for a newer authority generation cannot silently revalidate a permit issued under an older generation. Evidence for one capability therefore cannot be replayed to qualify or revalidate a different capability.

The resulting evidence should represent independently established propositions:

- signature verified;
- authority is currently not revoked;
- authority resolution is unambiguous;
- evidence is bound to the exact capability semantics;
- the evidence is bound to the exact authority generation/freshness state;
- the bounded freshness lease remains valid.

## Required qualification scenarios

The integration tranche is not complete until deterministic tests cover:

1. valid signed capability + authorized current grant;
2. valid signature + unauthorized issuer key;
3. valid grant identity + revoked current generation;
4. stale freshness lease;
5. generation advance after a previously issued permit;
6. conflicting current snapshots;
7. missing/unexpected authority subject;
8. changed delegation parent;
9. changed proof lineage;
10. partitioned authority state that cannot establish one current answer;
11. evidence verified for a different capability;
12. enforcement-time revocation after an earlier Allow;
13. advisory/model output attempting to expand capability.

## Evidence levels

- **D0:** this contract and design only.
- **D1:** deterministic unit/property tests.
- **D2:** multi-agent Holochain/Tryorama qualification.
- **D3:** partition, replay, revocation-race, and malformed-evidence scenarios.
- **D4:** independent security review.

No level may be inferred from a lower level.

## Integration rule

The next implementation should depend on the existing authority freshness and canonical grant identity work when those stacked drafts are integrated. Until then, this repository should not introduce a parallel authority identity, parallel generation model, or synthetic revocation registry merely to make the bridge compile.

The security kernel remains the enforcement boundary; the authority adapter supplies evidence, and Symthaea remains advisory.
