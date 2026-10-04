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
- bounded freshness lease;
- the existing current-freshness semantic digest used as the bridge authority binding.

The stable authority identity and the dynamic verification/lease metadata must not be conflated.

### Time-unit conversion and lease monotonicity

The authority freshness work expresses its bounded lease in milliseconds (`lease_until_ms`), while the bridge kernel expresses authorization timestamps in microseconds (`valid_until_us`, `now_us`). The adapter must perform an explicit checked conversion before crossing this boundary:

`lease_until_us = lease_until_ms.checked_mul(1_000)`

Qualification currently freezes this conversion with a checked test helper; the future authority adapter must perform the same checked conversion at its integration boundary rather than compare milliseconds directly with microsecond kernel timestamps. A multiplication overflow is invalid authority evidence and must fail closed; it must never wrap, saturate, or be compared across units. In particular, `lease_until_ms` must never be compared directly with a microsecond timestamp.

The adapter must pass the converted lease as an upper bound, never as a new source of authorization duration. The bridge's effective permit expiry remains the minimum of capability expiry, converted freshness expiry, and the kernel maximum permit lifetime. A refreshed lease may therefore support a **new** authorization flow, but must not extend an already-issued permit. The authority-freshness semantic digest remains independent of lease metadata, so a lease/proof refresh without a semantic authority-generation change does not by itself create a new authority identity; a generation/state change must produce a different freshness binding and invalidate the older permit domain at enforcement.

## Capability-to-authority binding

For a signed capability:

`SignedCapability.issuer_public_key`

must first be bound to the expected issuer key and the Ed25519 signature must verify over the canonical capability bytes.

That proves possession/integrity of the signing key and signed capability bytes.

It does **not** prove institutional authorization.

The adapter must then establish:

`issuer key -> authorized institutional authority -> canonical AuthorityGrant identity -> current freshness -> policy authorization`

For the bridge hand-off, the authority-freshness commitment should be the existing `CurrentAuthorityFreshness.freshness_digest` (or the exact equivalent from the integrated authority stack). That digest represents the authoritative freshness domain; dynamic `verified_at` / lease metadata remains separate evidence.

Only after this chain is established may the bridge kernel receive trusted verification evidence.

The bridge binding is derived deterministically from that stable freshness commitment. The `mycelix-bridge-common::authority_binding_from_freshness_digest` helper is private to `security_kernel` so the derivation domain is not part of the bridge's crate-wide API; it commits the authority-binding domain separator, the bridge's pinned canonical freshness protocol/profile, and the exact `CurrentAuthorityFreshness.freshness_digest`. The protocol/profile are not caller-supplied, preventing an adapter from accidentally interpreting a digest under a different freshness identity scheme. The helper does not include `verified_at_ms`, `lease_until_ms`, transport metadata, or other dynamic proof fields.

Therefore:

- unchanged semantic freshness with a renewed verification lease preserves the bridge authority binding;
- a changed generation/state, which changes `freshness_digest`, necessarily changes the bridge authority binding; and
- a changed freshness protocol/profile cannot be interpreted as the same binding domain.

This wrapper is not a second authority identity. The authoritative semantic identity remains PR #75's canonical grant identity plus PR #74's generation-bound freshness commitment; the bridge only domain-separates that existing commitment for its permit/evidence boundary.

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

Authorization is intentionally bounded in two layers:

1. obtain and verify current authority;
2. mint a short-lived permit;
3. revalidate current revocation/authority evidence at enforcement time;
4. enforce only if the permit remains valid.

This closes the authorization-to-enforcement revocation race represented by the bridge kernel.

A later generation must invalidate the old authority domain for new execution even if the historical event remains cryptographically reconstructable.

A freshness-lease renewal is different: it may authorize a new permit under the unchanged authority domain, but it must never extend an already-issued permit. Existing permits are historical bounded artifacts; renewed evidence can only support a subsequent authorization flow.

## Holochain boundary

Holochain integrity validation should validate deterministic relationships and signed evidence, not select mutable "latest" authority state.

Current authority status belongs in the runtime/application authority provider. If the authoritative current snapshot cannot be resolved deterministically, the security kernel must fail closed rather than select a record by timestamp, DHT order, or local preference.

## Evidence hand-off

`VerificationEvidence` remains opaque and non-serializable outside the bridge crate.

The integration must therefore create trusted evidence only inside the bridge's verifier boundary. The proposition types and evidence constructors are private to `security_kernel`; they are not crate-wide construction APIs. Public callers must not receive a constructor that accepts arbitrary booleans such as `verified: true`. Production bridge evidence construction derives its opaque authority binding from the canonical freshness digest; the raw authority-binding constructor is test-only and exists solely to exercise mismatch/fail-closed paths.

The resulting evidence must also be bound to the exact capability and authority-freshness state it verifies. Audit metadata is likewise treated as untrusted input: recovery correlations pass through their validating constructor and provenance sequences are bounded during wire decoding before the event is admitted. Capability wire decoding is constructor-gated, and the adapter must not treat a deserialized capability as validated until the normal verification boundary has accepted it. Security-event provenance references are likewise constructor-gated on wire decode and expose private invariant-bearing fields, so malformed provenance cannot bypass `ProvenanceRef::new()` through serialized input or direct struct construction. The bridge kernel derives a stable capability commitment from the canonical capability semantics and records that commitment in the opaque evidence and any issued permit. The adapter must also supply the existing current-freshness semantic digest as the opaque bridge authority binding. Serialized capability and authorization-request inputs are accepted only through their strict constructor-gated wire schemas; unknown fields must not be interpreted as authority semantics. It must change when the authoritative freshness domain changes, including an authority generation/state change, while remaining stable across proof/lease refreshes that do not change that semantic domain. The permit carries that commitment through enforcement, so evidence for a newer authority generation cannot silently revalidate a permit issued under an older generation. The permit itself is opaque to downstream callers until `EnforcementRequest::from_permit()` consumes it; no public permit-inspection API may become an alternative enforcement path. Both the permit and the resulting enforcement request are marked `#[must_use]`, and neither implements `Debug`, `Serialize`, `Deserialize`, `Clone`, or `PartialEq/Eq`, preventing common logging, serialization, duplication, or equality traits from widening the enforcement boundary. Evidence for one capability therefore cannot be replayed to qualify or revalidate a different capability.

The resulting evidence should represent independently established typed propositions:

- `SignatureVerification::Verified` / `Invalid` — the capability signature check result;
- `RevocationStatus::Current` / `Revoked` — the authoritative current-status result;
- `AuthorityResolution::Unambiguous` / `Ambiguous` — the current authority-resolution result.

These types prevent category confusion, but they are not provenance attestations by themselves. The adapter must establish each proposition from the corresponding verifier/authority check before crossing the kernel boundary.
- authority-freshness binding is present and non-zero;
- evidence is bound to the exact capability semantics;
- the evidence is bound to the exact authority generation/freshness state;
- the bounded freshness lease remains valid.

## Required qualification scenarios

A zero authority-freshness commitment is treated as missing authority evidence and fails closed.

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
