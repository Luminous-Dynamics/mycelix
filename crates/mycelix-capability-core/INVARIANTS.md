# CORE-CAP-001 — Domain-Neutral Bounded Capability Lease v1

Status: implementation candidate; not qualified.

## Governing separations

`structural lease != proof mint basis was legitimate`

`structural lease != proof issuer was authorized`

`presenter possession != mint authority`

`serialized/hash-valid lease != portable authority`

`lease != authoritative consumption state`

`lease != external effect`

CORE-CAP-001 is a small reusable structural waist. A consuming domain must independently qualify its mint basis and issuer authority before wrapping this product as executable authority.

## Exact scope

V1 preserves exact `(subject, purpose, resource, action)` UTF-8 strings. It defines no prefix, hierarchy, wildcard, inheritance, Unicode normalization, or implicit narrowing relation.

Every opaque semantic reference is:

- non-empty;
- free of leading/trailing whitespace;
- free of Unicode control characters; and
- bounded to 256 UTF-8 bytes.

Byte-distinct Unicode spellings remain distinct identities in v1.

## Evidence horizons

The mint constructor consumes the exact qualified `EvidenceLease` type from #181 for:

- the domain mint basis; and
- the local issuer authorization.

It computes the support horizon by exact intersection:

`verified_at = max(inputs)`

`valid_until = min(inputs)`

The capability mint time itself must satisfy:

`minted_at >= support_lease.verified_at`

and the usable interval must satisfy:

`valid_from >= minted_at`

`current_until <= support_lease.valid_until`

No transport object may claim that a capability was minted before its supporting evidence had been verified, even if the attacker recomputes the structural commitment.

No empty, future, expired, inverted, or zero-time evidence input becomes usable authority through the mint constructor.

Nested EvidenceLease transport fields are deserialized through a local `deny_unknown_fields` wrapper before reconstruction of the exact #181 type. This preserves the closed CORE-CAP v1 transport schema without changing #181's lease algebra.

## Presenter binding

The closed v1 presenter vocabulary is:

- `Key`
- `Session`
- `Principal`

The exact presenter-binding commitment is retained. Possession or successful presentation is not itself mint authority.

## Attenuation and defaults

V1 requires:

- positive issuer maximum use budget;
- positive requested use budget;
- both budgets <= 64;
- requested budget <= issuer maximum;
- `transferable = false`;
- `redelegation_allowed = false`;
- `delegation_depth = 0`;
- non-empty bounded replay domain;
- non-empty bounded revocation handle;
- non-empty bounded policy epoch;
- optional assurance profile, exact-match when present.

These values are committed. Future transferable/redelegable semantics require a new qualified profile.

## Wire type != validated structural type

CORE-CAP v1 uses two distinct Rust types:

```text
BoundedCapabilityLeaseWireV1
    [Serialize + Deserialize, untrusted transport]
        ↓ explicit structural validation
BoundedCapabilityLeaseV1
    [Serialize, deliberately NOT Deserialize]
```

A deserialized object therefore cannot inhabit the validated structural lease typestate directly.

`BoundedCapabilityLeaseV1::try_from_wire(...)` must re-check the complete closed schema, evidence intersection, time/budget attenuation, transfer/redelegation defaults, and exact commitment before the validated structural type exists.

This is still only a **structural** positive type:

`validated structural lease != domain-qualified executable capability`

A domain such as STEW must retain a separate non-forgeable wrapper that proves the exact mint basis and issuer authority were qualified under that domain's theorem.

## Closed schema and commitment

The v1 mint request, scope, presenter binding, wire lease, and nested EvidenceLease objects are closed schemas.

The exact lease has an explicit SHA-256 framed commitment domain:

`MYCELIX/CORE-CAP/LEASE/v1`

The commitment covers protocol version; mint basis; issuer authorization; local domain; exact scope; presenter; replay/revocation/policy bindings; assurance profile; both input EvidenceLeases; exact support intersection; mint/use horizon; issuer maximum; use budget; and transfer/redelegation defaults.

The conformance corpus freezes one exact deterministic golden commitment vector:

`adee00934208e3675266d1830e577bfc21e595c9bc5c0a27d16424dac6e20e55`

The commitment gives deterministic structural identity only:

`commitment-valid != provenance-valid != authorized`

## Live presentation checks

`validate_at(...)` checks the already-validated structural lease plus current time, exact authority domain, exact scope, presenter binding, revocation handle, policy epoch, and optional assurance profile.

A successful structural validation means only that the supplied lease is internally consistent with the exact context. The consuming domain still owns provenance and authority qualification.

The replay domain remains committed in CORE-CAP-001 and is consumed by CORE-CAP-002 invocation/state semantics rather than by the presentation-context check.

## Relationship to CORE-CAP-002

CORE-CAP-001 owns immutable structural lease identity and attenuation only.

CORE-CAP-002 owns deterministic consumption state, idempotency, replay-domain use matching, and CAS mechanics.

`CORE-CAP-001 PASS != CORE-CAP-002 PASS`

## Qualified design precedents

This candidate is built on exact selective convergence of:

- qualified LEX-NET-040 R5 `f5614e6d9c9e2ddb9d6690880b77dffdf6bf8b39`, run `35343124716` PASS; and
- qualified EvidenceLease #181 `58fa357e53d7e529362c5f766965498ee557d6ce`, run `34794620487` PASS.

Its first-parent capability mechanics also retain qualified LEX-NET-025 R3-Q3 ancestry.

These qualified subjects are design/parity evidence. They do not automatically qualify this extracted profile.

## Qualification prerequisites

The hosted qualifier must independently run exact-parent/tree/blob checks, rustfmt, all-target tests, warnings-denied Clippy, rustdoc warnings-denied, bare WASM where dependency-compatible, strict wire/typestate/closed-schema/no-authority audits, the independent golden vector, relevant LEX-NET-025 attenuation/presenter parity cases, exact #181 no-widening behavior, and immutable exact-head postflight.

The final qualification subject must also bind a reviewed Cargo-generated dependency capsule rather than treating a floating registry resolution as reproducible evidence.

No hosted Rust result exists until that exact candidate executes.

## Nonclaims

No factual truth, identity authentication, legal/cultural legitimacy, mint-basis authority, issuer authority, runtime authorization for any particular domain, authoritative consumption state, backend atomicity, external effect, finality, reproducible binary identity, runner hermeticity, or production security is established.
