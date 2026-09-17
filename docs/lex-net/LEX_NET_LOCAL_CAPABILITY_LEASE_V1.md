# LEX-NET-025 — Local Capability Lease Envelope v1

Status: executable research contract, R2. Language-neutral, deterministic, zero-network, and local-authority scoped.

## Governing theorems

`qualified 028 mint result != presentation-ready local capability envelope`

`presenter possession != authority`

`lease envelope != authoritative consumption state`

`portable evidence != portable authority`

`lease envelope validity <= underlying lease validity`

LEX-NET-028 R3 qualifies evidence-set satisfaction, support horizons, local mint authorization, and the underlying immutable `LocalCapabilityLease`. LEX-NET-025 does not re-prove that theorem. It qualifies the additional envelope constraints required before a locally minted lease can be presented to a local effect adapter.

LEX-NET-040 owns authoritative latest consumption state, concurrency, stale-state replay, and double-consume prevention.

## Qualified parent

This tranche is a direct child of qualified LEX-NET-028 R3 exact head `10c9cbd5f3044dbd18106ca17737c24d75aa1482`.

Parent qualification: run `35278138912`, attempt 1, job `105393578846`, conclusion `success`.

## Product domains

Source lease:
`LEX-NET/AUTHORITY/v3 :: LocalCapabilityLease`

Presentation envelope:
`LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1 :: LocalCapabilityLeaseEnvelope`

Cross-domain export:
`LEX-NET/EVIDENCE/v3 :: EvidenceAboutForeignAuthority`

The envelope is not an evidence subtype. Re-labeling evidence or a foreign wrapper does not create authority.

## Source lease requirements

Only a commitment-valid **and structurally valid** LEX-NET-028 `LocalCapabilityLease` is accepted. Before envelope construction the reference profile requires:

- non-empty local authority domain;
- non-empty local grant commitment;
- non-empty subject, purpose, resource, and action;
- non-empty satisfaction-evaluation commitment;
- non-empty mint-authorization commitment;
- integer mint/expiry/support horizons;
- `mint_time <= current_until <= support_current_until`;
- positive integer use budget.

`commitment-valid != structurally valid`.

Tampered, wrong-kind, or structurally malformed source products are rejected before envelope construction.

## V1 exact-scope policy

V1 defines no resource/action hierarchy. The envelope preserves exact `(subject, purpose, resource, action)`. Scope widening is forbidden; scope narrowing is deferred until a separately frozen attenuation lattice exists.

## Presenter binding

Presenter binding is distinct from authority. V1 requires binding type exactly one of `key`, `session`, or `principal`, plus a non-empty presenter-binding commitment.

Proof of possession or session control does not create the source authority.

## Envelope attenuation

`BindLeaseEnvelope(...)` may only attenuate:
- `valid_from >= source.mint_time`;
- `current_until <= source.current_until`;
- `current_until >= valid_from`;
- use budget is positive and no greater than source use budget;
- local domain and exact scope are preserved;
- transferability is `false`;
- redelegation is `false`;
- delegation depth is `0`.

V1 also requires a non-empty replay/idempotency domain, revocation-handle commitment, and local policy-epoch commitment. An optional assurance-profile commitment, when present, is exact-match at use time.

## Evaluation states

- `IssuedCurrent`
- `NotYetValid`
- `Expired`
- `EnvelopeInvalid`
- `SourceLeaseInvalid`
- `DomainMismatch`
- `PresenterBindingRequired`
- `PresenterBindingFailed`
- `ReplayDomainRequired`
- `RevocationBindingRequired`
- `RevocationStateMismatch`
- `PolicyEpochRequired`
- `PolicyEpochMismatch`
- `AssuranceProfileMismatch`
- `TransferDenied`
- `RedelegationDenied`
- `EnvelopeAttenuationViolation`

These states are not flattened into one boolean.

## Historical / executable split

An expired or otherwise unusable envelope may remain historical evidence about what was issued; it is not executable authority.

Cross-domain export always becomes `EvidenceAboutForeignAuthority` with:

`grants_local_authority = false`

`grants_external_effect_authority = false`

The destination must independently recognize that evidence and mint its own authority.

## Relationship to LEX-NET-040

`025 envelope PASS != 040 atomic consumption PASS`

The envelope carries replay-domain and use-budget ceilings but does not establish authoritative latest consumption state. Caller-supplied history cannot prove completeness, concurrency safety, or double-spend prevention.

## Golden/adversarial corpus

The corpus covers source commitment tampering, wrong source kind, exact-domain confinement, presenter binding, temporal/use-budget attenuation, transfer/redelegation defaults, replay and revocation/policy bindings, assurance matching, expiry, cross-domain export, wrapper/type substitution, and commitment-domain separation.

## Nonclaims

This tranche does not establish factual truth.

This tranche does not establish legal authority.

This tranche does not establish identity truth.

This tranche does not establish presenter cryptographic strength.

This tranche does not establish revocation-distribution freshness.

This tranche does not establish authoritative latest consumption state.

This tranche does not establish concurrency safety or double-spend prevention.

This tranche does not establish execution success or external finality.

This tranche does not establish cryptographic authenticity or provenance of local authority products beyond the frozen commitment-binding model.

This tranche does not establish production security.

A PASS establishes only the frozen local lease-envelope binding, exact-scope preservation, attenuation, presenter/domain/revocation/policy binding, and cross-domain degradation model.
