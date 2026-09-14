# Institutional Core Typed-Identifier Hardening v0.1

## Scope

This tranche closes one constructor/deserialization bypass class in `mycelix-institutional-core` without changing the authority vocabulary, wire field layout, valid-ID grammar, or `ValidationError` taxonomy.

The existing public ID tuple types can be constructed directly or deserialized without calling `IdType::new(...)`. Therefore any positive authority path must validate the contents of those typed IDs again at the semantic boundary.

## IC-ID-001 — Constructors are convenience, not proof

A value having Rust type `CapabilityId`, `RoleId`, `PrincipalId`, `InstitutionId`, `JurisdictionId`, or `AuthorityGrantId` is not by itself proof that the wrapped string satisfies the institutional-core ID invariant.

Every ID type exposes `validate()` using the same bounded non-empty rule as `new()`.

## IC-ID-002 — Authority grants validate every authority-bearing typed ID

`AuthorityGrant::validate()` must validate:

- grant ID;
- holder principal;
- institution;
- optional jurisdiction;
- every role;
- every capability;
- optional `delegated_from` grant ID;
- rulebook;
- every authority source; and
- grant proof/lifetime requirements already enforced by v0.1.

A malformed role/capability may never become usable authority merely because the requirement contains the same malformed wrapper value.

## IC-ID-003 — Authority requirements validate their complete typed scope

`AuthorityRequirement::validate()` must validate:

- institution;
- optional jurisdiction;
- every required capability;
- every accepted role;
- every evidence requirement; and
- exact rulebook.

`evaluate_authority()` must therefore deny malformed requirement scope before capability/role comparison can produce `Allow`.

## IC-ID-004 — Evidence issuer identity is validated

`EvidenceRef::validate()` validates optional issuer `PrincipalId`.

An evidence item with a malformed issuer must not satisfy even an issuer-open evidence requirement, because the evidence object itself is invalid.

## IC-ID-005 — Accepted evidence issuers are validated

`EvidenceRequirement::validate()` validates every `accepted_issuers` principal.

A malformed allow-list identity makes the requirement invalid; it cannot be paired with an equally malformed evidence issuer to create authority.

## IC-ID-006 — Error taxonomy and valid wire data are preserved

This hardening adds no new `ValidationError` variant and changes no serialized field layout.

Constructor-valid v0.1 authority data remains valid. Malformed directly constructed/deserialized typed IDs now fail through the existing `Empty` / `TooLong` validation errors and are surfaced by `evaluate_authority()` as the existing `invalid_grant` or `invalid_requirement` denial domains.

## IC-ID-007 — This is not canonicalization

v0.1 does not yet claim duplicate elimination/canonical set identity for roles, capabilities, sources, or accepted issuers.

That is a distinct representation/canonicalization theorem and should not be smuggled into this patch under the guise of validation.

## IC-ID-008 — No new authority source

This change only makes existing authority evaluation more fail-closed.

It does not add authority sources, roles, capabilities, evidence types, currentness, persistence, runtime provider trust, signatures, Holochain, or external effects.

## Adversarial acceptance corpus

The integration tests directly construct malformed public tuple IDs—intentionally bypassing `new()`—and require denial for:

- malformed grant capability;
- malformed grant role;
- malformed delegated parent grant ID;
- malformed required capability;
- malformed accepted role;
- malformed requirement institution/jurisdiction;
- malformed accepted evidence issuer; and
- malformed actual evidence issuer.

A constructor-valid grant/requirement remains allowed as the non-regression control.
