# ECON-005 — MYCEL Domain Credential Fabric

**Status:** Draft policy model. Existing recognition, fee, staking, and mediator runtime behavior is not changed by this tranche.

## Purpose

ECON-005 changes the conceptual source of truth for MYCEL from a universal scalar into a set of evidence-backed, domain-scoped credentials.

A credential states that a subject has evidence relevant to a bounded capability in a bounded domain. It has an issuer, evidence references, issuance time, expiry, challenge state, and revocation state.

The policy does not treat MYCEL as money, human worth, or automatic authority.

## Credential-first model

A valid credential requires:

- subject reference;
- non-empty domain;
- non-empty capability;
- issuer reference;
- at least one evidence reference;
- a finite validity window with expiry after issuance;
- no universal human-worth scope.

A challenged, revoked, not-yet-valid, or expired credential cannot support a role review.

## Domain separation

Evidence from one domain cannot silently establish competence in another. A music/audio credential does not establish mediation competence; a mediation credential does not establish medical, financial, engineering, or governance competence.

Matching a credential to a bounded role still returns only **support for independent review**. It does not grant the role automatically.

## Prohibited direct uses

MYCEL or a MYCEL-derived scalar must not directly determine:

- fundamental civic rights;
- human-worth ranking;
- automatic SAP fee discounts;
- automatic TEND credit expansion;
- automatic economic payouts.

These prohibitions make current MYCEL-derived fee tiers explicit migration debt rather than canonical design.

## Derived views

A compact score may remain as a user-interface/cache view for a declared domain, but it must retain references to the supporting credentials and is never the source of authority. The credential/evidence graph remains primary.

## Economic evidence boundary

SAP and TEND activity may be relevant evidence for some credential, but ECON-002 requires independent review before that evidence affects standing. There is no fixed payment→MYCEL or TEND→MYCEL conversion formula.

This directly changes the target semantics for current TEND quality-rating validation and similar automatic standing promotion paths.

## Migration implications

Later runtime tranches should replace or reclassify:

- MYCEL-derived SAP fee tiers;
- TEND-rating auto-promotion into general MYCEL;
- universal MYCEL thresholds for mediator eligibility;
- recursively MYCEL-weighted recognition without sufficient caps/diversity/contestability;
- general MYCEL use as a proxy for economic risk or privilege.

Those changes require separate runtime patches and tests; ECON-005 does not claim them complete.

## Qualification target

Target commands:

- `cargo fmt --manifest-path mycelix-finance/mycel-policy/Cargo.toml -- --check`
- `cargo test --manifest-path mycelix-finance/mycel-policy/Cargo.toml --locked`
- `cargo clippy --manifest-path mycelix-finance/mycel-policy/Cargo.toml --all-targets --locked -- -D warnings`

Tests cover evidence/expiry requirements, challenge/revocation, domain separation, bounded-role review, prohibited rights/economic uses, decomposable derived views, and economic-event review requirements.

## Non-goals

This tranche does not define a credential cryptographic format, claim W3C VC conformance, select issuers, change constitutional voting, alter current zome storage, or automatically decide any role.
