# MYC-CAP-002H2 — Current public-service distribution composition v1

Status: candidate executable composition profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Compose one narrow #1020 authority dimension, `public_service_distribution_state`, from exactly two qualified inputs: MYC-CAP-002F service-distribution eligibility and MYC-CAP-002F1 service-evidence currentness.

Core separation:

`historically ELIGIBLE != currently eligible for distribution`

`CURRENT service evidence != service covenant PASS`

`ELIGIBLE_CURRENT != payment authority`

## Qualified parent subjects

Service gate exact subject:

`92fe8408b52ce1b63808201e9f70ab5280a6f489`

Service currentness exact subject:

`caf653560310c9d47b036ef88a2207bff6d4b06f`

The H2 Git parent is the exact qualified F1 subject. The profile separately binds the exact qualified F subject named by F1.

## Positive theorem

A positive result requires F `distribution_eligibility = ELIGIBLE`, F `claim_modified = false`, F1 `currentness_state = CURRENT`, F1 `claim_modified = false`, F1 `uses_local_wall_clock = false`, exact project agreement, exact qualified F subject, service-profile agreement, measurement-ID agreement, exact F receipt digest, and eligibility-echo agreement.

Only then:

`public_service_distribution_state = ELIGIBLE_CURRENT`

## State precedence

After exact semantic binding:

- F `NO_ACTIVE_CLAIM` -> `NO_ACTIVE_CLAIM`;
- active claim + F1 `STALE` -> `STALE`;
- active claim + F1 `PENDING` -> `PENDING`;
- active claim + F1 `REVOKED` -> `REVOKED`;
- F1 `CURRENT` + F `BLOCKED` -> `BLOCKED_SERVICE`;
- F1 `CURRENT` + F `ELIGIBLE` -> `ELIGIBLE_CURRENT`.

A stale historical service measurement is therefore neither a permanent permission nor a permanent service verdict.

## Authority boundary

Every receipt fixes:

`claim_modified = false`

`execution_authority_established = false`

`payment_authority_established = false`

`legal_distribution_authority_established = false`

H2 establishes only current service-covenant distribution eligibility relative to the supplied qualified currentness lineage.

## Frozen semantic commitments

Composition profile SHA-256:

`34f6a89cc1ea79b045a55c538b2786e6e7ecd4171978ceade6be47017906725e`

Qualified F receipt semantic SHA-256:

`347cf7cc038ba918cade9e1e210871ddb6039440d9af5ea78c3745594acc8969`

Qualified F1 receipt semantic SHA-256:

`2d64bfa8fe3f0351c04c297d8ae4ef66fac930d2e193a2bfffe2072802a94bf0`

Canonical H2 receipt semantic SHA-256:

`56f9c0437f509c21ab6b80e938fb1050c3836761bc92dfb7203882ce2d93a76a`

## Local preflight

The stdlib suite passes **20/20** locally, covering ELIGIBLE_CURRENT, STALE/PENDING/REVOKED, BLOCKED_SERVICE, NO_ACTIVE_CLAIM, superseded F-subject rejection, project/measurement/profile/receipt substitution, eligibility echo mismatch, claim-authority contamination, local-wall-clock rejection, raw snapshot/designation substitution, authority-amplifying unknown fields, deterministic blocker ordering and replay, and strong nonclaims.

Local PASS is not hosted qualification.

## Nonclaims

Even hosted PASS would not establish payment authorization, legal distribution entitlement, tariff legality, measurement authenticity, utility/regulatory compliance, financial satisfaction, handback readiness, custody currentness, democratic legitimacy, or execution authority.
