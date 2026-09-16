# MYC-CAP-002G2A — Governance decision currentness v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Derive whether one historical G1 stewardship decision remains current relative to one exact designated constitution profile and explicit revocation set.

Core separation:

```text
historical authorization
!= current authority

current constitution designation
!= constitution amendment authority

CURRENT
!= execution authority
```

G2A answers only **which constitution is designated current** and whether an existing G1 receipt remains current under it. It does not authorize constitutional amendment or execute the decision.

## States

The deterministic currentness states are:

```text
CURRENT
HISTORICAL
REVOKED
PENDING
NOT_ASSESSED
```

v1 uses no wall-clock time as an authority source.

## Protected constitutional invariants

Before a designated G1 profile can be treated as current, it must:

- validate against the G1 structural schema;
- bind the exact project;
- retain the exact allowed chamber topology;
- contain no `CAPITAL` chamber;
- keep `ASSET_LOCK_REMOVAL` prohibited and constitutional with enforcer protection;
- keep `STEWARD_SEAT_SALE` prohibited and constitutional with enforcer protection.

A registry pointer alone therefore cannot bless a constitution that has silently weakened these protections.

## Currentness theorem

```text
designation == PENDING
    -> PENDING

designation == REVOKED
    -> NOT_ASSESSED

G1 authorization != AUTHORIZED
    -> NOT_ASSESSED

exact decision receipt is explicitly revoked
    -> REVOKED

G1 receipt profile != designated active profile
    -> HISTORICAL

otherwise
    -> CURRENT
```

A `HISTORICAL` decision remains valid audit evidence. It does not silently authorize a new execution under a successor constitution.

## Explicit revocation

Revocations are project-, registry-, epoch-, authority-, and exact decision-receipt bound.

The revocation set is canonicalized by target receipt digest so harmless input ordering does not change the semantic commitment.

There is no `clear=true` or resurrection operation in v1. Re-authorization requires a new governance decision under the then-current constitution.

## Output authority boundary

Every `GovernanceDecisionCurrentnessReceipt` fixes:

```text
execution_authority_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

`CURRENT` means only current relative to the exact designated constitution and revocation set.

## Deterministic commitments

G1 designated constitution semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

G1 decision receipt semantic SHA-256:

`5a330393381cdce9bd3d509a3aeab50fe66bb9c688cfc479c1bcb5efc0cbdccc`

Currentness profile semantic SHA-256:

`750981209aaab465dc4e54195b711a9e59f3a29f2d3e0af71bb63f65fa5e543f`

Canonical fixture SHA-256:

`63c56620f2276c7f0f14b92c7f6ae9d266e5155562a539f65bad441ce321c9d4`

Frozen receipt SHA-256:

`929f5cdd8eef25e56d557a17a4f4a5ab59694579555d60ad75dc90a937973a1e`

Verifier SHA-256:

`478e80cf8d33a687a7d411f51c404e9ec36780d6aef0ad0b7603805392945323`

Regression-suite SHA-256:

`97d4bad29afcef5df8188b5f6dfc91d8c43aaac4af682bf4a92ef73eaec9a746`

## Local preflight

The stdlib suite passes **20/20** in a repository-shaped layout, covering:

- positive current derivation;
- pending/revoked designation;
- blocked G1 receipt;
- old-profile historical state;
- explicit decision revocation;
- designation authority/project/registry/epoch substitution;
- designation/profile-digest substitution;
- asset-lock protection weakening;
- steward-seat-sale protection weakening;
- capital-chamber injection;
- protected chamber-topology change;
- revocation authority/epoch substitution;
- duplicate revocation;
- G1 authority contamination;
- deterministic and revocation-order-independent reconstruction.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only currentness of one supplied G1 decision relative to one designated constitution profile and revocation set.

It would not establish:

- execution authority;
- legal or constitutional-law validity;
- democratic legitimacy;
- social consensus;
- identity/signature authenticity;
- amendment authorization;
- designation-authority legitimacy;
- moral correctness or wisdom of the decision.
