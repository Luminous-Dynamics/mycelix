# MYC-CAP-002G1 — Stewardship decision authority gate v1

Status: candidate executable profile; local preflight only until hosted exact-head qualification executes.

## Purpose

Qualify whether one stewardship decision satisfies one frozen Capital-to-Commons governance authority profile without treating a generic community vote as universal authority.

Core separation:

```text
community vote
!= universal authority

ORDINARY
!= CONSTITUTIONAL
!= EMERGENCY

emergency authority
!= permanent asset-lock authority
```

## Frozen decision classes

v1 supports exactly:

```text
ORDINARY
CONSTITUTIONAL
EMERGENCY
```

Each known action code is assigned exactly one frozen class. A caller cannot upgrade or downgrade authority by relabeling the action in the decision evidence.

Unknown action codes fail closed.

## Conservative constitutional boundary

The v1 example constitution marks these actions prohibited regardless of vote totals:

```text
ASSET_LOCK_REMOVAL
STEWARD_SEAT_SALE
```

This is deliberate. v1 proves that an ordinary majority, constitutional relabel, or emergency path cannot silently privatize the commons or make stewardship seats privately transferable.

A later jurisdiction-specific theorem may model a lawful extraordinary path if one should exist. v1 does not invent one.

## Chamber model

The example profile contains four explicit chambers:

```text
USERS
WORKERS
PUBLIC
GUARDIAN
```

The profile—not the decision—freezes which chambers are required for each action and their quorum/approval thresholds.

All vote arithmetic uses bounded non-negative integers and parts-per-million thresholds. Floating point is forbidden.

A global or aggregate majority cannot bypass a required chamber.

## Conflicts and recusals

The profile freezes conflict codes that require recusal for particular actions.

For a required recusal, the supplied record must show:

```text
disclosed = true
recused = true
counted_vote = false
```

Disclosure alone is insufficient.

A record claiming both recusal and a counted vote fails closed.

v1 validates the supplied conflict/recusal evidence but does not independently reconstruct all chamber counts from individual voter identities. That remains an explicit nonclaim.

## Constitutional concurrence

Actions marked constitutional may require independent-enforcer concurrence in addition to chamber quorum and approval thresholds.

The enforcer reference is frozen by profile. A different reference or missing concurrence blocks authorization.

## Emergency authority

Emergency actions require an explicit supplied emergency declaration with:

- exact action scope;
- exact frozen emergency authority;
- `state = ACTIVE`;
- evidence reference.

Pending or revoked emergency state blocks authorization.

Non-emergency decisions may not inject emergency evidence.

Core asset-lock and steward-seat-sale actions remain prohibited in v1 and are not made possible by emergency state.

## Output

The deterministic `StewardshipDecisionReceipt` exposes:

- action and decision class;
- per-chamber quorum/approval results;
- conflict/recusal results;
- enforcer/emergency requirements;
- exact blockers;
- `authorization_state = AUTHORIZED | BLOCKED`.

Every receipt fixes:

```text
asset_lock_removed = false
legal_validity_established = false
democratic_legitimacy_established = false
```

## Deterministic commitments

Canonical fixture file SHA-256:

`3a0a22eb368b13ac67103bc94d596ebdbe77df7921053526d9be65c5d185a58a`

Frozen receipt SHA-256:

`5a330393381cdce9bd3d509a3aeab50fe66bb9c688cfc479c1bcb5efc0cbdccc`

Verifier source SHA-256:

`c7b53e5d7e3e7f534b827581dafd225f2583495b3f03d417361bea0604685692`

Regression-suite SHA-256:

`00c55a662bf13e122df277ae92a391240a3d162ea41f0808b95614c2eb4a44d5`

Constitution profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

## Local preflight

The stdlib suite passes **23/23** locally, covering ordinary authorization, constitutional relabeling, prohibited asset-lock removal, prohibited steward-seat sale, required recusal, counted-recused vote rejection, quorum and approval failures, chamber-bypass attempts, enforcer concurrence, emergency state/scope, unknown actions, project/profile substitution, invalid authority arithmetic, duplicate conflict identities, legal-authority injection, and deterministic reconstruction.

Local PASS is not hosted qualification.

## Nonclaims

Even a hosted PASS would establish only that one supplied decision satisfies one frozen authority/quorum/conflict/emergency profile. It would not establish democratic legitimacy, constitutional-law validity, municipal-law compliance, identity/signature authenticity, social consensus, moral correctness, wisdom of the decision, or whether the frozen constitution itself is normatively desirable.
