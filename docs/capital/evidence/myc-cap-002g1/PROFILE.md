# MYC-CAP-002G1 — Stewardship decision authority gate v1

Status: candidate executable profile; hosted qualification must bind the exact Git subject, exact parent, exact six-file scope, deterministic receipt bytes, and strong nonclaims.

## Purpose

Qualify whether one stewardship decision satisfies one frozen Capital-to-Commons governance authority profile without treating a generic community vote as universal authority.

Core separation:

```text
community vote != universal authority
ORDINARY != CONSTITUTIONAL != EMERGENCY
emergency authority != permanent asset-lock authority
```

## Frozen authority model

v1 supports exactly `ORDINARY`, `CONSTITUTIONAL`, and `EMERGENCY` decision classes. The profile—not the caller—assigns each known action to a class and freezes required chambers, integer parts-per-million quorum/approval thresholds, recusal requirements, optional enforcer concurrence, and emergency authority/scope.

Unknown actions fail closed. Floating-point authority arithmetic is forbidden.

The v1 constitution keeps these actions prohibited regardless of vote totals:

```text
ASSET_LOCK_REMOVAL
STEWARD_SEAT_SALE
```

Emergency state cannot unlock them.

## Conflict / recusal rule

Where the profile requires recusal, supplied evidence must establish:

```text
disclosed = true
recused = true
counted_vote = false
```

Disclosure alone is insufficient, and a recused counted vote fails closed.

G1 validates supplied chamber totals and conflict records; independent reconstruction from individual eligibility/ballot evidence belongs to G1A.

## Output boundary

The deterministic `StewardshipDecisionReceipt` exposes action/class, chamber results, conflict results, enforcer/emergency results, blockers, and:

```text
authorization_state = AUTHORIZED | BLOCKED
asset_lock_removed = false
legal_validity_established = false
democratic_legitimacy_established = false
```

Governance authorization is not custody or execution authority.

## Semantic commitments

Constitution profile semantic SHA-256:

`945b558049d7570cd47b52d7d822b2dff142a46b3d5fa117ed24dadbcd1059cb`

Frozen receipt semantic/file SHA-256:

`5a330393381cdce9bd3d509a3aeab50fe66bb9c688cfc479c1bcb5efc0cbdccc`

Implementation/test/fixture byte identity is not duplicated as manually copied SHA literals. Hosted qualification instead requires the exact Git commit, exact sole parent, and exact six-file diff scope; the commit tree is the review-byte commitment. The generated receipt must still reproduce byte-for-byte from the checked-in fixture.

## Test surface

The stdlib suite contains **23** fail-closed regressions covering ordinary authorization, class substitution, protected actions, required recusal, counted-recused ballots, quorum/approval failure, required-chamber bypass, enforcer concurrence, emergency state/scope, unknown action, project/profile substitution, invalid authority arithmetic, duplicate conflict identity, authority-field injection, and determinism.

The test harness stages the canonical checked-in fixture explicitly before invocation; an absent or different fixture cannot be silently supplied from a developer workstation.

## Nonclaims

Even hosted PASS would establish only that one supplied decision satisfies this frozen authority/quorum/conflict/emergency theorem over the exact reviewed subject. It would not establish democratic legitimacy, legal or constitutional-law validity, identity/signature authenticity, social consensus, moral correctness, wisdom, custody, or execution authority.
