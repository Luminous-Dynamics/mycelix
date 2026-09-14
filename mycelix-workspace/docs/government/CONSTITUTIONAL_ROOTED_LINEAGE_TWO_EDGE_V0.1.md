# GOVSYS-003C-B — Two-Edge Constitutional Rooted-Lineage Conformance v0.1

## Purpose

This tranche extends the qualified one-edge Root-C composition theorem with a fresh, independently generated two-edge cryptographic corpus:

```text
external Root-B pin for G0
        ↓
G0 --K0--> G1 --K1--> G2
                         |
                         └─ immutable terminal root
```

K0 and K1 are distinct Ed25519 authorities. The private fixture keys were used only to create the two frozen signatures and were discarded before the public fixture was committed. The repository contains only canonical public SPKI material, signatures, semantic roots, nonces, and expected commitments.

## Qualified parent

This child must be exactly one commit above qualified Root-C-A head:

`b30a3cf30df7fc925834240b89ae0edcc0e42f74`

Parent hosted run `34900250671` is PASS.

No inherited Root-A, Root-B, transition-verifier, one-edge Root-C, or CORE-LINEAGE theorem blob may change in this tranche.

## What the two-edge corpus proves

The fixture requires all of the following:

1. G0 is independently Root-B pinned as generation zero.
2. G0's exact rotation-authority commitment authenticates K0.
3. K0 verifies only the exact G0→G1 transition transcript.
4. G1 carries a new rotation-authority commitment authenticating K1.
5. K1 verifies only the exact G1→G2 transition transcript.
6. Reusing K0 as authorization material for G1→G2 fails closed.
7. Mutating either signed transition candidate fails cryptographic verification.
8. Both transitions preserve the derived constitutional lineage-domain identity.
9. Replay-nonce uniqueness is enforced across the complete supplied evidence set.
10. Transition arrival order does not change the stable structural history.
11. Exact semantic duplicates normalize without changing the stable history.
12. Parallel transitions and forks fail closed in CORE-LINEAGE.
13. A missing first edge cannot make G1→G2 reachable from G0.
14. G2 is an immutable root, so ordinary predecessor-authorized rotation stops there.

The two-edge CORE-LINEAGE commitment is:

`3802a7e337444b8252dde31d619c14d2c443f3ab35dea825801cb6eb265169e4`

The stable constitutional-history identity is:

`2bceed3c055a2a48443d5db5d4e7163580f7aa6ff8cb8c69100bf61ebe8c67c5`

The separate Root-B-bound qualification identity is:

`7833d1860d5d54c9c22fea00022c61512eb3fac9a04f4f0b1223946f6cf2dc22`

## Stable history versus authority provenance

The structural history commits to Root-A identities, source-descriptor identities, transition semantic identities, generations, effective times, and the derived lineage domain. It does not embed private verifier material or dynamic proof leases.

Cryptographic authorization is re-executed before transition facts are projected into CORE-LINEAGE.

```text
valid signature
!= historical structure

historical structure
!= complete global history

complete global history
!= current constitutional root
```

## Immutable terminal semantics

G2 deliberately carries:

```text
rotation_mode = immutable
rotation_profile = absent
rotation_authority_anchor = absent
```

This proves only that the observed ordinary-rotation chain terminates at G2. It does **not** prove that G2 is current, globally unique, judicially valid, legally supreme, or impossible to supersede through a separately specified migration/recovery procedure.

Root-D owns closed-world source coverage and current-head promotion.

## Observed-set boundary

Root-C can reject conflicts present in the verified evidence supplied to it. Root-C cannot prove that the caller supplied every authorized transition that exists.

Therefore:

```text
fork-free supplied evidence
!= globally fork-free constitutional history
```

A caller cannot acquire a global uniqueness claim merely by withholding a competing branch. Closed-world completeness belongs to the source-coverage theorem, not this historical projection theorem.

## Authority nonclaims

This tranche maintains:

```text
grants_currentness = false
grants_effect_authority = false
```

It grants no ordinary policy authority, administrative authority, judicial authority, execution authority, legal finality, external-effect authority, or network sovereignty.

The network remains infrastructure for institutions. It is not the sovereign.
