# MYC-CAP-002D3 — Custody continuity and successor state v1

Status: exact replacement candidate over hosted-qualified D2; hosted qualification required for this exact D3 head.

## Purpose

Preserve historical custody acceptance while deriving current operational-custody state from a separate append-only event lineage.

```text
historical CUSTODY_ACCEPTED != custody CURRENT forever
current regression != retroactive falsification of historical acceptance
```

## Exact genesis binding

The custody profile binds exact hosted-qualified D2 acceptance subject:

`d5a3258fb1713cf0fe95ab7434600d6240cefad7`

and the canonical D2 acceptance receipt semantic SHA-256:

`7a073b1b012c60ac645c9cc98c00d78392a2adadbf2474b0cd39df7a7c03b832`

Qualification additionally proves that the acceptance receipt embedded in the D3 fixture is byte-for-object identical to:

`docs/capital/evidence/myc-cap-002d2/example_receipt.json`

in the exact D2 parent tree. Git parent identity and semantic genesis therefore have to agree.

The D2 receipt in turn binds the qualified D1 readiness evidence through its frozen readiness receipt/profile/currentness fields; D3 does not invent a second readiness-subject commitment plane.

## State machine

Supported events are exactly:

```text
GenesisCurrent
MaterialRegression
VerifiedCure
SuspendCustody
ResumeCustody
SuccessorAcceptance
TerminateCustody
```

State derivation:

```text
SUPERSEDED / TERMINATED -> terminal
else suspended          -> SUSPENDED
else active blockers    -> DEGRADED
else                    -> CURRENT
```

Material regressions add only frozen blocker codes. Verified cures remove only exact active blockers. Suspension preserves blockers. Resume is rejected until all blockers are cured. SUPERSEDED and TERMINATED are terminal.

## Canonical history

```text
GenesisCurrent
-> MaterialRegression(BACKUP_RESTORE_FAILED)
-> VerifiedCure(BACKUP_RESTORE_FAILED)
-> CURRENT
```

The incident remains committed in the event history after recovery.

## Recomputed commitments

Custody profile semantic SHA-256:

`99478af7e37e6d878c42051bb88657feaf083fe9d55afa6726aa8fa78e8fca3a`

Event-history semantic SHA-256:

`5ec741fe98ae16872cd1a6a954b22fa7ea679ff48075cca96a622ed8ee864807`

Event-chain tip SHA-256:

`82b13fad92801eee7038b237b0a5e47aebb198ef4ec0e63d3f4ed139dfe28744`

Canonical fixture file SHA-256:

`9ffcc6a9b22916beafecb1187ded59412dddfe46229bc6e8168acc55bf1ab8ea`

Frozen receipt file SHA-256:

`f778ce32c405c5653c144ae5d3b1796d7b8f3e9d6711502c261d8599981f92c7`

Custody receipt semantic SHA-256:

`26f9f23e24d6a120c3cf210e920e6111fdefbd1355fb0069052bdfb5a68f01e1`

Canonical D2 acceptance receipt semantic SHA-256:

`7a073b1b012c60ac645c9cc98c00d78392a2adadbf2474b0cd39df7a7c03b832`

## Qualification contract

The exact-head workflow requires:

- literal PR head checkout with sufficient ancestry;
- exact sole parent `d5a3258fb1713cf0fe95ab7434600d6240cefad7`;
- exact six-file D3 scope;
- clean checkout;
- Python compile;
- unchanged fail-closed custody suite;
- deterministic receipt reconstruction;
- frozen fixture/receipt file commitments;
- embedded D2 acceptance receipt equality to the exact parent-tree receipt;
- semantic D2 receipt digest equality to the profile commitment;
- exact acceptance subject equality to the Git parent;
- strong legal/constitutional nonclaims.

## Authority boundary

Every receipt preserves:

```text
historical_handover_accepted = true
legal_title_transition_established = false
constitutional_stewardship_transition_established = false
```

Current custody health remains operational evidence at the supplied event-chain tip only. `CURRENT` does not prove that the supplied tip is the latest designated registry tip.

## Nonclaims

PASS would establish only current custody-state derivation under this frozen profile and exact hosted-qualified D2 acceptance lineage. It would not establish legal title, constitutional stewardship legitimacy, democratic legitimacy, physical engineering fitness, service quality, cybersecurity certification, external identity/signature authenticity, or future custody health.
