# MYC-CAP-002H1 — Current stewardship-governance dimension composition v1

Status: executable candidate; local preflight only until hosted exact-head qualification executes.

## Purpose

Compose exactly one qualified currentness receipt (G2A) with exactly one qualified active-constitution receipt (G2C) into one bounded `stewardship_governance_state`.

This theorem does **not** implement the full Capital-to-Commons readiness vector.

Core rule:

```text
G2A CURRENT/AUTHORIZED
+ G2C ACTIVE checkpoint
+ exact project/registry/epoch/profile equality
-> stewardship_governance_state = CURRENT
```

## Hosted prerequisites

This candidate is created only after hosted PASS for:

- G1 subject `9c23142b7711e2981595ee6969f6a7fa4382b910`, run `35125040864`;
- G2A subject `95dea3490a514f8e34514e626884a8978bcc4581`, run `35125260516`;
- G2C subject `42f6c75f3947bd409cc012edb9ea2c92b63ad9ab`, run `35125477347`.

G1A and G2B are independently qualified siblings but are not sufficient H1 authority inputs.

## Permanent negative control

The checked-in qualified examples deliberately do **not** compose to CURRENT:

```text
G2A example: epoch 7 / profile A / CURRENT
G2C example: epoch 8 / profile B / ACTIVE
```

H1 must derive `STALE` with:

```text
GOVERNANCE_EPOCH_MISMATCH
GOVERNANCE_PROFILE_MISMATCH
```

Historical currentness relative to epoch 7 cannot outrank the epoch-8 registry checkpoint.

## Positive fixture

The H1 positive fixture contains a separately derived post-cutover G2A-shaped receipt at epoch 8/profile B. It is only composition test evidence; it is not a claim about a live deployment decision.

## States

```text
CURRENT
BLOCKED
PENDING
STALE
NOT_ASSESSED
UNSUPPORTED
```

No scalar `governance_ok` or `transition_complete` field exists.

## Authority non-amplification

Every receipt fixes:

```text
execution_authority_established = false
legal_validity_established = false
democratic_legitimacy_established = false
```

H1 cannot strengthen either input.

## Frozen example commitments

Canonical positive fixture SHA-256:

`b49450cb22d6e059af26d64de4d5c012cedfc5f72ad587d5647a886aa2c48792`

Frozen positive receipt SHA-256:

`c6b536faabef5002048b1372847d4c45a0169928cc577d63454ebfb4de4130bd`

The permanent negative control additionally consumes the exact ancestor G2A/G2C example receipts from their qualified subjects.

## Local preflight

The stdlib regression suite passes **22/22** locally, including:

- post-cutover positive CURRENT;
- qualified epoch-7 G2A + epoch-8 G2C -> STALE;
- epoch mismatch and profile mismatch;
- project/registry substitution;
- G2C stale/non-active/malformed checkpoint;
- G2A historical/revoked/pending/not-authorized states;
- unsupported action;
- raw G1/G2B shape substitution;
- authority contamination;
- unknown-key injection;
- deterministic blocker ordering and replay.

Local PASS is not hosted qualification.

## Nonclaims

Even hosted PASS would establish only that one supplied governance decision is current relative to one supplied exact active constitution checkpoint under this frozen composition profile. It would not establish execution authority, legal validity, democratic legitimacy, identity/signature authenticity, service quality, solvency, handback readiness, operational sovereignty, or wisdom.
