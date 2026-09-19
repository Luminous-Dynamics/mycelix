# Constitutional Event Admission Repair Bridge v0.1

Status: **PendingRepairQualification / non-activating**  
Tranche: **MYC-CONST-003D1D-E1AR1**  
Tracks: **#1896**  
Parent theorem: **MYC-CONST-003D1D-E1A** semantic `249753017c3781f7a3259061aae16d7e0075e984`

## Purpose

E1A is a qualified source-bound theorem over exact historical D1C and E0 subjects. Those historical Rust subjects subsequently failed their own hosted qualification at the source-format gate and were repaired as new Git commits. Because E1B correctly treats semantic-head identity as part of admission evidence, a green receipt for a repaired subject cannot be silently substituted for the old SHA.

E1AR1 makes that migration explicit without changing runtime code or rewriting E1A history.

```text
qualified E1A theorem
        |
        | binds old exact objects
        v
old D1C 47d1d764... ---- pinned-rustfmt repair ----> D1C-R1 90ff00c3...
old E0  36a4fcff... ---- pinned-rustfmt repair ----> E0-R1  ac3f71e3...
        |                                           |
        +---------------- E1AR1 -------------------+
                            |
                            v
                future E1B-R1 exact subject census
```

The bridge does **not** assert that the old and new commit IDs are interchangeable. It states the exact proof obligation that can justify carrying the already-qualified E1A relationships onto repaired subjects.

## Qualified upstream theorem

E1A exact verifier `ebe85798557a04c75d0bb9ce6c02349576e12b4f` completed successfully in hosted run `35360097507`, job `105648954365`.

Retained evidence:

- artifact ID `10583687692`;
- artifact digest `sha256:4edc41c9172ee221332fe1501cb05291b4f6f9eadaf9d4af84d39ab71810cd2f`;
- exact cross-lineage materialization: PASS;
- independent validator + mutation controls: PASS;
- semantic-source immutability: PASS.

This receipt establishes E1A only over the exact old D1C/E0 source objects named by the frozen E1A profile.

## D1C repair edge

Historical D1C:

- semantic head `47d1d764323dbfaf991b5574cfde83abb7a3e4a4`;
- ledger blob `bb3b8d6a865bbf514ff1e43a41c520b890a5515d`.

Prepared D1C-R1:

- semantic head `90ff00c371d2dd875b9bb7f23f1c5ee4b293f39c`;
- ledger blob `6f5d6c2a3e8f27cace3b37c23c82b8b4c64c1f0e`;
- exact verifier `a03273ecdc095cd60c5ebfa9a10e9e88bc64dd40`;
- changed-file census: only `constitutional-effect-ledger/src/lib.rs`.

The R1 qualifier must independently prove:

```text
pinned_rustfmt(old exact ledger bytes) == D1C-R1 ledger bytes
```

before this repair edge can be promoted by any successor profile.

## E0 repair edge

Historical E0:

- semantic head `36a4fcffb6ca806570ebf439f9c36cf76401e7b2`;
- event-provider blob `5d110f95c0d78f494b61a16d5afe5776401820bb`;
- D1C identity-test blob `897668782e09fac5ffbc506e1e8b11f653de2950`.

Prepared E0-R1:

- semantic head `ac3f71e37c480a9c6f99578fe2106285fe567a5b`;
- event-provider blob `8c57b38e1c304fdfb042485c666bac5dc8d40960`;
- D1C identity-test blob `77dff6764b329c6ba3ce263dbfd34956efb8eac0`;
- exact verifier `42cbfaa5cfb043e124db84f7762e1ca7142a6f08`;
- changed-file census: exactly those two files.

The R1 qualifier must independently prove:

```text
pinned_rustfmt(old exact provider bytes) == E0-R1 provider bytes
pinned_rustfmt(old exact mapping-test bytes) == E0-R1 mapping-test bytes
```

before this repair edge can be promoted by any successor profile.

## Carried-forward E1A invariants

E1AR1 requires all E1A security relationships to remain visible on the repaired source subjects:

- D1C operation ID, proposal ID, action ID, action commitment, and ClaimBinding reference are reused by E0 rather than regenerated;
- ClaimBinding target authorization remains bound to the exact event target descriptor;
- ClaimBinding payload authorization remains bound to the exact canonical payload commitment;
- authenticated DHT author identity must equal event publisher identity;
- constitutional event truth is immutable;
- the action-key index is integrity-relevant and cannot become last-write-wins state;
- same action + same semantics is `ExistingSame`;
- same action + conflicting semantics is `IntegrityConflict`;
- signal projection remains outside constitutional completion.

Formatting repair is not authority to weaken any of those invariants.

## Why the bridge remains pending

At this freeze point, D1C-R1 and E0-R1 have prepared exact verifiers but do not yet have inspected hosted PASS receipts. The repository Actions queue is saturated, so creating more execution-only trigger PRs would add load without improving evidence.

Therefore this profile is frozen as:

```text
status = PendingRepairQualification
activation_allowed = false
```

A later hosted PASS does not retroactively mutate this profile. A successor exact-head repair-bridge profile must bind the inspected D1C-R1 and E0-R1 receipts.

## E1B consequence

E1B intentionally hard-codes the exact old dependency heads and explicitly rejects subject mismatch. That behavior must remain unchanged.

After both repair edges are qualified, a future **E1B-R1** revision should name:

```text
MYC-CONST-003D1C     -> 90ff00c371d2dd875b9bb7f23f1c5ee4b293f39c
MYC-CONST-003D1D-E0  -> ac3f71e37c480a9c6f99578fe2106285fe567a5b
```

and cite a qualified successor of this bridge. It must not accept old-subject receipts as though they were R1 receipts, nor accept R1 receipts under the old semantic-head identifiers.

## Independent validator

`tools/formal/validate_constitutional_event_admission_repair.py` checks:

1. exact inspected E1A receipt metadata;
2. frozen E1A profile still binds the historical D1C/E0 heads and blobs;
3. each R1 semantic commit is a direct child of its historical subject;
4. exact repair changed-file census;
5. exact old/new Git blob identities;
6. no Holochain zome is changed by either repair;
7. repaired D1C/E0 source still exposes the source-visible identity continuity required by E1A;
8. migration remains blocked and non-inheriting;
9. non-claim ceiling remains intact.

Mutation self-tests reject premature activation, repair-head drift, changed-file omission, fake qualification promotion, upstream receipt drift, SHA-substitution weakening, and removal of the SHA non-claim.

## Non-claims

E1AR1 does not establish:

- D1C-R1 qualification;
- E0-R1 qualification;
- Git SHA interchangeability;
- retroactive modification of E1A evidence;
- an E1B repaired dependency census;
- a Holochain event entry/link implementation;
- active constitutional-event admission;
- live `GovernanceAction::EmitEvent` routing;
- deployment currentness.
