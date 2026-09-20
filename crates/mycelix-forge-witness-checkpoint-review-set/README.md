# Witness Checkpoint Review Set — FORGE-007G

This crate joins one FORGE-007F witness-qualified review-head checkpoint to the exact FORGE-007D project-policy-trusted authenticated review revisions named by the underlying snapshot.

## Exact theorem

```text
WitnessQualifiedReviewHeadSnapshotV1
+ exact EvidenceBoundReviewHeadSnapshotV1
+ exact ProjectPolicyTrustedReviewRevisionV1 set
+ 1:1 reviewer identity
+ 1:1 ReviewRevisionId
+ 1:1 reviewer-local sequence
+ no missing or extra supplied reviewer
    -> WitnessCheckpointReviewSetV1
```

The positive set preserves the exact head decision for every reviewer:

```text
Approve | RequestChanges | Withdraw
```

Non-approval heads are evidence, not errors and not approvals.

## Fail-closed substitution boundary

The join rejects:

- another project or proposal;
- another authority, project-policy, or repository-policy context;
- another snapshot evidence commitment;
- another provider namespace or provider checkpoint;
- duplicate trusted revisions for one reviewer;
- snapshot heads with no matching trusted revision;
- trusted revisions not named by the snapshot;
- reviewer substitution;
- sequence substitution;
- exact revision-id substitution;
- a revision structurally observed after the witness checkpoint's common observation time.

The time comparison is structural only. Both times remain caller supplied at these layers; this crate establishes no trusted-clock theorem.

## Critical non-claims

`WitnessCheckpointReviewSetV1` means only that the exact authenticated trusted revisions supplied to this function are exactly the heads named by the exact witness-qualified checkpoint.

It does **not** establish:

- no successor revision exists after the checkpoint;
- checkpoint freshness at merge time;
- trusted wall-clock time;
- approval threshold satisfaction;
- whether `RequestChanges` blocks merge under a project policy;
- repository/source/build correctness;
- merge authorization.

The next tranche should derive a checkpoint-relative review decision/quorum theorem from this positive set. Final merge authorization must still require an explicit checkpoint freshness/finalization rule and the independent source/execution assurance chain.
