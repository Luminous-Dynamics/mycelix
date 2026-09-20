# Mycelix Forge Checkpoint Review Set

This crate proves that every reviewer head named by one witness-qualified review-head checkpoint corresponds one-to-one with an exact project-policy-trusted authenticated review revision.

```text
WitnessQualifiedReviewHeadSnapshotV1
+ exact EvidenceBoundReviewHeadSnapshotV1
+ exact ProjectPolicyTrustedReviewRevisionV1 set
+ reviewer/revision-id/sequence bijection
    ↓
WitnessQualifiedCheckpointReviewSetV1
```

## Why this layer exists

A witnessed snapshot still contains opaque review-head identifiers. Without this join, a caller could present a valid witness checkpoint and then substitute different review-revision objects when deriving decisions.

The projection therefore requires an exact bijection:

- same number of snapshot heads and trusted revisions;
- one trusted revision per reviewer;
- no duplicate trusted reviewer;
- exact proposal identity;
- exact project-policy state;
- exact reviewer-local sequence;
- exact `ReviewRevisionId`;
- trusted revision observation no later than the witness checkpoint time.

The resulting entries retain the exact authenticated:

- reviewer;
- revision id;
- sequence;
- `Approve | RequestChanges | Withdraw` decision;
- immutable review context;
- original structural-eligibility observation time;
- project-policy-trusted revision evidence commitment.

## Critical non-claim

The positive type is deliberately named:

`WitnessQualifiedCheckpointReviewSetV1`

not `CurrentReviewSet`.

```text
state proven at witnessed checkpoint C
!= proof that no later review revision exists after C
```

A later approval/currentness theorem may evaluate the checkpoint-relative state under a finalization policy, re-check reviewer eligibility at the finalization time, and decide whether the checkpoint is fresh enough to use. Merge authority must still join source/repository/build/history evidence separately.
