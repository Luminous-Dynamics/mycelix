# Checkpoint Approval Quorum — FORGE-007H

This crate evaluates the exact `ReviewSource` threshold over `Approve` states in one FORGE-007G witness-checkpoint review set.

## Theorem

```text
WitnessCheckpointReviewSetV1
+ exact AuthorityEpoch
+ authority epoch valid at checkpoint time
+ every checkpoint-head reviewer eligible for ReviewSource at checkpoint time
+ distinct checkpoint-head Approve count >= ReviewSource threshold
    -> CheckpointApprovalQuorumV1
```

The checkpoint review set is already canonical and contains at most one exact witnessed head per reviewer, so one reviewer cannot be double-counted.

## Decision semantics

The evaluator partitions exact checkpoint heads into:

- counted `Approve` heads;
- `RequestChanges` reviewers;
- `Withdraw` reviewers.

Only `Approve` counts toward threshold.

A positive quorum may still report `has_request_changes() == true`.

```text
approval threshold satisfied
!= conflict-free
!= merge-policy satisfied
!= merge authorized
```

This distinction is deliberate. Forge currently has no stronger typed review-acceptance policy on this line that says whether a checkpoint-head `RequestChanges` must block every merge, blocks only for selected principals, or is merely non-counting. That policy must be modeled explicitly rather than inferred by this threshold crate.

## Time boundary

Reviewer eligibility is re-evaluated at the FORGE-007F witness checkpoint's common observation time. That time is still caller supplied at this layer.

```text
structurally coherent checkpoint time
!= trusted current time
```

A finalization/freshness theorem remains necessary before using a historical checkpoint for merge authority.

## Next

Define a typed review-acceptance / merge-review policy and bind it to proposal-significant project policy before deriving a `ConflictFree` or `ReviewPolicySatisfied` positive type. Only then should merge authorization join review state to source/repository/build/history evidence.
