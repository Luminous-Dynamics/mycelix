# Mycelix Forge Review-Policy-Satisfied Checkpoint

This crate is the first Forge layer that evaluates the semantics of an explicitly project-authorized review policy over an exact closed-world witnessed checkpoint.

## Exact theorem

```text
PolicyBoundWitnessCheckpointReviewSetV1
+ exact ReviewAcceptancePolicyV1
+ exact ProposalReviewPolicyAuthorityQuorumV1
+ exact AuthorityEpoch
+ policy RequestChanges semantics
+ policy proposer/self-review semantics
+ policy required-approver semantics
+ ReviewSource eligibility at checkpoint time
+ policy-filtered approval count >= exact ReviewSource threshold
    -> ReviewPolicySatisfiedCheckpointV1
```

## Threshold ownership

The numeric `ReviewSource` threshold remains exclusively in `AuthorityEpoch`.

The review policy only controls which exact checkpoint-head approvals may count. This matters for proposer self-review: FORGE-007H intentionally counts all eligible approvals, whereas this layer can exclude the proposer when `MayReviewButDoesNotCount` is the adopted rule.

## RequestChanges semantics

`BlocksAcceptance` makes any exact checkpoint-head `RequestChanges` a hard policy failure.

`NonCountingOnly` preserves the request-changes head but does not count it toward approval. A positive result may therefore have:

```text
has_request_changes() == true
```

That is not a conflict-free theorem. It means the exact adopted policy permits that state while its other requirements are satisfied.

## Proposer semantics

- `MayCountTowardThreshold`: proposer `Approve` may count normally.
- `MayReviewButDoesNotCount`: proposer may have review state, but proposer `Approve` is retained as a non-counting approval.
- `Prohibited`: any proposer checkpoint head fails policy evaluation.

## Required approvers

Every exact required approver must appear in the checkpoint and its exact head must be `Approve`. `RequestChanges`, `Withdraw`, or absence fail closed.

A required proposer can still be required while `MayReviewButDoesNotCount` prevents that approval from contributing to the numeric threshold; this permits an explicit “must sign, but cannot satisfy independence threshold” policy.

## Structural time boundary

The `ManagePolicy` review-policy adoption quorum must not occur after the witnessed checkpoint. Every checkpoint reviewer is also rechecked for `ReviewSource` eligibility at the checkpoint's common observation time.

All of those times remain caller supplied:

```text
review-policy satisfied at structural checkpoint
!= trusted current time
!= checkpoint still current now
```

## Capability separation

Review-policy adoption is governed by `ManagePolicy`; `ManageAuthority` remains reserved for authority-structure changes. This evaluator does not blur those capabilities.

## Non-claims

`ReviewPolicySatisfiedCheckpointV1` does not establish:

- conflict-free state when policy permits non-counting objections;
- trusted wall-clock time;
- checkpoint freshness/finality at merge time;
- repository/source/build/history correctness;
- `MergeProtected` authority;
- merge authorization.

The next major boundary should establish protected review finalization over this exact checkpoint before review satisfaction can be consumed by merge authorization.
