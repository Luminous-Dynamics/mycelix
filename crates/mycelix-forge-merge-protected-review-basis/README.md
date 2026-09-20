# Mycelix Forge Merge-Protected Review Basis

FORGE-007N gives protected-merge authority an explicit way to select one exact `ReviewPolicySatisfiedCheckpointV1` as the review basis for a later source transition.

## Exact theorem

```text
ReviewPolicySatisfiedCheckpointV1
+ exact ReviewBasisSubjectV1
+ exact MergeProtectedReviewBasisStatementV1
+ project-policy-trusted Xenia authentication
+ Capability::MergeProtected
+ MergeProtected eligibility
+ distinct MergeProtected threshold
    -> MergeProtectedReviewBasisQuorumV1
```

## Why this is not called "current reviews"

A historical checkpoint can satisfy review policy without proving that no later review revision exists. A caller-supplied timestamp cannot close that gap.

This layer therefore models an explicit governance action instead of an inferred absence claim: the exact `MergeProtected` threshold signs one exact review-basis statement.

```text
protected authority selected checkpoint X
!= checkpoint X is globally latest
!= no review exists after X
```

Later review records do not mutate the identity of the selected review basis. A later merge/execution theorem must decide how the protected basis is consumed and must not silently substitute another checkpoint.

## Portable subject

`ReviewBasisSubjectV1` is a deserializable, non-authoritative projection of the positive FORGE-007M checkpoint. It binds:

- exact project and proposal;
- authority epoch;
- project policy;
- repository policy;
- adopted review policy and review-policy context;
- exact `ReviewPolicySatisfiedCheckpointV1` evidence commitment;
- structural checkpoint observation time.

Before authority is granted, the portable subject is rejoined to the live positive checkpoint.

## Exact action subject

Each protected-merge principal authenticates the exact `MergeProtectedReviewBasisStatementId`. The statement commits to the exact review-basis subject plus an immutable `finalization_context` commitment.

Changing the selected checkpoint, proposal, policy evidence, or finalization context changes the authentication subject.

## Quorum semantics

The evaluator requires:

- exact `MergeProtected` threshold from `AuthorityEpoch`;
- distinct principals, with duplicates rejected rather than deduplicated;
- authority epoch valid at one common supplied quorum time;
- every counted principal still eligible for `MergeProtected` at that common time;
- no attestation before the selected checkpoint;
- no attestation after the claimed quorum time.

## Critical non-claims

`MergeProtectedReviewBasisQuorumV1` does **not** establish:

- that the selected checkpoint is globally latest/current;
- trusted wall-clock time;
- absence of later review revisions or checkpoints;
- Git object/source correctness;
- protected-history/gittuf correctness;
- build/Nix/isolation qualification;
- repository tip currentness;
- actual merge authorization;
- successful or atomic merge execution.

Those remain later joins. In particular, merge authorization should bind this protected review basis to the exact source/execution evidence chain and a concrete repository transition/consumption theorem.
