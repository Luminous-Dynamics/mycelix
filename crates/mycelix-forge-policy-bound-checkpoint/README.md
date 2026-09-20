# Mycelix Forge Policy-Bound Witness Checkpoint

This crate composes two already-strong Forge theorems:

1. `WitnessCheckpointReviewSetV1` — the exact authenticated review revisions named by one project-witnessed checkpoint;
2. `PolicyAdoptionBoundTrustedReviewRevisionV1` — an exact trusted review revision whose authenticated `review_context` matches the project-authorized review policy for the proposal.

## Exact theorem

```text
WitnessCheckpointReviewSetV1
+ ProposalReviewPolicyAuthorityQuorumV1
+ exact PolicyAdoptionBoundTrustedReviewRevisionV1 set
+ 1:1 reviewer / revision / sequence / decision equality
+ 1:1 trusted-review evidence equality
+ 1:1 review-observation-time equality
+ no missing or extra policy-bound reviews
    -> PolicyBoundWitnessCheckpointReviewSetV1
```

## Closed-world join

Every checkpoint head must have exactly one matching policy-bound review. Extra supplied reviews are rejected rather than ignored.

The join additionally requires each policy-bound review to carry:

- the exact review-policy commitment accepted by project authority;
- the exact proposal-review policy-context commitment;
- the exact policy-authority quorum evidence;
- the exact underlying project-policy-trusted review evidence already frozen into the checkpoint head;
- the same trusted Xenia verifier identity;
- the same structural review-observation time.

This prevents a policy-bound review for one authenticated revision from being borrowed to strengthen a different checkpoint head.

## Non-claims

`PolicyBoundWitnessCheckpointReviewSetV1` does not establish:

- whether the policy's semantic acceptance rules are satisfied;
- whether `RequestChanges` blocks under the exact policy;
- whether proposer self-review may count under the exact policy;
- whether required approvers all approve;
- trusted current time or checkpoint freshness;
- repository/source/build/history correctness;
- merge authorization.

The next theorem may evaluate `ReviewAcceptancePolicyV1` over this exact closed-world checkpoint set without relying on an arbitrary review subset.
