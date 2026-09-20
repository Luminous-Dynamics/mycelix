# Mycelix Forge Review Policy Adoption

This crate lets the exact project `ManagePolicy` threshold accept one exact review-acceptance policy for one exact change proposal without modifying frozen Forge v1 proposal bytes.

## Exact chain

```text
ReviewAcceptancePolicyV1
+ ProposalReviewPolicyContextV1
+ ProposalReviewPolicyAdoptionStatementV1
+ project-policy-trusted Xenia authentication
+ Capability::ManagePolicy
+ ManagePolicy eligibility
+ distinct ManagePolicy threshold
    -> ProposalReviewPolicyAuthorityQuorumV1
```

Every policy-governance principal authenticates the same exact adoption statement. The authentication action subject is the exact statement id.

## Why proposal-specific adoption

Forge core currently supports protocol v1 only. Rather than reinterpret `ProjectPolicyStateV1` or `ChangeProposal`, this layer authorizes the review policy specifically for an exact existing `ChangeProposalId`.

The review policy context itself also binds that same proposal. A later review-binding tranche can therefore require authenticated review revisions to carry the exact adopted context in `ReviewRevisionV1.review_context`.

## Policy-governance semantics

The adoption quorum requires:

- project-policy-trusted Xenia provider evidence;
- exact proposal authority epoch;
- `Capability::ManagePolicy`;
- distinct policy-governance principals;
- exact `ManagePolicy` threshold;
- one common supplied quorum observation time;
- eligibility at that common time;
- no attestation observed after the claimed common quorum time.

Duplicate principals are rejected rather than silently deduplicated.

`ManageAuthority` is intentionally not used here: it remains reserved for governance over authority structure itself.

## Non-claims

`ProposalReviewPolicyAuthorityQuorumV1` does not establish:

- trusted wall-clock time;
- that the adopted policy is still current at merge time;
- that any review revision actually committed the adopted context;
- checkpoint review-policy satisfaction;
- conflict-free review state;
- repository/build/history correctness;
- merge authorization.

The next tranche should bind project-policy-trusted review revisions to this exact adopted review-policy context before policy semantics are evaluated over a witnessed checkpoint.
