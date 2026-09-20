# Mycelix Forge Review Acceptance Policy

`mycelix-forge-review-acceptance-policy` defines the first typed review-policy contract above the witnessed-checkpoint approval theorem.

## Boundary

Forge protocol v1 already freezes `ProjectPolicyStateV1` and `ChangeProposal` canonical bytes. This crate does not reinterpret either object.

Instead it defines:

- `ReviewAcceptancePolicyV1`
- `RequestChangesRuleV1`
- `ProposerReviewRuleV1`
- canonical required approvers
- `ProposalReviewPolicyContextV1`

The intended forward path is:

```text
ChangeProposalId
+ ReviewAcceptancePolicyV1
    -> ProposalReviewPolicyContextV1
    -> ReviewRevisionV1.review_context
    -> exact authenticated ReviewRevisionId
```

A policy change therefore requires a distinct policy-context commitment and, once used in a review revision, a distinct authenticated revision identity.

## Policy dimensions

`RequestChangesRuleV1` distinguishes:

- `BlocksAcceptance`
- `NonCountingOnly`

`ProposerReviewRuleV1` distinguishes:

- `MayCountTowardThreshold`
- `MayReviewButDoesNotCount`
- `Prohibited`

Required approvers are canonical sorted unique Forge principals. The numeric `ReviewSource` approval threshold remains exclusively in `AuthorityEpoch`.

## Important non-claims

A `ReviewAcceptancePolicyV1` is portable policy data. Its existence does not establish that the project authorized or adopted it.

Likewise:

```text
ProposalReviewPolicyContextV1
!= review revision actually used this context
!= policy project-authorized
!= checkpoint review-policy satisfied
!= conflict-free review state
!= merge authorization
```

A later tranche should bind the exact context to project-policy-trusted authenticated review revisions. Project authorization/adoption of the policy remains a separate authority theorem.

## Migration discipline

Forge core currently supports protocol version 1 only. A future proposal-v2 migration may eventually move review-policy commitment directly into proposal identity, but that migration must be explicit and must not reinterpret existing v1 proposal bytes.
