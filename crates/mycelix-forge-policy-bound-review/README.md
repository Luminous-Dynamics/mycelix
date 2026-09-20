# Mycelix Forge Policy-Bound Trusted Review

This crate joins project-policy governance of review-policy adoption to the exact project-policy-trusted authenticated review revision that used that adopted context.

## Exact theorem

```text
ProposalReviewPolicyAuthorityQuorumV1
+ ProjectPolicyTrustedReviewRevisionV1
+ exact ProposalReviewPolicyContextV1
+ review_context == adopted context digest
+ structural policy-adoption time <= structural review observation time
    -> PolicyAdoptionBoundTrustedReviewRevisionV1
```

## Why this boundary exists

A project-authorized review policy does not prove a reviewer actually reviewed under that policy. `ReviewRevisionV1` already authenticates an immutable `review_context`, so the safe v1 bridge is to require that exact field to equal the canonical proposal+policy context accepted by `ManagePolicy` governance.

Therefore:

```text
policy A adopted
+ review authenticated under arbitrary context B
!= policy-A-bound review
```

and:

```text
policy A review
!= reusable as policy B review
```

Changing review-policy semantics changes the canonical policy context and requires a distinct authenticated review revision.

## Structural chronology

The policy-authority quorum observation must not be later than the review's structural observation time.

This is only a consistency rule over caller-supplied timestamps:

```text
adoption_observed <= review_observed
!= trusted wall-clock chronology
```

Trusted current time remains a separate theorem.

## Positive type

`PolicyAdoptionBoundTrustedReviewRevisionV1` retains the full `ProjectPolicyTrustedReviewRevisionV1` rather than flattening it to portable fields. Downstream code can therefore consume the exact provider/project-trust theorem rather than rebuilding a weaker approximation.

## Capability separation

The adoption theorem consumed here is governed by `Capability::ManagePolicy`. `ManageAuthority` remains reserved for authority-structure governance and is not accepted as a substitute for policy adoption.

## Non-claims

This crate does not establish:

- trusted wall-clock time;
- policy currentness at merge time;
- witnessed checkpoint membership/currentness;
- approval quorum;
- review-policy satisfaction;
- conflict-free review state;
- repository/build/history correctness;
- merge authorization.

The next checkpoint tranche should require these stronger policy-bound trusted revisions when joining review heads to an adopted review policy.
