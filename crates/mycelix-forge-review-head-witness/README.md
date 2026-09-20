# Mycelix Forge Review-Head Witness

This crate turns one exact evidence-bound review-head snapshot into a **witness-qualified checkpoint** only when the project has explicitly authorized enough distinct Forge Witness principals to authenticate that exact completeness statement.

```text
EvidenceBoundReviewHeadSnapshotV1
+ ClaimsCompleteForProposal
+ exact completeness-witness statement
+ project-policy-trusted Xenia authentication under Capability::Witness
+ exact Witness eligibility
+ exact distinct Witness threshold at one common observation time
    ↓
WitnessQualifiedReviewHeadSnapshotV1
```

## Why witnesses are separate from the collaboration-state provider

A provider can report what it observed and can even claim proposal-wide coverage. That statement is not automatically authority.

For Holochain in particular, ordinary DHT/link observations are eventually consistent and cannot prove global absence. A source-chain proof can establish deterministic bounded lineage relative to an explicit chain top, but it does not create a global consensus snapshot across every reviewer.

Forge therefore keeps these claims separate:

```text
provider observation
!= provider completeness claim
!= project-authorized witness quorum
!= globally current state
!= merge authorization
```

## Witness statement

`ReviewHeadCompletenessWitnessStatementV1` binds:

- the exact `EvidenceBoundReviewHeadSnapshotV1` evidence commitment;
- the exact Witness `PrincipalId`;
- an immutable witness-context commitment.

The exact statement id must be the `PrincipalAuthenticationRequest.action_subject` and the authentication capability must be exactly `Capability::Witness`.

The Xenia authentication must then cross the generic FORGE-005C project-policy trust boundary before it can become a `WitnessedReviewHeadAttestationV1`.

## Quorum semantics

`evaluate_review_head_witness_quorum_v1` requires:

- the exact proposal and snapshot;
- snapshot coverage `ClaimsCompleteForProposal`;
- one exact `AuthorityEpoch`;
- its exact `Capability::Witness` threshold;
- distinct witness principals;
- every attestation observed no later than one common quorum observation time;
- every counted witness still eligible for `Witness` at that common time.

Duplicate witnesses are rejected, not silently deduplicated. Under-threshold input cannot construct the positive quorum type.

## Critical non-claims

`WitnessQualifiedReviewHeadSnapshotV1` does **not** establish:

- global Holochain consensus;
- that the checkpoint remains current after its witness time;
- trusted wall-clock time;
- absence of actions authored after the checkpoint;
- current approval quorum;
- source/build/history qualification;
- merge authorization.

A later finalization/currentness theorem must define a freshness window or stronger checkpoint-finalization protocol and join this checkpoint to exact authenticated review revisions before merge authority can rely on it.
