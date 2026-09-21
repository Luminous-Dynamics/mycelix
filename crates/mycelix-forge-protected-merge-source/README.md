# FORGE-009B — protected merge source qualification

FORGE-009B rejoins the **source side** of one exact `ProtectedMergeRequestV1` to live positive repository and Git-source evidence without importing the obsolete collaboration ancestry of the earlier FORGE-008 proposal-source branch.

## Exact theorem

```text
ProtectedMergeRequestV1
+ exact ChangeProposal
+ exact RepositoryAdoption
+ exact RepositoryPolicyState
+ exact RepositoryVerificationRequest
+ QualifiedRepositoryVerification
+ ProposalSourceObservationV1
+ concrete ProposalSourceVerifierV1
        ↓
SourceQualifiedProtectedMergeRequestV1
```

The qualifier rechecks the protected merge request against the immutable proposal and reconstructs the exact repository-verification request from the proposal/adoption/policy context.

It then closes four explicit references carried by FORGE-009A:

```text
repository_request
repository_verification
proposal_source
source_state
```

The repository-evidence, source-observation and qualified-source canonical domains intentionally preserve the earlier FORGE-008A byte semantics so the concrete pinned-Git verifier can be ported onto the current authority line without changing the meaning of those evidence commitments.

## Anti-substitution boundary

The positive join rejects:

- another proposal/project/authority/project-policy/repository-policy context;
- another target ref/base/proposed commit/resulting tree;
- a repository request not exactly reconstructed from the proposal;
- positive repository evidence for another request/tip/policy state;
- a source observation from another verifier;
- another proposal/request/base/proposed/tree in the source observation;
- a different repository-verification reference;
- a different proposal-source reference;
- a different source-state reference.

## Deliberately unresolved M0 references

FORGE-009B does **not** rejoin:

```text
offline_evidence
execution_subject
```

Those are preserved as:

```text
pending_offline_evidence
pending_execution_subject
```

inside the positive source-qualified result.

Therefore:

```text
SourceQualifiedProtectedMergeRequestV1
!= M0 OfflineEvidence qualified
!= hermetic execution qualified
!= repository-tip current
!= merge authorized
!= merge executed
```

The next theorem must consume actual positive frozen-M0-v6 evidence and require exact equality with both pending references. It must not accept serialized M0-looking digests as authority.

## Current M0 state

The frozen M0 v6 product head remains unchanged. Its original admission workflow stopped at repository-wide Rust 1.96 formatting drift before lint/tests. A separate never-merge qualification harness exists to test the exact frozen source bytes without rewriting them; its result is independent of FORGE-009B.

## Next boundary

After the M0 rejoin, Forge still needs a repository-tip lease / atomic compare-and-swap theorem. A source-qualified and M0-qualified request is evidence that an exact transition *may be considered*; it is not proof that the target ref still equals the expected base or that the request was consumed exactly once.
