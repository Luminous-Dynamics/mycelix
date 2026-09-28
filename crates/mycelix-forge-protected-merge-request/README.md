# Mycelix Forge Protected Merge Request

FORGE-009A defines the exact portable subject that a later merge-authorization theorem and repository executor must consume.

It deliberately creates **no merge authority**.

## Exact subject

```text
ChangeProposal
+ MergeProtectedReviewBasisQuorumV1
+ explicit source/execution evidence references
+ exact target ref / expected base / proposed commit / resulting tree
+ per-attempt merge nonce
    -> ProtectedMergeRequestV1
```

Construction from live inputs rechecks that the protected review basis and proposal have the same exact:

- project;
- `ChangeProposalId`;
- authority epoch;
- project-policy state;
- repository-policy state.

The transition fields are copied directly from the immutable proposal.

## Source/execution references are not authority

`SourceExecutionEvidenceReferencesV1` names the evidence the next qualifier must prove:

- proposal-source profile (`PinnedGitV1`);
- offline execution profile (`M0OfflineEvidenceV6`);
- exact repository request digest;
- exact positive repository-verification evidence commitment;
- exact proposal-source evidence commitment;
- exact source/object-store state commitment;
- exact OfflineEvidence commitment;
- exact hermetic execution subject.

These values are intentionally serializable/deserializable. An attacker can write arbitrary digests into them.

```text
source evidence references
!= source evidence verified
!= OfflineEvidence qualified
!= merge authorized
```

A later theorem must recompute every reference from positive source/execution objects.

## Exact transition / CAS preparation

The request repeats the exact protected transition material already committed by `ChangeProposal`:

- target ref;
- expected base revision;
- proposed revision;
- resulting tree.

That is deliberate. A concrete repository executor should later be able to require an atomic compare-and-swap style transition:

```text
observed target ref == expected_base
then atomically update target ref -> proposed_revision
```

This crate does not perform or prove that operation.

## Per-attempt nonce

A fixed 32-byte `merge_nonce` is part of request identity. Reusing all proposal/evidence material with another nonce creates another `ProtectedMergeRequestId`.

The nonce alone does not prove freshness or one-time consumption. A later repository/provider lease or consumption theorem must establish that.

## Frozen M0 boundary

The `M0OfflineEvidenceV6` profile is only an evidence-contract identifier here. FORGE-009A does not modify, reformat, requalify, or claim PASS for the frozen M0 subject.

At the time this tranche was authored, the frozen M0 v6 GitHub qualification was blocked at repository-wide Rust 1.96 `cargo fmt --check`; lint and tests did not execute. Therefore its evidence remains unqualified and cannot yet satisfy a future positive merge-authorization join.

## Non-claims

`ProtectedMergeRequestV1` does not establish:

- source/Git correctness;
- repository/gittuf correctness;
- OfflineEvidence qualification;
- review currentness beyond the selected protected review basis;
- trusted time;
- merge authorization;
- repository-tip currentness;
- one-time request consumption;
- atomic ref update;
- successful merge execution.
