# FORGE-004A Evidence-Binding Hardening

This note records two additional positive-type boundaries added before FORGE-004A qualification.

## 1. Repository-policy genesis binding

A structurally valid `RepositoryPolicyState { sequence: 0, ... }` is not automatically the policy lineage adopted by a project.

`BoundRepositoryPolicyGenesis` succeeds only when:

- adoption and genesis name the same `ProjectIdentity`;
- the supplied policy state is sequence zero;
- the policy digest in the genesis state exactly equals `RepositoryAdoption.repository_policy`.

Therefore:

```text
sequence-zero repository policy state
!= adopted repository policy lineage root
```

Consumers that need the latter must require the positive binding type or independently reproduce its checks.

## 2. Monotonic-policy evidence commitment

`PolicyLineageMonotonic` is no longer consumable as a bare adapter capability assertion.

The public qualification path uses `EvidenceBackedAdapterObservation`. If the wrapped observation declares `PolicyLineageMonotonic`, the wrapper requires a non-empty `policy_lineage_commitment`.

This mirrors the existing structural rules:

- `FullHistory` requires `history_commitment`;
- `OfflineEvidence` requires `evidence_commitment`;
- `PolicyLineageMonotonic` requires `policy_lineage_commitment`.

Custom deserialization re-runs the invariant, and the positive qualification function checks it again defensively.

The concrete FORGE-004B gittuf adapter must define how its policy/RSL history is deterministically committed and must emit that commitment rather than merely setting a boolean capability.

## Canonical-vector impact

None. The existing adoption, repository-policy-state, and verification-request canonical formats are unchanged, so their frozen FORGE-004A vectors remain stable.
