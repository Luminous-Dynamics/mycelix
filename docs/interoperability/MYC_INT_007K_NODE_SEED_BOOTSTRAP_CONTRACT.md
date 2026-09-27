# MYC-INT-007K — Node Seed Package and Independent Bootstrap Contract

Status: design/fixture only. Child of MYC-INT-007I.

## Purpose

Turn the node-maturation story into an exact bootstrap subject that can be replayed and challenged.

```text
seed package
!= receiving-node identity
!= local governance
!= local certification
!= federation membership
```

007K freezes two subjects: `NodeSeedPackageV1` (what A offers B) and `NodeBootstrapRunV1` (what B accepts, rejects, adapts, regenerates and proves). The same contract must later support B→C without special-case logic.

Fresh bootstrap requires new node, host and application-agent identities. No private identity is derived from the source node or package hash. `FreshNode` and `SameNodeRecovery` are separate modes; 007K covers FreshNode only.

Receiving-node component decisions are `AcceptAsIs`, `AcceptWithLocalAdaptation`, `Reject`, `Defer`, or `MoreEvidenceRequired`. Local adaptation creates a new local generation and declares loss where applicable.

The independence test is:

```text
bootstrap complete
+ seeder unavailable
→ receiving node retains its declared local capabilities
```

Remaining external dependencies stay explicit. Federation is a post-bootstrap local choice, and foreign certification, reputation, standing, balances, training, donations, or introductions do not become local authority automatically.

The eventual recursive test is stronger:

```text
A seeds B
A becomes unavailable
B operates independently
B emits B-owned seed package K2
B seeds C under the same contract
A is not required
```

Lineage/provenance may retain A-origin public artifacts; authority lineage does not.

## Nonclaims

007K does not establish physical/economic self-sufficiency, governance legitimacy, legal compliance, ecological sustainability, production safety, or successful real-world community replication.