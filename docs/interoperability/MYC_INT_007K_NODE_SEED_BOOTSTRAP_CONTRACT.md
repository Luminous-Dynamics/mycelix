# MYC-INT-007K — Node Seed Package and Independent Bootstrap Contract

Status: design/fixture only. Child of MYC-INT-007I.

The contract freezes two subjects: a `NodeSeedPackageV1` produced by a seeder and a `NodeBootstrapRunV1` owned by the receiving node.

```text
seed package
!= receiving-node identity
!= local governance
!= local certification
!= federation membership
```

Fresh bootstrap requires new node, host, and application-agent identities; no private identity is copied or derived from the seeder or package hash. `FreshNode` and `SameNodeRecovery` remain separate modes; 007K covers FreshNode only.

Receiving-node component decisions are `AcceptAsIs`, `AcceptWithLocalAdaptation`, `Reject`, `Defer`, or `MoreEvidenceRequired`. Adaptation creates a new local generation and declares loss where applicable.

The independence condition is: after a complete package transfer and bootstrap, the seeder can become unavailable while the receiver retains its declared local capabilities and keeps remaining external dependencies explicit. Federation is a later local choice.

The recursion condition is stronger: A seeds B; A becomes unavailable; B operates independently; B emits a B-owned seed package; B seeds C using the same contract; A is not required. Public provenance may retain A-origin lineage, but authority does not.

007K does not establish economic self-sufficiency, social legitimacy, legal compliance, ecological sustainability, production safety, or successful physical replication.