# MYC-INT-007K — Node Seed Package and Independent Bootstrap Contract

Status: design/fixture only. Child of MYC-INT-007I.

The contract freezes `NodeSeedPackageV1` and `NodeBootstrapRunV1`.

```text
seed package != receiving-node identity != local governance != local certification != federation membership
```

Fresh bootstrap requires new node, host, and application-agent identities; no private identity is copied or derived from the seeder or package hash. `FreshNode` and `SameNodeRecovery` are separate modes.

Receiving-node component decisions are `AcceptAsIs`, `AcceptWithLocalAdaptation`, `Reject`, `Defer`, or `MoreEvidenceRequired`. Adaptation creates a new local generation and declares loss where applicable.

Independence means a completed receiver retains its declared local capabilities while the seeder is unavailable, with remaining external dependencies explicit. Federation is a later local choice.

Recursive seeding requires B to emit a B-owned seed package and seed C under the same contract without A being required. Public provenance may retain A-origin lineage; authority does not.

007K does not establish economic self-sufficiency, social legitimacy, legal compliance, ecological sustainability, production safety, or successful physical replication.