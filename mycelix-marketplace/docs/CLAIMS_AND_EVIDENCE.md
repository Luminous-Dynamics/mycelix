# Marketplace claims and evidence ledger

This ledger distinguishes implemented mechanisms from demonstrated system properties.

| Claim | Current permitted wording | Evidence | Important limitation |
|---|---|---|---|
| Decentralized marketplace | Holochain-based peer-to-peer marketplace prototype | DNA/zome source and manifests | Deployment, availability, and global convergence are not proven |
| Transaction integrity | Buyer/seller authorship and guarded lifecycle transitions are integrity-validated | Integrity tests and lifecycle contract | Rust tests must pass in a toolchain-enabled checkout |
| Conflict handling | Narrow automatic safety projections remain authored-head-only; unsafe conflicts may be projected only by matching buyer/seller approvals or a conflict-bound arbitration result | `transaction-conflict-policy-v2`, `transaction-conflict-resolution-v1`, reducer tests, wire fixtures, and network evidence contract | This is explicit authority over one recorded branch, not semantic merge, rollback, legal finality, or general consensus |
| Arbitration | Equal-vote, conflict-aware arbitration prototype with deterministic result validation | Arbitration contract and scenario bundle | Permissionless registration is not Sybil-resistant or globally fair |
| Reputation | Explainable ratio derived from immutable fulfillment and arbitration events | Finance/reputation contract | Not an authorization weight or Byzantine-tolerance proof |
| Settlement | Marketplace can project a matching external Finance record by stable reference | Companion Finance patch plus artifact-bound settlement receipt | Requires an active patched Finance role; indeterminate Processing fails closed |
| 45% Byzantine tolerance | **Do not claim as demonstrated** | No accepted threat model and reproducible end-to-end proof in this repository | Historical phase documents overstate the evidence |
| Multi-conductor conflict safety | In the controlled network profile, two distinct conductors must first preserve shipped/cancelled as an unresolved conflict, then observe independent buyer and seller approvals and converge on the same authorized shipped projection while retaining both original heads | `network-promotion-v1` version 3, network receipt, separate conductor logs, and signed bundle | This proves one bilateral protocol under a controlled topology; arbitration-authorized network convergence, internet-scale behavior, and arbitrary-partition availability remain unproven |
| Deployment promotion | A specific build may be promoted for a named profile when artifact-bound live receipts verify it | `deployment-promotion-v1` contract and promotion receipt | Promotion is profile- and artifact-specific; it is not general production readiness |
| Production ready | **Do not claim** | No broad adversarial, operational, and availability evidence | Pre-alpha |

## Evidence rule

A public claim should identify:

- exact code revision;
- threat model and property measured;
- reproduction command;
- dependency and conductor versions;
- inputs, agent count, and network topology;
- pass/fail criteria;
- raw output or durable release artifact;
- limitations and unresolved counterexamples.

Historical completion reports are project history, not automatically current evidence.
