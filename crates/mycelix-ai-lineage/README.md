# mycelix-ai-lineage

Pure `#![no_std]` AMSAP-004 operational lineage semantics for artificial systems.

The crate keeps four identity concepts separate:

```text
ModelId != LineageId != InstanceId != CivicId
```

- `ModelId` identifies an explicit model artifact/version.
- `LineageId` identifies an operational history branch.
- `InstanceId` identifies one runtime execution branch.
- `CivicId` is an externally granted civic/legal reference and is never created by lineage events.

The lineage validator supports explicit fork, merge, historical-checkpoint restore, and model-mutation events. Forks must create distinct child lineages and instances; merges preserve all parent histories and create a distinct result lineage; restoring an old checkpoint creates a new branch rather than rewriting intervening history; model mutation is recorded explicitly.

This crate deliberately does **not** answer whether identical or diverged executions are metaphysically one person, multiple persons, or no persons. It preserves the operational facts needed for later scientific and constitutional reasoning.

Validated lineage events create no CivicId, legal standing, governance authority, currentness, voting entitlement, or external-effect authority. Copying software therefore has no native path to manufacturing citizens or votes.
