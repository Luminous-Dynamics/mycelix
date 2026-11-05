# Pulse AuthorityCheckpoint lineage — bounded model-check receipt

**Result:** PASS for the stated finite lineage model and exact model-check run  
**Workflow:** [Pulse Authority Lifecycle Model, run #134](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41945984183)  
**Artifact:** [TLC logs, receipt and counterexamples](https://github.com/Luminous-Dynamics/mycelix/actions/runs/41945984183/artifacts/13379245045), ID `13379245045`  
**Artifact ZIP SHA-256:** `c56658f77793f63003611c01da60af059fe43d9a8103273093d083aa874d95`  
**PR head checked:** `67ec7a6f64ebd940fc7b4db1931fdc55cbb3342e`  
**Merge checkout:** `3f3905e8` (merge into base `9e12dd46bdd8371e8ed9626a17bc06f21c141a52`)  
**TLC:** 1.8.0 / build `2026.08.11.125311`  
**TLC JAR SHA-256:** `ab323b79802aedc3203b3f9af37c6aca3ed43f4e0225b36f2aa77b26de46c05f`

## Results

| Configuration | Result | States generated | Distinct states | Outcome |
|---|---|---:|---:|---|
| `PulseAuthorityCheckpointLineage.cfg` | PASS | 31 | 14 | Queue exhausted; no invariant error |
| `PulseAuthorityCheckpointLineage-bypass-mutant.cfg` | Expected counterexample | 11 | 8 | `NoForeignLineageCanBeTrusted` violated |

The negative-control workflow step passes only when TLC exits non-zero and reports the intended invariant. A parser error or unrelated invariant failure cannot satisfy the mutant check.

## Counterexample

TLC found the following unsafe schedule when the reader compares only numeric epoch:

1. Canonical history starts at DNA `dna-canonical`, grantor `agent-grantor`, epoch 0, with the grant active.
2. Canonical host revocation commits. The authority epoch advances to 1 and the required checkpoint binds to canonical DNA, author and `action-revoke`.
3. Before the canonical observer projection converges, the observer sees a foreign history from `dna-foreign` with a different `action-foreign` at the same integer epoch 1. It is explicitly not descended from the required checkpoint.
4. The mutant checks only `projectionEpoch >= requiredEpoch` and returns `TRUSTED_ACTIVE`.

The resulting read has:
- observed epoch = required epoch = 1;
- canonical revocation = true and host grant = inactive;
- projection DNA = foreign DNA;
- projection action = foreign action;
- ancestry proof = false;
- decision = `TRUSTED_ACTIVE`.

TLC catches that result with `NoForeignLineageCanBeTrusted`. The safe configuration requires matching DNA/cell identity, author/source-chain identity and ancestry evidence, and therefore returns `UNKNOWN` for the foreign/conflicting projection.

## Modeled contract

The model keeps separate:
- live conductor host CapGrant state;
- canonical authority history (DNA/cell identity, author, ActionHash, epoch, revocation);
- required checkpoint identity (DNA, author, action, epoch);
- observer projection identity and state;
- proof that the observer projection descends from the required checkpoint;
- the final read classification.

A checkpoint is a *causal lineage fence for an application-state read*. It is not a bearer token, capability, grant or proof that the conductor will authorize a call. Host calls remain subject to the current conductor grant and resource/domain validation.

## Relationship to the scalar-epoch model

The earlier `PulseAuthorityCheckpoint.tla` is a useful preliminary model for the simple stale-active case. It proves that an epoch-constrained reader returns UNKNOWN while the observed epoch is behind the requested epoch. It is **not sufficient as the checkpoint design**: a foreign or conflicting history can have the same integer epoch and still be unrelated to the requested source-chain transition.

The lineage model supersedes the scalar-only freshness rule. Numeric height is necessary but not sufficient; the read must also establish the correct DNA/cell and source-chain ancestry. The project should not invent a global total order across independent Holochain source chains or treat equal epoch numbers as proof of causal equivalence.

## Boundaries

This is a finite abstract model with one canonical grantor history, one foreign history, one revocation, and one observer. It does not implement Holochain conductor APIs, DHT validation receipts, source-chain cryptographic signatures, network partitions, multiple authors, competing valid checkpoints, DHT authority gossip, rollback detection, checkpoint persistence or production latency. It does not establish that Holochain provides a global snapshot token or a built-in ancestry proof equivalent to this abstraction.

A production design must derive its lineage evidence from actual Holochain identities and validated records. If a peer cannot prove that its projection descends from the requested checkpoint, the correct result is UNKNOWN or explicitly stale advisory—not trusted-active.

**Claim ceiling:** bounded model-check evidence for this exact TLA+ model/configuration and pinned TLC build only; not a conductor test, general fork-resolution proof, cryptographic proof, or production qualification.
