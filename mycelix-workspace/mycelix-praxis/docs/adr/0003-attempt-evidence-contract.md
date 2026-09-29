# ADR-0003: Attempt Evidence Contract Before DNA Materialization

- Status: Proposed
- Subject: `PRAX-EVIDENCE-002A`
- Depends on: `PRAX-EVIDENCE-001` / PR #2467
- Follow-up: `PRAX-EVIDENCE-002B` Holochain materialization

## Context

Praxis still carries the historical adaptive API:

```text
record_attempt(mastery_hash, correct, response_time_ms)
```

That call records too little information to support a provenance-complete learning-evidence claim. It does not identify the source task/activity, capability dimension, assistance state, original evaluator/version, or an independently traceable source record/artifact.

At the same time, the Praxis Holochain workspace remains on the repository's isolated 0.6-era migration authority. Adding a new integrity entry type is a DNA change, not a coordinator-only refactor. Mixing the semantic evidence repair with an unrelated Holochain migration would make both lineages harder to qualify.

## Decision

Freeze the attempt-evidence semantic contract in `praxis-core` first, without changing any integrity-zome entry definitions.

The contract distinguishes:

```text
complete attempt observation
!= legacy-incomplete attempt observation
!= admitted evidence
!= capability estimate
!= credential decision
```

### Complete attempt observation

A `Complete` `AttemptEvidence` requires:

- stable event identity;
- learner identity;
- capability identity;
- source activity/task/assessment identity;
- explicit capability dimension;
- observed outcome;
- explicit assistance classification;
- producer identity and version;
- at least one source-record or artifact trace reference;
- observation time.

A complete attempt can be projected into the generic `LearningEvidenceEvent` contract, but that projection is still only an observation.

### Legacy-incomplete attempt observation

Historical `record_attempt` calls remain useful observations, but missing history must stay visible.

The compatibility constructor therefore preserves only what was actually recorded:

- learner and capability supplied by the migration boundary;
- binary correctness;
- response time;
- observation time;
- truthful identity/version of the compatibility adapter.

It explicitly does **not** invent:

- a source task/activity;
- a capability dimension;
- an assistance classification;
- an original evaluator;
- a source record or artifact digest.

Those missing fields are enumerated in `AttemptEvidenceCompleteness::LegacyIncomplete`.

A legacy-incomplete attempt cannot silently convert into a provenance-complete `LearningEvidenceEvent`.

## Holochain materialization boundary

`PRAX-EVIDENCE-002B` will map this contract into the adaptive Holochain surface only under the correct migration authority.

The intended Holochain shape is:

1. private, append-only attempt entries;
2. create-author binding for learner identity;
3. updates rejected at integrity validation;
4. delete semantics explicitly qualified rather than assumed;
5. new provenance-complete recording API;
6. explicit legacy compatibility adapter;
7. source-chain-local discovery by default.

### Privacy rule: no public evidence index by default

Private Holochain entries must not gain a public metadata side channel merely for convenient lookup. `002B` should therefore avoid learner→attempt or skill→attempt DHT links by default. If indexing is required, it must have a separate privacy analysis and explicit disclosure semantics.

The stable evidence-event identifier for a materialized private attempt should preferably be its entry-creation `ActionHash`, which already binds authorship and creation time. The semantic contract deliberately accepts an opaque `EvidenceEventId` so the adapter can make that mapping without changing the core ontology.

## Legacy adapter provenance

The compatibility boundary identifies itself as:

```text
praxis-adaptive:legacy-record-attempt-adapter
```

That identifies the adapter producing the wrapped record. It is **not** represented as the historical evaluator. `OriginalEvaluator` remains explicitly missing.

## Non-goals

This tranche does not:

- change the adaptive integrity zome;
- change the DNA hash;
- write source-chain data;
- make BKT event-derived yet;
- admit evidence for a credential;
- claim historical attempts are provenance-complete;
- infer capability dimensions from correctness;
- infer that tool-assisted or collaborative work is independent performance.

## Next executable theorem

`PRAX-EVIDENCE-002B` must prove at the Holochain boundary:

```text
private persistence
+ author binding
+ append-only source semantics
+ complete provenance
!= admission
!= mastery
!= credential
```
