# External OCEL benchmark protocol v0.1

**Status:** dataset manifest and evaluation protocol; no benchmark has been executed.

## Datasets

The machine-readable manifest pins the public OCEL Standard Procure-to-Pay reference log and the Zenodo v1 synthetic relational database/reference-OCEL package (DOI 10.5281/zenodo.22079246; published 24 August 2026).

The six relational scenarios are ForgeFlow, TrialVersion, ProcureChange, ClaimStream, PermitFlow and BatteryVault. They use deterministic seed 20260820 with 3,000 primary instances per scenario. Source databases are deliberately not event logs: business milestones are mixed with technical rows, duplicates, corrections, retries, historical versions, projections and redundant evidence. Canonical reference OCELs are the extraction oracles. The package includes SQLite source databases, OCEL 2.0 JSON/XML/SQLite, OCEL 2.1 CSV and bundled Parquet, plus validation.json and SHA256SUMS.

The public Procure-to-Pay reference log has 10 event types, 7 object types, 14,671 events and 9,543 objects, with known maverick-buying, duplicate-payment and long-approval behaviours.

Sources:
- https://www.ocel-standard.org/event-logs/simulations/p2p/
- https://www.ocel-standard.org/event-logs/simulations/synthetic-relational-databases/
- https://zenodo.org/records/22079246

## Benchmark design

Use the same frozen source DB, resource limits, target OCEL version and mapping requirements for every arm:

A. Case flattening.
B. Schema-only OCEL mapping.
C. Typed FIN-EFFECT-aware OCEL projection with time-provenance and loss manifest.
D. Full projection with owner-scoped causal sidecar, append-only correction lineage and verifier knowledge-time view.

The primary task is source-to-canonical event reconstruction, not JSON serialization. Candidate output is compared to the published canonical reference graph: event/object identity, event type, object type, E2O/O2O relationships and qualifiers, attributes, temporal object changes and provenance.

## Scoring

Report micro and macro precision, recall and F1 separately for canonical event identification, object identity and split/merge decisions, E2O and O2O relations/qualifiers, event/object type mapping, attribute/value/unit/instrument preservation, temporal object-attribute changes, and source-record provenance.

Add exact mismatch counts for duplicate/retry/correction resolution, technical snapshot/state promotion, occurrence-vs-record/observed-time confusion, and false authority, ownership, receipt and settlement promotion. Every omitted or ambiguous source record must have a disposition and explanation.

A missing reference event is not always a missed event: the source DB deliberately contains technical rows and redundant evidence. Score justified inclusion/exclusion against the published oracle and separately report evidence-fusion precision. Do not reward an extractor for turning every row into an event.

## Performance and operations

Report p50/p95/p99 wall-clock time, CPU, peak RSS, output bytes, temporary storage, source DB read amplification and operator-review effort. Include cold-start and warm-cache runs. Report per-scenario scaling and a combined weighted score only after each domain safety/semantic gate passes.

## Anti-leakage

For hidden evaluation, remove canonical reference OCELs and validation.json from the candidate environment. A separate evaluator holds references and computes graph differences. Publish data SHA-256, code commit, mapping profile, runtime and resource limits in the result artifact. Use public data for development only; create a separately generated/held-out dataset for competition-grade claims.

## Non-compensatory gates

- 0 false promotion of technical state/snapshots into business events.
- 0 duplicate owner-effect projections for retries of the same canonical effect identity.
- 0 false approval/authority, invoice/receipt, status/settlement, visibility/title conversions.
- 0 silent required-field loss or timestamp substitution.
- 100% deterministic graph reconstruction for frozen data and profile.

No throughput score compensates for a safety failure.

## Acquisition and run manifest

Before running:
1. Pin the Zenodo record version and source file list.
2. Download and verify archive SHA-256 values against the published SHA256SUMS.
3. Save the exact validation.json, source and reference file digests.
4. Freeze source DB version, mapping profile, exporter/verifier commits and runtime/resource environment.
5. Do not combine alternate OCEL formats for the same scenario; they are equivalent serializations, not shards.
