# Evidence-Centric Temporal Query Benchmark Protocol v0.1

Status: proposed qualification protocol. No external dataset has been downloaded or executed by this crate yet; no comparative performance claim is made.

## 1. Research question

Can a temporal evidence engine reconstruct what was valid at business time **V**, according to a particular verifier **X** by knowledge-time cutoff **K**, without allowing later corrections, missing observations, uncertain clocks, or competing branches to rewrite what that verifier was justified in knowing?

This differs from a conventional two-column bitemporal query. The benchmark treats verifier-relative knowledge as a first-class query dimension, while source record time, receiver observation time, and owner-local causal order remain distinct evidence dimensions. In the v0.1 kernel K is a timestamp cutoff; the stored `knowledge_frontier_ref` is opaque and is not authenticated or resolved. The stronger partial-order frontier query is specified separately in `VERIFIER_RELATIVE_FRONTIER_QUERY_V0.1.md`.

## 2. Comparator classes

Use matched-semantics comparators, not product-name comparisons:

| Comparator | Documented baseline | Required adapter for matched semantics |
|---|---|---|
| Microsoft SQL Server temporal tables | System-versioned row history and FOR SYSTEM_TIME AS OF reconstruction | Append-only fact/receipt tables and explicit verifier identity + knowledge-time filter |
| XTDB | Separate valid-time and system-time querying over immutable history | Explicit verifier-relative knowledge assertions and frontier filter |
| Prevalidated scan baseline | Full scan of the same typed assertions after one-time structural validation | None |
| Mycelix preindexed kernel | Rust in-memory index keyed by claim/dimension and fact/verifier receipt; validates once at construction | None for in-memory semantic tests |
| Independent reference interpreter | Simple, separately implemented oracle over the same fixture semantics | Required to avoid sharing selection logic with the system under test |
| OCEL / PM4Py process-mining path | Object-centric events, objects, and qualified relationships | Project source records into the same normalized event/object evidence model |

The SQL Server and XTDB comparisons must not imply their native temporal features are semantically identical to verifier-relative knowledge. Query syntax, transaction-time semantics, storage configuration, and isolation level must be documented for each run.

## 3. Public data

### 3.1 Synthetic relational source systems

Use the OCEL Standard's deterministic synthetic relational benchmark, generated with seed 20260820. It contains six source databases, each with 3,000 primary instances:

| Scenario | Domain | Reference events | Reference objects |
|---|---|---:|---:|
| ForgeFlow | Engineer-to-order manufacturing | 97,884 | 59,036 |
| TrialVersion | Clinical-trial execution | 74,661 | 33,858 |
| ProcureChange | Industrial procure-to-pay | 100,504 | 46,353 |
| ClaimStream | Insurance claims | 99,715 | 40,610 |
| PermitFlow | Building-permit approval | 118,084 | 32,783 |
| BatteryVault | Circular battery lifecycle | 88,346 | 25,779 |

The sources deliberately include technical state, duplicates, corrections, retries, historical rows, projections, and redundant evidence. Use the source SQLite database and canonical reference OCEL as a matched pair. Preserve the original package, SHA256SUMS, validation.json, scenario name, seed, and the exact extraction code revision. The published reference OCELs are not to be treated as proof that Mycelix's own projection is correct; they are the external comparison target.

Source: https://www.ocel-standard.org/event-logs/simulations/synthetic-relational-databases/

### 3.2 Established process-mining logs

Add the OCEL Standard's Procure-to-Pay (P2P) reference log as a separate compatibility workload. Record its exact downloaded format and checksum, because the event-log catalogue exposes multiple formats and versions.

Catalogue: https://www.ocel-standard.org/event-logs/

### 3.3 Synthetic adversarial evidence

Use the repository's 36-case temporal corpus and add generated families for:
- late, missing, duplicate, contradictory, and reordered knowledge receipts;
- correction chains and competing branches;
- two verifiers with divergent knowledge frontiers;
- valid-from / valid-to boundary equality and uncertainty windows;
- malformed timestamps, unpinned offsets, leap seconds, and precision mismatches;
- revoked or rotated source authorities (as structural inputs only until authentication exists);
- missing end-boundary evidence versus a genuinely absent end boundary;
- digest syntax valid but digest content mismatched (the current kernel must not claim to detect the latter).

## 4. Workload matrix

Run deterministic synthetic workloads at 1K, 10K, 100K, and 1M assertions. Scale independently:

| Dimension | Values |
|---|---|
| Assertions | 1K / 10K / 100K / 1M |
| Verifier identities | 1 / 10 / 100 / 1,000 |
| Corrections per claim | 0 / 1 / 2 / 8 / 64 |
| Competing branches | 0 / 2 / 4 / 16 |
| Metadata-to-fact ratio | 1:1 / 2:1 / 4:1 / 8:1 |
| Query locality | 100% hot / Zipf-like / uniform |
| Knowledge frontier | before fact / between corrections / after latest correction |
| Temporal quality | asserted / estimated / uncertain / contested |
| Event-time order | ordered / shuffled / adversarially reordered |

The first example benchmark sweeps smaller sizes to validate the harness and compare the prevalidated scan baseline against the preindexed engine. Both validate once at construction; only their lookup structures differ. The independent reference interpreter remains a semantic oracle and is not conflated with the performance baseline. This smoke test is not the external SOTA benchmark.

## 5. Correctness gates

Correctness is a hard gate, not a weighted score:

1. **Oracle parity:** 100% identical dispositions and selected assertion IDs between the independent oracle, prevalidated scan engine, and indexed engine for all generated valid inputs.
2. **No false allow:** zero cases where NotKnown, Indeterminate, or Conflict becomes ValidAtQuery or Known.
3. **Correction visibility:** exact cutoff behavior; a correction changes the as-known answer only after that verifier's qualifying receipt.
4. **Verifier isolation:** no knowledge transfer between verifier identities without an explicit receipt.
5. **Boundary semantics:** half-open [from,to) behavior and no inferred open interval when an end-boundary assertion exists but is unknown.
6. **Uncertainty:** no confident point answer inside a contested/uncertain boundary region.
7. **Time discipline:** no silent timezone conversion, clock substitution, precision upgrade, or leap-second coercion.
8. **Projection fidelity:** report event/object/relationship precision, dropped or held rows, provenance coverage, and timestamp-source coverage against the published reference.
9. **Mutation isolation:** these query results cannot by themselves authorize a journal, transfer, payment, or policy effect.

A single false allow blocks qualification.

## 6. Performance and resource metrics

For each system, publish:
- index or database build time;
- cold-start and warm p50 / p95 / p99 point-query latency;
- throughput at fixed concurrency;
- memory high-water mark and resident bytes per assertion;
- bytes read / scanned per query where available;
- ingest / correction-append throughput;
- query latency as unrelated-claim count grows;
- query latency as correction-chain depth and branch fan-out grow;
- adapter / normalization time separately from query time;
- semantic disagreement count beside every performance result.

Record CPU model, RAM, storage device, OS/kernel, compiler/runtime versions, database configuration, transaction isolation, dataset checksum, warm-up protocol, query seed, concurrency, and exact source commit. Report medians across repeated runs and publish raw machine-readable output.

## 7. Provisional targets (not achieved results)

- 100% parity with the independent reference oracle.
- Zero false allows across the full adversarial suite.
- At 1M assertions, at least 10x lower p95 point-query latency than the unindexed scan reference for claim-local queries, with no semantic disagreement.
- Query latency should be driven primarily by the selected claim's evidence/correction fan-out, not by unrelated claims elsewhere in the corpus.
- Keep index memory overhead at or below 3x the serialized assertion payload as an initial target; revise only with measured workload evidence.
- Do not claim superiority over XTDB or SQL Server until both systems are run with a matched verifier-knowledge adapter and the same data, hardware, consistency policy, and query mix.

## 8. Frontier-relative extension (future work, not v0.1)

The next stage replaces timestamp-only K selection with an authenticated verifier-specific frontier F over owner-scoped causal histories. K may remain an optional reporting constraint, but F controls receipt membership. Frontiers are partially ordered; concurrent cuts are not assigned an artificial total order. An assertion can be classified as `NotKnown` only when the relevant owner prefix is proven complete and no qualifying receipt is included. Incomplete coverage or missing absence evidence is `Indeterminate`.

The frontier extension must add frontier width, owner cardinality, fork/merge frequency, unresolved-gap density, inclusion/consistency proof size and verification cost to the performance matrix. Its own correctness oracle must measure false-`NotKnown` as well as false allows. See [VERIFIER_RELATIVE_FRONTIER_QUERY_V0.1.md](VERIFIER_RELATIVE_FRONTIER_QUERY_V0.1.md) and [Issue #6680](https://github.com/Luminous-Dynamics/mycelix/issues/6680).

## 9. Claim ceiling

The current kernel validates assertion structure, digest shape, correction lineage, and verifier-relative selection over caller-supplied data. It does not recompute evidence digests, authenticate signatures or issuers, calibrate clocks, persist knowledge frontiers, or establish legal/financial authority. External data projection, full-schema validation, database adapters, and production replay require independent qualification.

This protocol is designed to make a failed comparison visible. It does not presume that Mycelix wins.


## 10. Validity-episode identity extension (v1.1)

A claim may have multiple disjoint or overlapping validity episodes. The benchmark therefore treats `validity_interval_ref` as a first-class join key, not as a display label.

Add deterministic episode workloads at:
- 1 / 2 / 10 / 100 / 1,000 episodes per claim;
- disjoint, overlapping, adjacent, nested and equal-boundary intervals;
- same claim and subject with different episode identities;
- duplicate start/end assertions within one episode;
- endpoint corrections that preserve or attempt to change episode identity;
- same-episode corrections accepted before / after the historical knowledge cutoff;
- no VALID_TO, unknown VALID_TO, and explicit-open disposition (only when a separately versioned open-end profile is implemented).

Episode-specific correctness gates:
1. 100% scan / prevalidated-scan / index parity with an independent endpoint-pairing oracle.
2. Zero cross-episode endpoint pairs.
3. Zero corrections accepted when they change the predecessor's episode identity.
4. Zero legacy untagged pairs promoted to episode-complete answers.
5. Zero missing VALID_TO records treated as OPEN solely due to caller policy.
6. Zero historical result changes when a correction is only visible after K.
7. Report ambiguity and abstention separately from false rejection; a conservative Indeterminate is not scored as correct Supported.

For the current v1.1 implementation, explicit open-end disposition is not yet implemented. A missing end remains Indeterminate, including when the legacy caller boolean is true. The benchmark must not award an OPEN success until a separately typed, frontier-bound open-end assertion/profile and its negative controls are implemented.

## 11. Matched-semantics industry comparison

Compare systems at three separate levels and never mix their guarantees:

- **Bitemporal storage/query:** XTDB and SQL Server system-versioned temporal tables. Compare valid-time and system/transaction-time reads, backdated corrections, query latency, index build cost, storage amplification and historical replay. These systems already provide meaningful bitemporal history; Mycelix does not claim to replace that function.
- **Event/observation semantics:** OpenTelemetry distinguishes source event `Timestamp` from `ObservedTimestamp`. Use it as a mapping baseline for ingestion/observation separation, not as an evidence-authentication or owner-frontier verifier.
- **Authenticated append-only proofs:** RFC 9162 consistency proofs establish append-only extension between Merkle tree heads and inclusion proofs establish membership under a root. Mycelix adds application-specific owner identity, accepted-frontier coverage and receipt-to-interval projection obligations; those application semantics are not supplied by Certificate Transparency itself.

Run each adapter against the same frozen source events, correction schedule, exact verifier cut, policy revision, and expected output. Native product features must be configured to expose equivalent output claims. Where they cannot express verifier-relative frontier membership or episode pairing, report the missing semantics explicitly rather than silently reducing Mycelix's obligations or declaring the comparator incorrect.

Report:
- correctness and false-allow counts before performance;
- p50/p95/p99 query/build/update latencies;
- storage and memory amplification;
- independent owner count and receipts examined;
- proof bytes and verification time;
- correction-impact fan-out and recomputation cost;
- abstention/Indeterminate rate and false holds;
- mutation detection by risk tier;
- complete raw measurements, hardware/software versions and commit SHAs.

A defensible SOTA claim requires Mycelix to pass the stronger correctness matrix and to report the added cost of those guarantees. A throughput win is not valid when the compared system was not asked to prove the same historical, completeness, provenance and episode semantics.
