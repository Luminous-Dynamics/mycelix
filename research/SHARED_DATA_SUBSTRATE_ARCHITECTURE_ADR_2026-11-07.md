# Shared Data Substrate Architecture — ADR and Qualification Plan

**Status:** Proposed architecture decision for review; no production database migration is authorized by this document.

**Date:** 2026-11-07

**Scope:** Symthaea, Mycelix, Symtropy and Sol-Atlas.

**Decision owner:** Luminous Dynamics engineering.

## 1. Executive decision

Do not select one database to perform every job. Adopt a deliberately small polyglot data architecture with explicit authority boundaries:

1. **Holochain remains Mycelix's peer-authored, validation-governed distributed authority substrate** for the civic/commons/identity/governance claims and actions that are designed to live on its DHT/source-chain model. A relational projection must not silently become the authority for those records.
2. **PostgreSQL 18.6 is the default shared transactional and query-projection database** for service-owned relational state, operational records, temporal projections, GIS, and cross-domain queries. Use PostgreSQL 18, not PostgreSQL 19 RC, for production qualification at this date.
3. **pgvector 0.8.7 is the first integrated vector-search baseline**, but not a universal answer for Symthaea's 16,384-dimensional HDC vectors. Benchmark Qdrant as the dedicated vector candidate, and Vespa as the advanced hybrid-ranking candidate.
4. **Keep Symthaea's existing SQLite persistence for per-node/local memory** unless measured multi-writer or query requirements justify a service boundary. Keep DuckDB for local/offline analytical work, not as the transactional system of record.
5. **Symtropy's simulation hot loop remains in memory.** Persist snapshots, event/replay records and operational observations outside the physics step. Use PostgreSQL for scenario/asset/operational metadata; evaluate TimescaleDB only if time-series workloads justify it; use DuckDB + Parquet for experiment analytics.
6. **Sol-Atlas should use PostgreSQL + PostGIS for authoritative infrastructure/geospatial records**, with its existing H3 code for hierarchical cell aggregation. Evaluate pgRouting for road/utility/transport network routing. Browser/WASM code talks to a service API, never directly to a database.
7. **Use immutable content-addressed object storage for large source artifacts, simulation artifacts and evidence payloads.** Adopt Parquet and an Iceberg table format only when the scale and multi-engine analytics needs justify them.
8. **Do not add Neo4j, Redis, Kafka, Milvus, InfluxDB or a second graph/temporal ledger by default.** Each must earn its place through a workload-specific benchmark.

The central architecture principle is:

> Authoritative events and evidence have one declared owner. Relational, vector, graph, geospatial and analytical databases are projections or domain stores with explicit ownership—not competing sources of truth.

## 2. Why a single database is the wrong abstraction

The four projects have materially different data workloads:

- Mycelix needs peer-authored, signed, validated, decentralized records, privacy boundaries, and historical provenance.
- Symthaea needs low-latency local cognitive state, memory persistence, high-dimensional HDC representations, temporal evidence retrieval and analytical evaluation.
- Symtropy needs deterministic simulation state transitions, replay/checkpointing, asset/scenario metadata and potentially high-volume telemetry.
- Sol-Atlas needs geographic features, infrastructure networks, spatial joins, H3 aggregation, routes and historical asset state.

A database optimized for one of these workloads is not automatically good at the others. In particular:
- A DHT is not a general-purpose SQL analytics engine.
- A vector index is not a source-of-truth ledger.
- A time-series database is not a substitute for relational transactions and rights checks.
- An analytical columnar engine is not a multi-writer operational database.
- An in-memory simulation state is not a durable event history.
- A web frontend is not a safe place to embed database credentials.

We should optimize each data path while sharing identity, provenance, temporal and rights contracts across systems.

## 3. Repository audit: what exists today

The following is a source-level inspection of the public GitHub `main` branches, not a claim that every deployment or private workspace matches them.

### 3.1 Symthaea

The current root workspace declares:
- `rusqlite = 0.37` with bundled SQLite.
- Optional `duckdb = 1.1` for analytical/epistemic-audit work.
- Existing SQLite persistence paths include knowledge-graph persistence, conversation/memory storage, Nixward and Muse genealogy.

This is a reasonable embedded/local starting point. Do not migrate all local memory to a remote database simply to standardize the stack. Keep local state available when the network is unavailable. Promote only cross-agent/shared, multi-writer or service-query workloads to PostgreSQL.

The critical vector-specific finding is that ordinary pgvector ANN indexes are not a drop-in solution for all Symthaea HDC vectors. The current pgvector documentation distinguishes storage dimensionality from index dimensionality: `vector` and `halfvec` can store up to 16,000 dimensions, but their standard HNSW/IVFFlat index limits are lower; binary `bit` indexes support up to 64,000 dimensions. Symthaea's 16,384-dimensional representations therefore require a representation-aware decision:
- For genuinely binary HDC hypervectors, evaluate packed bit/Hamming or Jaccard search.
- For continuous 16,384-dimensional hypervectors, evaluate Qdrant, which documents dense vectors up to 65,535 dimensions, or a different indexed representation.
- Do not silently down-project or binary-quantize an HDC representation unless task-level fidelity is measured against the original similarity function.
- Retain the original exact representation and a versioned transform/quantization identity; the ANN index is a candidate generator, not the evidence source.

### 3.2 Mycelix

The current repository's architecture is explicitly Holochain-first: domain clusters are separate DNAs/zomes, composed into hApps, with cross-cluster interaction through sanctioned bridge zomes. Holochain's DHT stores addressable entries/actions and connects them with typed links; public and private records have different storage/privacy semantics. Holochain's CRUD model preserves history by attaching metadata rather than overwriting immutable DHT content.

The repo also already has PostgreSQL/SQLx-backed service paths (including Pulse/Music APIs and DeSci authority-service code) and Redis in some service configurations. This is a useful precedent for separating:
- Holochain authority / peer validation;
- service-owned relational state;
- query and analytics projections;
- ephemeral caching.

There is an important Cargo integration constraint. The current Mycelix workspace has Holochain 0.6.1 dependencies and a workspace SQLx dependency with default features disabled. The DeSci code documents why its SQLx/Postgres authority backend and its Holochain DHT bridge were separated into different workspaces: Holochain's `rusqlite` dependency and SQLx's optional `sqlx-sqlite` dependency declare incompatible `links = "sqlite3"` requirements in one Cargo lockfile. Keep server crates explicitly on SQLx's Postgres feature set; do not activate SQLx SQLite in the unified Holochain workspace. Preserve the existing dependency boundary unless the upstream SQLite linkage issue is deliberately resolved and tested.

**Version alignment warning:** the public Mycelix `main` workspace inspected for this ADR pins Holochain/HDK to 0.6.1 (and HDI 0.7.1), while current upstream Holochain documentation recommends 0.7.1. This is a separate migration decision, not a reason to change the database architecture. The official 0.6→0.7 upgrade guide says there is no data migration path and DNA hashes change; 0.7.0→0.7.1 also has a compatibility break. Audit deployed conductors, data, DNA hashes, hApp bundles and rollback/restore before any upgrade. Do not perform a blind version bump as part of this ADR.

### 3.3 Symtropy

The public Symtropy root package is a Rust/Bevy simulation and physics workspace. Its architecture describes deterministic in-memory stepping and replay/snapshot structures. The root package does not currently declare PostgreSQL, TimescaleDB, DuckDB or a dedicated database dependency. Its lockfile includes rusqlite through a `symtropy-foundry` package, but that alone is not evidence that the physics core should depend on SQLite.

Keep the physics engine independent of database I/O. A database write, network request, index refresh or blocking transaction must never sit in the collision/constraint/physics-step path.

### 3.4 Sol-Atlas

The inspected `sol-atlas-leptos` repository is presently a Leptos CSR/WASM frontend plus a `sol-atlas-core` crate. The core crate depends on Serde/JSON and `h3o`; neither the frontend nor core currently declares a database driver. H3 is already part of the project model.

Add a backend service only where shared, durable infrastructure data and multi-user editing require it. The frontend must call authenticated APIs; no database credentials or direct database connections belong in browser WASM.

## 4. Recommended logical storage architecture

| Layer | Recommended technology | Owns | Must not own |
|---|---|---|---|
| Peer-authored decentralized authority | Holochain source chains + DHT | Signed actions/entries, integrity validation, typed links and peer-held public records | General ad-hoc SQL analytics |
| Shared transactional/query store | PostgreSQL 18.6 | Service-owned relational records, projections, temporal tables/views, API queries and operational metadata | Mycelix DHT authority by implication |
| Integrated vector baseline | pgvector 0.8.7 | Smaller text/image embeddings and vector search that fits verified index limits | Rights, truth or proposition-version authority |
| Dedicated HDC/vector search | Qdrant benchmark candidate | High-dimensional vectors, payload filters, ANN retrieval and quantized search projections | Canonical source spans or evidence provenance |
| Advanced hybrid retrieval | Vespa benchmark candidate | BM25 + dense/sparse/tensor retrieval, multi-phase ranking and search at scale | Canonical lifecycle/rights ledger |
| Geospatial | PostGIS on PostgreSQL; pgRouting where needed | Geometry/geography, spatial relationships and route/network queries | H3 aggregation replacement |
| Hierarchical spatial indexing | Existing H3/`h3o` | Cell aggregation, coarse joins, hierarchical regional summaries | Exact geometry/topology or road-network routing |
| Local analytics | DuckDB + Parquet | Offline experiment analysis, data preparation, batch joins and reproducible reports | Multi-writer operational truth |
| Large immutable artifacts | Content-addressed files + S3-compatible object store | Source snapshots, audio, simulation runs, evidence payloads and exports | Relational metadata/transactions |
| Large analytical lake (later) | Apache Iceberg over Parquet/object storage | Snapshot-based large tables, schema/partition evolution and multi-engine analytics | Initial operational database |
| Local node persistence | Existing SQLite/rusqlite | Local/private agent memory, local caches and offline state | Cross-node shared authority |

This is deliberately a *logical* architecture. It does not require one PostgreSQL cluster for every project. Keep service ownership, operational blast radius, privacy and availability boundaries separate even if some environments initially share a PostgreSQL server.

## 5. PostgreSQL: recommended shared foundation

Use PostgreSQL 18.6 as the production baseline at the date of this ADR. PostgreSQL 19 RC 1 was released on 15 October 2026, with general availability scheduled for 19 November 2026. PostgreSQL's own beta guidance says release candidates are not intended for production. Test PostgreSQL 19 RC in an isolated compatibility lane; do not anchor production workloads to it before GA and extension/driver qualification.

Recommended extension candidates:
- **pgvector 0.8.7** for integrated vector search where dimensions and filters fit its actual ANN capabilities. Pin at least 0.8.7: it was released on 5 October 2026 with a security fix for an IVFFlat index-build buffer overflow (CVE-2026-103484).
- **PostGIS** for geospatial data, GiST/BRIN/SP-GiST spatial indexes and geometry predicates.
- **pgRouting** only for network-routing workloads that need graph algorithms over road/utility/transport topology.
- **TimescaleDB** only if measured time-series ingest, retention and continuous-aggregate workloads justify a specialized extension and its current license/operational requirements are reviewed.

Use PostgreSQL range types and explicit event columns for temporal correctness. Do not assume PostgreSQL automatically gives the application a complete bitemporal/legal amendment model. The application must explicitly represent:
- valid-time applicability;
- knowledge/transaction time;
- source publication/acquisition/receipt;
- transform/materialization time;
- index revision and exact serving-replica visibility observations.

Use append-only source events/operation records and rebuildable projections. A current-state table can be a materialized view of that history, not the only retained record. Use constraints and transaction boundaries to prevent overlapping or contradictory current states where the domain rules require it. Exclusion constraints and GiST indexes are candidates for valid-time interval invariants; domain-specific precedence still belongs in the resolver contract.

For Rust, use SQLx with only the explicit Postgres/runtime features needed by service crates, or tokio-postgres where a lower-level driver is preferable. Avoid enabling SQLx's SQLite feature in the same Cargo workspace as Holochain's rusqlite-linked dependencies.

## 6. Vector database decision: baseline plus two challengers

### 6.1 pgvector

Strengths:
- Same transaction and backup/replication boundary as PostgreSQL.
- Exact nearest-neighbour search is available as a baseline.
- HNSW and IVFFlat approximate indexes.
- Iterative index scans (since pgvector 0.8.0) can continue scanning to satisfy filtered queries, subject to configured scan limits.
- Binary `bit` indexes support Hamming/Jaccard and up to 64,000 dimensions.

Important limitation:
- Do not confuse vector storage limits with ANN index limits. At current docs, standard indexed `vector` and `halfvec` dimensions are materially lower than their storage limits; binary `bit` indexing has a higher limit. The 16,384-dimensional HDC workload must be measured in its real binary or continuous representation.
- Approximate search plus WHERE filters can still return too few eligible candidates. Iterative scans help but are not proof of exhaustive candidate recall. Measure filter selectivity and compare against exact search.

### 6.2 Qdrant

Make Qdrant the first dedicated-vector challenger because:
- It has an official Rust gRPC client.
- It documents dense vectors up to 65,535 dimensions, covering a 16,384-dimensional dense HDC experiment.
- Payload indexes are used for filtering/cardinality estimation and filter-aware search planning.
- It has documented strict-filter search improvements (including ACORN) and tenant-aware indexing approaches.

Cautions:
- Payload indexes must be designed and created for every hard filter field.
- Index configuration and filter selectivity affect recall and query plan.
- Rebuilding an HNSW index may be required when adding payload indexes after ingestion.
- A vector payload is still not a canonical evidence record. Keep the exact source/proposition/version/span, rights and lineage record in the authoritative store; the vector point carries stable references and a reproducible transform identity.

### 6.3 Vespa

Benchmark Vespa as the advanced search challenger if the product needs:
- Combined lexical and vector candidate retrieval;
- Multi-phase ranking and tensor features;
- Complex filtering/ranking and high-throughput serving;
- More control over ranking than a simple vector database exposes.

Vespa supports nearest-neighbour retrieval combined with lexical query operators and filters, exact search for bounded workloads, and phased ranking. It is more of a search/ranking platform than a minimal vector sidecar, so its operational and schema complexity must be justified by measured quality/scale gains.

### 6.4 Milvus

Keep Milvus as a later distributed-vector challenger, not a first dependency. Its standard and iterative filtered-search modes are relevant, but it introduces another service and operational surface. Add it only if Qdrant/Vespa/pgvector fail explicit scale, recall, availability or cost gates.

### 6.5 Proposed selection rule

- Start with pgvector for 768–4,000 dimensional dense embedding workloads where operational simplicity is valuable and the exact filtered-recall tests pass.
- For 16,384-dimensional continuous HDC, benchmark Qdrant first; test Vespa if advanced lexical/HDC/reranking combinations are required.
- For binary HDC, benchmark pgvector `bit` Hamming/Jaccard indexing against Qdrant's supported vector/quantization modes and Symthaea's own exact HDC candidate search.
- Do not move to Qdrant just because it is a vector database. Move when the measured quality/latency/scale gain is worth operating a separate service.
- Keep exact top-k search over a frozen corpus as the oracle for ANN recall.

## 7. Per-project recommendation

### 7.1 Symthaea

**Keep now**
- SQLite/rusqlite for per-node local cognitive memory and local persistent state.
- DuckDB for bounded offline analytical/evaluation workloads where it is already optional.
- Existing HDC-native retrieval for small or specialized candidate sets.

**Add/benchmark**
- PostgreSQL 18 service for cross-agent/shared evidence, query-visible receipts, proposition metadata, temporal resolution results and shared operational APIs.
- pgvector 0.8.7 for ordinary lower-dimensional embeddings.
- Qdrant for 16,384D continuous HDC and larger dedicated ANN workloads; compare binary HDC with pgvector `bit` before selecting.
- Content-addressed object store for source snapshots, large evidence payloads and immutable benchmark artifacts.

**Do not do**
- Do not force all local memory into PostgreSQL.
- Do not use SQLite as the shared multi-writer/federated evidence service.
- Do not put the 16,384D HDC representation into a vector type/index without checking both the exact storage limit and the actual ANN index limit.
- Do not let a vector search result mutate evidence confidence or corroboration counts.

### 7.2 Mycelix

**Authority**
- Keep Holochain as the authority substrate for data whose semantics are signed peer authorship, integrity validation, source-chain history, DHT replication and typed link relationships.
- Keep private data in its designed private/encrypted boundary; do not mirror sensitive payloads into a globally accessible relational projection.
- Do not replace Holochain with PostgreSQL for civic/commons/identity/governance truth.

**Relational services**
- PostgreSQL for service-owned state (for example, application/API data), administration, accounting projections, search indexes and high-performance reporting.
- Use explicit ownership: for every aggregate, declare whether Holochain or a service PostgreSQL database is authoritative. Avoid two independent writers with no reconciliation protocol.
- Project Holochain records into PostgreSQL through idempotent, replayable ingestion keyed by canonical action/entry/agent identity and DHT observation/validation state.
- Record projection watermarks and source references; do not represent a locally observed item as globally complete or validated without the required Holochain evidence.

**Rust/Cargo**
- Use SQLx Postgres-only features for PostgreSQL services.
- Keep the DeSci Holochain bridge / SQLx Postgres authority backend workspace separation while the SQLite `links` incompatibility remains.
- Redis is cache/rate-limit/ephemeral coordination only, never the source of truth for trust, identity, financial state or governance.

### 7.3 Symtropy

**Hot loop**
- PhysicsWorld, contact/constraint solver, body maps and deterministic per-step simulation state remain in memory.
- Database and network work must not occur inside physics stepping, collision callbacks or deterministic replay computation.

**Durability**
- Persist scenario definitions, entity/asset metadata, run manifests, parameter sets, checkpoints and replay references in PostgreSQL.
- Emit bounded batches of timestamped events/measurements at an explicit sampling cadence; do not persist every internal solver iteration unless the experiment requires it.
- Use object storage for large replay tapes, snapshots and simulation artifacts, with BLAKE3 digest and exact engine/build/config identity.
- Use DuckDB + Parquet for large sweeps, experiment comparison, calibration and offline scientific analysis.
- Evaluate TimescaleDB for sustained telemetry only after measuring event rate, retention, time-window queries and continuous aggregates. PostgreSQL partitioning is the baseline.

**Governance**
- If simulation entities represent Mycelix identity/trust/ownership, store the simulation state as a projection and bind it to Mycelix references; do not turn physics output into authority over civic records.

### 7.4 Sol-Atlas

**Recommended backend**
- PostgreSQL + PostGIS as the canonical operational store for infrastructure assets, project records, sites, routes, corridors, utility networks, environmental layers and change history.
- Use GiST spatial indexes and spatial-index-aware predicates such as ST_Intersects/ST_DWithin where appropriate.
- Keep H3 as an aggregation/indexing layer for cell-based rollups, regional summaries and hierarchical spatial joins; do not substitute H3 for exact geometry/topology.
- Evaluate pgRouting for transport, port, road, canal, rail, utility and evacuation network routing.
- Use DuckDB + spatial/Parquet for offline source ingestion, cleaning, regional analysis and reproducible batch outputs.
- Expose an authenticated Rust API to the Leptos/WASM frontend. Keep secrets and database credentials server-side.

**Versioning**
- Infrastructure edits should be append-only or historically reconstructable. Bind each feature to its source dataset/version, geometry digest, temporal applicability, jurisdiction, rights and projection revision.
- A map rendering is a read model, not evidence that a project exists or is authorized.

## 8. Temporal, provenance and lifecycle contract across every store

Every derived/indexed item must carry or resolve to:
- canonical artifact/source identity and immutable content digest;
- proposition/claim/version identity where applicable;
- exact source-span reference and coordinate system;
- valid-time and known-at semantics;
- jurisdiction/scope;
- provenance family;
- rights/access decision and revocation state;
- transform/model/index revision;
- parent lineage for summaries, translations, bridges, chunks and embeddings;
- exact materialization and serving visibility observations where the serving architecture exposes them.

The database may accelerate a query, but it cannot create authority. In particular:
- A Qdrant/pgvector/Vespa point is not an independent source witness.
- A cache hit is not proof that rights remain current.
- A query plan filter is not proof the provider applied the filter.
- A current index state does not prove historical query visibility.
- A vector similarity score does not establish proposition identity or corroboration.
- A source digest proves byte binding, not authenticity, legal authority, truth or entailment.

Use QVF-R / ALC to keep resolver correctness, index eligibility, rights and serving visibility as separate qualification dimensions.

## 9. Event, projection and replay pattern

Use a narrow event/projection pattern rather than introducing Kafka or a new generic ledger immediately.

1. The owning system commits an authoritative event (Holochain action/entry, or a service-owned PostgreSQL transaction).
2. The event carries an immutable identity, source/version, and idempotency key.
3. A projection worker consumes/replays the event and writes a PostgreSQL read model transactionally.
4. Vector/spatial/analytical indexes are derived from that projection and retain the projection revision and transform digest.
5. Corrections, rights revocations or parent changes produce invalidation/rebuild events for descendants.
6. A failed projection is retryable and cannot silently advance the projection watermark.
7. A query response records the exact source/projection/index/replica frontier that actually served it when available.

Start with a transactional outbox for PostgreSQL-owned services and an idempotent Holochain observation/indexing worker for DHT-owned records. Consider Debezium/Kafka/Redpanda only when throughput, fan-out or replay operations exceed the simpler pattern.

## 10. What not to add yet

| Candidate | Why not by default | Trigger for reconsideration |
|---|---|---|
| Neo4j or another graph DB | Existing Holochain links + typed PostgreSQL edge tables + recursive SQL cover the first graph workloads | A graph query benchmark proves a material path/query advantage and the system can preserve provenance/temporal semantics |
| Redis | Cache/coordination adds operational state and invalidation risk | Measured cache benefit for ephemeral, reconstructable data |
| Kafka/Redpanda | Adds cluster operations, partition/schema/replay design and another failure domain | Measured sustained event fan-out / replay throughput requires it |
| Milvus | Separate vector infra before Qdrant/Vespa has been measured | Qdrant/Vespa fail agreed throughput, filter recall, replication or cost gates |
| InfluxDB 3 Core | Excellent time-series candidate, but its own docs distinguish Core from Enterprise capabilities such as historical query, HA and read replicas | Edge/high-rate telemetry benchmark demonstrates a clear advantage and the product's retention/HA semantics match requirements |
| XTDB | Its native bitemporal model is attractive, but it is a separate JVM-backed database and would be another operational substrate | A controlled ALC/QVF-R comparison shows significant semantic/engineering value over explicit PostgreSQL temporal schemas |
| Iceberg | Excellent analytical table evolution/snapshots, but overkill before a large shared data lake exists | Multiple engines, long retention, large Parquet datasets and snapshot/branch workflows become first-class needs |

## 11. Shared Data Substrate Qualification (SDSQ) benchmark

Do not choose the database from marketing claims or a generic vector leaderboard. Create one small, reproducible cross-project benchmark contract with workload-specific tracks.

### 11.1 Candidate configurations

- A0: PostgreSQL 18.6, relational only, exact lexical/relational queries.
- A1: PostgreSQL 18.6 + pgvector 0.8.7.
- A2: PostgreSQL 18.6 + pgvector 0.8.7 + PostGIS.
- A3: PostgreSQL 18.6 + PostGIS + pgvector + optional TimescaleDB.
- A4: PostgreSQL metadata/authority projection + Qdrant vector index.
- A5: PostgreSQL metadata/authority projection + Vespa hybrid search.
- A6: XTDB bitemporal pilot compared with PostgreSQL's explicit valid-time/knowledge-time model.
- A7: Holochain canonical DHT records + PostgreSQL projection + vector search projection.
- A8: DuckDB + Parquet (and later Iceberg) analytical path, evaluated separately from OLTP.

Do not run all candidates as permanent infrastructure. The benchmark is a decision instrument.

### 11.2 Workload tracks

**Temporal/evidence**
- Overlapping amendments, partial clause replacement, repeal, scope exceptions and split/merge.
- Queries by valid_at and known_at.
- Delayed source acquisition, delayed materialization and delayed replica visibility.
- Rights revocation after indexing, cache invalidation and descendant/bridge rebuild.
- Provenance family and source-span integrity.

**Vector/hybrid**
- 768/1536/3072-dimensional text embeddings.
- 16,384-dimensional continuous HDC.
- 16,384-bit binary HDC where the representation is genuinely binary.
- Dense + BM25 + HDC candidate fusion.
- Filter selectivity strata: 0.1%, 1%, 10%, 50%.
- 100k, 1m and 10m-point scale tiers, subject to machine budget.
- Compare approximate results against exact exhaustive top-k and exact eligible-candidate sets.

**Operational and time-series**
- Burst ingest, steady ingest, corrections, deletes/revocations, restarts and recovery.
- Query by asset/entity, time window and tenant/authority.
- Write amplification, index maintenance and retention.
- Symtropy simulation checkpoint/replay and telemetry batch ingestion.

**GIS**
- Point-in-polygon, nearest infrastructure, line intersection, buffer/coverage and spatial joins.
- H3 rollups at declared resolutions.
- Network shortest-path / constrained routing for transport and utilities.
- Historical geometry queries and source dataset replacement.

**Local/analytics**
- Local SQLite durability and restart/replay for one Symthaea node.
- DuckDB query over Parquet versus loaded analytical tables.
- Reproducibility of experiment manifests and artifact digests.

### 11.3 Required metrics

Report separately:
- nDCG@k / Recall@k / MRR where relevance labels apply;
- exact ANN Recall@k against exhaustive top-k;
- eligible-candidate recall after all hard constraints;
- false-negative rate by selectivity/filter mode;
- false-eligibility and post-revocation payload leakage;
- temporal version correctness and historical replay parity;
- source-span/provenance/lineage completeness;
- time-to-invalidation and projection lag;
- p50/p95/p99 latency and throughput;
- ingest/update/rebuild time;
- memory, resident set, storage and index size;
- backup/restore and crash/restart behaviour;
- cost per million ingested items and per thousand qualified queries;
- operator burden, deployment complexity and recovery time.

### 11.4 Qualification gates

Deterministic safety gates:
- Zero ineligible/unauthorized representations admitted in frozen controls.
- Zero Unknown visibility promoted to Visible.
- Zero payload returned after a required rights revocation.
- Zero invalid source-span/version bindings.
- 100% agreement with independent temporal/version oracle on frozen fixtures.
- 100% deterministic replay/result-digest parity for frozen test cases.

Performance gates:
- Do not set p95 or throughput targets until the reference machine, corpus, query mix and retention policy are frozen.
- Compare at equal candidate budgets, exact vector representation, filter selectivity, warm/cold cache conditions and resource budgets.
- Require a measured Pareto improvement before adding a separate database.
- Preserve non-regression controls; improved ANN recall is not useful if rights or temporal correctness deteriorates.

## 12. Implementation sequence

1. **Inventory and ownership registry:** enumerate each existing DB/file/DHT/embedded store, its authoritative aggregates, owners, schemas, retention and backups.
2. **Pin PostgreSQL 18.6 + pgvector 0.8.7 local development service:** establish the relational baseline; no production migration yet.
3. **Define shared IDs and projection envelopes:** canonical artifact, statement/version, source span, provenance family, rights decision, transform, index revision and source frontier.
4. **Add SDSQ fixtures:** temporal/rights/filtered ANN/GIS/telemetry data with exact oracles and negative controls.
5. **Benchmark pgvector vs Qdrant for Symthaea HDC:** binary 16K and continuous 16K separated; include Qdrant payload-filtered recall and pgvector bit-index tests.
6. **Benchmark Vespa only if hybrid ranking complexity warrants it.**
7. **Build Mycelix PostgreSQL projection from Holochain with replay/idempotence and explicit DHT observation status.** Do not make Postgres a competing authority.
8. **Define Symtropy checkpoint and telemetry event envelope; keep database work out of the physics loop.**
9. **Define Sol-Atlas PostGIS schema/API and spatial benchmark; keep frontend database-agnostic.**
10. **Evaluate TimescaleDB / XTDB / Iceberg only after the baseline reveals a workload gap.**
11. **Publish exact-head result packets:** source digests, database versions, extension versions, configs, workload manifests, raw per-query outputs and failure cases.

## 13. Risks and open decisions

- Holochain upgrades are not database migrations. The current public Mycelix workspace pin and the upstream recommended release must be reconciled separately with a real data/backup plan.
- PostgreSQL extension compatibility must be checked against exact PostgreSQL/OS versions and the Nix deployment.
- TimescaleDB and other extensions require a current license and product-tier review before adoption.
- Qdrant/Vespa indexes are derived data and must be rebuildable from authoritative records and transform manifests.
- High-dimensional HDC similarity semantics must be measured before quantization or projection.
- Personal/health/identity data must not be copied to broad analytics/vector indexes without a specific lawful, consented and technically enforceable access design.
- Serving-replica visibility may be opaque in managed search products; Unknown must remain Unknown.
- A single shared database server can create a cross-project blast radius. Logical architecture does not imply one shared production cluster.

## 14. References

### Database and storage documentation
- PostgreSQL 18 release notes: https://www.postgresql.org/docs/18/release.html
- PostgreSQL 19 RC release announcement / schedule: https://www.postgresql.org/about/news/postgresql-19-rc-1-released-3409/
- PostgreSQL RC production warning: https://www.postgresql.org/developer/beta/
- pgvector 0.8.7 security release: https://www.postgresql.org/about/news/pgvector-087-released-3392/
- pgvector capabilities and limits: https://github.com/pgvector/pgvector
- Qdrant filtering and payload indexes: https://qdrant.tech/documentation/search-patterns/vector-search-filtering/
- Qdrant vector dimension limits: https://qdrant.tech/documentation/faq/qdrant-fundamentals/
- Qdrant Rust client: https://docs.rs/qdrant-client/latest/qdrant_client/
- Vespa nearest-neighbour search and hybrid retrieval: https://docs.vespa.ai/en/querying/nearest-neighbor-search.html
- Milvus filtered search: https://milvus.io/docs/filtered-search.md
- XTDB bitemporal model: https://docs.xtdb.com/intro/what-is-xtdb.html
- Holochain data/DHT model: https://developer.holochain.org/build/working-with-data/
- PostGIS spatial indexes: https://postgis.net/docs/using_postgis_dbmanagement.html
- InfluxDB 3 Core capabilities and limits: https://docs.influxdata.com/influxdb3/core/
- DuckDB Parquet querying: https://duckdb.org/docs/current/guides/performance/file_formats
- Apache Iceberg schema/partition evolution: https://github.com/apache/iceberg/blob/main/docs/docs/evolution.md

### Holochain versioning
- Holochain 0.7.1 release information: https://blog.holochain.org/dev-pulse-157-holochain-0-7-1-brings-chain-restore-direct-signal-security/
- Holochain 0.7 compatibility table: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Holochain 0.6→0.7 upgrade guide: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/

---

**Decision requested:** review the logical boundaries and approve the SDSQ benchmark plan. No new production database, service or extension should be deployed solely because it appears in this candidate architecture.
