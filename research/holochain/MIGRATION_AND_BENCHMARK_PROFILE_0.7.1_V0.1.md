# Mycelix Holochain 0.7.1 Migration and Qualification Profile

**Status:** proposed target profile; not yet qualified  
**Snapshot date:** 2026-11-02  
**Scope:** migration from the repository's Holochain 0.6 family to the current stable 0.7 line

## Executive decision

Treat **Holochain 0.7.1 as the preferred evaluation candidate, not yet the approved deployment baseline**. The stable 0.7.1 release exists, and the current 0.6→0.7 upgrade guide specifies the 0.7.1 dependency tuple; however, the published 0.7 compatibility table still explicitly calls 0.7.0 the recommended version. This is a real upstream documentation/release-policy discrepancy. The project must resolve it against exact release tags/artifacts and, where needed, upstream maintainer clarification before promoting 0.7.1 as the production baseline. Treat current 0.7.0 work as a migration intermediate/canary, not as automatically qualified. Do not target Holochain 0.8 development builds for production qualification.

**Candidate version floor (verify from locked artifacts in every workspace):**

| Component | Candidate pin | Qualification rule |
|---|---:|---|
| Holochain conductor / Rust core | 0.7.1 | exact binary and source tag |
| HDK | 0.7.1 | exact Cargo pin |
| HDI | 0.8.1 | exact Cargo pin |
| `holochain_serialized_bytes` | 0.0.57 | exact compatibility pin |
| JavaScript client | 0.21.1 | lockfile and client E2E |
| Lair keystore | 0.7.1 | exact runtime version |
| Kitsune2 bootstrap | 0.5.0 | exact service/runtime identity |
| Holonix | `main-0.7` line | pin the resolved flake input revision in `flake.lock` |
| Node.js | 24 | use where required by the upstream 0.7 guide |
| `hc-spin` | 0.700.1 | only where the project uses hc-spin |
| Rust client | resolve and pin the exact compatible release | do not infer from a stale compatibility table |

The table is a **candidate profile**, not proof that every crate or package listed is the newest compatible release. The qualification workflow must record the exact resolved package graph, binaries, source tags, and hashes.

## Live repository baseline found

The default branch is not globally on 0.7:

- `mycelix-workspace/flake.nix` references Holonix commit `d21b3543`.
- `mycelix-workspace/flake.lock` resolves the Holochain input to ref `holochain-0.6.0`.
- The root README quick start still advertises Holochain 0.6.0.
- The Governance and Fabrication Cargo manifests inspected on `main` still contain 0.6-family pins.
- The consumer qualification census (#3617) records multiple independent 0.6 consumers and 0.7 canaries; it is an open work item, not a completed repository-wide migration.
- Governance 0.7 assembly PR #3627 is draft/unqualified. Pulse migration PR #4891 is also draft/unqualified. Their existence is not evidence that `main` or a deployed conductor is on 0.7.

Consequently, do not describe Mycelix as “migrated to Holochain 0.7” until every active consumer has an explicit qualified-or-isolated disposition and all required runtime gates have evidence.

## Canonical source and exporter boundary

The standalone public repository is not the only source boundary. The private umbrella repository `Luminous-Dynamics/luminous-dynamics` contains the active exporter `mycelix-workspace/scripts/sync-to-standalone.sh`. That script exports committed monorepo HEAD content to `Luminous-Dynamics/mycelix`, maps cluster directories such as Governance into the public tree, rewrites selected path dependencies, and also syncs the workspace subtree.

The canonical monorepo Governance mirror was inspected on 2026-11-02 and still declares the 0.6.1-era tuple (HDK 0.6.1 / HDI 0.7.1 / Holochain types 0.6.1). The monorepo ecosystem Sweettest package still declares `holochain = "0.6"`. Pulse also contains a deliberate exception in `mycelix-workspace/mycelix-pulse/crates/pulse-smtp-gateway/Cargo.toml`: its optional bridge path pins `holochain_client = 0.9.0-dev.10` and `holo_hash = 0.6` for a pre-release federated-gateway track. The source comments explicitly say this is an intentional exception, not accidental drift. Classify it as an isolated optional client boundary and qualify that feature separately; do not blindly force it onto the zome workspace's pin. This means a standalone-only 0.7 migration can be overwritten by a normal monorepo export. Existing canonical-source work is tracked in `Luminous-Dynamics/luminous-dynamics#3478`.

Every migration tranche must therefore record and verify the three-way relationship:

`canonical monorepo source SHA + path`
→ `export transform + sync-script SHA`
→ `standalone path + content digest`.

The migration must update canonical source first (or in the same reviewed change set), then prove the exported tree is equivalent after documented path rewrites. Add a regression guard that fails if a subsequent export downgrades a migrated workspace. Do not run the exporter as a migration tool while its canonical source remains 0.6. Do not qualify a standalone-only branch as reproducible canonical source.

## Compatibility boundaries that must remain explicit

### 0.6 → 0.7

The upstream migration guide identifies three system-level changes:

1. The action model changes from per-variant action structs to an `Action` composed of a common header and typed data. Integrity `validate` callbacks and coordinator `signal_action` consumers must be audited.
2. tx5/WebRTC are removed; Iroh over QUIC becomes the transport. Remove obsolete conductor configuration such as `signal_url` and `webrtc_config`; do not assume WebRTC network modes remain available.
3. There is no in-place 0.6 conductor database migration. DNA hashes change and the database layout changes. A 0.6 conductor database must not be pointed at a 0.7 binary.

The operational replacement is **semantic export/import**, not database-file conversion:

`0.6 source data → versioned export → validation/reconciliation → 0.7.1 fresh install → import as new authored records`.

The export/import layer must preserve original authorship, timestamps, signatures, provenance, consent, revocation, and source references as data where technically and legally appropriate. It must not fabricate original 0.7 action hashes or imply that old 0.6 actions were authored on the new network.

### 0.7.0 → 0.7.1

This is a second migration boundary, not a harmless patch bump:

- Capability-grant types and serialization change. Grants already written on a 0.7.0 source chain cannot be read by 0.7.1 and must be re-issued.
- Direct signals require an explicit `Capability::DirectSignal` grant, including signals to the same agent.
- Direct-signal wire encoding is incompatible between 0.7.0 and 0.7.1 conductors. A 0.7.1 conductor drops 0.7.0 direct signals and vice versa.
- `Signal::AppDirect` carries `from_agent`.
- The JS client chain-op types are aligned to the 0.7 serialization; test code that reads `ChainOp` / `OpEntry` or calls `getChainOpAction`, `getChainOpEntry`, or `getChainOpSignature` must be audited against 0.21.1.

Therefore a 0.7.0 canary must not be silently promoted to a 0.7.1 deployment. Re-issue capability grants, test direct-signal grants and denial paths, and treat mixed-version direct signaling as unsupported.

## Migration programme

### Workstream A — closed consumer census

Extend the existing #3617 census into a generated, machine-readable matrix across both the private canonical monorepo and public standalone export. Discover every Cargo workspace root and every active Holochain-consuming package, including nested and excluded workspaces, isolated hApps, test harnesses, SDKs, Nix flakes, JavaScript packages, build scripts, manifests and CI workflows. Record exporter path mappings and compare the resulting canonical and standalone source digests after explicitly allowed path rewrites.

Each row must include:

- canonical workspace root and owning hApp/cluster;
- active, archived, generated, fixture, vendored, or isolated classification, with evidence;
- direct and transitive Holochain dependency closure;
- exact `hdk`, `hdi`, `holochain`, `holo_hash`, type-crate and serialized-bytes versions;
- Rust, Nix/Holonix, Lair, Kitsune2, JS client, Rust client and CLI versions where applicable;
- manifest and lockfile digests;
- 0.6 / 0.7.0 / 0.7.1 / other status;
- compile, WASM, DNA-pack, conductor, Sweettest and independent-client evidence;
- exact source SHA and qualification run identity;
- disposition: migrate, isolate, archive, or remove;
- unresolved/unknown consumers.

**Invariant:** `repository_07_mergeable = every_active_consumer_qualified_or_isolated && unknown_consumers == 0`.

A path under `mycelix-workspace/` is not automatically an active consumer, and a path outside it is not automatically isolated. Resolve the actual workspace and build graph.

### Workstream B — exact substrate closure and source parity

- Treat canonical monorepo source and standalone export as one source lineage with two independently auditable Git heads.
- Before changing a standalone migration target, identify whether it is generated from the monorepo and migrate/guard the canonical source as well.
- Run exporter parity checks that bind monorepo HEAD, sync-script digest, standalone HEAD, path rewrites and output digests.



- Freeze the candidate 0.7.1 profile in one machine-readable source of truth.
- Pin Holonix and all Nix inputs by resolved revision and lock hash.
- Regenerate Cargo and JS lockfiles with their native tools; never hand-edit generated checksums.
- Audit feature flags and target-specific dependencies, not only package version strings.
- Reject a build if Cargo, Nix, CLI, conductor, test runtime, and client identities disagree.
- Prove that no 0.6 package is pulled into a 0.7.1 runtime closure except in an explicitly isolated legacy test/tool boundary.

### Workstream C — API and semantic migration

For every consumer:

- Migrate all 0.6 `FlatOp` variants and per-action structs to the 0.7 typed-action model.
- Audit both integrity validation and coordinator action/signal consumers.
- Replace direct struct-field reads with the appropriate header/data accessors.
- Recheck create/update/delete/link/agent-activity dispatch and update-chain predecessor semantics.
- Audit capability grants, direct signals, client serialization, conductor configuration, network assumptions and removed host APIs.
- Preserve the pre-migration accept/reject boundary with differential tests. A migration must not quietly introduce a new authorization, governance, identity, or economic claim.

### Workstream D — real runtime qualification

Compilation is only the first rung. Qualify the exact source head through:

1. source and dependency guards;
2. native workspace tests and Clippy;
3. WASM compilation for every declared zome;
4. DNA/hApp packaging from those exact WASM artifacts;
5. manifest, zome-name and dependency-topology verification;
6. fresh 0.7.1 conductor installation;
7. multi-agent Sweettest on the packed DNA;
8. independent Rust/JavaScript client E2E;
9. restart, backup/restore and export/import tests;
10. sealed evidence receipt binding source SHA, toolchain, lockfiles, artifact hashes, conductor version and test results.

Keep the excluded Sweettest workspace in the qualification chain. A passing root workspace does not establish that the real hApp can be packed, loaded or exercised.

### Workstream E — user data and network transition

For any real 0.6 data or 0.7.0 canary data:

- Inventory the data classes and authority for each class.
- Define a versioned export schema and deterministic digest.
- Preserve source-chain provenance as provenance; do not counterfeit new-network authorship.
- Explicitly classify data that cannot or must not be migrated (including secrets, capability grants, revoked credentials, expired permissions and private material).
- Require owner consent and privacy policy for personal/private data.
- Import into a fresh 0.7.1 conductor and reconcile counts, digests, signatures and links.
- Test interruption, retry, duplicate import, partial failure and idempotent resume.
- Keep old network read-only during cutover where possible; publish a signed cutover manifest and a rollback/containment plan.

No live user's conductor database should be cleared as part of a developer migration test.

## Paradigm shifts

### 1. From version bump to proof-carrying substrate

A release should carry a machine-verifiable compatibility statement: exact source commit, workspace roots, resolved dependency graph, Nix derivations, lockfiles, WASM digests, DNA hashes, conductor identity, client versions, runtime tests and explicit non-claims. Consumers inherit a substrate claim only through evidence, never through directory layout or naming.

### 2. From database migration to semantic continuity

Because 0.6→0.7 has no in-place conductor DB migration, build a semantic continuity layer: canonical export schemas, provenance-preserving import, content-addressed manifests, idempotent replay and independently checked reconciliation. This turns a hard runtime break into a controlled application-level transition without pretending histories are identical.

### 3. From “works on my conductor” to a multi-agent protocol laboratory

Run deterministic scenarios across independent conductors and clients. Inject partitions, delayed DHT visibility, duplicate delivery, restarts, stale capabilities, malformed records, competing updates, invalid validation warrants and version skew. Record both safety (what must never happen) and liveness (what must eventually happen under stated assumptions).

### 4. From test counts to adversarial coverage and calibrated claims

Count scenario families, negative controls, state transitions, network topologies, fault schedules and reproducible seeds—not just unit tests. Every result should state its claim ceiling and distinguish a local projection from conductor-enforced authorization, a successful publish from DHT convergence, and a valid receipt from semantic correctness.

### 5. From raw TPS marketing to workload-equivalent systems science

Holochain is an agent-centric source-chain/DHT architecture, not a globally ordered blockchain. Do not compare raw transactions-per-second with Ethereum or a replicated database as if the semantics were identical. Compare equivalent operations, consistency requirements, hardware, network topology, payload sizes, security checks and failure conditions.

## Benchmark programme and proposed targets

All numerical values below are **qualification targets**, not achieved results. Record the hardware, network, workload generator, seed, version, payload sizes and statistical interval for every measurement.

| Benchmark | Measurement | Initial target |
|---|---|---|
| Consumer closure | Active Holochain workspaces classified | 100%; zero unknown consumers |
| Version coherence | Dependency/runtime/client mismatch | 0 unexplained mismatches |
| Behavior preservation | Differential accept/reject corpus | 100% parity for in-scope rules |
| Security negative controls | Unauthorized, replayed, malformed or stale inputs accepted | 0 false accepts in the defined corpus |
| WASM/DNA integrity | Declared zomes, dependencies and artifact digests | Exact-set match; no missing/extra zomes |
| Build reproducibility | Rebuilds from identical pinned inputs | Identical artifacts where reproducibility is supported; otherwise explain variance and never claim deterministic builds |
| Data continuity | In-scope export/import records | 100% count + digest reconciliation; explicit exceptions |
| Retry safety | Duplicate/interrupted import effects | At-most-once effect per idempotency identity |
| Network convergence | Publish-to-observation p50/p95/p99 | Establish 0.6 and 0.7.1 baselines; then target ≥20% p95 improvement without safety regression |
| Partition recovery | Convergence after controlled partition heals | 100% valid in-scope records converge under declared assumptions; 0 invalid records accepted |
| Capability correctness | Positive, negative, revocation and delayed-visibility tests | All specified grants enforced by conductor; stale app projections never override denial |
| Resource efficiency | CPU, RSS, disk, bandwidth and joules per accepted operation | Establish matched baseline; target ≥20% improvement in at least one resource dimension with no material regression elsewhere |
| Client interoperability | Rust/JS clients against fresh 0.7.1 conductors | 100% supported protocol cases; incompatible version pairs explicitly rejected |
| Operational recovery | restart, backup/restore, interrupted import | 100% of declared recovery scenarios meet RPO/RTO policy |

For performance, run at least 2, 10, 50 and 100 simulated agents where hardware permits, then expand only after bottlenecks are understood. Report p50/p95/p99 latency, throughput, convergence lag, message/byte volume, CPU, RSS, disk and energy. Include cold-start and warm-cache runs, small and large entries, and both normal and adversarial traffic.

### Comparator set

- **Holochain upstream Wind Tunnel:** the primary substrate-specific baseline. Re-run equivalent scenarios on 0.6 and 0.7.1 where the same semantics can be represented.
- **Local-first/CRDT systems (for example Yjs or Automerge):** compare offline editing, conflict resolution and convergence, while separately testing Holochain's integrity validation and authored source-chain semantics.
- **P2P transport stacks (for example libp2p):** compare transport establishment, churn, partition healing and message delivery under the same topology; do not treat transport throughput as application correctness.
- **Replicated databases (for example CouchDB):** compare operational recovery, replication lag and query workload where the data model and consistency level can be matched.
- **Blockchain systems:** compare only explicitly specified trust/consensus and settlement workflows. Do not present an agent-centric DHT operation as equivalent to a globally ordered transaction.

Use matched hardware and topology, pinned software releases, repeated seeded trials, confidence intervals, and public raw result manifests. The “better than SOTA” claim is earned only by an independent, reproducible head-to-head result on a named workload—not by architecture descriptions or one synthetic fixture.

## Exit criteria

Mycelix may claim **Holochain 0.7.1 qualified** only when:

- upstream compatibility guidance and the 0.7.1 release tuple discrepancy is resolved, or the project explicitly records a 0.7.0 hold;
- canonical monorepo source, exporter transform, and standalone output have exact-head parity for every mirrored consumer;

- all active consumers are classified and every consumer is either 0.7.1-qualified or demonstrably isolated;
- all 0.6 and 0.7.0 canaries are labelled with their exact status and network boundary;
- exact source/toolchain/dependency/artifact identities are recorded;
- every required zome builds to WASM and every target DNA/hApp packs;
- real 0.7.1 conductor and multi-agent Sweettest pass on exact-head artifacts;
- independent client E2E and capability/direct-signal tests pass;
- export/import reconciliation and recovery tests pass for any data being transitioned;
- all negative controls reject as expected;
- independent review has checked the evidence and claim ceilings.

Queued, pending, skipped, cancelled, or superseded runs are not PASS. No CI result may be carried from a parent SHA to a changed head.

## Upstream references

- [Holochain 0.6→0.7 upgrade guide, including 0.7.0→0.7.1 breaking changes](https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/)
- [Holochain releases](https://github.com/holochain/holochain/releases) — 0.7.1 stable release published 2026-10-14; 0.8 development releases are prereleases.
- [Holochain 0.7 compatibility table](https://developer.holochain.org/resources/compatibility/holochain-0.7/) — note the observed 0.7.0 recommendation and reconcile it against the newer 0.7.1 upgrade instructions before final pin approval.
- [Holonix](https://github.com/holochain/holonix) — versioned `main-0.7` development environment.

## Related Mycelix work

- #3534 — Holochain 0.7 toolchain baseline
- #3617 — closed consumer qualification matrix
- #3627 — Governance 0.7 integrity assembly
- #4246 / #4248 — Fabrication 0.7 migration and actual hApp/Sweettest qualification
- #4317 — Finance substrate truth
- #4371 — Fabrication authenticated-anchor resolver action-model port
- #4891 — Pulse Holochain 0.7 migration canary
- #4929 / #4903 — Pulse exact-head workflow and delayed-DHT capability currentness
- #5245 / #5293 — CivOS multi-agent correctness/resilience benchmark and executable corpus

**Claim ceiling:** This document defines a migration target and qualification programme. It does not assert that any Mycelix workspace has migrated, that any current head passes, that a 0.6 database has been migrated, or that Mycelix outperforms any industry comparator.
