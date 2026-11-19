# Mycelix Platform Stewardship and Topology Governance v0.1

**Status:** Research proposal; review required  
**Tracking:** [PLAT-STEWARD-001](https://github.com/Luminous-Dynamics/mycelix/issues/6322)  
**Scope:** Public-repository proposal only. This file does not replace the canonical monorepo architecture map or authorize runtime changes.

## Executive decision

Do not create another application cluster or DNA in this tranche.

Establish Platform Stewardship as an operator-side control plane that inventories and qualifies Mycelix deployments. It observes and reports; it does not become an authority over identity, consent, governance, finance, property, health or other application records.

The first deliverable is not a new hApp. It is a reliable way to answer, with evidence:

- Which hApp profiles and DNA roles are actually deployed?
- Which source commit and manifest generated each role?
- Which Holochain, HDK, HDI, client, Lair, Kitsune2, Rust and Nix inputs form the effective runtime?
- Which cross-role calls are valid in the selected hApp topology?
- Which operations remain safe during partition, revocation, restart, restore and migration?
- What has actually been tested on the exact source head, and what remains unknown?

## Repository observations

The public Mycelix main branch inspected for this proposal was:

- Repository commit: `f51334e48f686fc294a423180c2f7f3fd9b2d283`
- Unified hApp manifest: `mycelix-workspace/happs/mycelix-unified-happ.yaml`
- Unified manifest Git blob: `c375b9f9851315ee9773cbd609273c8300491d34`
- Resilience manifest: `mycelix-workspace/happs/mycelix-resilience-happ.yaml`
- Resilience manifest Git blob: `643c399ff4229a6e3a06ba3f8dde8887bbe37248`

The unified manifest contains 19 DNA roles. The resilience profile contains 8 roles. These are deployment compositions, not 19 or 8 new application clusters.

The unified role list is:

| Role | DNA artifact path in unified manifest |
|---|---|
| personal | `./personal/dna/mycelix_personal.dna` |
| identity | `./identity/dna/mycelix_identity_dna.dna` |
| hearth | `./hearth/dna/mycelix_hearth.dna` |
| commons_land | `./commons/dna/mycelix_commons_land.dna` |
| commons_care | `./commons/dna/mycelix_commons_care.dna` |
| civic | `./civic/dna/mycelix_civic.dna` |
| attribution | `./attribution/dna/mycelix_attribution_dna.dna` |
| finance | `./finance/dna/mycelix_finance.dna` |
| governance | `./governance/dna/mycelix_governance_dna.dna` |
| health | `./health/dna/health.dna` |
| supplychain | `./supplychain/holochain/dna/supplychain.dna` |
| manufacturing | `./manufacturing/dna/manufacturing.dna` |
| music | `../../mycelix-music/dnas/mycelix-music/mycelix_music.dna` |
| energy | `../../mycelix-energy/dna/mycelix_energy.dna` |
| knowledge | `../../mycelix-knowledge/dna/mycelix_knowledge.dna` |
| climate | `../../mycelix-climate/dnas/climate/workdir/climate.dna` |
| praxis | `../../mycelix-praxis/dna/praxis.dna` |
| craft | `../../mycelix-craft/dna/mycelix_craft.dna` |
| core_fl | `../../mycelix-core/dna/mycelix_core.dna` |

The resilience profile lists identity, finance, commons_care, civic, governance, hearth, knowledge and supplychain.

There is documented topology drift to resolve: the legacy `mycelix-core/architecture/clustermap.yaml` describes only Sovereign, Civic and Commons tiers and lists Knowledge and Finance as planned, although those roles are present in the unified hApp. The unified manifest description also mentions Marketplace, while the inspected 19-role list contains no `marketplace` role. Treat this as an audit finding; do not silently infer or mutate deployment topology from descriptive prose.

## Architecture vocabulary

Keep these entities separate in code, manifests and evidence:

1. **Application domain** — a bounded problem space such as property, housing, justice, food or energy.
2. **Logical subcluster** — a group of domains sharing a trust, privacy or operational boundary.
3. **DNA** — a Holochain validation and DHT boundary.
4. **hApp role** — a named DNA installation slot inside a particular hApp manifest.
5. **Deployment profile** — a composition of roles selected for a use case, such as unified or resilience.
6. **Service / projection** — an API or read model that may project validated source data but is not automatically an authority.
7. **Authority boundary** — the exact principal, policy, evidence and state transition that can authorize a mutation.

A role name, directory name, branch, issue, PR, passing unit test or deployment profile does not itself confer authority.

## Platform Stewardship: proposed responsibilities

Platform Stewardship should initially be an operator-side control plane with read-only observation by default.

### 1. Architecture registry and topology auditor

- Parse the actual hApp manifests rather than maintaining a parallel hand-written list.
- Emit a versioned snapshot with source commit, manifest path and Git blob digest.
- Record role names, DNA artifact paths, clone limits, provisioning strategy, modifiers and profile membership.
- Mark unclassified trust/privacy/authority fields as unknown rather than guessing from names.
- Compare every statically discoverable OtherRole target with the roles in the selected hApp profile.
- Distinguish same-hApp OtherRole calls from client/orchestrator-mediated calls between separately installed hApps.
- Report description/role mismatches, missing bundle artifacts and role-path drift.

The manifest remains the deployment source of truth. A generated registry is a verifiable index, not a second authority.

### 2. Runtime and compatibility inventory

- Record exact conductor, hc, HDK, HDI, Holochain types, Rust client, JavaScript client, Lair, Kitsune2 and bootstrap/relay inputs.
- Close Cargo, JavaScript and Nix dependency graphs; record generated lockfile digests.
- Detect unclassified consumers and unsupported mixed-version graphs.
- Bind every positive qualification result to a source SHA, environment identity, artifact digests and exact workflow/job result.
- Distinguish source-audited, compiled, WASM-built, DNA-packed, conductor-loaded, multi-agent-tested and client-E2E-qualified states.

### 3. Network and DHT observability

Observe, without claiming more than the instrumentation proves:

- peer discovery and reachability;
- DHT integration/convergence by record class and expected authority set;
- validation queue depth and validation outcomes where exposed;
- signal delivery, sender identity and capability failures;
- network partitions, reconnects and peer catch-up;
- resource consumption and operational health.

A network-health score must not be used as an authorization substitute. A peer can be reachable but untrusted; an unavailable peer can make a decision not evaluable.

### 4. Recovery and migration stewardship

- Separate private source-chain data, public DHT data, key material, local projections and external-system receipts.
- Define application-level semantic export/import for 0.6-to-0.7 data movement.
- Test the 0.7.1 public source-chain restoration path separately from application export/import and private-data backup.
- Preserve identity/key continuity requirements, record lineage and source provenance.
- Support dry-run migration rehearsals, reconciliation and explicit cutover/rollback decisions.
- Never represent an uncertain external outcome as failure or success; reconcile before retrying a potentially mutating operation.

### 5. Qualification receipts

A receipt should include at minimum:

- exact repository, source commit and source tree/blob digests;
- exact hApp manifest and DNA artifact hashes;
- exact toolchain, lockfile and environment inputs;
- scenario IDs, workload seed, topology and time bounds;
- raw trace and artifact digests;
- independent verifier identity/version;
- per-gate status and failure classification;
- explicit limitations and claim ceiling.

The status vocabulary must keep pending, queued, skipped, cancelled, failed, superseded and exact-head passed distinct.

## Cluster and subcluster decisions

### Keep

- Commons Land and Commons Care as separate DNA roles. The existing split is a useful physical-infrastructure versus social/care boundary and already has a deployment representation.
- Civic as a bounded application DNA for now, with independently specified logical domains.
- Personal as the source-chain/private-data boundary; never assume that records in this DNA are publicly replicated.
- Existing Identity, Governance, Finance, Hearth and other specialized roles as separate hApp roles where the manifests and existing trust boundaries warrant them.

### Formalize logically before splitting DNA

Civic should have three explicit logical subdomains:

- **Justice:** cases, evidence, arbitration, restorative processes, appeals and enforcement.
- **Emergency:** incidents, triage, resources, shelters, dispatch and emergency communications.
- **Media:** publication, attribution, fact-checking, curation and provenance.

A separate DNA is justified only when measured privacy, validator population, governance, scaling, availability or failure-isolation requirements materially differ. A directory rename or organizational preference is insufficient.

### Defer

Do not create a dedicated Land Rights DNA yet. Continue the current Property / DeSci / Sol-Atlas references and qualify external authority, source currentness, spatial identity, challenges, exact registry receipts and legal-effect boundaries first. Preserve the distinction between an internal Mycelix record and an external legally effective registration.

Do not create a Platform Stewardship DNA yet. The control plane should first prove its read-only observation and qualification value. A DNA is a later option only if a genuine peer-authored, independently governed stewardship record domain emerges.

## Holochain migration boundary

The current migration programme already has the appropriate workstreams:

- #3617 — consumer qualification matrix;
- #5718 — 0.7.1 migration and benchmark profile;
- #5719 — explicit 0.7.1 target qualification;
- #4891 — earlier Pulse 0.7.0 migration canary;
- #5914 — substrate pin audit.

The migration should be a 0.7.1 qualification, not a blind bump to 0.7.0. Holochain's official 0.6-to-0.7 guide documents the Action/FlatOp model change, transition to Iroh/QUIC, DNA-hash changes and absence of an in-place conductor database migration. The 0.7.1 release notes additionally require explicit DirectSignal capabilities and account for 0.7.0/0.7.1 signal incompatibility and capability-grant changes.

There is an important upstream maturity distinction: the 5 November 2026 Dev Pulse says the 0.7.1 tools, libraries and documentation have been updated/tested and are ready for development, while the 17 November roadmap still classifies the 0.7.x line as beta and lists Wind Tunnel infrastructure, workflow tuning, core stability and local-first confirmation among ongoing work. Treat 0.7.1 as the correct migration candidate, not as an automatic production-readiness claim.

The 0.7.1 `restore_from_dht` path is not a complete backup strategy: upstream documents that it requires the old agent's private key in Lair, restores public data only, waits in `AwaitingRestore` until all cell chains restore, and then requires an explicit `enable_app`. Private source-chain data, key backup and application-level export/import remain separate recovery obligations.

Upstream references: [0.6 to 0.7 upgrade guide](https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/), [Dev Pulse 157](https://blog.holochain.org/dev-pulse-157-holochain-0-7-1-brings-chain-restore-direct-signal-security/), and [Holochain roadmap](https://www.holochain.org/roadmap/).

The Pulse 0.7.1 evaluation branch is a candidate, not a qualified runtime. The inspected manifest pins HDK/HDI 0.7.1/0.8.1, while the inspected Cargo.lock still resolves important Holochain packages to 0.7.0. Its Nix flake also references a mutable Holonix branch and a Kitsune2 tag that must be reconciled against the exact 0.7.1 profile before qualification. No exact-head compile, WASM, DNA pack or fresh-conductor result is claimed here.

Canonical source lineage matters: the public repository is synchronized from the private umbrella monorepo. Do not make a public-only change to canonical source files that the exporter can overwrite. This proposal is deliberately additive and public; canonical registry generation must be integrated with the exporter after source ownership is resolved.

## Exit criteria

- A machine-readable registry schema and manifest-derived snapshot with exact provenance.
- Automated comparison between generated registry and actual deployment manifests.
- A role-target audit for OtherRole calls and deployment profiles.
- A reviewed Platform Stewardship boundary with no application-authority escalation.
- An MCRB benchmark plan with frozen scenarios, metrics and claim rules.
- A separately tracked fail-closed implementation plan for the Net Steward Holochain transport placeholders.
- No new DNA until a measurable boundary justifies it.


## Cross-hApp topology audit: OtherRole is a local installation contract

Holochain `CallTargetCell::OtherRole(role)` is a bridge call to another cell in the same agent's installed hApp. It is not a global service lookup and does not call a separately installed hApp. The exact hApp manifest therefore determines whether a call target exists for a given deployment.

Holochain's documentation distinguishes bridge calls between cells within one hApp from client/orchestrator-mediated interaction with other applications. A client may bridge separate app APIs, but that is a different trust and correctness boundary.

This motivates [PLAT-TOPOLOGY-AUDIT-001 #6334](https://github.com/Luminous-Dynamics/mycelix/issues/6334), a source-aware Rust AST audit. It will map each `OtherRole` call site to its caller DNA and every deployment profile containing that DNA, then classify the target as same-hApp-valid, profile-dependent, cross-installed-hApp-invalid, missing, dynamic-unresolved or source-owner-unresolved.

The audit must not use a global list of all Mycelix role names. It must also not fail closed on an incomplete source-to-DNA map by pretending the map is complete. Report attribution coverage, unknowns and per-profile resolution before promoting the audit to a hard gate.

A concrete precursor is [#4249](https://github.com/Luminous-Dynamics/mycelix/issues/4249): the Fabrication hApp declares only the `fabrication` role while selected bridge/verification call sites target Identity, Commons and Knowledge. Those targets may exist in the unified hApp, but that alone does not make them reachable from the standalone Fabrication hApp.

The eventual registry receipt should bind caller path, caller DNA, hApp profile, exact target role/zome/function, source SHA and manifest digests. A source-level mock or successful compile is not runtime proof that the installed topology contains the target cell.

