# Maritime Commons Current-Main Owner API Census v0.1

**Audit date:** 9 November 2026  
**Purpose:** pin the code actually present on each owner repository's current `main` branch before writing cross-stack adapters.  
**Status:** source-tree inspection; no owner-specific maritime integration has been built or qualified by this census.  
**Parent:** Mycelix #5968 / PR #5969.

This census intentionally distinguishes:
- a current, buildable owner API;
- a generic substrate that may be composed;
- a historical proposal on a stale/diverged PR branch;
- a missing maritime-specific contract.

A source file on `main` is not automatically a stable public API, validated physical model, or production-ready integration. Git blob SHA-1 values below identify exact repository objects; they are not SHA-256 evidence digests or source-authenticity proofs.

## 1. Exact repository heads inspected

| Owner | Repository | Branch | Commit |
|---|---|---|---|
| Mycelix | `Luminous-Dynamics/mycelix` | `main` | `6a0d072d3b65f4a2cf08af3ae71e541788e33ae9` |
| Symthaea | `Luminous-Dynamics/symthaea` | `main` | `d11bc49866edb3213a68c308ce6045e5631b6c95` |
| Symtropy | `Luminous-Dynamics/symtropy` | `main` | `d7e7751fb7a65db37d5fb870c4c3d70b19a2fdd6` |
| Sol-Atlas | `Luminous-Dynamics/sol-atlas-leptos` | `main` | `5c097476e3c0d0d155cae0f04cd1de3680b6a3f6` |

All paths and blob IDs in this document are interpreted at the corresponding commit above.

## 2. Mycelix — commitment and hand-off authority

### Current status

The current `main` head predates the new cross-stack verifier and oracle branches. The following are reviewable PR-head artifacts, not merged/current-main APIs:

- #5972: `feat/maritime-handoff-verifier-v0-1`, exact qualified PR head `25e8768ae4c770744cdb309bc6a5db83bdc01d76`.
- #5980: `feat/maritime-cross-stack-golden-oracle-v0-1`, exact qualified PR head `8c256b50f5803f0da99bcb2c244ca928246f64c8`.

#5980's exact-head workflow run #42570868329 passed the #5972 verifier (31/31 tests), the independent oracle (15/15 tests), and 35/35 synthetic cross-owner dispositions. Those results qualify the two PR-head implementations and their declared synthetic fixtures only. They do not make those artifacts part of Mycelix main or establish owner outputs from the other repositories.

### Integration owner boundary

Continue to use Mycelix for:
- provider/community identity and local recognition;
- provider-owned offers, holds, commitments and effect receipts;
- capability leases and rights-bounded offline allocation;
- stay disposition, reconciliation and historical reconstruction.

Do not move provider authorization into a map, simulation, or cognitive decision type.

## 3. Symthaea — current observation and execution-evidence substrates

### Current files

| Current-main path | Blob SHA-1 | What it currently provides | Maritime integration limit |
|---|---|---|---|
| `crates/core/symthaea-core/src/observation.rs` | `1ac6d14ce1ac6cfdecbaea0f34b46b08ceef16d4` | Narrow `CognitiveObservation` trait; currently only `state_dimensions()` | Not a maritime observation graph, receipt, DecisionBrief or source-verification contract |
| `crates/core/symthaea-evidence-plane/src/lib.rs` | `540ff055b017990bf8174034d214a53ac7a5f37b` | Run-level declared-mechanism vs measured-counter integrity checks; `RunEvidence`, `EvidenceCounters`, expectations | Useful to test Symthaea's own execution path; not a source evidence ledger or claim authority |
| `crates/core/symthaea-evidence-plane/Cargo.toml` | `2b9fa9679e77353dae0be511039d2bae99479f7e` | Buildable crate manifest, depends on workspace Serde | No maritime-specific DecisionBrief dependency/API |

### Unpackaged source requiring reconciliation

The directory `crates/core/symthaea-observation-attestation/src/lib.rs` exists at current main (blob SHA-1 `c6799431bd8b6d4c9621294695e72c2d3aefb167`), but the inspected directory listing contained no `Cargo.toml`. Do not treat it as a buildable workspace crate or production dependency until the owner resolves its package/workspace status.

Its code comments also state that cryptographic attestation does not itself provide key resolution, authorization, revocation or substantive truth. Preserve those boundaries.

### Required Symthaea contribution

#11173 should define a small, buildable, read-only DecisionBrief contract under current main. It should reference Mycelix commitment/effect records, Sol-Atlas projections and Symtropy run manifests. Use the evidence-plane to check the declared/measured behavior of the reasoning/evaluation run; do not repurpose it as maritime source provenance.

## 4. Symtropy — current generic simulation substrate, no current maritime run contract

### Current files

| Current-main path | Blob SHA-1 | What it currently provides | Maritime integration limit |
|---|---|---|---|
| `crates/domains/luminous-sim-core/src/lib.rs` | `2f8adf20c0a27881fc72e65b21577d6138b36245` | Generic simulation config/module/report orchestration | Not a maritime scenario or provider-commitment authority |
| `crates/domains/luminous-sim-core/src/report.rs` | `155fa52509d507825b3ae526d61c58c12fb6a273` | `StandardizedReport` and `ReproducibilityInfo` with seed, tick count, version and config hash | Executive summary contains civilization-specific metrics; insufficient as a maritime feasibility report without a domain adapter |
| `crates/domains/luminous-sim-core/src/module_trait.rs` | `831955e3b8a70a4f4acf619b8ca62428b1a0da1d` | Generic module execution, metrics, warnings and self-assessment | Does not declare maritime physical model validity, CRS/datum, source lineage or claim ceiling |
| `crates/core/symtropy-core/src/lib.rs` | `2382bb0437cc51ab81599d9c5a341e8efdf3d317` | General Bevy physics/core re-exports | Generic kernel capability does not establish maritime physics qualification |

The current `crates/domains` listing has no maritime-specific domain crate. Historical maritime PRs #1642 and #1940 are materially behind/diverged from current main; their old scenario-manifest, S-100 profile and PCO evidence files must not be imported as though they were current owner APIs.

### Required Symtropy contribution

#4276 should define only a maritime-specific **run-result wrapper/manifest** around a current, identified deterministic execution. It must bind the exact scenario/input references, model capability census, configuration/seed/time-step, per-constraint dispositions, supported/unsupported physical semantics, output digest, evaluator and claim ceiling.

Do not re-use the current generic `StandardizedReport` as if it already means maritime feasibility; its fields are not sufficiently domain-neutral. Do not create a second simulator or event store.

## 5. Sol-Atlas — current layer metadata and geospatial rendering substrate

### Current files

| Current-main path | Blob SHA-1 | What it currently provides | Maritime integration limit |
|---|---|---|---|
| `sol-atlas-core/src/types.rs` | `131a14f5d1d035982ce31d6d8b3e87183cd63a6e` | `Layer`, `DataKind` (Observed/Curated/Scenario), layer-level `DataProvenance` | Static `source` and `snapshot_date`; not a dynamic source receipt/asset provenance graph |
| `sol-atlas-core/src/data.rs` | `a3248a5506ef3d6b33c4b450538c1067a7b3620d` | Typed JSON loaders into current `LoadedData` | No versioned maritime product profile or source-byte custody contract |
| `sol-atlas-core/src/geo.rs` | `629e708a87f627312cca2532df3215ee1da7b033` | Lat/lon rendering conversion, inverse conversion and haversine distance | Rendering geometry is not hydrographic CRS/datum transformation or navigation authority |
| `sol-atlas-core/src/visual_validation.rs` | `957a60ae7107400a325952b1e9a878308537e995` | PNG/render-frame visual checks | Visual correctness does not establish source, spatial or physical correctness |
| `sol-atlas-core/src/lib.rs` | `7407357e8e78a3b158065a825de5aacb857c886d` | Current module surface | No dedicated maritime source-reversible projection module |

Historical Sol-Atlas PRs #55/#58/#62 (provenance graph, source-attributed assertions, validation report) are divergent from current main and their modules are not present in the inspected current-main source tree. The geospatial asset foundation remains owned by #65; provenance/validation remain #54/#59. Do not create competing `AssetId`, `ArtifactId`, `ActivityId`, provenance graph or validation report types.

### Required Sol-Atlas contribution

#654 should first resolve/reuse the live #65 asset and #54/#59 provenance/validation owner contracts. Then add only a maritime profile/projection reference binding source product/version, dataset/cell, coverage, CRS, horizontal/vertical datum, temporal roles, uncertainty, rights and transformation lineage.

## 6. Cross-stack readiness classification

| Capability | Current evidence | Readiness |
|---|---|---|
| Mycelix local hand-off validation | PR #5972 exact-head CI pass; unmerged | Reviewable implementation |
| Mycelix cross-owner disposition oracle | PR #5980 exact-head CI pass on 35 synthetic fixtures; unmerged | Executable synthetic composition |
| Symthaea run integrity counter checks | Current-main buildable generic crate | Reusable generic substrate |
| Symthaea maritime DecisionBrief | No current-main maritime package identified in this census | Not implemented |
| Symtropy maritime run-evidence artifact | No current-main maritime domain crate identified in this census | Not implemented |
| Sol-Atlas maritime source-reversible projection | Current-main layer metadata is coarse; planned asset/provenance foundations unresolved | Not implemented |
| Real owner-native cross-stack fixture | None in this census | Not available |
| South African provider / DGT field integration | DGT source rights and semantics remain under #4003; provider authorizations not obtained | Not authorized / not performed |

## 7. Required gate before replacing synthetic golden-trace owner outputs

Do not replace the synthetic owner-output fixture tuples in Mycelix #5980 until each required owner delivers a current-main, exact-pinned artifact:

1. Sol-Atlas: one synthetic or permissioned source projection produced by its current owner-native asset/provenance types.
2. Symtropy: one deterministic run-result artifact produced by a current-main execution, with model capability and per-constraint dispositions.
3. Symthaea: one read-only DecisionBrief produced by a current-main implementation, including a positive and an abstain/human-review result.
4. Mycelix: one independently verified provider commitment/effect receipt and historical stay view.

Each artifact must carry its own repo commit, file/artifact identity, exact bytes digest, profile version and claim ceiling. The cross-stack oracle validates composition; it must not recreate any owner domain logic.

## 8. Explicit nonclaims

This census does not establish that any generic substrate is validated for maritime use, that any old PR implementation is integrated, that an external source is authentic or licensed, that a provider API exists, or that any South African government MSW/PCS interface is operational. It is a dated code-surface inventory used to prevent stale-contract and ownership mistakes.
