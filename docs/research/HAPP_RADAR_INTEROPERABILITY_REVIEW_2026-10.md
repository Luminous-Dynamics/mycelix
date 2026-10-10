# Mycelix / hAppRadar Comparative Interoperability Review
Date: 2026-10-10
Status: research findings and proposed engineering gates; source builds/tests were not run as part of this review.

## Executive finding

The strongest near-term improvement is not to increase the number of Mycelix clusters. It is to make the economic ledger and cross-hApp trust boundaries independently verifiable, then align interoperable parts with existing Holochain ecosystem vocabularies.

This review compared the public hAppRadar directory, hREA's ValueFlows implementation and migration documentation, ValiChord's commit/reveal design, and the current Mycelix root/Finance documentation and source.

## Evidence-based findings

### 1. Version compatibility is a hard boundary, not a packaging detail

- Mycelix's root README documents Holochain 0.6.0 / HDK 0.6.0 / HDI 0.7.0. The Finance workspace pins HDK 0.6.1 and HDI 0.7.1 in `mycelix-finance/Cargo.toml`.
- hREA's current in-repository release guide describes `happ-0.5.0-beta` on Holochain 0.7.x, with HDK 0.7.0, HDI 0.8.0, `@holochain/client` 0.21.x and the 0.700.x GraphQL adapter line. hREA documents breaking 0.6-to-0.7 changes in validation callback signatures, action wire format, and WASM `getrandom` configuration.
- hAppRadar's cached project metadata can describe a released version while repository docs describe a newer development/release line. Record the exact source ref, release artifact, conductor, HDK/HDI, client and adapter versions for every compatibility claim.

**Decision:** do not treat hREA zomes as drop-in dependencies for current Mycelix Finance. First prototype against a pinned artifact. Native DNA composition requires a deliberate compatibility/migration plan. A separate-process/API integration is a different trust and networking boundary and must be documented as such; it does not make 0.6 and 0.7 DHTs one network.

Sources:
- Mycelix root: https://github.com/Luminous-Dynamics/mycelix/blob/main/README.md
- Finance manifest: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/Cargo.toml
- hREA release and compatibility guide: https://github.com/h-REA/hREA/blob/sprout/docs/consuming-a-release.md
- hREA ValueFlows types: https://github.com/h-REA/hREA/blob/sprout/modules/vf-graphql-holochain/src/types.ts
- hAppRadar directory: https://happradar.com/projects

### 2. ValueFlows/REA is a useful interoperability vocabulary for economic facts

hREA's model gives applications shared concepts for agents, economic resources, commitments, agreements, processes and economic events. Its GraphQL adapter exposes relations such as a commitment being fulfilled by an event and an event affecting a resource.

Mycelix Finance already has application-specific concepts for SAP, TEND and MYCEL, Payment, Receipt, SAP balances, treasury contributions, and cross-hApp events. These are not interchangeable with ValueFlows concepts, but a carefully defined mapping would make economic facts easier to exchange with other applications.

**Decision:** build a mapping specification and a small read-only/export prototype before changing ledger semantics:

| Mycelix concept | Candidate interoperability representation | Guardrail |
|---|---|---|
| SAP balance/transfer | Resource specification + resource-affecting economic events | Preserve integer micro-SAP units, exact currency identity, and signed debit/credit semantics; do not use floating point for authoritative amounts |
| TEND time-credit obligation/settlement | Commitment plus fulfillment/settlement event, scoped to the relevant parties/community | Preserve zero-sum accounting, unit meaning (hours), credit limits, disputes and settlement evidence |
| MYCEL recognition/reputation | Agent-scoped attestations/claims or another explicitly non-transferable representation | Never map it into a transferable balance; distinguish evidence about reputation from the authority to compute a score |
| Commons pool/reserve | Scoped resource and events with policy metadata | Keep reserve floors, demurrage, mint caps and governance rules enforceable by Mycelix integrity validation |

This is a candidate mapping, not a claim that the external schema already expresses every Mycelix policy. ValueFlows should describe interoperable economic facts; Mycelix remains responsible for its own monetary-policy and authorization invariants.

Source: https://github.com/h-REA/hREA/blob/sprout/modules/vf-graphql-holochain/src/types.ts

### 3. ValiChord offers a useful pattern for claims that must be hidden until a common reveal point

ValiChord describes private local attestations bound to a commitment hash, publication of a content-free commitment anchor, deterministic serialization shared by seal and reveal paths, a fresh nonce, duplicate-seal guards, and reveal eligibility driven by network-observed phase state. Its source also explicitly separates what this proves (the committed statement was not changed after commitment) from what it does not prove (that the validator is correct).

Potential Mycelix applications are governance ballots, blind peer reviews, pre-registered evaluations, or contested claims where seeing another participant's answer early would bias or enable adaptation. This pattern is not automatically appropriate for every payment receipt: adding a reveal phase to an ordinary payment may add latency without a clear privacy or integrity benefit.

**Decision:** write a protocol-level design and adversarial tests before reusing code. Require:
- Canonical, versioned and domain-separated commitment preimages.
- A cryptographically random nonce generated by a trusted platform source; never publish the nonce before reveal.
- The same serialization/hash routine on both sides of the boundary.
- Integrity validation that rejects mismatched reveals and duplicate or phase-invalid operations.
- Idempotent retry behavior when a commitment anchor write/cross-zome call fails.
- Multi-agent tests for early reveal, tampered payload, replay, duplicate commit, missing participant, network delay/dropout and conflicting outcomes.
- Clear trust assumptions for any credential issuer, quorum, Sybil-resistance scheme or governance override.

**Important external-project caveat:** ValiChord documents a permissioned validator membrane and a single trusted certificate issuer in its current phase, with issuer federation/rotation planned. Its README also notes its hosted demo runs Holochain 0.6.2 while `main` targets 0.7.0. Treat its described architecture as a source to inspect, not as evidence that Mycelix should inherit those trust assumptions or that its demo verifies the exact current branch.

Sources:
- https://github.com/ValiChord/ValiChord
- https://github.com/ValiChord/ValiChord/blob/main/docs/15_How_a_Validation_Round_Works.md
- https://github.com/ValiChord/ValiChord/blob/main/valichord/dnas/validator_workspace/zomes/validator_workspace_coordinator/src/lib.rs

### 4. A source comment records an unresolved SAP balance-conservation step

In `mycelix-finance/zomes/payments/integrity/src/lib.rs`, `SapBalance.justified_by` is documented as a field intended to bind a balance delta to a mint record or counterpart payment. The same comment says it is **not yet enforced** by integrity validation and producers currently write `None`.

This is a directly visible implementation gap, not a conclusion inferred from test counts. Before making strong claims about conservation of SAP, the implementation must prove which operation justifies every non-genesis positive delta and reject unmatched increases.

**P0 gate before broader economic interoperability:**
1. Inventory every writer/producer of `SapBalance`, including mint, transfer, treasury/compost, bridge, restoration and exit paths.
2. Define the accepted justification types and exactly how the validator matches owner, currency, amount/delta, predecessor and operation identity.
3. Enforce justification in integrity validation, not only in coordinator logic or UI.
4. Ensure a justification cannot be replayed to authorize multiple balance increases; explicitly model partial consumption if any use case requires it.
5. Preserve a narrowly defined zero/genesis exception rather than treating `None` as general authorization.
6. Add multi-agent tests for unauthorized increase, changed amount, wrong owner, wrong currency, duplicate/replay, mismatched transfer legs, failed/retried cross-zome operations and valid mint/transfer/compost cases.
7. Update all producers in one coherent migration; don't introduce a permissive fallback to keep old tests green.

Source: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/zomes/payments/integrity/src/lib.rs

### 4a. Align implementation work with existing exact-head Finance tracks

This gap already has substantial tracked work; do not open a parallel implementation path or treat the presence of a design kernel as a migrated ledger:

- [#4519 / AC-176](https://github.com/Luminous-Dynamics/mycelix/issues/4519) tracks removal of public arbitrary SAP credit.
- [#4520](https://github.com/Luminous-Dynamics/mycelix/pull/4520) is the stacked source PR that makes `credit_sap` private; [#4521](https://github.com/Luminous-Dynamics/mycelix/pull/4521) is its qualification-only mirror.
- [#4110 / AC-092](https://github.com/Luminous-Dynamics/mycelix/issues/4110) and [#654 / FIN-SAFE-012](https://github.com/Luminous-Dynamics/mycelix/issues/654) track typed provenance, predecessor binding, balance deltas and fork-explicit state.
- [#4594 / AC-153](https://github.com/Luminous-Dynamics/mycelix/pull/4594) is the cause-bound/conservation-aware balance update tranche.
- [#4662 / FIN-SAFE-013 kernel](https://github.com/Luminous-Dynamics/mycelix/pull/4662) and [#4708](https://github.com/Luminous-Dynamics/mycelix/pull/4708) add a pure economic-effect theorem layer; their own claim ceilings state that this does not migrate legacy SAP or raw `credit_sap`.
- [#4664 / AC-153E](https://github.com/Luminous-Dynamics/mycelix/issues/4664) tracks prevention of compound mutation classes in one balance successor.

When checked on 2026-10-10, the above source/qualification PRs were still open and stacked rather than merged to `main`. Accordingly, the current `main` paths still expose the legacy public credit primitive; the existence of a source PR or a qualification PR is not evidence that the behavior has landed or passed exact-head execution.

**Additional caller-census finding raised in #4520:** the PR patch removes `#[hdk_extern]` from the two payments coordinator copies, but the same branch still contains cross-zome calls to `payments::credit_sap` in the bridge collateral-deposit path, bridge fiat-deposit verification path, and staking return path. These calls are dynamically dispatched by zome/function name and will not be caught by a Rust compile. The finding was posted for review on [PR #4520](https://github.com/Luminous-Dynamics/mycelix/pull/4520). Before that change is qualified, either migrate those paths to explicitly authorized source-specific entrypoints or prove, by exact-head multi-zome runtime tests, that each path has been replaced. The qualification-only changed-file list does not include the bridge or staking coordinator files.

**Sequencing decision:** use the existing AC-176 / AC-153 / FIN-SAFE track and its exact-head gates as the source of truth. The new comparative research should feed those tracks (e.g. interoperability mapping and claim/reveal protocol design); it should not create a competing credit/conservation implementation. Do not call the SAP ledger conserved, exactly-once, or fully provenance-bound until the precise integrated subject passes its required validation/runtime evidence.

### 4b. Further projects: patterns to benchmark, not dependencies to import blindly

**Nondominium — ValueFlows plus resource-scoped peer networks.** The public `Sensorica/nondominium` README describes a multi-DNA application using hREA/ValueFlows, a lobby → group → NDO hierarchy, group-scoped cloned cells, resource/NDO-scoped cells, and a governance operator separated from resource state. It names Rust Sweettest suites across the core, lobby and group DNAs plus Playwright against real conductors. Its README's canonical technology table currently lists Holochain 0.6.0 / client 0.20.0. That makes it a useful, concrete case study in the same broad architectural problem as Mycelix: how to separate resource data from policy and prevent one network's membership/state from automatically becoming another network's authority.

Candidate Mycelix lesson:
- Compare which domains truly need one shared DHT versus their own network/membrane.
- Model cross-cluster access as explicit adapters/capabilities rather than inheriting authority from a broad unified deployment.
- Benchmark the test pyramid: pure rules, integrity tests, two-/three-agent Sweettests per DNA, and user workflows against live conductors.
- Compare Nondominium's PPR (Private Participation Receipt) and resource lifecycle with Mycelix attribution/reciprocity, provenance, commons/property and economic-event models before considering shared types.

Source: https://github.com/Sensorica/nondominium/blob/dev/README.md

**Moss — composable collaboration spaces.** `lightningrodlabs/moss` describes a runtime/frame for composing custom collaboration “Tools” into groups, with the group and each tool represented by their own private peer-to-peer networks. This is a product/architecture lesson: users should assemble a useful workspace from small domain apps without each domain needing to become a monolithic all-in-one platform. For Mycelix, a similar composition contract could improve app boundaries and user choice, while the unified hApp remains a deployment option rather than the only security boundary.

Source: https://github.com/lightningrodlabs/moss/blob/main/README.md

**AD4M — semantic interoperability and signed agent-authored expressions.** `coasys/ad4m` describes pluggable “Languages” for bridging protocols/storage systems, cryptographically signed expressions, semantic perspectives, and a local agent-centric runtime. It also documents both Holochain-backed synchronization and a self-hosted server-backed link-language option, plus multi-node test commands. This is worth studying for Mycelix's federation of personal, commons, and civic meanings: how to preserve provenance while allowing different applications to interpret shared relations. Do not assume its semantic layer substitutes for integrity-zome validation, authorization, or a domain ontology.

Its repository uses the Cryptographic Autonomy License (CAL-1.0). Review the actual CAL conditions and compatibility before reusing code; source inspiration and independent protocol implementation are different from copying a licensed implementation.

Source: https://github.com/coasys/ad4m/blob/dev/README.md

**Flowsta Vault — user sovereignty as product behavior.** Its public README describes local key derivation and custody, encrypted local vault data, an approval-gated localhost bridge, identity linking, per-app backups and full data export/recovery. The useful benchmark is not the specific BIP39 or encryption implementation by itself; it is the end-to-end user contract: keys remain local, actions are visible/approved, and users can recover both identity and app data without a service operator retaining the only copy. Mycelix identity/personal vault UX should be tested against those workflows.

A metadata discrepancy is visible in the source: the README and root `LICENSE` say MIT, while `src-tauri/Cargo.toml` has `license = ""`. That does not prove a licensing violation, but it does mean the Rust package metadata is incomplete and should not be inferred from the top-level badge alone.

Sources:
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/README.md
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/LICENSE
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/src-tauri/Cargo.toml

### 5. Resolve licensing ambiguity before copying code

The Finance README currently says Apache-2.0 at its footer, while its `Cargo.toml`, source SPDX header, Finance `LICENSE`, and root `LICENSING.md` identify the Finance cluster as AGPL-3.0-or-later. This review corrects the stale Finance README footer in the accompanying change.

hREA is Apache-2.0, but license compatibility alone does not resolve version, architecture, attribution, and resulting-work obligations. Prefer a clean adapter/specification first; any source reuse must retain notices and receive an explicit license review.

Sources:
- https://github.com/Luminous-Dynamics/mycelix/blob/main/LICENSING.md
- https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/Cargo.toml
- https://github.com/h-REA/hREA/blob/sprout/docs/README.md

## Prioritized execution plan

### P0 — Ledger truth and explicit maturity
- Complete the `SapBalance.justified_by` conservation path described above, after inventorying all producers.
- Keep the known pre-alpha/multi-agent-test maturity caveats visible; a unit-test count does not establish DHT validation behavior.
- Correct contradictory license metadata and make license state machine-readable and consistent across README, Cargo manifests, license files and source headers.

### P1 — External economic interoperability
- Freeze a versioned mapping document for SAP, TEND, MYCEL, commitments, events, resources, receipts and disputes.
- Pin exact hREA artifacts and compatibility tuple. Do not combine hREA 0.7 artifacts into the existing 0.6 Mycelix DNA.
- Implement a read-only adapter/export fixture and round-trip tests before enabling external writes.
- Preserve Mycelix rules as integrity invariants; never let an external GraphQL adapter become the authority for balance changes.

### P1 — Selective commit/reveal
- Identify one suitable Mycelix use case (prefer blind evaluation or governance, not ordinary payments).
- Define the threat model and commitment envelope, then test it across independent agents before production use.
- Use ValiChord as a comparative reference, not a dependency by default.

### P2 — Broader hAppRadar survey
Expand the research to Moss, Flowsta Vault, AD4M, Nondominium and other active projects by category. For each, capture exact repo/ref/license/toolchain; actual test and runtime evidence; integration cost; security assumptions; reuse/adapter/independent-implementation decision. Prioritize by gap relevance, not stars, activity or project count.

## Acceptance criteria

- All claims in the comparison point to a specific source and revision or are clearly marked as project documentation rather than independently verified behavior.
- No external implementation is described as compatible until its exact runtime/toolchain is tested.
- SAP balance increases fail closed when they lack a valid, single-use justification; valid paths have tests at integrity and multi-agent layers.
- The economic mapping preserves exact integer units, non-transferable MYCEL semantics, TEND zero-sum behavior, and the boundary between economic facts and monetary policy.
- Commit/reveal tests demonstrate phase enforcement, tamper rejection, replay resistance, failure recovery and auditable disagreement.
- No production-readiness claim is inferred from a passing build or a stated test count.

## Scope and limitations

This is an evidence-backed research/design change. It does not claim that external repositories have been independently audited, that all findings in their READMEs are verified, or that any builds/tests were run during this review. The SAP conservation comment should trigger a complete source/test investigation before modifying ledger code; a narrow patch without tracing all producers could break valid paths or preserve a bypass.
