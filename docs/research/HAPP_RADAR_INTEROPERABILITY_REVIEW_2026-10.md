# Mycelix / hAppRadar Comparative Interoperability Review
Date: 2026-10-15 (hREA release evidence refreshed; individual sections retain their own inspection dates)
Status: comparative research and proposed engineering gates. The upstream release metadata and asset digests below were inspected, but Mycelix/hREA runtime integration is not claimed.

## Executive finding

The strongest near-term improvement is not to increase the number of Mycelix clusters. It is to make the economic ledger and cross-hApp trust boundaries independently verifiable, then align interoperable parts with existing Holochain ecosystem vocabularies.

This review compared the public hAppRadar directory, hREA's ValueFlows implementation and migration documentation, ValiChord's commit/reveal design, and the current Mycelix root/Finance documentation and source.

## Evidence-based findings

### 1. Version compatibility is a hard boundary, not a packaging detail

- Mycelix's root README documents Holochain 0.6.0 / HDK 0.6.0 / HDI 0.7.0. The Finance workspace pins HDK 0.6.1 and HDI 0.7.1 in `mycelix-finance/Cargo.toml`.
- As of 2026-10-15, hREA's latest listed release is [`happ-0.5.0-beta.1`](https://github.com/h-REA/hREA/releases/tag/happ-0.5.0-beta.1), published 2026-09-10. The release notes claim ValueFlows 1.0 + Holochain 0.7.x and require `@holochain/client` `^0.21.0`; the compatible release tuple is HDK 0.7.0, HDI 0.8.0, with the 0.700.x GraphQL adapter line. The release tag points at commit `79c8d4fb98c4f7f9a8cf3f7dd69690d6d1334d85`. The published `hrea.dna` and `hrea.happ` asset SHA-256 digests are pinned in the corpus and matrix below.
- The separately pinned `sprout` source snapshot `8c99160bbf6c4ffab0b4d05a3edf8ec629c09ab4` is a later source commit, not the release-tag commit. Do not substitute the source branch's code for a versioned release artifact. The checksummed release is tracked as research evidence only; it has **not** been selected as a Mycelix integration dependency, and no hREA/Mycelix runtime compatibility is claimed.
- The Holochain 0.6-to-0.7 upgrade changes validation callback/action APIs and network transport, and has no built-in 0.6 conductor database migration path. Native DNA composition therefore remains a compatibility/migration task, not a packaging-only change.
- hAppRadar's cached project metadata can describe a released version while repository docs describe a newer development/release line. Record the exact source ref, release artifact, conductor, HDK/HDI, client and adapter versions for every compatibility claim.



- The pinned hREA release tree's root `LICENSE` identifies Apache License 2.0. The corpus now records the exact license-file URL, but keeps `license_review_approved_for_reuse=false`; identifying a license is not a code-reuse approval or proof that every dependency/artifact obligation has been satisfied. The dedicated ValueFlows qualification workflow now also downloads the two public release assets from the fixed tag URL and checks the bytes against the pinned SHA-256 values; a successful step would establish artifact identity, not runtime compatibility or integrity-zome correctness. The same workflow now pins checkout and Rust-toolchain Actions by immutable commit SHA; this narrows third-party tag drift but does not make the PR-controlled workflow an independent verifier.
- The pinned release guide explicitly says the `happ-0.4.0-beta` → `happ-0.5.0-beta` upgrade changes the DNA hash and has no in-place data migration; treat it as a new network. This is distinct from the Holochain conductor 0.6→0.7 database migration limitation. A composition spike must not silently promise continuity of existing hREA state.

**Decision:** do not treat hREA zomes as drop-in dependencies for current Mycelix Finance. The release tag and SHA-256 asset digests are now pinned as evidence, but an artifact is not a Mycelix dependency until it is deliberately selected, license-reviewed, and tested against a fully pinned conductor/HDK/HDI/client/adapter tuple. Start with a loss-aware read-only export. Native DNA composition requires an explicit compatibility/migration plan; a separate-process/API integration is a different trust and networking boundary and does not make 0.6 and 0.7 DHTs one network.

Sources:
- Mycelix root: https://github.com/Luminous-Dynamics/mycelix/blob/main/README.md
- Finance manifest: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/Cargo.toml
- hREA release guide: https://github.com/h-REA/hREA/blob/sprout/docs/consuming-a-release.md
- Exact hREA release tag and public asset checksums: https://github.com/h-REA/hREA/releases/tag/happ-0.5.0-beta.1
- Pinned hREA source branch commit: https://github.com/h-REA/hREA/commit/8c99160bbf6c4ffab0b4d05a3edf8ec629c09ab4
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

**Additional caller-census finding raised in #4520:** the PR patch removes `#[hdk_extern]` from the two payments coordinator copies, but the same branch still contains four cross-zome calls to `payments::credit_sap` in both canonical and workspace projections: (1) thermodynamic genesis issuance in `currency-mint`, (2) collateral deposits in `bridge`, (3) fiat-deposit verification in `bridge`, and (4) staking return / un-slashed stake recovery in `staking`. The calls are dynamically dispatched by zome/function name and will not be caught by a Rust compile. The findings were posted on [PR #4520](https://github.com/Luminous-Dynamics/mycelix/pull/4520). Before the extern is removed, each path needs an explicitly authorized, source-specific entrypoint and exact-head composed-hApp runtime coverage—or must be disabled with clear behavior. The qualification-only changed-file list does not include the currency-mint, bridge, or staking coordinator files.

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

**Requests and Offers — a small, concrete economic user journey.** `happenings-community/requests-and-offers` implements a peer-to-peer request/offer board, is explicitly marked alpha/not production ready, and its package manifest wires a Sweettest suite plus UI unit/integration tests. Its documented MVP is intentionally a listing board with contact, ownership controls, and search; agreement, exchange, reputation and matching are later features. That scope discipline is useful for Mycelix: first prove one simple request → offer → commitment → delivery/receipt → SAP/TEND settlement path works end-to-end, instead of trying to launch all economic and civic domains at once.

The repository also provides a compatibility caution: its current `download-hrea` script fetches the pinned `happ-0.4.0-beta/hrea.dna` artifact, while hREA's latest listed release as checked on 2026-10-15 is `happ-0.5.0-beta.1`, which claims Holochain 0.7.x. Requests and Offers therefore appears pinned to an older hREA artifact/0.6.1 stack; do not infer 0.7 compatibility from the upstream project's newer release. Pin and qualify the exact artifact and matching conductor/client/GraphQL package. The project's own README calls it alpha, so its documented test surfaces are a pattern to benchmark, not proof of production readiness.

Sources:
- https://github.com/happenings-community/requests-and-offers/blob/main/README.md
- https://github.com/happenings-community/requests-and-offers/blob/main/package.json

**AD4M — semantic interoperability and signed agent-authored expressions.** `coasys/ad4m` describes pluggable “Languages” for bridging protocols/storage systems, cryptographically signed expressions, semantic perspectives, and a local agent-centric runtime. It also documents both Holochain-backed synchronization and a self-hosted server-backed link-language option, plus multi-node test commands. This is worth studying for Mycelix's federation of personal, commons, and civic meanings: how to preserve provenance while allowing different applications to interpret shared relations. Do not assume its semantic layer substitutes for integrity-zome validation, authorization, or a domain ontology.

Its repository uses the Cryptographic Autonomy License (CAL-1.0). Review the actual CAL conditions and compatibility before reusing code; source inspiration and independent protocol implementation are different from copying a licensed implementation.

Source: https://github.com/coasys/ad4m/blob/dev/README.md

**Flowsta Vault — user sovereignty as product behavior.** Its public README describes local key derivation and custody, encrypted local vault data, an approval-gated localhost bridge, identity linking, per-app backups and full data export/recovery. The useful benchmark is not the specific BIP39 or encryption implementation by itself; it is the end-to-end user contract: keys remain local, actions are visible/approved, and users can recover both identity and app data without a service operator retaining the only copy. Mycelix identity/personal vault UX should be tested against those workflows.

A metadata discrepancy is visible in the source: the README and root `LICENSE` say MIT, while `src-tauri/Cargo.toml` has `license = ""`. That does not prove a licensing violation, but it does mean the Rust package metadata is incomplete and should not be inferred from the top-level badge alone.

Sources:
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/README.md
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/LICENSE
- https://github.com/WeAreFlowsta/flowsta-vault-app/blob/main/src-tauri/Cargo.toml

### 4c. Latest exact-head CI triage (2026-10-10)

This section records the latest source-level follow-through so a documented fix is not mistaken for a qualified fix.

**Root cause of the downstream compiler failure:** the failed Mycelix CI run #4750 on AC-176's former head `cedb056565a15825d7b1dbf2dc8acb1aa2555838` reported a mismatched closing delimiter at `mycelix-finance/zomes/payments/integrity/src/lib.rs:822`. The expression came from AC-171 / PR #4490's new `validate_mint_cap_counter_link` helper: `wasm_error!(WasmErrorInner::Guest(format!(...)))` had one missing closing parenthesis. It affected both canonical and workspace integrity projections.

**Source correction made:** both integrity files on [AC-171 / PR #4490](https://github.com/Luminous-Dynamics/mycelix/pull/4490) were corrected to close the macro expression. The resulting file blobs are identical (`94d74d546d4f544f00e9087ae3a56d2b8da0cdff`). Head: `c242e721699fd87375a29a40347cae69e07968c1`.

**Independent formatting correction made:** the same failed run showed rustfmt differences in `checked_channel_transfer_balances` around coordinator line 1467 and at EOF. Both canonical/workspace checked-arithmetic helpers were moved to rustfmt's multiline `checked_add/checked_sub` layout and EOF whitespace normalized on [AC-175 / PR #4517](https://github.com/Luminous-Dynamics/mycelix/pull/4517). The same correction was forward-ported to AC-176's source branch so its CI can inspect a coherent tree. The matching coordinator blobs are identical in each checked pair (`66e9679b9d19b4738a2e03f6d19c509669aa93e1` on AC-175; `5decb81386775bbb4c4af5bd1bbc18e4cf8fb4a3` on AC-176).

**Exact-head synchronization:** AC-176's source head first moved to `dc0c6fa7587a0b0990a819dab4df5ccc6f12f0a1`, then advanced again to `1bc4c492639847f65a2be526138bfb86db46f779` when the fail-closed audit was added. The qualification-only branch for [PR #4521](https://github.com/Luminous-Dynamics/mycelix/pull/4521) was fast-forwarded to the latest exact subject (two additional commits; no merge or force update).

**Qualification state as of the latest check:** AC-171's Mycelix CI #6541, D6S #2202, and Finance exact-head #82 remained `queued`; AC-176's latest Mycelix CI #6545 was `pending`, D6S #2206 was `queued`, and Finance exact-head #86 was `queued`. AC-175's corrected head had no associated PR-triggered workflow run returned. The prior AC-176 #4750 failure remains historical evidence against its old, now-superseded SHA; it does not tell us whether the corrections pass. **No PASS is claimed until a run on the exact new head completes and its material steps are inspected.**

**Fail-closed guard added:** the latest AC-176 head adds `scripts/security/finance_raw_credit_caller_audit.py` and makes its CI job a required dependency of `ci-pass`. It scans coordinator sources in both Finance projections, excludes internal Payments-coordinator calls, and fails if any other zome still uses a string-dispatched raw `credit_sap` reference or if the two caller inventories drift. Applying the same pattern to the fetched source files found four references per projection, at `currency-mint:179`, `bridge:663`, `bridge:2417`, and `staking:379`. This was a source-level check of the reference pattern; **the new Python script has not completed in CI yet**. Given these calls remain, the guard is expected to fail until they are migrated or disabled. This is an intentional merge gate, not a claimed pass.

The outstanding AC-176 caller-census concern is unchanged and has one additional call site: `currency-mint::mint_genesis_sap` also dynamically calls `payments::credit_sap`. The full current census is one genesis-mint call, two bridge-deposit calls, and one staking-return call, mirrored in the workspace tree. Those four paths must be migrated to source-specific authorization or explicitly disabled with user-visible behavior before raw-credit removal is integration-qualified. The syntax/format patches above do not prove SAP conservation or close the caller gap.

### 4d. Route legacy SAP credit callers through AC-154 source proofs, not generic wrappers

A fresh source review of the current AC-176 branch reconfirmed that each of the four external `payments::credit_sap` calls has an identical canonical/workspace projection. More importantly, the newer Finance safety stack already defines the right direction: [AC-154 / #4589](https://github.com/Luminous-Dynamics/mycelix/issues/4589) says unsupported external positive credits remain fail-closed until typed source-specific proofs exist. [AC-153 / PR #4594](https://github.com/Luminous-Dynamics/mycelix/pull/4594) explicitly leaves governance/bridge/staking and other external positive credits blocked until that follow-up.

#### Source-specific gaps from the legacy caller paths

| Caller | Fresh source-level finding | Required evidence boundary |
|---|---|---|
| Thermodynamic genesis (`currency-mint::mint_genesis_sap`) | The coordinator rejects only empty `proof_bytes`; the source comments say actual STARK verification is future work. The persisted `ThermodynamicGenesis` entry contains sensor ID, yield, timestamp and location, not the proof/verifier receipt. The read-links-then-create-link replay marker is not a global concurrent exact-once theorem. | Authenticated, versioned proof verification; exact measurement/evidence object; circuit/verifier and conversion profile; canonical issuance identity; recipient and integer μSAP amount; conflict/replay semantics. See [#4439](https://github.com/Luminous-Dynamics/mycelix/issues/4439) and [#4180](https://github.com/Luminous-Dynamics/mycelix/issues/4180). |
| Legacy collateral bridge | `deposit_collateral` accepts a caller-supplied finite positive `oracle_rate`; integrity verifies that `sap_minted` is arithmetically consistent with that same supplied value, not that the rate/custody evidence was authenticated. | Consume an exact issuance receipt from the authenticated FIN-SAFE-010/011 collateral settlement pipeline; bind the qualified deposit, valuation/custody facts, mint ID, recipient and amount. Do not treat the legacy rate field as an oracle attestation. |
| Fiat bridge verification | The call supplies an `ExternalResourceAudit`; the shown flow only checks `is_speculative == false`. It does not authenticate the institution/compliance reference or prove bank settlement through a configured authority. The visible update validator checks structure and disallows transition back to Pending, but does not bind verifier authorization to the `Update` action. | Trusted issuer/bank attestation and explicit trust-root policy; canonical external deposit identity; exact amount/currency/conversion profile; one-time claim semantics; make verification evidence distinct from a balance mutation. |
| Staking return / slash remainder | `withdraw_stake` attempts the credit before recording `Withdrawn`. A multi-write partial/ambiguous completion therefore needs explicit retry protection; a reason string and the current mutable stake state are not a proof that the locked amount has not already been returned. | Exact stake-lock and release/slash evidence, bounded return amount, canonical stake/return identity, owner binding, and duplicate/conflict handling that survives retry after ambiguous completion. |

**Mutation-order hazard while the raw extern is private (source-order deduction):**

- Genesis writes a `ThermodynamicGenesis` entry and its sensor/timestamp dedup link before the credit call. If that call errors, the source evidence/dedup marker can remain with no SAP credit, and a retry may be rejected.
- Collateral bridge creates the `CollateralBridgeDeposit` and both lookup links before credit. If credit errors, a pending indexed deposit can claim an SAP quantity that never reached the balance.
- Fiat bridge changes the deposit status from Pending to Verified before calling Payments. If credit errors, the record can remain Verified without the balance effect, while the current coordinator refuses a second attempt because the status is no longer Pending.
- Staking withdrawal calls the return helper before updating to Withdrawn, so an error leaves the stake unwithdrawn and the retry path repeats the failed call. Slashing is more problematic: it creates a slashing event and link before attempting the un-slashed return, so an error can leave the event linked while the stake remains Active/Unbonding.

These outcomes are inferred from source ordering on the current draft, not demonstrated runtime results. They show why merely hiding the public ABI is not integration-safe. Migration needs exact effect identity, replay-safe receipts, and recovery/response-loss tests; where a path cannot yet prove those invariants, it should fail before any source-state writes with a clear disabled response.

These are source-inspection findings, not runtime exploit demonstrations. The source was inspected on 2026-10-10; no Rust/Sweettest/WASM runtime test was performed for these paths during this review.

#### Use the current V2 stack without overstating its readiness

The current Finance work already includes:
- [FIN-SAFE-014 issue #663 / draft PR #665](https://github.com/Luminous-Dynamics/mycelix/pull/665): owner-authored collateral claims derived from an exact FIN-SAFE-010 issuance receipt and economically de-duplicated by canonical `mint_id`.
- [FIN-SAFE-015 / draft PR #671](https://github.com/Luminous-Dynamics/mycelix/pull/671): pure value-note/transfer theorem, with exact conservation and explicit double-spend conflict semantics.
- [FIN-SAFE-016 / draft PR #677](https://github.com/Luminous-Dynamics/mycelix/pull/677): disabled-by-default Holochain adapter that exact-loads collateral claims and issuance receipts; it does not change legacy `SapBalance`, `credit_sap`, or `transfer_sap`.

All three source PRs remain open/draft in the inspected state, and shipped `sap_account_v2.enabled` / `sap_transfer_v2.enabled` are `false`. This is the right architectural direction for collateral-backed value, but it is **not** a drop-in fix for the four legacy callers and does not yet support genesis, fiat, governance, staking return, demurrage, or a globally final spend protocol.

**Decision:** keep AC-176's caller audit fail-closed. Do not add generic `credit_from_x` wrappers, whitelist the four callers, or enable the V2 lanes as a shortcut. Use AC-154's source-specific authorization contract and promote each source only after exact-origin validation, adversarial retry/replay/conflict tests, and exact-head hosted execution.

#### Runner-assignment diagnosis is a separate infrastructure boundary

[CI-OPS-001 / #697](https://github.com/Luminous-Dynamics/mycelix/issues/697) records jobs queued before runner assignment, with no checkout or test steps. On 2026-10-10, the public [GitHub Status page](https://www.githubstatus.com/) reported all systems operational and no incidents for October 8–10; its recent October 5 Actions/hosted-runner incident is marked resolved. This rules out neither repository/organization configuration nor a stuck account-specific queue; it does mean we should not infer a current global incident or a repository code failure from `queued`. Check Actions Settings (active jobs/usage), repository/organization Actions policy, quota/spending/suspension and UI errors; if clean, escalate representative stuck run IDs to GitHub Support. More source churn is not a substitute for this diagnostic.

### 5. Resolve licensing ambiguity before copying code

The Finance README currently says Apache-2.0 at its footer, while its `Cargo.toml`, source SPDX header, Finance `LICENSE`, and root `LICENSING.md` identify the Finance cluster as AGPL-3.0-or-later. This review corrects the stale Finance README footer in the accompanying change.

hREA is Apache-2.0, but license compatibility alone does not resolve version, architecture, attribution, and resulting-work obligations. Prefer a clean adapter/specification first; any source reuse must retain notices and receive an explicit license review.

Sources:
- https://github.com/Luminous-Dynamics/mycelix/blob/main/LICENSING.md
- https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/Cargo.toml
- https://github.com/h-REA/hREA/blob/sprout/docs/README.md

## Follow-up qualification snapshot — 2026-10-11, after static-name audit hardening

The AC-176 source PR #4520 and qualification-only PR #4521 are now aligned at exact source SHA b81a0308c827e1b3a2f2354445ed531b46d0672f. The mirror advanced using a guarded fast-forward; it remains a qualification-only subject and must not be merged.

### Audit coverage and local evidence

- The caller audit checks the raw-credit ABI in both Payments coordinator projections; inventories the canonical and workspace Finance zomes trees byte-for-byte; rejects nested/root symlinks before reading Rust sources; and scans coordinator call sites outside Payments.
- The scanner catches exact normal/raw "credit_sap" literals independent of constructor spelling, escaped normal/Unicode-escaped strings that decode to credit_sap, and statically composed Rust concat! calls whose string and character literal arguments resolve to credit_sap (including normal/raw fragments and parentheses inside comments). It deliberately errs toward false positives, including matching literals inside comments. Arbitrary const indirection, other macro/literal transformations, and runtime string construction remain outside this regex-oriented static scanner; this is not an AST-level proof.
- The checked-in regression suite now has **24 cases**: the previous 20 cover ABI visibility and attributes/comments, literal/constructor forms, full-audit fail-closed behavior, CI trigger/required-gate policy, full projection parity, and broken/nested/root symlink rejection. The 21st detects statically composed concat! names split across normal/raw literal fragments, including a parenthesis in a comment, while rejecting unrelated concatenation.
- An isolated local Python probe of the scanner logic detected hex- and Unicode-escaped literals, normal/raw concat forms, string/character concat fragments including a Unicode-escaped character, nested/comment-parenthesis forms, and rejected unrelated concatenation. This is focused fixture evidence, **not** execution of the checked-in test module. A reconstructed 16-case harness passed before later regression additions; that historical result is not a pass for the current suite. The 22nd detects a hex-escaped ordinary string literal that decodes to credit_sap; the 23rd detects Rust Unicode escape syntax such as \\u{5f}. The checked-in 24-test suite has **not** been run against a full local repository checkout, so no 24/24 claim is made.
- The exact source census previously fetched all **18 non-Payments Finance coordinator Rust files** and found the four remaining external raw-credit calls at currency-mint/coordinator/src/lib.rs:179, bridge/coordinator/src/lib.rs:663, bridge/coordinator/src/lib.rs:2417, and staking/coordinator/src/lib.rs:379. The latest commits changed only the audit and its test, not Finance zome source, so those four callers remain an expected blocker. The two Finance zomes projections previously matched 57/57 file blob SHAs; the new commits do not alter those trees.

### CI process and exact-head state

The audit has a dedicated finance_raw_credit_audit path-filter output covering canonical Finance, workspace Finance, the audit/test files, and .github/workflows/ci.yml. ci-pass depends on the change-filter job and the audit; it rejects a non-success change-filter result and requires a successful audit whenever selected by that dedicated path filter, a push, or manual dispatch. Focused local policy fixtures exercised relevant/irrelevant PR, push, manual-dispatch, skipped-audit, and failed-filter outcomes.

For exact SHA b81a0308c827e1b3a2f2354445ed531b46d0672f, direct GitHub Actions records show these PR-triggered runs, all **queued**:

- Finance Exact-Head Qualification: run 38095349383
- Mycelix CI: run 38095349394
- D6S Canonical Qualification: run 38095349463

Finance Exact-Head Qualification (run 38095349383), Mycelix CI (run 38095349394), and D6S Canonical Qualification (run 38095349463) remain queued. The commit combined status is pending with no completed status checks. Queued is not PASS.

Two D6U runtime-executor push-event runs on the same SHA (38095346264 and 38095322919) concluded failure, but the Jobs API returned total_count=0 for both. Neither record has job, step, test, log, or runtime evidence; these are not evidence of a code/test failure. The repeated event/branch-versus-configured-filter anomaly is tracked in [CI-OPS-002 / #4948](https://github.com/Luminous-Dynamics/mycelix/issues/4948). [CI-OPS-001 / #697](https://github.com/Luminous-Dynamics/mycelix/issues/697) remains the distinct hosted-runner assignment/queue issue.

### Merge boundary

AC-176 remains **unqualified and not ready to merge**. Four external raw-credit call sites remain; the audit is expected to fail until each caller is migrated to source-specific authorization or deliberately disabled with explicit behavior. The scanner’s success report now matches its actual scope: direct literals, escaped normal/Unicode literals, and static concat! names are covered. It still does not resolve arbitrary const aliases, every macro/literal transformation, or runtime-built names, and it does not prove source authorization, SAP conservation, exactly-once settlement, retry/recovery correctness, or runtime behavior. No queued, skipped, cancelled, or zero-job run is a PASS, and the legacy V2 account/transfer lanes remain draft and disabled by default.

## Recommended Mycelix proof-of-integration slice

The comparative work suggests the next useful demonstration is a small, complete and independently verifiable journey, not a new domain cluster.

1. A participant posts a resource request or offer.
2. The counterpart accepts an explicit scoped commitment/agreement.
3. Completion is recorded as an economic event with a receipt/evidence reference.
4. Settlement uses the selected SAP/TEND path, with exact typed effects and idempotency.
5. A verifier can independently inspect the provenance trail and report unresolved or forked states honestly.
6. The UI supports cancellation, dispute, retry/recovery and data export—not only the happy path.

Implement it as a thin vertical slice across the existing Mycelix domain code, backed by an explicitly versioned ValueFlows mapping. For the first iteration, keep the ValueFlows/hREA edge read-only/export-only until conductor/client compatibility and economic mutation guarantees are established. Do not let the adapter write authoritative balances. Reuse the existing AC-176/AC-153/FIN-SAFE source tracks rather than duplicating their ledger work.

**Qualification ladder:** (a) pure transition/economic-effect unit tests, (b) integrity validators, (c) two- and three-agent Sweettests on the exact target toolchain, (d) response-loss/replay/fork/network-partition tests, (e) packaged clean installation and multi-step UI E2E, (f) exact-head CI evidence. A queued or skipped run is not a pass. Record each layer independently so that a successful unit test never gets reported as a completed distributed-safety theorem.

This slice would let Mycelix demonstrate breadth through one coherent outcome while preserving sovereignty and evidence discipline. It would also produce a useful reference integration for future hApps.

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


## Follow-up qualification snapshot — 2026-10-14

This update adds an explicit, machine-readable decision matrix and refreshes the runtime-compatibility boundary against the Holochain developer documentation. It does not claim external hApps were built, integrated, or independently security-audited in this pass.

### Current Holochain compatibility decision

The official Holochain compatibility table currently recommends the 0.7 stack: conductor/CLI 0.7.0, HDK 0.7.0, HDI 0.8.0, JavaScript client 0.21.0, Rust client 0.9.0 and Lair 0.7.1. Holochain 0.8.0-dev releases exist, including dev.11 dated 2026-10-12, but a development prerelease is not the correct default qualification target for a stable integration. Sources: https://developer.holochain.org/resources/compatibility/holochain-0.7/ and https://github.com/holochain/holochain/releases

The official 0.6 → 0.7 guide identifies three system-level consequences that must shape Mycelix planning:

1. The action model and integrity validation API change substantially, including Action/ActionData and FlatOp changes.
2. tx5/WebRTC are removed in favor of Iroh over QUIC.
3. There is no built-in data migration path: 0.7 cannot read 0.6 conductor databases, and DNA hashes change. Existing installs have to clear the old conductor data to join the 0.7 network.

Source: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/

**Decision:** a Mycelix subproject is not “0.7-compatible” merely because a different subproject has migrated, a dependency manifest has changed, or unit tests compile. Each deployable hApp needs an exact compatibility tuple and its own source/runtime evidence. Any user data that must survive a 0.6 → 0.7 transition requires a separately designed application-level export, validation, re-import and recovery procedure; do not describe a database clear as a data migration or silently imply old and new DNA networks interoperate.

Minimum compatibility record per deployable target:

- exact source commit and hApp/DNA artifact identity;
- conductor and CLI versions;
- HDK/HDI versions and all direct Holochain dependencies;
- JavaScript/Rust client and Tryorama/Sweettest versions, where used;
- network transport and conductor configuration;
- whether the target is a clean install, migration/import test, or an existing-user upgrade;
- source-level, unit, integrity, multi-agent, packaged UI and recovery evidence recorded as separate states.

### Interoperability contract derived from the ecosystem review

Use standards and external projects to reduce translation cost, not to outsource Mycelix authority:

- **ValueFlows/hREA:** map economic facts and relationships through a versioned, read-only export first. Preserve exact asset identity, units, integer amounts, issuer/community scope, provenance and correction lineage. ValueFlows defines economic concepts; it does not make an incoming event authoritative for Mycelix balances, membership, policy or settlement.
- **Nondominium:** benchmark resource-scoped network boundaries and private participation receipts. A person’s membership in one group must not silently grant access or authority over another group’s resources.
- **Moss:** benchmark user-composable tools and private tool/group boundaries. Favor small interoperable experiences rather than assuming every domain must be exposed as one large application.
- **AD4M:** evaluate portable signed expressions and semantic adapters for exchange, but do not substitute a semantic graph for integrity validation, authorization or an explicit domain model.
- **ValiChord:** consider commit/reveal for blind review or pre-registered evaluation where early visibility creates a real attack surface. A commitment proves only that the revealed payload matches the earlier commitment under the specified hash/encoding; it does not prove the payload is true or the validator is competent.
- **Epistemic Resonance Protocol:** represent claims, evidence and critiques as structured, attributable records; preserve disagreement and corrections rather than reducing truth to a single reputation/confidence number.
- **Hearth and Flowsta Vault:** benchmark person-controlled care data, local custody, visible action approval, backup, export and recovery. Treat early-demo limitations and licensing/implementation details as explicit review items; do not place real health data into a demo.
- **Requests and Offers:** benchmark a narrow request → offer → agreement/commitment → delivery evidence → settlement journey. The source describes an alpha, so reuse its scope discipline rather than treating it as a production dependency.

Machine-readable decision matrix: [HAPP_RADAR_INTEROPERABILITY_MATRIX_2026-10.json](HAPP_RADAR_INTEROPERABILITY_MATRIX_2026-10.json).

### New negative controls for future adapter/qualification tests

The matrix records expected dispositions for high-value failure classes. At minimum, the adapter and vertical-slice qualification should reject or leave indeterminate:

- a 0.6 artifact/database presented as a qualified 0.7 deployment;
- a successful external API/GraphQL response represented as proof of DHT validation or settlement;
- an external ValueFlows event treated as authority to mutate SAP/TEND balances;
- MYCEL recognition exported as a transferable economic balance;
- TEND events translated without preserving time units, issuer/community scope, and zero-sum obligations;
- identical resource identifiers from different networks treated as the same resource without an explicit cross-network binding;
- a valid signature or commitment interpreted as proof that the signed statement is true;
- a commit/reveal payload accepted after mismatch, replay, or a phase violation;
- dissent, unknown status, stale evidence, or unavailable verification collapsed into a positive/negative fact;
- unit-test or build success represented as multi-agent or deployment qualification;
- README feature claims, hAppRadar listing status or a live demo represented as independent security review or production adoption.

### Recommended execution order

1. **Compatibility inventory:** close the current version truth for each hApp and select an exact stable target; keep 0.8 development builds out of the baseline unless a separate experimental lane is explicitly intended.
2. **Read-only interoperability fixture:** freeze one ValueFlows mapping corpus with exact-unit examples and adversarial cases. Export only; external input must not directly mutate an authoritative Mycelix balance.
3. **One vertical workflow:** complete a small requests/offers or commons-resource journey, including cancellation, dispute, idempotent replay, evidence export and recovery.
4. **Independent qualification:** exercise at least two agents against the exact target conductor/DNA, include response loss and duplicate operations, and have a verifier that does not reuse the production reducer as its oracle.
5. **Only then broaden:** consider write-through adapters, blind commit/reveal or multi-domain UI expansion only after the relevant threat model and exact-head evidence pass.

This ordering is intentionally conservative: Mycelix's differentiator should be cross-domain coordination with verifiable authority boundaries, not an unverified claim that it contains more features than other hApps.


## Source-pinned project review — 2026-10-14

The JSON matrix now pins **10 inspected source commits across 9 project profiles**. The SHA pins make this snapshot reproducible even when a project’s default branch moves; the corresponding Holochain/HDK/HDI/client fields below are still project- or hAppRadar-reported metadata, not compatibility tests run by Mycelix.

### New product and security findings from current hAppRadar pages

- **Hearth:** the current listing reports HDK 0.7.0, HDI 0.8.0 and client 0.21.0, with release v0.3.4. Its README still describes an early demo, not a clinical record. The listing says entries are not encrypted at rest and that other circle members retain their plaintext copies when one member leaves. Source: https://happradar.com/projects/hearth and the pinned source revision in the JSON matrix.

  **Mycelix lesson:** consent revocation is not retroactive deletion. A recipient may have already read, copied, or retained data. Health and personal-data flows must explain which access can be revoked, what is replicated, what can actually be deleted, and what remains with prior recipients. Never use Hearth's demo with real health data; benchmark the consent/portability problem, not its storage-security model.

- **Epistemic Resonance desktop:** hAppRadar now lists a v0.2.0 standalone desktop release published on 2026-10-12. Its page describes a built-in Holochain 0.7 conductor and a graph of claims, critiques, evidence and retractions, without numeric agent/claim rankings. The same source warns that the packages are unsigned. The project’s INSTALL documentation says peer discovery currently uses Holochain’s development bootstrap server and that data does not survive an incompatible upgrade. Sources: https://happradar.com/projects/epistemic-resonance-desktop, https://happradar.com/projects/epistemic-happ, and https://github.com/StateofIntent/epistemic-happ/blob/main/INSTALL.md.

  **Mycelix lesson:** the packaged-app route reduces installation friction, while its unsigned distributions, development bootstrap and upgrade/data-continuity caveats remain explicit operational risks. Copy the claims/evidence/retraction design ideas, not assumptions that a successful desktop launch means stable discovery, secure updates or preserved data.

- **Requests and Offers:** the current listing identifies the project as alpha/not production-ready and reports HDK 0.6.1, HDI 0.7.1, client 0.20.5, with a 0.6.0 release note; hAppRadar lists v0.6.0-alpha.1. Source: https://happradar.com/projects/requests-and-offers.

  **Mycelix lesson:** its narrow request/offer journey remains a useful scope benchmark, but it is not a drop-in 0.7 DNA dependency merely because hREA has a 0.7 release branch. Keep the workflow idea separate from binary/DNA composition and pin a compatible artifact for any integration experiment.

- **AD4M:** the inspected default-branch commit is dated 2026-10-14. This is fresh source activity, not evidence of compatibility with a specific Mycelix hApp or proof that the current architecture meets Mycelix's trust requirements.

Source revision pins are recorded in [HAPP_RADAR_INTEROPERABILITY_MATRIX_2026-10.json](HAPP_RADAR_INTEROPERABILITY_MATRIX_2026-10.json), with commit URLs for hREA, Nondominium, Moss, AD4M, ValiChord, Hearth, both Epistemic Resonance repositories, Flowsta Vault and Requests and Offers. Branch names remain as provenance labels; the exact SHA is the reproducibility anchor.

### Additional testable privacy controls

The next privacy/interop test fixture should include:
1. Revoking a recipient's future access does not claim it erased data already copied or replicated to that recipient.
2. A recipient leaving a group is not reported as erasing other participants' copies.
3. No clinical or other sensitive record is used in a demo/test environment unless it is synthetic.
4. An unsigned release and development bootstrap are surfaced as operational-risk metadata, not silently upgraded to trusted distribution or production discovery.
5. A 0.6 dependency tuple cannot be promoted to a 0.7-compatible DNA based on feature/API similarity.
6. “Retraction” is represented as a new attributable event, not as proof that prior readers or replicas no longer possess the original claim.

These controls sharpen the intended differentiator: user sovereignty must be visible in the real access, replication, revocation and recovery semantics—not only in policy text or UI language.


## Executable read-only ValueFlows contract — 2026-10-14

The research now includes a reference corpus and an isolated Rust contract checker inside the AGPL-licensed Finance cluster:

- Corpus: `mycelix-finance/tests/reference/valueflows-interop/mapping-corpus-v1.json`
- Contract/readme: `mycelix-finance/tests/reference/valueflows-interop/README.md`
- Checker: `mycelix-finance/tools/valueflows-interop-checker/`
- Path-scoped workflow: `.github/workflows/valueflows-interop-qualification.yml`

The checker is intentionally not an adapter and has no authority to change balances. It fails closed on unqualified source/ontology version claims, direct external authority promotion, lost identity/scope/provenance, MYCEL transferability, unsafe quantity encodings, missing correction lineage, changed negative-control dispositions, unknown fields and removed required guards. It includes regression tests that mutate the corpus to confirm these contract violations are rejected.

### Source-level quantity finding: TEND export remains blocked

Inspection of the pinned Mycelix source found `TendExchange.hours: f32`, `TendBalance.total_provided: f32`, and `TendBalance.total_received: f32`, while `TendBalance.balance` is `i32`. TEND describes 1 TEND as one hour of service, but its event-hour fields are binary floating point. Therefore the contract now marks lossless TEND numeric export as blocked until a versioned quantization/precision profile, tolerated error bound, and round-trip tests are defined. Formatting an `f32` into a decimal string does not make its economic semantics exact.

The current SAP types use `u64` micro-unit amounts, but the source itself documents that `SapBalance.justified_by` is threaded through and not yet enforced by integrity validation. Consequently, the proposed export mapping does not promote external facts to authoritative SAP balances, and this fixture does not establish source-ledger conservation.

### Validation claim ceiling

The corpus JSON was fetched back and parsed. Source-level consistency checks confirm the expected 3 instrument mappings and 12 registered negative controls. Rust source now defines 13 unit tests and a dedicated workflow for formatting, test execution and CLI validation. **Those Rust tests have not been run in this environment** because a Cargo toolchain is unavailable here; CI results on the exact branch head must be inspected before reporting a pass. This does not qualify full ValueFlows conformance, hREA runtime compatibility, DHT validation, settlement, or security.



## Qualification correction and trust-boundary note — 2026-10-15

### Initial workflow finding and correction

The first observed run for the ValueFlows workflow stopped at `cargo fmt --check`; all test/run steps were skipped. It did **not** produce a Rust compile or unit-test result. The log identified formatting differences in the newly added checker. These reported formatting edits were applied on the research branch.

The workflow originally used `actions/checkout@v4` without a `ref`. On a `pull_request` event GitHub checks out the synthetic merge ref by default, not necessarily the proposed source head. The workflow now explicitly selects:

```yaml
ref: ${{ github.event.pull_request.head.sha || github.sha }}
```

This makes the intended subject explicit. The next successful workflow log must still be inspected to confirm the checked-out SHA and that format → tests → CLI validation actually execute. Queued, pending, skipped or missing job results are not passes.

### Self-consistency is not independent verification

The checker, its expected-value tables, tests, workflow and corpus are all in this same PR-controlled trust domain. It can make contract assumptions executable and catch many accidental or incompletely updated changes, but it **cannot protect against a malicious change that weakens the checker and corpus together**. Do not call it an independent verifier or an independent security control.

For any high-assurance qualification:
1. Keep the canonical required-check policy protected on the base branch, or use a separately pinned verifier whose identity is not controlled by the proposed change.
2. Bind each result to the exact candidate head SHA and verifier identity.
3. Retain negative controls and verify that the trusted runner actually executed the named tests.
4. Treat this in-PR checker as a reference contract until that stronger trust boundary exists.

### Current technical claim ceiling

The TEND numeric export remains blocked pending an explicit quantization/precision profile, error bound and round-trip tests because the source event-hours fields are `f32`. The SAP source comment still says `SapBalance.justified_by` is not enforced by integrity validation. Nothing in this reference fixture repairs either production path, establishes settlement, or qualifies hREA interoperability.



## TEND transferability refinement — 2026-10-15

The mapping now replaces its coarse boolean transferability field with explicit semantic profiles:

- SAP: `transferable_units_with_domain_rules`
- TEND: `community_scoped_mutual_credit_position`
- MYCEL: `non_transferable_recognition_attestation`

This avoids implying that TEND is a bearer-like token. The TEND source defines each exchange against a service provider, receiver, DAO/community, and hours of service; its unit and obligation context must travel together. The new `VF-TEND-BEARER-001` negative control rejects promotion to freely transferable units unless an explicit, separately authorized instrument/conversion profile is introduced. The ecosystem matrix carries the same failure case.

The reference corpus now has **13 negative controls** and the Rust checker has **14 test functions** defined in source. This is source/corpus-level verification only. The dedicated workflow previously failed at formatting before test execution; after applying the reported formatting corrections and changing checkout to the exact PR head, a new executed result is still needed. Do not call the Rust suite passed until a run on the exact current head completes all required steps successfully.
