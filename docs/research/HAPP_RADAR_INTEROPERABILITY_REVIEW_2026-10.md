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

The repository also provides a compatibility caution: its current `download-hrea` script fetches the pinned `happ-0.4.0-beta/hrea.dna` artifact, while the current hREA in-repository guide describes the later `happ-0.5.0-beta` / Holochain 0.7 line. Pinning an older supported artifact is fine, but integration claims must follow the exact artifact and matching conductor/client/GraphQL package—not the newest README alone. The project's own README calls it alpha, so its documented test surfaces are a pattern to benchmark, not proof of production readiness.

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

## Follow-up qualification snapshot — 2026-10-10, refreshed after workflow-path hardening

The latest inspected AC-176 source SHA is `f61907259eaf6482da713dc3fae39a156e145e6f`. [PR #4520](https://github.com/Luminous-Dynamics/mycelix/pull/4520) and qualification-only [PR #4521](https://github.com/Luminous-Dynamics/mycelix/pull/4521) point to this exact SHA. The mirror was advanced with a guarded fast-forward, not a force update.

### What is now locally exercised

- The caller audit checks the raw-credit ABI in both Payments coordinator projections and scans external coordinator files for the exact normal/raw Rust string literal `"credit_sap"`, independent of constructor spelling.
- The scan deliberately errs toward false positives (for example, a quoted literal in a source comment gets reported for review); it does not detect names dynamically assembled at runtime. This is a static gate, not an AST-level proof.
- The regression suite now has **18 cases**. The first 17 cover ABI visibility, outer attributes/comments, literal/constructor spellings, full-audit fail-closed behavior, clean/private success, known-caller detection, canonical/workspace caller drift, and the CI trigger/required-gate policy. The 18th checks that non-caller content drift anywhere under the two Finance `zomes/` trees fails closed.
- A reconstructed local harness passed 16 audit cases before the 17th workflow-policy and 18th projection-parity cases were added; that is historical evidence, not a pass for the current suite. A focused local fixture exercised the workflow path-filter/job-condition/`ci-pass` assertions. A separate focused local fixture exercised byte-identical projection success, content-drift rejection, and symlink rejection. The checked-in 18-test suite has **not yet been run against a full local repository checkout**, so no 18/18 result is claimed.
- A separate exact-subject census fetched all **18 non-Payments Finance coordinator Rust files** from the source tree at SHA `2b7283a0b098a0c519b4e91f2663b55e76df0119`. It found exactly four external raw-credit literals, at `currency-mint/coordinator/src/lib.rs:179`, `bridge/coordinator/src/lib.rs:663`, `bridge/coordinator/src/lib.rs:2417`, and `staking/coordinator/src/lib.rs:379`; no extra literal-form callers. Subsequent source changes affected only audit/test/CI files, not those callers. At exact source head `f61907259eaf6482da713dc3fae39a156e145e6f`, the recursive Git tree reports 57 files under each Finance `zomes/` root and all 57 corresponding Git blob SHAs match. The audit now enforces byte-for-byte equality across this full file inventory, not only coordinator caller-inventory parity.

### CI process improvement

The raw-credit audit no longer reuses the broad Finance test job's `finance` path-filter output. A separate `finance_raw_credit_audit` path filter includes:
- `mycelix-finance/**`;
- `mycelix-workspace/mycelix-finance/**`;
- the audit Python script and its regression suite;
- `.github/workflows/ci.yml`.

The audit job condition uses this dedicated output (and still runs on branch pushes); `ci-pass` already has it in `needs` and fails if the audit job fails. This closes a process gap where a workspace-only or audit-only change could otherwise skip the gate. A focused local fixture exercising the policy assertions passed. The 18th test protects full Finance zome projection parity; focused fixtures exercised equality, content drift and symlink rejection. The checked-in 18-test suite still needs execution on a full checkout before any suite-wide pass claim.

### Exact-head workflow state

For exact SHA `86cdb4c08f81639224c63165c5625757805fa657`, direct GitHub Actions API records show the three PR-triggered runs, all **queued**:

- Mycelix CI: run `38090495461`
- Finance Exact-Head Qualification: run `38090495451`
- D6S canonical qualification: run `38090495467`

Queued is not PASS and there were no executed steps shown at the last query.

Two D6U runtime-executor runs on the same SHA (`38090492007` and `38090460642`) show workflow conclusion `failure`, but the Actions Jobs API returned `total_count=0` for both. They expose no steps, test logs, or artifacts, and do not demonstrate a Rust/test failure or runtime execution. The mismatch between these `event=push` records on the AC-176 branch and the configured D6U `workflow_run` branch filter is tracked separately in [CI-OPS-002 / #4948](https://github.com/Luminous-Dynamics/mycelix/issues/4948).

[CI-OPS-001 / #697](https://github.com/Luminous-Dynamics/mycelix/issues/697) remains the distinct runner-assignment blocker: new ordinary hosted-runner jobs remain queued before steps. The public GitHub Status page was operational at the last check; that does not rule out repository/account/org policy, spending/quota restrictions or a stuck queue. Inspect Actions settings/usage and UI error banners, and escalate representative IDs to GitHub Support if settings are clean.

### Merge boundary

AC-176 remains **unqualified and not ready to merge**. The source audit should fail until all four external callers are migrated to source-specific authorization or deliberately disabled with explicit behavior. The ABI guard, source census and locally tested cases improve process confidence but do not prove source authorization, SAP conservation, exactly-once behavior, or runtime success. The existing V2 account/transfer lanes remain draft and disabled by default; they are not a shortcut for wiring the legacy callers.

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
