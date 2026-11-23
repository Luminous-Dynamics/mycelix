# Legacy SAP Writer → Owner-Authored V2 Architecture Crosswalk v0.2

**Audit base:** `e949664ceb2f9ddde26704c70e23a19380f33dde`  
**Source inventory:** [PR #6561](https://github.com/Luminous-Dynamics/mycelix/pull/6561)  
**Status:** source-audit crosswalk and migration planning only; no production code or DNA authority changed.

## 1. A correction to the v0.1 proposal

The first draft introduced a generic mutable scalar-transition schema. Comparing it with the existing FIN-SAFE roadmap shows that this would be the wrong canonical destination: it risks creating a second mutable scalar model alongside the stronger owner-authored account and immutable value-note design.

**This v0.2 replaces that proposal.** The existing architecture is the target:

- FIN-SAFE-013: remove cross-agent writes; V2 account effects are owner-authored.
- FIN-SAFE-014: collateral value is an owner-authored claim keyed by canonical `mint_id`.
- FIN-SAFE-015: scarce value is represented as immutable value notes and spends, with checked conservation and explicit conflict.
- FIN-SAFE-016: the Holochain transfer adapter exact-loads owner-claimed note origins and stays disabled.
- FIN-SAFE-017: recipient claim, output existence and spendability are distinct states.
- FIN-SAFE-018: trusted witness policy and authenticated quorum evidence are independently rooted.
- FIN-SAFE-019: each witness refuses conflicting allocations in its own immutable allocation journal.
- AC-153B/E/G and AC-154: exact conservation, one mutation class, complete value-surface classification and typed external-source proofs.

These are separate theorem layers. A merged pure kernel is not the same as an enabled Holochain lane, and a recipient claim is not the same as spendability. This PR adds a migration crosswalk; it does not claim these existing theorem tracks are all complete.

## 2. Why the industry comparison points toward value notes, not another scalar

The strongest transferable model is the UTXO/value-note pattern: a spend consumes exact prior outputs and creates deterministic outputs; output sum cannot exceed input sum; one input cannot be spent twice without a conflict. Bitcoin's developer guide describes exactly this input/output structure and the one-use constraint. Mycelix already has a Rust pure theorem with deterministic transfer identity, canonical input ordering, checked conservation and conflict classification. The advantage to pursue is not novelty of the UTXO idea; it is combining that proven accounting shape with owner-authored Holochain evidence, source-specific issuance claims, and explicit assurance levels.

Payment processors provide strong retry discipline but narrower scope: Adyen idempotency keys are company-account scoped and retained for 7–14 days, with regional duplication caveats and explicit transient-error handling. AWS transactional outbox guidance emphasizes atomic local event/state writes, at-least-once delivery, and idempotent consumers. Mycelix should improve on the combined semantics by retaining durable economic identity beyond API-cache windows, making source/recipient claims independently authored, and preserving an explicit Indeterminate state across remote effects.

Holochain is useful for local source-chain atomicity, not a distributed ledger transaction: writes in a single zome call commit or fail together, while a competing call may cause a chain-head-moved rollback. Countersigning coordinates a shared write but is not, by itself, a double-spend solution. FIN-SAFE-018/019's separately rooted witness policy and allocation journal are the right place to make the stronger assurance claim—only after host-level qualification.

## 3. Sixteen legacy writer classes mapped to the V2 stack

The machine-readable crosswalk maps each of the 16 source-audited writer classes to an existing V2 theorem or an explicitly missing source-specific effect. Key decisions:

| Legacy writer | V2 destination |
|---|---|
| zero genesis | owner-authored account-open marker |
| Amber grant/revoke | separate owner-authored, authority-bound metadata effects |
| demurrage | deterministic owner-authored effect with formula/interval evidence |
| raw credit | remove; replace with source-authenticated issuance claim or recipient-owned transfer-output claim |
| debit | exact value-note spend or typed burn/redemption/fee effect |
| transfer | FIN-SAFE-015 spend + exact recipient output/claim + FIN-SAFE-017 spendability gate |
| governance mint | source-authorized receipt + recipient-owned claim |
| payment | stable payment identity + outbox + recipient claim + independent Treasury receipt/reconciliation |
| Hearth contribution/withdrawal | new FIN-SAFE-020 pool effect theorem; no mutable member credit |
| membership gate | freshness-qualified authority evidence; membership is not pool allocation entitlement |
| collateral / fiat / physical issuance | independently authenticated source receipt + owner claim |
| staking lock/unlock | typed escrow-position effects with exact owner/position lineage |

The mapping is a crosswalk, not a claim that each destination adapter is implemented. Each row names its proof blockers and the exact cutover condition.

## 4. Important migration traps

### Legacy incoming transfer

The existing `transfer_sap` can credit the recipient's legacy `SapBalance` from a sender-authored invocation. That record is not a recipient claim. Do not silently reinterpret it as owner-authored V2 value or reject the member's valid historical value without a migration/claim path. V2 should leave the legacy history quarantined and let the recipient claim an exact authenticated sender output through the explicit owner-authored protocol.

### Claim is not spendability

An exact sender spend action and a recipient acknowledgement do not by themselves prove that an output is safe to re-spend. The FIN-SAFE-017 status gate must authenticate the required assurance, and the high-assurance witness path must bind a trusted policy and the exact spend/input set. Do not promote `ForkDetectionOnly` output to spendable.

### Countersigning is not global exactly-once

Holochain countersigning provides a shared multi-chain write protocol; it does not establish a global double-spend theorem. Use the independently rooted FIN-SAFE-018 policy and FIN-SAFE-019 witness-side conflict journal for the stronger risk tier, and keep the claim local to the actual authenticated witness set/quorum.

### Hearth is a separate communal authority problem

Membership alone does not establish contribution acceptance, pool ownership, withdrawal entitlement or available liquidity. FIN-SAFE-020 now tracks the distinct member-input spend, pool receipt, allocation authority, disbursement and beneficiary claim protocol. This must not be collapsed into the member-account theorem.

## 5. Exact next work

1. Keep legacy writers explicitly unqualified; do not turn on a V2 feature flag during this crosswalk.
2. Complete the pure/adapter stack in FIN-SAFE-014 through FIN-SAFE-019, preserving the disabled-by-default posture.
3. Add source-specific effects that are not covered by the value-note spend theorem: demurrage, Amber grant/revoke, burn/fees, governance issuance, fiat/custody/physical issuance and staking.
4. Execute FIN-SAFE-020 for Hearth pool contribution/withdrawal.
5. Make payment completion contingent on recipient claim and required remote Treasury receipt; use a durable outbox and original operation identity for reconciliation.
6. Migrate legacy inbound claims without rewriting historical Action authorship or stranding valid credits.
7. Only then deprecate raw `credit_sap` / `debit_sap` / `transfer_sap` as authoritative V2 capabilities and qualify every value-bearing surface under AC-153G.

## 6. Benchmarks to beat — safety first

The crosswalk retains 25 normative known-answer/adversarial scenarios. The next executable harness should report:

- **Accounting:** zero unauthorized value, uncaused issuance, conservation mismatch, duplicate claim, reused input note or wrong-owner mutation.
- **Idempotency:** 100 concurrent same-ID/same-payload attempts → at most one effect; 1,000 same-ID/changed-payload attempts → 100% conflicts; 10,000 duplicate outbox deliveries → at most one remote effect.
- **Fork/witness:** classify duplicate physical observations separately from distinct economic spends; zero winner-by-time/arrival/hash; all conflict descendants frozen; incomplete DHT state stays Indeterminate.
- **Recovery:** inject response loss before and after each local/remote commit, restart both conductors, and prove that the original operation identity is reconciled without a second spend.
- **Performance after safety passes:** p50/p95/p99 effect/claim latency, sustainable throughput, witness-read amplification, DHT bytes per note/effect, outbox age, claim lag, recovery time and cost per 1,000 valid effects.
- **Comparators:** Bitcoin-like UTXO conservation/double-spend semantics; matched SQL ledger transaction; outbox-based payment service; and Adyen-style API idempotency under the same retry, retention and failure model. Never compare raw TPS with different finality or authority semantics.

Candidate research target: at least 20% lower p95 total processing cost or 20% more sustainable valid effects under equal p99 latency/resource budgets, with all safety invariants equal or stronger. This is a target, not a measured result.

## 7. Claim ceiling

This PR is a crosswalk and structural CI artifact only. It does not implement V2 account/transfer/Hearth logic, validate issuers, prove network finality, enable any DNA config, migrate legacy balances or claim SOTA performance. The referenced PRs/issues retain their own exact-head qualification and claim limits.

## 8. Runtime capability blocker: fork warrants and countersigning

A V2 value-note design must not imply Holochain runtime guarantees that have not been verified.

The current official Holochain developer documentation states:
- chain-fork warrants are not operational yet; ordinary invalid-operation warrants and chain-fork evidence are distinct;
- agent activity / DHT observations are eventually consistent; the absence of a known fork or warrant is not proof that no conflict exists;
- countersigning is labelled unstable and disabled by default in Holochain 0.4 and newer;
- countersigning synchronizes a shared write, requires all participants online for the session, and is explicitly not by itself a double-spend prevention mechanism.

The current Finance workspace records conductor 0.6.1 via Holonix commit `d21b3543`, HDK 0.6.1 and HDI 0.7.1. Holochain's current compatibility matrix lists 0.6 as maintenance mode (0.6.3 latest compatible) and recommends 0.7.1 for general use. A runtime migration is not assumed here; it needs a matched build/DNA/conductor proof.

This gap is tracked by [FIN-SAFE-021 / #6576](https://github.com/Luminous-Dynamics/mycelix/issues/6576). The witness and spendability roadmap is conditional on that exact-runtime qualification.

**Required status vocabulary:** `NotAvailable`, `GoodSoFar`, `BadActor`, `Conflict`, `Indeterminate`, and `Provisional`. `GoodSoFar` means no conflict has yet been observed by the queried authorities; it is not finality. Missing fork warrants, incomplete authority coverage, unavailable countersigning or a timed-out witness session must never promote an output to `SpendableWitnessed`.

The crosswalk therefore requires:
1. an exact runtime capability manifest for the shipped conductor, HDK, HDI, DNA, feature flags and Holonix/Nix inputs;
2. reproducible capability tests on the current 0.6.1 pin, latest 0.6 maintenance (0.6.3) and recommended 0.7.1;
3. explicit countersigning enablement evidence before relying on its session API;
4. host-level tests for source-chain fork visibility, delayed discovery, `get_agent_activity` status, witness timeout, offline signers, equivocation and no-warrant `GoodSoFar`;
5. a claim ceiling that stays fork-detection-only/provisional until the actual runtime and independent witness policy are qualified.

This is not a claim that countersigning cannot be enabled on the current conductor. It is a requirement to prove that it is enabled and behaves as assumed before any high-assurance output is spendable.

## 9. Machine-readable runtime evidence boundary

The companion `holochain-runtime-capability-matrix.v0.1.json` records the Finance source pins and the exact limits of the current official runtime documentation. It is deliberately labelled `repository_source_and_official_docs_audit_not_runtime_qualification`.

It records:
- repository-documented conductor 0.6.1 and Holonix ref `d21b3543`;
- repository-documented HDK 0.6.1 / HDI 0.7.1 and matching Holochain type pins;
- exact conductor binary, actual runtime feature enablement, DNA/WASM pack digests and live countersigning session tests as **not proven**;
- chain-fork warrants as **not operational per current official docs**;
- countersigning as unstable/disabled by default in 0.4+ docs, with its exact Mycelix 0.6.1 enablement left **unknown**;
- `GoodSoFar` as provisional observation, never finality.

This is an evidence classification, not a claim that Holochain 0.6.1 cannot enable countersigning. FIN-SAFE-021 / #6576 owns the exact runtime manifest and integration tests. The 25-case adversarial corpus includes delayed fork discovery, unavailable countersigning, and incomplete witness sessions; all must remain Indeterminate until proven otherwise.

