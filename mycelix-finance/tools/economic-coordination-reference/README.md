# Economic Coordination Reference v1

**Status:** research/reference implementation; not production finance, custody, or settlement software.

This standalone, dependency-free Rust crate implements one narrow vertical slice proposed by the Mycelix Economic Coordination Contract: a resource/service need moves through authorization, bounded capacity reservation, a policy-bound obligation, delivery claim, independent verification, and an internal double-entry reference-ledger posting.

Related work:
- Mycelix public PR #4959, *Economic OS evidence gates and Economic Coordination Contract v1*.
- Mycelix issue #3343, *Fiat, Web3, stablecoin, mutual-credit and external-instrument interoperability fabric*.
- Mycelix issue #3345, *Settlement rail profiles, finality algebra, and reconciliation receipts*.
- Symthaea issue #7188 / PR #7189, *Socioeconomic interoperability laboratory*.
- [ValueFlows 1.0 specification](https://www.valueflo.ws/specification/vfspec/) for economic intents, commitments, claims, resources, and observed events.

## Interoperability boundary

This crate intentionally does not serialize ValueFlows objects yet. A follow-on adapter should propose explicit mappings rather than equating similarly named records:

- `NeedRequest` is a candidate projection to a ValueFlows `Intent` (request/offer), not proof that the request is authorized or that capacity exists.
- `ObligationTerms` is a candidate commitment/contract projection with Mycelix-specific instrument, policy, and settlement metadata preserved.
- `DeliveryClaim` is a provider assertion pending verification; it must not be emitted as an observed `EconomicEvent` simply because it was recorded.
- A ValueFlows `Claim` has its own semantics: a claim for a future economic event in reciprocity for an event that already occurred. It is not a generic replacement for reputation, authority, an obligation, or an internal settlement receipt.
- A verified delivery may support an `EconomicEvent` projection, while the financial settlement record remains separately bound to a defined claim/settlement profile.

The ValueFlows specification distinguishes planned flows (`Intent` / `Commitment`) from observed flows (`EconomicEvent`) and defines `Claim` separately. Any adapter must retain those distinctions and declare which fields are native, profile extensions, or unrepresented. This README records an adapter proposal, not an implemented or qualified mapping.

## What it demonstrates

- Integer quantities and instrument-specific atomic units; no floating-point settlement arithmetic.
- Explicit separation of financial obligation, resource capacity, delivery claim, verification, and internal ledger posting.
- Verification status and evidence origin are separate axes: evidence may be `Verified` as a simulation and still have `SyntheticSimulation` origin. Authorization fails closed unless authority, rights, and ecological evidence are both `Verified` and `LiveDomain`.
- A commitment must bind the policy digest that authorized it.
- Capacity reservation is bounded in this serial reference model, and pre-commit cancellation releases its reservation.
- Only the contract's named provider may submit its delivery claim; an unrelated source cannot enter the verification path. Delivery remains a claim until an independent verifier confirms the exact resource, unit, quantity, and deadline.
- Independent verification binds to the exact provider delivery-claim ID as well as a separate verifier evidence ID.
- Unknown/stale/contested verification stays indeterminate; short or mismatched delivery becomes disputed. A `SyntheticSimulation` verification result, delivery claim, or dispute reassessment cannot promote a delivery to `Verified`; a synthetic-origin obligation cannot be committed.
- A dispute may be reassessed only by a different independent resolver bound to the exact delivery-claim ID. A resolver can contribute only one reassessment per dispute; only confirmation of full delivery resolves it to Verified. Adverse/incomplete reassessment is journaled and leaves the obligation disputed without erasing the obligation. When reassessment resolves the dispute, the primary verification accessor points to the accepted decision while the append-only verification history retains the earlier verdict and all later review decisions.
- Reassessment returns the resulting lifecycle state whether it resolves or leaves the dispute open, avoiding an error return after an already-recorded state change.
- Lifecycle event times cannot move backwards.
- Verified delivery must first consume its reserved physical capacity, producing a distinct `CapacityConsumed` lifecycle state. Consumption remains recorded even if a later financial posting fails.
- Consumed capacity is not made available to another reservation; the reference tracks reserved and consumed quantities separately.
- Settlement is attempted only after verified delivery and capacity consumption; account identity, denomination, arithmetic, and both balance bounds are checked before either balance mutates.
- Transaction IDs are single-use for reservations, including after cancellation; obligation IDs are single-use for ledger postings.
- Duplicate account/capacity declarations are non-destructive.
- Replayed lifecycle transitions cannot post the same contract twice.

The v1 reference, partial-fulfilment tranche experiment, escrow-rights ledger, isolated-replica experiment, and coordinated claim-identity registry currently contain **78 authored unit tests** (38 v1, 14 tranche-ledger, 7 escrow-ledger, 9 isolated-replica, and 10 claim-registry tests), plus **3 authored integration tests**: **81 authored test functions total**. This is a source-level count, not a claim that the tests have executed successfully.

## Experimental partial-fulfilment tranche ledger

The additive `fulfillment_tranches` Rust module explores a ValueFlows-aligned gap: one request may be fulfilled by multiple delivery tranches, each independently verified. It does not change the existing v1 single-claim lifecycle and is not wired into the v1 capacity book or settlement engine.

- Pending, indeterminate, and disputed tranches reserve their full claimed quantity. Verified accepted quantities count only when both evidence status and origin are acceptable.
- Partial acceptance counts only the accepted amount and releases the unaccepted remainder for a replacement tranche.
- Duplicate claim IDs and evidence IDs are rejected. Final tranches cannot be accepted twice. Disputed resolution requires a different verifier.
- Claims and verification decisions are retained in an append-only journal. This is a serial reference model, not a distributed concurrency protocol or financial posting authorization.
- Fourteen unit regression tests cover multiple tranches, partial acceptance, replay, overcommitment, over-fulfilment, unknown/synthetic evidence, independence, dispute resolution, deadlines, and journal ordering. They remain unexecuted until qualification records show otherwise.
- `tests/fulfillment_generated_sequences.rs` adds three dependency-free integration tests: 4,096 seeded ledgers × 32 generated claim attempts (131,072 attempts) checking quantity, journal-time, and aggregate invariants; 100 same-seed replay comparisons; and an intentional counterexample where two isolated ledgers each accept 10 units against the same 10-unit request. The counterexample demonstrates a known distributed-safety gap; it is not a desired protocol property. These tests are authored, not yet reported as executed.

## Partial-fulfilment experiment: proposed benchmarks

These are acceptance targets for the tranche prototype, not measured results:

| Benchmark | Target |
|---|---:|
| Conservation | 0 accepted units above request quantity across 100,000 generated transition sequences |
| Evidence gate | 0 synthetic/unknown-origin units counted as accepted |
| Replay resistance | 0 duplicate claim IDs or evidence IDs accepted across 10,000 replay attempts |
| Pending-claim reservation | 0 overcommitted quantities across 100,000 generated claim interleavings |
| Dispute audit | 100% of accepted decisions preserved in append-only verification history |
| Deterministic replay | 100/100 same-input runs produce identical ordered journal digests |
| Partial fulfillment | Exact agreement between per-tranche accepted quantities and aggregate accepted quantity in every generated case |

The implementation is currently a serial in-memory reference. The generated-state harness targets 131,072 claim attempts per invocation and 100 exact-seed replay comparisons; these are test workload sizes, not measured pass results. A dedicated counterexample asserts that two disconnected replicas can accept 20 units combined against a 10-unit request. A successor protocol must either use a shared reservation authority or specify bounded escrow/partition semantics and then demonstrate zero global over-allocation across generated partitions, retries, and message reordering. Concurrency/interleaving campaigns must not be presented as evidence of distributed safety until a concurrent or replicated protocol is modeled and qualified. A future integration should map tranche claims and acceptance decisions to ValueFlows semantics explicitly; ValueFlows defines an economic event as fulfilling a commitment completely or partially, but this module is not yet a ValueFlows adapter.

## Experimental replica escrow rights

The additive `fulfillment_escrow` module explores a way to bound local reservations without a globally available coordinator: partition a request quantity into disjoint, preallocated per-replica rights. A replica can reserve only rights it already holds. Pending reservations remain locked; finalization consumes only accepted units and returns the rejected remainder. Transfers move available rights and are single-use inside this serial reference ledger.

This is an accounting model, not a distributed implementation. It does not authenticate transfer certificates, prevent a partitioned replica from forking its local state, or establish Byzantine safety. A production protocol must either prevent rights transfers while disconnected or add durable authenticated single-spend transfer semantics and prove recovery/reconciliation safety.

See [FULFILLMENT_ESCROW_BENCHMARKS.md](FULFILLMENT_ESCROW_BENCHMARKS.md) for adversarial test targets. All targets remain proposals until exact-head execution evidence exists.

## Isolated-replica fixed-rights experiment

The additive `fulfillment_escrow_partition` module models the quantity-accounting contract across separate per-replica states. Each replica owns a disjoint preallocated share; reservations mutate only that replica's balance, and partial finalization consumes the accepted amount while returning the rejected remainder. It deliberately exposes no transfer or timeout-release API during a partition. The aggregate wrapper acts as an independent test oracle across replicas.

The deterministic interleaving campaign covers 1,000 seeds × 128 attempted transitions (128,000 operation steps) with 2–8 replicas and request quantities up to 97 units. These are authored generated checks, not a distributed proof and not an observed test pass.

This model's guarantee depends on assumptions that remain outside it: initial grants are issued uniquely and durably; local spent state cannot be rolled back or forked; accepted evidence is genuine; and a replica cannot use duplicated credentials to instantiate a second copy of its share. It also does not deduplicate the same external delivery identity across replica-local histories: `same_external_claim_replayed_across_replicas_is_a_known_idempotency_counterexample` demonstrates that a repeated claim ID may be accepted twice while the quantity invariant still holds. Quantity conservation therefore does not imply exactly-once delivery evidence. A successor needs a globally unique/single-use evidence or reservation identity with explicit partition behavior, plus crash/restart, snapshot rollback, transfer single-spend semantics, and reconciliation rules.

## Coordinated claim-identity experiment

The additive `fulfillment_claim_identity` module models the alternate consistency choice: all claims pass through one serialized authority that binds an external claim ID to one request, one replica, one exact SHA-256-shaped delivery digest, and one claimed quantity. Exact retries are idempotent; conflicting reuse, including the same claim ID on another request or replica, is rejected. Finalization is also idempotent only when the accepted quantity is identical; conflicting finalizations fail closed.

This demonstrates the stronger exactly-once *identity bookkeeping* available under a single authority. It is not a distributed consensus protocol, and the digest's syntax is not signature or evidence authentication. During a network partition, a replica unable to reach the registry must stay pending or decline finalization. An in-memory registry cannot prove durable single-spend behavior after crash, restore, fork or backup cloning.

The experiments therefore present two distinct operational choices: fixed local quantity rights improve partition availability within a finite allocation, while globally deduplicated claim identities require a shared serialization point or an explicitly proven distributed equivalent. Do not claim both properties from either toy model alone.

## Explicit limitations

- Evidence status and origin are caller-supplied inputs. Separating `EvidenceStatus` from `EvidenceOrigin` prevents a legitimately verified simulation result from being treated as live-domain evidence, but the enum itself does not prove provenance: a dishonest caller can mislabel origin as `LiveDomain`. This crate does not authenticate signatures, establish physical truth, or qualify who may issue evidence.
- The capacity book is in-memory and serial. It does not prove distributed reservation safety under DHT concurrency, network partitions, or Byzantine peers.
- The ledger is in-memory and internal-only. It does not move fiat, SAP, TEND, stablecoins, bank deposits, or any external asset; it does not establish legal settlement or discharge.
- The model does not decide which socioeconomic policy is legitimate, optimize resource allocation, establish fairness, or prove real-world economic outcomes.
- It is not wired into Holochain zomes or production finance authority paths.

## Reproduce

From the repository root:

```sh
cargo fmt --manifest-path mycelix-finance/tools/economic-coordination-reference/Cargo.toml -- --check
cargo test --locked --manifest-path mycelix-finance/tools/economic-coordination-reference/Cargo.toml
```

The focused GitHub Actions workflow runs these commands against the exact pull-request head. Until that run has completed successfully, the test suite should be described as authored—not passed.


## Capability-first sourcing (experimental)

The new `sourcing` module evaluates a capability-level request against supplier offers, rejects hard-constraint failures before comparison, and returns a Pareto frontier over total landed cost, arrival time, and a risk measure bound to a versioned risk-model digest. It requires each offer to carry a source URI, source-payload digest reference, and extraction-profile identity, and includes these in accepted/rejected evaluations so candidate decisions remain attributable. The reference layer checks only that these fields are present; it does not authenticate URLs or recompute/verify the supplied digest. It reuses the crate-wide evidence-origin and evidence-status types, and rejects mixing live-domain evidence with synthetic-simulation inputs; these status/origin values still have to be supplied by an upstream verifier. Quote validity and inventory-observation freshness use separate timestamps, expiry times and maximum-age policies; a fresh quote does not rescue stale stock. Accepted and rejected report rows retain the exact quote/inventory/credential-check times, validity bounds, status and evidence-origin fields used for the decision. Supplier credential expiry and credential-status-check freshness are separate gates. It uses integer minor currency units, checked arithmetic, explicit timestamps, exact capability-profile/unit/currency matching, and stable offer-ID ordering. It does not choose a winner when legitimate trade-offs remain.

The sourcing structs are minimal evaluator projections, not the canonical supplier listing, physical-component, or substitution ontology; future adapters should compose with Symthaea's component/substitution work and Mycelix EPI/supply-chain records rather than duplicate their identity semantics.

A capability profile digest is not itself proof of equivalence. A reviewed versioned mapping must establish that a candidate product meets the requested capability. Evidence-status fields are caller-supplied reference inputs; this module does not authenticate suppliers, verify live stock, search the web, issue purchase orders, or authorize spending.

See SOURCING_DECISION_BENCHMARKS.md for the hard safety gates, independent-oracle plan, baseline comparisons, proposed scale targets and cross-project boundaries. The Pareto classifier uses a deterministic cost/arrival/risk sweep and Fenwick prefix-min index, reducing classification from quadratic pairwise comparison to O(n log n) after sorting. Twenty focused inline unit tests and a separate brute-force frontier oracle (2,048 deterministic cases × 32 offers, plus an exact-tie fixture) are authored but have not been qualified by an exact-head runner in this PR. A dependency-free Rust example emits raw timings and p50/p95/max summaries at 10, 100, 1,000 and 10,000 offers; it has not yet been run, so performance remains unmeasured.


## Exact portfolio procurement (experimental)

The additive `portfolio` module is a bounded, effects-disabled reference for selecting feasible allocations across multiple supplier offers after upstream eligibility/evidence checks. This extends the single-offer `sourcing` evaluator without treating one offer's capacity ceiling as if it had to satisfy the full request on its own.

- Requires explicit request, capability-profile, unit, currency, policy, scenario-set and input-manifest identities.
- Checks qualification-receipt, quote and capacity observation status/origin, freshness and validity separately.
- Requires shared offers that point to the same capacity pool to agree on the pool quantity, observation window, evidence state and snapshot digest; allocations against that pool are summed and cannot double-count the ceiling.
- Enforces demand conservation, per-offer capacity, minimum lots/order increments, maximum number of suppliers, maximum supplier share, a minimum count of pairwise-disjoint evidenced failure-domain groups, delivery deadline and landed-cost budget with checked integer arithmetic.
- When disruption scenarios or a multi-source independence constraint are required, dependency evidence must be verified and fresh. Supplier identity and capacity-pool identity are included as common-cause domains automatically; declared shared sites/routes/sub-tiers also prevent a pair from being counted as independent. It reports unavailable allocation by named scenario and worst scenario; it does not invent probabilities.
- Returns the full non-dominated set over total cost, latest arrival, and worst-scenario unavailable quantity. It does not select an award winner or move stock or money.
- Includes bounded recursive enumeration (at most 16 offers, 500,000 search nodes and 2,000 retained feasible allocations). Exceeding a bound returns an error, not a partial frontier presented as complete.
- Output allocations preserve source locator, payload-digest reference, qualification receipt, capacity-pool identity and snapshot-digest reference. The module checks references are present, not authentic.

The in-module tests include a split-award case where neither supplier can individually satisfy the full demand, a no-split negative control, repeated-capacity-pool accounting, shared-failure-domain concentration, direct-supplier outage, stale-capacity and unverified-dependency cases, duplicate offer identities, and a small brute-force objective-frontier oracle. They are **authored tests, not an observed pass** until the exact-head workflow succeeds.

This remains a research prototype. The module does not authenticate suppliers, prove source claims, determine semantic product substitution, fetch live offers, authorize awards, issue RFQs/purchase orders, execute inventory mutations, or process payments. The independent portfolio oracle, broader frozen adversarial corpus, measured performance and any live/shadow integration remain follow-up work. See [robust portfolio sourcing contract](ROBUST_PORTFOLIO_SOURCING_V1.md) for the design and benchmark contract.


## Adaptive procurement recourse (experimental)

Benchmark and acceptance contract: [ADAPTIVE_RECOURSE_BENCHMARKS_V1.md](ADAPTIVE_RECOURSE_BENCHMARKS_V1.md). The first code slice evaluates a fixed baseline and already-committed options; it does not yet optimize which backup options to contract ex ante.

The additive `recourse` module evaluates a fixed baseline allocation and pre-committed flexible/backup options against an explicit scenario set. For each scenario it calculates which baseline quantities fail, filters contingency options by evidence freshness, failure-domain exposure and activation-plus-transit deadline, then solves a bounded allocation problem that maximizes on-time recovery first, minimizes incremental recourse cost second, and minimizes latest arrival third. The tie-break is versioned and deterministic.

- Normal/base allocations must conserve requested quantity and satisfy explicit lot, capacity, evidence, currency, unit, capability and deadline constraints.
- Flexible/backup options have separate option-retainer fees, activation fees, unit costs, activation leads and transit times. Retainer fees are included in the baseline commitment cost even when the option is unused; activation fees are charged only when used.
- Shared capacity snapshots must agree, and the baseline allocation plus reserved contingency quantities cannot exceed the declared capacity pool. This is deliberately conservative when multiple options describe the same pool.
- Scenario failures can target declared dependency tags or reserved `supplier:<id>` / `capacity-pool:<id>` names. A backup that shares a failed region/site/sub-tier cannot be counted as a recovery path.
- Evidence age is rechecked at scenario observation time. A quote/credential/capacity snapshot that expires before recourse cannot be promoted because the initial plan had passed.
- Reports carry the exact request, policy, scenario-set and input-manifest identifiers, per-scenario lost/recovered/unfilled quantities, exclusions, selected recourse allocations and search counts.
- A global search-node budget is divided across scenarios. Search-budget exhaustion returns an error, not a partial result described as complete.

The current authored tests cover flexible-plus-backup recovery, late backup rejection, evidence expiry, shared-capacity reservations, common failure domains, order-invariant reporting, baseline-demand conservation and a small independent enumeration oracle. They are **authored only** until exact-head qualification reports a successful run.

Research trail: two-stage robust supplier selection using normal/flexible/backup procurement modes (Dai, Zou & Zhang, 2026, https://doi.org/10.1016/j.ijpe.2026.110060); distributionally robust supplier allocation when disruption probabilities are poorly known (Li, Doan & Branke, 2026, https://doi.org/10.1016/j.ijpe.2026.110160); resilience with disruption dependencies, backup sourcing and reactive recovery (Hosseini et al., 2019, https://doi.org/10.1016/j.ijpe.2019.03.018). These motivate testable hypotheses, not universal performance guarantees.

The module is a research/reference model; it does not search for suppliers, execute contracts, issue orders, mutate stock, make payments, or prove source authenticity. It evaluates a supplied scenario set without assuming synthetic scenarios have calibrated probabilities.


## Ex-ante procurement strategy (experimental)

The additive `strategy` module exhaustively evaluates subsets of up to eight already-described flexible/backup options against the same frozen scenario set, invoking the current recourse evaluator for each subset. It returns a Pareto frontier over commitment cost, worst-case shortage, shortage summed across the declared scenarios, worst-case scenario cost and reserved maximum quantity. The scenario sum is a declared Pareto axis and descriptive corpus metric, not an expected value; scenarios are unweighted, and duplicate scenarios can change that axis.

The first slice treats reservations as all-or-nothing at each option's maximum activation quantity. It does not size partial reservations, model mutually exclusive options, allocate across multiple simultaneous requests, establish supplier truth, execute contracts or choose an authorized award. It fails closed on exhausted search bounds. The full acceptance contract and baselines are documented in [STRATEGY_BENCHMARKS_V1.md](STRATEGY_BENCHMARKS_V1.md).

The subset enumerator currently uses the same recourse reference module to score candidate sets. The test suite contains one test-local tiny two-option subset/activation-quantity oracle, but it is not yet a generalized independent recourse oracle or broad qualification corpus. Exact-head CI is also required before correctness claims. No compile/test pass or runtime measurement is claimed yet.


## Procurement outcome evidence ledger (experimental)

The additive `outcomes` module is an append-only in-memory reference for linking forecasts, authorized commitments, shipment claims, receipts, acceptance/rejection, returns, disputes and finalized outcomes. Exact retries with a stable upstream claim ID are idempotent; conflicting claim/event identity reuse fails closed. Corrections append a replacement and supersede the prior event in the derived view without deleting the original. Active events replay in deterministic occurrence-time order, independently of ingestion order.

Only caller-labelled `Verified` + `LiveDomain` events contribute to trusted quantities. Untrusted, stale, contested and synthetic events remain visible as excluded counts. An open dispute suppresses OTIF and forecast-error outputs, and final quantities reconcile against detailed acceptance/return evidence when present. The module does not authenticate status, issuer role, signatures, digests or physical truth; source claim IDs cannot stop duplicate real-world facts submitted under unrelated IDs.

Benchmark and claim contract: [OUTCOME_LEDGER_BENCHMARKS_V1.md](OUTCOME_LEDGER_BENCHMARKS_V1.md). [OUTCOME_LEDGER_GOLDEN_CORPUS_V1.md](OUTCOME_LEDGER_GOLDEN_CORPUS_V1.md) registers 40 stable fixture IDs and separates authored coverage from planned cases. Four standalone integration golden tests are authored in `tests/outcome_ledger_golden.rs`, in addition to module-local tests; the registry is not a claim that all 40 fixtures are implemented. No Rust test or performance pass is claimed before exact-head CI. This is not a durable ledger, distributed protocol, buyer-authority service, payment adapter or supplier ranking engine.


## Joint capacity allocation feasibility (experimental)

The additive \`joint_capacity\` module independently checks a proposed allocation across several concurrent requests. It binds assignments to explicit versioned request/offer capability mappings; enforces current Verified + LiveDomain evidence, per-request budget and demand, offer-local quantity and order-lot rules, unit/currency/deadline matching, and a single shared ceiling per capacity-pool ID; and returns canonical request/offer/pool reconciliation. Identical input facts in different vector orders must produce the same report.

This layer is deliberately a validator, not an optimizer. It does not award scarce units according to priority, compute a Pareto frontier, model fixed freight/tax/FX costs, couple cumulative capacity across time buckets, or run disruption recourse. Those are subsequent layers, and no purchase, reservation, dispatch or payment authority is exposed. Evidence statuses and digests remain caller-supplied assertions, not cryptographic proof.

The benchmark contract is [JOINT_CAPACITY_BENCHMARKS_V1.md](JOINT_CAPACITY_BENCHMARKS_V1.md), and stable fixture IDs are tracked in [JOINT_CAPACITY_GOLDEN_CORPUS_V1.md](JOINT_CAPACITY_GOLDEN_CORPUS_V1.md). Nineteen adversarial unit tests are authored; all require exact-head CI qualification before being called passing. The registry currently has 46 stable fixture IDs; only 19 Rust tests are authored, and remaining cases are planned.


The report includes canonical line-level allocation witnesses: request/offer/supplier IDs, offer source URI/digest and qualification receipt, offer evidence status/origin/freshness, mapping ID/digest, capacity-pool snapshot and time bucket, exact quantity and currency, unit cost, line cost, and deadline fields. It refuses allocations for already-expired requests. This makes each accepted line reviewable; it does not cryptographically authenticate the referenced records.


## Revision-fenced capacity reservations (experimental)

The additive reservation_cas module separates a sourcing/optimization proposal from an authoritative reservation. A reserve command must match a specific capacity-pool revision, fencing token and evidence-snapshot digest; successful mutations increment revision/fence and append a decision event. Exact retries are idempotent, conflicting idempotency-key reuse fails closed, and an expired lease only releases capacity through an explicit expiry event. Committed or consumed capacity cannot be released as if unused.

This is a serial in-memory reference state machine, not a concurrent service or distributed protocol. It does not implement persistence, store-level compare-and-swap, atomic multi-pool reservations, consensus, physical evidence authentication or any live purchasing path. The 100-contender test simulates stale writers presenting the same original revision; it is not a stress test of real concurrent threads or network requests.

The benchmark and claim boundary is recorded in [RESERVATION_CAS_BENCHMARKS_V1.md](RESERVATION_CAS_BENCHMARKS_V1.md), with 40 fixture IDs in [RESERVATION_CAS_GOLDEN_CORPUS_V1.md](RESERVATION_CAS_GOLDEN_CORPUS_V1.md). Eighteen module tests plus one integration test-local oracle test are authored, not yet claimed passing. The integration oracle compares 108 small reserve sequences against independently computed capacity and revision state. A future SQL-backed authority must commit capacity mutation and receipt/event atomically and retry the whole transaction on serialization failures.
