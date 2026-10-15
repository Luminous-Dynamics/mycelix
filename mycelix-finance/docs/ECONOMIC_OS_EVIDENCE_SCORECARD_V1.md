# Economic OS Evidence Scorecard v1

**Status:** research contract, not a claim that Mycelix is the best economic operating system.
**Review date:** 2026-10-11.
**Scope:** public Mycelix finance, commons, governance, and economic-transition components.

## Judgment

Mycelix has the ingredients for a distinctive economic coordination stack: plural monetary instruments (MYCEL/SAP/TEND), mutual credit, commons treasury rules, governance, resource coordination across everyday domains, and a conservative transition/recovery reference model. That is broader than a standalone community-currency wallet.

It is **not yet justified to call Mycelix the best economic OS**. Breadth of components is not proof of a working, usable, resilient whole. Public READMEs document intended mechanisms; test guides enumerate categories of testing; the transition validator is a reference model. The integrated cross-domain workflow, independent comparative evidence, field outcomes, and external replication remain the bar—not assumptions to fill in.

**Falsifiable hypothesis:** Mycelix can become unusually strong if it connects economic accounting to real resource fulfilment, preserves community sovereignty, interoperates with existing mutual-credit networks, and demonstrates safety and distributional outcomes under stress.

## What counts as an economic OS?

An economic OS coordinates eight things as one auditable lifecycle:

1. Needs and offers, with capacity, availability, and acceptable terms.
2. Real resources—housing, care, food, water, transport, energy, labor, capital, and commons assets—with units, constraints, custody, and lifecycle.
3. Multiple instruments—currencies, mutual credit, time credits, grants, obligations, reserves, collateral, and external settlement—without conflating them.
4. Allocation under legitimate authority and explicit policy.
5. Reservations, delivery, accounting, defaults, reversals, disputes, and recovery.
6. Federation between communities that retain authority over their rules and data.
7. Constitutional limits: rights floors, due process, exit, privacy, and appeal.
8. A learning loop where simulations are compared to baselines and field observations, preserving provenance and negative results.

A currency issuer is not an economic OS. Neither is an optimizer without legitimate authority, proof of fulfilment, or a right to appeal.

## Evidence labels

Every public capability claim should identify its actual maturity:

- **Specified:** a requirement exists in a design.
- **Implemented:** code exists on a named branch/commit.
- **Tested:** exact-commit automated tests pass.
- **Integrated:** multiple components complete an end-to-end scenario.
- **Observed:** a deployment has operational data.
- **Evaluated:** results are compared with a baseline and limitations disclosed.
- **Replicated:** an independent operator reproduces the results.

Do not collapse these labels. A passing unit test is not adoption; a README is not a deployment; a simulation is not a demonstrated real-world outcome. Each evaluation should bind commit, scenario-corpus digest, input-data provenance, environment, evaluator version, and retained run receipt.

## Mycelix capability map: the gaps that matter

This is a source-oriented self-assessment, not an independent code audit.

| Capability | Publicly documented direction | Evidence still needed |
|---|---|---|
| Monetary pluralism | Finance describes MYCEL reputation, SAP circulation, TEND mutual credit/time exchange, treasury pools, fees, and demurrage. | Accounting/property tests, parameter sensitivity, adversarial cases, integrated cross-currency lifecycle, and evidence users understand obligations. |
| Real-resource coordination | Commons describes property, housing, care, mutual aid, water, food, and transport. | End-to-end scenarios from a real need to verified delivery, cost/accounting, dispute resolution, and resource release or reconciliation. |
| Democratic authority | Governance describes proposals, voting, delegation, optional quadratic voting, and timelocks. | Adversarial tests for Sybil attacks, wealth-weighted capture, collusion, low turnout, coercion, emergency-power persistence, appeal, and minority rights. |
| Safe external effects | CIV-ECON develops a bounded local journal and pure recovery-policy reference model. | Integration with a production transition driver, authenticated provider evidence, adapter-level crash/partition tests, and evidence that retries cannot duplicate real effects. |
| Inter-community exchange | Mycelix describes bridges and cross-community clearing. | Two independently operated nodes, frozen contracts, partition/reconciliation scenarios, compatible dispute semantics, and non-repudiable histories. |
| Macroeconomic evidence | FIN-SYS defines source identity and semantic limits for financial observations. | Versioned adapters, revision-aware replay, source-release comparisons, coverage reporting, and proof classifications never exceed source scope. |
| Inclusion and user experience | Not established by architecture descriptions alone. | Accessible task completion, low-bandwidth/offline behavior, onboarding and exit/export, localization, and reasons users abandon workflows. |
| Privacy and sovereignty | Local-first and sovereignty are architectural goals. | Threat models, metadata-leakage tests, revocation, key recovery, community export, jurisdictional review, and measured limits on cross-domain data sharing. |
| Economic outcomes | The designs imply possible resilience and access benefits. | Independent baselines and longitudinal measures of fulfilment, affordability, defaults, utilization, distribution, resilience, and unintended harm. |

References:
- [Mycelix Finance README](../README.md)
- [Economic Coordination Contract v1 — proposed, not implemented](ECONOMIC_COORDINATION_CONTRACT_V1.md)
- [Mycelix Commons](../../mycelix-commons/README.md)
- [Mycelix Governance](../../mycelix-governance/README.md)
- [FIN-SYS source profile](../../docs/finance/FIN_SYS_SOURCE_PROFILE_V1.md)
- [CIV-ECON transition-validator PR](https://github.com/Luminous-Dynamics/mycelix/pull/4942)

## Competitive landscape: compare strengths, not slogans

This initial scan summarizes public project descriptions available on the review date. It does **not** establish comparative quality, security, adoption, or impact; the projects below should be candidates in a fair task-by-task evaluation.

| Project | Publicly documented focus | What Mycelix should learn or prove |
|---|---|---|
| [Credit Commons Org](https://gitlab.com/credit-commons/credit-commons-org) | Federated mutual-credit networks, local-first principles, specifications, and a reference implementation. | Do not reinvent federation without reason. Compare clearing, limits, dispute, governance, and inter-network settlement semantics. Interoperate if the contract is compatible. |
| [Mutual Credit Services](https://www.mutualcredit.services/projects) | Applied collaborative-finance projects, including multilateral invoice clearing for businesses. | Benchmark real operator needs: cash-flow relief, working-capital resilience, support, and sustainable operating cost. |
| [MCCS](https://github.com/ic3software/mccs) | Mutual-credit transfers plus trade discovery; the public roadmap describes interconnected nodes as a later phase. | Make trade discovery a first-class capability. Measure whether participants find counterparties and complete exchange, not just whether transfers validate. |
| [Komunitin](https://github.com/community-exchange-network/komunitin) | Local community-currency wallets and marketplace, including exchange between decentralized community currencies. | Compare usability, marketplace liquidity, currency-to-currency flows, onboarding, and federation overhead. |
| [Community-Currency](https://github.com/segovro/Community-Currency) | Configurable, community-specific zero-reserve mutual credit and separate monetary/governance roles. | Compare parameter simplicity and transparency; test whether Mycelix complexity earns measurable benefits. |
| [Commons OS](https://github.com/Commons-Engineering/commons-core) | Broad, forkable commons-oriented operating-system concept spanning coordination and production. | Compare scope carefully. Conceptual breadth and advertised pattern counts are not evidence of reliable economic mechanisms or field outcomes. |

There is no single leaderboard for these projects: they have different units of success. Mycelix should be willing to interoperate with a narrower system that performs one job better rather than force every job into a Mycelix-only path.

## Economic hypotheses to test adversarially

These are risks to measure, not findings that a mechanism is necessarily wrong.

- **Demurrage:** who bears the cost; does circulation improve; do low-income users suffer disproportionately; does activity shift to another store of value? Account for every fee and distinguish circulating assets from protected reserves.
- **Reputation:** do early members or high-reputation stewards gain self-reinforcing power? Do cliques dominate recognition? Can newcomers recover from error or exclusion? Reputation is not objective truth.
- **Time credit:** an hour is auditable but hours are not economically or socially interchangeable. Test scarce skills, care intensity, disability, danger, training, and informed consent; allow local policy variation.
- **Counter-cyclical credit:** expanded limits during stress may sustain exchange but may also raise correlated defaults. Test simultaneous income shocks, supplier failure, and delayed repayment.
- **Reserve requirements:** do inalienable reserves protect commons assets at the cost of essential liquidity or service continuity? Measure both over a full cycle.
- **Voting/delegation:** test whale influence, strategic delegation, vote buying, identity duplication, retaliation, minority-rights erosion, and emergency-rule persistence. Quorum percentages are not an anti-capture proof.
- **Automated allocation:** optimize only inside a feasible rights-and-capacity set. Aggregate welfare scores must not conceal critical unmet needs, coerced participation, discrimination, or unfulfilled promises.
- **Federation:** test stale observations, partitions, duplicate requests, key compromise, incompatible versions, asymmetric reversals, and exit with open obligations.

Every hypothesis must declare its failure condition before execution. Record null and disappointing results as well as wins.

## Constitutional anti-Ponzi and finite-resource invariants

“Ponzi-like” is a useful systems metaphor here, not a legal diagnosis of every part of the global economy. The engineering target is a system that cannot silently sustain itself by rolling forward unfulfillable obligations, transferring losses to weaker parties, treating ecological sinks as infinite, or classifying coercion as efficiency.

This is not solved by declaring money “backed by real assets” in the abstract. Productive credit can finance future activity; the question is whether the future activity has a plausible cash-flow and physical-capacity path, who carries downside risk, and what happens when the path fails. Financial claims, real capacity, liquidity, solvency, valuation uncertainty, and ecological viability must be modeled separately.

The public evidence makes finite-resource and distributional limits material design inputs, not optional externalities. UNEP’s Global Resources Outlook 2024 reports that global material extraction has tripled over five decades and could rise by almost 60% from 2020 levels by 2060 without change; it also reports strongly unequal resource footprints and impacts across income groups. The 2025 Planetary Health Check reports that seven of nine planetary boundaries have been breached. The IPCC’s AR6 synthesis says equity, social justice, inclusion, and just-transition processes can enable stronger climate action and more sustainable outcomes. These are reasons to design explicit constraints and distributional tests, not evidence that a specific economic OS has already solved them.

### Non-negotiable system invariants

1. **No paper prosperity.** Credit issuance, token issuance, rising asset valuations, refinancing, or ledger transfers alone do not count as need fulfilment or net real wealth creation. Report financial claims and real deliveries as separate quantities.
2. **No invisible rollover.** Every obligation records its principal, unit/denomination, maturity, repayment or settlement source, counterparty, contingency, and loss-allocation rule. Refinancing moves the obligation forward; it does not erase the obligation or count as repayment. Stress scenarios must include correlated defaults and refinancing freezes.
3. **No “unknown equals zero.”** Missing prices, missing resource quantities, stale observations, and failed external attestations remain Unknown. They must not default to a zero cost, zero risk, healthy state, or proven insolvency.
4. **No unlimited physical throughput.** Every material resource or ecological impact relevant to a scenario records a unit, time interval, geography, provenance, and absolute budget/constraint. Efficiency gains must not be used to hide rising absolute throughput. Recycling assumptions need measured recovery rates, quality loss, energy, and material losses.
5. **No export of harm by accounting boundary.** Include relevant upstream/downstream supply-chain impacts, unpaid care burdens, displacement, occupational harms, pollution exposure, and public subsidy where data permit. Report missing coverage and uncertainty; do not mark a cost as internalized merely because a field exists.
6. **No systemic violence as an optimization win.** Forced or unsafe labour, violent eviction, forced displacement, injury or death from preventable workplace/environmental exposure, dispossession, retaliation for dissent, denial of essential needs as leverage, and suppression of exit/appeal are hard failures—not compensable losses that can be offset by aggregate output gains. Record who used or benefited from coercive power, who bore the harm, the evidence and uncertainty, and what remedy is available. Protect informed consent, collective voice, due process, non-retaliatory complaints, and safe exit. A model must not treat violence as an invisible enforcement service that makes a transaction look efficient.
7. **No rights-versus-efficiency trade-down.** Define critical-need floors and rights constraints before optimizing. A policy cannot trade away an individual or group's protected floor merely because aggregate utility or GDP-like throughput increases.
8. **No unaccountable loss socialization.** When a policy or institution gains privately and produces a loss borne by other communities, workers, taxpayers, or future generations, record the beneficiary, affected parties, transfers, and remediation. Make sure the people exposed to the risk have meaningful authority and recourse.
9. **No single global scalar.** Publish the Pareto profile of fulfilment, distribution, risk, ecological throughput, autonomy, and resilience. Any composite score must expose weights and show whether a worse outcome in one dimension was hidden by gains elsewhere.

### Failure tests to add to the frozen scenario corpus

- **Refinancing freeze:** demand falls, maturities cluster, collateral values drop, and lenders stop rolling obligations. Fail the policy if it calls rollover “repayment,” hides arrears, or meets balance-sheet targets only by denying essential needs.
- **Coercive debt enforcement:** a borrower misses payment and an institution can threaten eviction, unsafe work, physical force, or denial of necessities. The system must expose the enforcement mechanism and burden-bearers, require due process and proportionality, and treat rights violations as failures rather than collateral offsets.
- **Collateral-price outage:** an oracle becomes unavailable or returns a stale/degraded value. The system must preserve Unknown, stop new automated risk decisions that need the missing value, and never convert missing evidence into either “healthy” or liquidation evidence.
- **Physical-capacity shortfall:** the financial budget can buy more units than a local water, energy, housing, food, care, or material supply can deliver. Allocation must reserve only verified feasible capacity and surface unmet need, not synthesize supply.
- **Externalized production:** a cheaper supplier has unrecorded pollution, unsafe work, coercion, dispossession, or displacement. Compare apparent cost with the expanded impact ledger; any fundamental-rights violation triggers a hard failure, not a priced offset.
- **Rebound effect:** per-unit resource use falls while total output rises enough that absolute material/energy impact increases. Report both intensity and absolute impact; do not claim ecological progress from intensity alone.
- **Unequal transition:** the policy reduces carbon/material throughput but shifts costs onto low-income households, workers, or resource-poor regions while leaving high-consumption beneficiaries protected. Evaluate who gains, who pays, whether minimum needs remain met, and whether remediation/voice is effective.
- **Unpaid-care substitution:** a nominally efficient allocation reduces public/accounted cost by shifting additional unpaid care and time burden to already overburdened households. Time is a finite resource and its distribution must be measured.

For each test, publish predeclared failure criteria, beneficiary and burden analysis, ecological totals and uncertainty, and the exact decision that was prevented or permitted. An “ethics” section in a README is not evidence that these invariants hold.

### Research references

- [UNEP / International Resource Panel, Global Resources Outlook 2024](https://www.unep.org/resources/Global-Resource-Outlook-2024) — resource extraction trajectories, unequal footprints, and resource-use pathways.
- [Earth Commission, 2025 Planetary Health Check summary](https://earthcommission.org/planetary-health-check-7-of-9-boundaries-breached/) — reports seven of nine planetary boundaries breached.
- [IPCC AR6 Synthesis Report, Summary for Policymakers](https://www.ipcc.ch/site/assets/uploads/2023/03/Doc4_Approved_AR6_SYR_SPM.pdf) — equity, justice, inclusion, and just-transition findings.

These references establish external risk context; they do not establish that Mycelix's implementation satisfies the invariants above.

## Minimum reproducible economic benchmark

Create a versioned scenario corpus before reducing the OS to a single score:

1. **Essential need under scarcity:** competing housing or care needs, limited capacity, eligibility, appeal, cancellation, and resource release.
2. **Local liquidity shock:** supply and demand persist while settlement capacity contracts. Compare no intervention, fixed-limit, and counter-cyclical credit.
3. **Cross-currency exchange:** an offer in one instrument meets a supplier requiring another; include uncertain rates, fees, expiry, failed settlement, and reconciliation.
4. **Commons asset lifecycle:** proposal, authorization, reservation, actual delivery/maintenance, invoice or time credit, treasury accounting, dispute, and audit.
5. **Governance capture:** concentrated wealth/reputation, coordinated voters, duplicate identities, low turnout, emergency action, appeal, and recovery.
6. **Federation partition:** two nodes accept work offline; reconciliation must not create spendable value twice or erase valid disputes.
7. **External-effect crash window:** intent is durable, an effect may or may not have happened, then the process dies before the receipt. No retry until evidence and original-key binding justify it.
8. **Exit and migration:** a person/community exits with open obligations, private data, shared assets, consent, and dispute rights preserved.

Freeze scenario ID/version and corpus digest; initial state and data license; policy/configuration digest and authority; random seed or deterministic schedule; exact software commit, toolchain and environment; baseline and method; invariants and failure thresholds; run receipt, outputs, evaluator digest, and reconciliation evidence.

The evaluator must be independent from the policy implementation and must not infer success from the candidate’s own passed field.

## Metrics: no single “best” scalar

Publish a Pareto profile rather than one weighted winner score. Weights encode value judgments and may hide severe failures.

| Dimension | Minimum observations |
|---|---|
| Fulfilment | Critical-need fulfilment ratio, unmet needs by category, median and tail waiting time, cancellation/failed-delivery rate. |
| Resource productivity | Utilization, idle capacity, spoilage/waste, maintenance backlog, offer-to-fulfilment time. |
| Distribution | Changes in disposable resources and debt burden, concentration, group-level denial/error rates, people below predeclared rights floors. |
| Liquidity/resilience | Payment failures, credit utilization, defaults, arrears duration, shock recovery time, unmet needs during stress. |
| Accounting safety | Instrument-specific conservation/issuance, zero-sum where applicable, reserve constraints, no double spend, no duplicate effect, auditable reversals. |
| Governance | Participation, power/delegation concentration, policy reversals, appeal outcomes, correction time, minority-rights violations. |
| Inclusion | Completion by device/network/accessibility condition, onboarding time, abandonment, exit success, informed understanding of obligations. |
| Sovereignty/privacy | Data disclosed per workflow, metadata leakage, replication exposure, export/revocation success, operator dependence, recovery without global authority. |
| Operations | Latency distributions, availability, crash recovery, resource/storage growth, backup/restore results, support cost per active community. |

Agree the baselines and minimum acceptable outcomes with participating communities before a pilot. If evaluation criteria change after viewing a run, version the criteria and rerun the old and new evaluations.

## Gates toward a credible “best-in-class” claim

**A — Mechanism correctness.** Property/adversarial tests for every currency, pool, credit, governance, and lifecycle invariant. Malformed or missing evidence must fail closed.

**B — Cross-domain vertical slice.** One realistic need-to-fulfilment workflow crosses Commons, Finance, Governance, and transition/reconciliation, with a single auditable chain from authorization to service delivery and accounting. Inject faults at every boundary.

**C — Federation/interoperation.** At least two independently operated nodes complete discovery, authorization, clearing/settlement, dispute, version negotiation, and exit without a single always-online global authority.

**D — Independent simulation.** Run a frozen corpus against explicit baselines and at least one alternative for each comparable task. Publish counterexamples, distributional results, uncertainty, resource use, and evaluator receipts.

**E — Field evidence.** A voluntary, consent-based pilot compares outcomes to a predeclared baseline and reports failure, support burden, participant feedback, and harm. A pilot is not globally representative; state that limit.

**F — Replication.** An independent operator deploys from documented artifacts, replays the corpus, checks results, exports state, and recovers from failure without privileged access to the author’s machine or private data.

CI proves specific automation; it does not by itself pass these system and social evidence gates.

## Next engineering priority

The highest-leverage next step is a **versioned Economic Coordination Contract** shared by Finance, Commons, Governance, and the transition/reconciliation layer—not another currency feature. A proposal is now drafted in [ECONOMIC_COORDINATION_CONTRACT_V1.md](ECONOMIC_COORDINATION_CONTRACT_V1.md); it is a design contract, not implemented behavior. At minimum it needs:

- typed need/offer/resource identities and capacity units;
- authority and policy digests for allocation;
- reservation/commit/fulfilment lifecycle and expiry;
- instrument identity, amounts, units, and conversion profile;
- effect identity, exact request digest, provider profile, and original idempotency-key binding for external effects;
- source evidence and receipt digests;
- dispute/appeal state and unresolved mandatory items;
- explicit Unknown for missing facts rather than a guessed value.

Start with one housing-repair or care-fulfilment scenario. Do not integrate every domain at once, and do not authorize a real external effect until authority, reservation, rights floors, and recovery bind to the same frozen transaction identity.

## Decision

Mycelix should aim to be **a federated, rights-constrained economic coordination OS**, not simply the most feature-rich currency stack. Its potential differentiator is connecting multiple economic instruments to real resources, community authority, and verifiable fulfilment while preserving local sovereignty.

The “best” claim becomes credible only if reproducible evidence demonstrates that the integrated system is safer, more useful, more resilient, or more equitable than relevant alternatives on stated tasks. Until then, it remains a research hypothesis—not a result.
