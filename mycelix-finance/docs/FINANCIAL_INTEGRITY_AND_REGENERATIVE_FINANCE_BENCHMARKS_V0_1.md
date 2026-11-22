# Mycelix Finance Qualification and Comparative Benchmark Programme

**Document ID:** MYC-FIN-BENCH-0.1  
**Status:** Proposed benchmark specification; not yet executed  
**Date:** 2026-11-22  
**Scope:** SAP issuance/redemption, reserve safety, ledger integrity, collateral, lending, investment and cooperative finance

> No result is claimed by this document. All thresholds below are proposed qualification gates or research targets. They must be evaluated against immutable code/data heads and reported with raw evidence, failures and limitations.

## 1. Objective

The benchmark programme asks whether Mycelix can improve the safety, fairness, reach, cost and auditability of financial services relative to matched conventional and digital-finance baselines.

It does not optimize token price, total value locked, raw throughput or loan approval rate in isolation.

A system is better only if it improves the relevant outcome without degrading monetary integrity, legal enforceability, borrower protection, privacy, resilience or independent auditability.

## 2. Evaluation arms and fair comparisons

| Domain | Baseline | Mycelix candidate | What must be held equal |
|---|---|---|---|
| Payment ledger | Serializable relational double-entry ledger with reconciliation controls | Holochain agent-centric finance ledger using TEND-derived conservation patterns | Same events, concurrency, retries, outages, finality definition and hardware |
| Payment orchestration | Conventional API/PSP flow with idempotency and bank reconciliation | Mycelix intent-bound payment and settlement workflow | Same bank rail, amount, counterparties, settlement finality and exception rate |
| Monetary backing | Fully disclosed regulated reserve-backed payment claim / stablecoin arrangement | SAP-ZAR reserve architecture | Same legal claim, currency denomination, redemption SLA, reserve eligibility and stress scenario |
| Collateral lending | Conventional secured loan / warehouse receipt finance | Mycelix evidence-backed productive collateral | Same asset class, title, haircut, LTV, enforcement rights and recovery market |
| Credit underwriting | Rule-based affordability + conventional bureau/financial record baseline | Cash-flow/evidence underwriting with optional advisory model | Same applicant cohort, features legally available, time split, decision thresholds and loss horizon |
| Investment project finance | Conventional project-finance due diligence, escrow and milestones | Evidence-carrying, milestone-based Mycelix project finance | Same project, capital stack, security, monitoring, contract and outcomes |
| Transparency | Existing audit statements and financial reporting | Signed receipts, provenance and verifiable reconciliation | Same records, privacy constraints, audit scope and reviewer access |

Do not compare a local community credit network with a global card network on raw TPS. Compare the exact economic task and disclose system boundary.

## 3. Benchmark manifest

Every run must publish or privately escrow a machine-readable manifest containing:

- benchmark ID and semantic version;
- repository, branch, exact commit SHA and build digest;
- test harness version and dependency lock digest;
- operating system, hardware, runtime and network profile;
- input corpus digest, synthetic/real label, data rights and cohort definitions;
- random seed(s), generator version and scenario counts;
- policy, oracle, model and schema versions;
- baseline configuration and parameter parity;
- expected invariants and scoring rules;
- start/end time and run environment;
- raw result artifact digest;
- failures, skips, timeouts and inconclusive runs;
- independent reviewer and reproduction status.

A queued, skipped, cancelled or partial CI job is not a pass. A result on a different head is not evidence for the current head. A benchmark that omits failures or denominator counts is invalid.

## 4. Monetary integrity and settlement

### 4.1 Properties

Required invariants:
- internal transfer is net-zero;
- issuance has one valid author-bound issuance receipt;
- redemption reduces liabilities only after a final and reconciled external outcome;
- escrow/locked/frozen/pending units remain in supply until legally extinguished;
- repeated idempotency keys cannot create new side effects;
- conflicting concurrent spends cannot both commit;
- coordinator shortcuts cannot bypass integrity-layer authorization/conservation;
- treasury and reserve balances reconcile to the same canonical liability model.

### 4.2 Qualification scenarios

1. Duplicate mint intent delivered concurrently to 100 agents.
2. Same issuance receipt replayed across independent source chains.
3. Concurrent transfers spend the same balance.
4. Transfer commits but acknowledgement is dropped.
5. External bank transfer succeeds but callback times out.
6. External bank transfer fails but callback arrives late.
7. Redemption is retried with the same and then a different payload under the same key.
8. Two liabilities are created from the same asset/reserve reference.
9. Coordinator bypass attempts raw credit_sap or mutable balance update.
10. DHT/network partition while a pending balance adjustment exists.
11. Node restart during prepare, commit and reconciliation.
12. Governance/custodian key compromise and revocation.
13. Duplicate, out-of-order or stale ledger checkpoints.
14. Currency retirement while pending, escrowed and disputed balances exist.

### 4.3 Hard gates

- **Zero** unauthorized issuance, duplicate issuance, double spend or supply-conservation violation in the defined adversarial corpus.
- **100%** of money-moving external operations bind to exact intent, actor, amount, recipient, currency, policy version and idempotency key.
- **100%** of unknown outcomes remain indeterminate until reconciled.
- **100%** of reserve/liability snapshots reconcile with explicit treatment of pending, escrowed, frozen and disputed liabilities.
- Any invariant failure blocks qualification; it is not averaged against throughput.

Run at least 1 million generated state transitions, including bounded concurrency and fault injection, before a production claim. Publish actual counts and seeds; this is a target corpus size, not a claim that it has been run.

## 5. SAP reserves, liquidity and redemption

### 5.1 Measures

Report:
- gross redeemable liabilities;
- eligible liquid reserve assets by category, currency, custodian and maturity;
- asset haircuts and encumbrances;
- reserve coverage ratio;
- same-day / next-day / 7-day liquidity coverage under stated scenarios;
- reserve concentration and related-party exposures;
- redemption requests, completion times, failures, reversals and disputes;
- reconciliation exceptions and stale attestations.

### 5.2 Qualification targets

| Measure | Proposed gate |
|---|---|
| Reserve coverage | At least 100% of all redeemable liabilities after haircuts and encumbrance deduction |
| Asset double-pledge | Zero accepted duplicates |
| Liability completeness | 100% of ledger liabilities reconciled to the reserve snapshot |
| Redemption completion | At least 99.9% within the published SLA in normal test conditions; all failures explicitly accounted for |
| Unknown payout outcome | Zero blind retries or double payments |
| Reserve attestation freshness | 100% within the published cadence; stale assets become ineligible |
| Insolvency/recovery | Independently reviewed recovery, custody and resolution plan exists before launch |

A 100% reserve coverage ratio is necessary for the proposed SAP-ZAR model, not sufficient. It does not prove immediate liquidity, legal segregation or par redemption under stress. The liquidity ladder and independent legal/custody review are separate gates.

Stress scenarios:
- 10%, 25% and 50% redemption requests in one business day;
- bank/custodian outage for 1, 3 and 5 business days;
- sovereign instrument price haircut shock;
- operational settlement cut-off missed across a weekend/holiday;
- stale, incorrect or colluding reserve reports;
- issuer insolvency and custodian insolvency;
- FX disruption for cross-currency liabilities.

These are proposed stress magnitudes, not empirical forecast probabilities. Add sensitivity analyses and explain the rationale for the selected shocks.

## 6. Productive collateral and asset finance

### 6.1 Asset classes

Test independently by class:
- warehouse-held grain/produce;
- energy certificates and generation equipment;
- vehicles and machinery;
- property;
- inventory;
- contracted receivables;
- construction materials.

Do not pool incomparable collateral into one headline recovery rate.

### 6.2 Required measures

- title/ownership validation rate;
- duplicate pledge detection rate;
- independent valuation error;
- valuation staleness;
- custody/insurance failure rate;
- realized liquidation time and proceeds;
- legal recovery cost and duration;
- loss given default by asset class;
- fraud incidence and false positive holds;
- percentage of loans with perfected, enforceable security.

### 6.3 Gates

- No loan may rely on collateral whose ownership, pledge, custody or release cannot be established.
- No LTV bonus is awarded solely because an asset position contains multiple token categories.
- Haircuts must be based on observed or defensibly stressed liquidation data.
- Collateral release requires proof of discharge and a separate signed release event.
- An oracle outage blocks new lending against affected assets; it must not silently reuse a stale price.

## 7. Credit underwriting and fairness

### 7.1 Baselines

At minimum compare:
1. conventional bureau/financial-data score where lawful and available;
2. transparent affordability and rule-based underwriting;
3. cash-flow and verified-business-record model;
4. human underwriter using the same evidence pack;
5. Symthaea-assisted recommendation under human approval.

Do not train/test split at random when observations from the same borrower or business can leak across sets. Use time-based and entity-grouped holdouts, including out-of-time validation.

### 7.2 Predictive and economic measures

Report by loan product and cohort:
- AUROC and AUPRC, with class prevalence;
- Brier score and calibration plots;
- expected calibration error and confidence intervals;
- false approval/default rate;
- false rejection among applicants who would repay;
- expected loss versus actual realized loss;
- default and recovery rates by vintage;
- loss given default;
- cost per application and cost per performing loan;
- approval, take-up, delinquency and restructuring rates;
- time to decision and time to disbursement;
- adverse action/decision reason completeness.

AUROC alone is not an acceptance criterion. A model can rank applicants well while being badly calibrated or economically harmful.

### 7.3 Proposed quality gates

- ECE target ≤ 0.05 overall after calibration, with confidence intervals and cohort-level report; if sample size is inadequate, label the result inconclusive.
- Brier score must improve over the matched transparent baseline before the model is used to change lending decisions.
- No model deployment if expected loss worsens materially after fees, collections and operational costs.
- 100% of declined/changed offers have reason codes, evidence references and a human appeal route.
- Report false rejection and false approval disparities for legally and ethically appropriate cohorts, subject to privacy and minimum sample sizes.
- Proposed research flag: investigate any unexplained cohort gap >5 percentage points in false rejection, false approval or appeal reversal. This is an internal investigation trigger, not a universal legal fairness threshold or safe harbour.
- MYCEL or other reputation signals must be ablated. Demonstrate incremental value and harm before retaining them.

### 7.4 Out-of-distribution and abstention tests

- new borrower with no history;
- seasonal/freelance cash flow;
- informal business with partial records;
- sudden macroeconomic shock;
- incomplete or stale income statement;
- forged invoice or colluding customer;
- identity theft and synthetic identity;
- business and personal cash flows commingled;
- repayment failure caused by verified disaster/force majeure.

The model must be able to abstain, request more evidence or route to human review. Missing history is not a negative moral judgment.

## 8. Investment and project finance

Test:
- milestone release against exact contract;
- evidence spoofing and sensor disagreement;
- sponsor/contractor related-party conflicts;
- project schedule and cost overrun;
- energy/resource output below plan;
- cash-flow shortfall and distribution waterfall;
- investor transfer restrictions and beneficial ownership;
- valuation update and stale NAV;
- dispute, default, restructuring and insolvency.

Measures:
- funds disbursed versus independently verified milestone value;
- misallocated disbursements;
- project cost/schedule variance;
- operating cash flow versus forecast;
- distributions reconciled to contract;
- recovery and loss severity;
- investor reporting latency and completeness;
- beneficial-owner and rights record completeness.

Hard gate: no automated disbursement against a self-authored or self-verified milestone. Every milestone release has a separate evidence and authority chain.

## 9. Reliability, recovery and privacy

Test:
- database/source-chain restart at every state-machine boundary;
- DHT partition and partial quorum;
- custodian API unavailable;
- oracle conflict and common-source failure;
- stale credential or revocation service;
- key compromise and rotation;
- reconciliation of unknown outcomes;
- schema migration and historical replay;
- data export and verified exit;
- deletion request with minimal lawful audit-retention requirements;
- unauthorized data joins and inference from public records.

Report p50/p95/p99 latency, throughput, cost per successful operation, resource use, availability, recovery time and operator burden under a named workload.

Performance is measured only after correctness. A faster system that accepts duplicate issuance or loses a payment is worse.

## 10. Benchmark comparisons to industry

### Reserve-backed payment instruments

Compare the SAP-ZAR claim to the current BIS/FSB/IFWG expectations for reserves, redemption rights, liquidity, custody, capital, disclosure, recovery and resolution. Do not claim to outperform stablecoins until the same legal and operational properties exist.

### Conventional credit

Compare to transparent rule-based and bank-style underwriting, not just a weak or opaque credit score. Demonstrate improved access and/or reduced cost without increasing loss, discrimination or borrower harm.

### DeFi lending

Compare collateral valuation, liquidation, oracle dependencies, custody, governance, permissioning and insolvency rights. A lower collateral ratio is not a benefit if it is achieved by shifting hidden loss to another party.

### Cooperative and warehouse-receipt finance

Compare accessibility, local knowledge, asset verification, liquidation experience, community governance and resilience. Existing cooperative/warehouse-receipt finance is prior art; Mycelix should integrate and improve it rather than claim to invent it.

## 10.1 Position against leading financial infrastructure

### Wholesale tokenized settlement: integrate with BIS Project Agorá

Project Agorá has already demonstrated a prototype for wholesale cross-border payments using tokenized commercial-bank deposits and tokenized central-bank reserves on a shared programmable platform. The BIS and participating central banks have reported participation by more than 40 financial institutions. It is aimed at atomic multi-currency wholesale settlement, not community-scale retail credit or asset finance.

Mycelix should not claim to beat Agorá at central-bank-money settlement. The stronger strategy is to interoperate with regulated banks, PSPs and future wholesale tokenized-settlement rails. Mycelix's differentiation should be the upstream/downstream layer: evidence-backed productive collateral, small-business and cooperative financing, authority-bound agents, physical-project verification, local mutual credit, and portable community governance.

Benchmark the external rail as a dependency: exact settlement-finality semantics, fees, availability, counterparty risk, reconciliation delay, FX transparency and failure/recovery behavior. A token wrapper around a bank transfer does not create atomic settlement.

Sources:
- BIS Project Agorá prototype outcome, 27 May 2026: https://www.bis.org/press/p260527.htm
- BIS Project Agorá project page: https://www.bis.org/about/bisih/topics/cbdc/agora.htm

### Stablecoins and tokenized money: exceed the assurance perimeter, not the ticker

The FSB's 2025 peer review found material gaps and inconsistent implementation across jurisdictions for crypto-asset and global stablecoin recommendations, including reserve, redemption, liquidity, governance, service-provider and cross-border supervisory issues. The South African IFWG's 2026 consultation on rand-pegged stablecoin arrangements focuses attention on reserves, redemption/stabilization, operational and cyber risk, disclosures and financial interlinkages.

SAP-ZAR should therefore be compared on an instrument-equivalent basis:
- identical legal claim and redemption promise;
- identical currency denomination and redemption window;
- comparable eligible reserves, liquidity horizon, concentration and haircuts;
- comparable custody, segregation, insolvency and recovery arrangements;
- independent liability reconciliation and public disclosures.

A 100% reserve arithmetic test is not enough. The launch claim must include enforceable holder rights and evidence of actual custody and liquidity. Do not compare SAP-ZAR with an unbacked or non-redeemable token and then claim superiority.

Sources:
- FSB 2025 implementation review: https://www.fsb.org/2025/10/fsb-finds-significant-gaps-and-inconsistencies-in-implementation-of-crypto-and-stablecoin-recommendations/
- IFWG, 2026 rand-pegged stablecoin arrangements: https://www.ifwg.co.za/Reports/Functional%20analysis%20of%20existing%20regulatory%20frameworks%20and%20rand-pegged%20stablecoin%20arrangements.pdf

### Credit risk: complement Basel controls rather than replace them with an AI score

Basel's 2025 credit-risk principles organize the discipline around an appropriate credit-risk environment, sound granting, ongoing administration/measurement/monitoring, and adequate controls. Mycelix should preserve those core disciplines while testing whether richer cash-flow, contracted-revenue and asset-condition evidence improves access and loss prediction for underserved micro and small enterprises.

The comparison must include an affordability/rule-based baseline and a human-underwriter arm. Symthaea is an advisory model, not a bank's credit officer or legal decision-maker. Report expected loss, calibration, false rejection, borrower outcome, operational cost and fairness—not only discrimination metrics or application approval rate.

Sources:
- Basel Committee, Principles for the Management of Credit Risk (2025): https://www.bis.org/publications/202504-guidelines-principles-management-credit-risk
- Basel consolidated credit risk management guidance (1 January 2026): https://www.bis.org/committees/bcbs/basel-consolidated-guidelines/module/cri/10

### Productive collateral: build on warehouse-receipt finance, do not claim to invent it

Warehouse-receipt finance and movable-asset registries already provide established precedents for financing inventory and equipment. Mycelix's research claim should be narrower and testable: can the common evidence/authority layer reduce duplication, improve title/custody verification, shorten reconciliation and enable more asset classes without increasing loss or legal uncertainty?

Compare by asset class against actual recovery time, liquidation discount, custody failures, title disputes, duplicate pledges, financing cost and borrower inclusion. A digital receipt alone is not a perfected security interest.

### Mycelix's proposed competitive wedge

The candidate advantage is a cross-domain contract from verified asset or business activity to accountable financing and reconciled repayment:
1. prove identity/authority and consent;
2. verify the underlying business or asset evidence;
3. assess affordability, projected cash flow and downside scenarios;
4. disclose legal rights, fees, loss allocation and recourse;
5. fund through a regulated lender/settlement partner;
6. observe delivery/performance independently;
7. reconcile payments, hardship, restructuring and collateral release;
8. export a portable, privacy-preserving evidence history.

This is a research hypothesis. It only becomes an advantage if a matched field pilot demonstrates lower cost/time, wider appropriate access, comparable or lower realized losses, better borrower outcomes, and no increase in fraud, privacy harm or legal failure.

## 11. Research targets and claims policy

A benchmark result qualifies only if:
- test scope and denominator are defined before execution;
- code and data are pinned by digest;
- baseline receives the same task and comparable inputs;
- failed, skipped and inconclusive runs are visible;
- confidence intervals and sensitivity are reported;
- independent replication is possible;
- the conclusion is limited to the tested profile.

Do not say "better than industry" because a synthetic happy-path test passes. Use narrower claims such as "zero double-spends in 1,000,000 generated transitions under the tested fault model" only after that exact experiment is run and independently reproducible.

## 12. Research and implementation order

1. Re-audit exact-head SAP issuance, payment, reserve, bridge and redemption code.
2. Turn each known issue into a failing negative control.
3. Freeze an immutable scenario corpus.
4. Rebuild SAP conservation and reserve liability model.
5. Test collateral lock/release and duplicate-pledge prevention.
6. Add loan lifecycle with receipt-bound repayment.
7. Add cash-flow underwriting baseline/ablation before AI-assisted decisions.
8. Add investment milestones and payout reconciliation.
9. Run paired comparisons and publish complete scorecards.
10. Invite an external financial auditor, security reviewer and borrower/community advocate to review the results.

## 13. Machine-readable scenario corpus

The initial SAP adversarial corpus is published at https://github.com/Luminous-Dynamics/mycelix/blob/research/web4-regenerative-finance-v0.1-20261122/mycelix-finance/benchmarks/sap/sap-adversarial-scenarios.v0.1.json. It defines 14 replay, concurrency, reserve, collateral, key-compromise and recovery scenarios. The corpus is specification-only; no execution result is claimed. Each scenario must be converted into a harness test with exact-head source references, expected state, negative control and raw receipt evidence.

## 14. Research references

- Basel Committee, Principles for the Management of Credit Risk (30 April 2025): https://www.bis.org/publications/202504-guidelines-principles-management-credit-risk
- BIS, General Credit Risk Management, consolidated guidance (1 January 2026): https://www.bis.org/committees/bcbs/basel-consolidated-guidelines/module/cri/10
- BIS, Annual Economic Report 2026, "Anchoring trust in money": https://www.bis.org/publications/aer-2026/anchoring-trust-money
- FSB, 2025 stablecoin regulatory implementation review: https://www.fsb.org/2025/10/fsb-finds-significant-gaps-and-inconsistencies-in-implementation-of-crypto-and-stablecoin-recommendations/
- IFWG, 2026 rand-pegged stablecoin arrangements discussion paper: https://www.ifwg.co.za/Reports/Functional%20analysis%20of%20existing%20regulatory%20frameworks%20and%20rand-pegged%20stablecoin%20arrangements.pdf
- South African Prudential Authority, functions and cooperative financial institution requirements: https://www.sarb.co.za/en/home/what-we-do/Prudentialregulation/functions-of-the-prudential-authority
- W3C, Verifiable Credentials Data Model 2.0: https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- IETF RFC 9943, SCITT architecture: https://www.rfc-editor.org/rfc/rfc9943
