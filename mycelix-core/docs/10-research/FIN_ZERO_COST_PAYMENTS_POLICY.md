# FIN-ZERO-COST-001 — Transparent, sustainable near-zero-fee payments

**Status:** research and design proposal; no payment route is authorized or claimed live.  
**Owner boundary:** finance/payments policy and evidence; not a bank, payment service provider, FX desk, settlement guarantee, or SAP issuer.  
**Issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5322

## Thesis

Mycelix should aim to eliminate direct user fees for ordinary, low-risk payments wherever the economics permit, and reduce all-in payment costs wherever they do not. This is a systems goal—not a “free” marketing claim. Remove avoidable intermediaries, duplicated onboarding, hidden FX markups, manual reconciliation, and avoidable failure handling. Make unavoidable costs visible and contestable.

Initial product hypothesis: **R0 Mycelix platform fee for a defined standard domestic-ZAR person-to-person flow**, subject to an audited cost-coverage model and a qualified partner/route. Any provider or rail charge must be disclosed, with its payer identified. This target does not mean every transfer is costless.

## Four independent fee/cost coordinates

1. **Payer charge:** amount explicitly charged to the sender.
2. **Recipient charge:** any amount deducted from or billed to the recipient.
3. **Embedded price cost:** FX spread/markup and quote-to-execution deterioration against a timestamped reference.
4. **System resource cost:** provider and rail charges, liquidity, fraud/loss, compliance, support, disputes, reconciliation, capital and resilience cost, where observable.

A zero in one coordinate is not evidence that the others are zero. Attribute each observed cost to a payer or label it unknown. Never silently convert “unknown” into “zero.”

## Evidence motivating this work

- The BIS Working Paper 1385 (published 2026-10-19) compares bank wires, fintech money-transfer operators and stablecoin-based transfers. Within its small bank-transaction sample, FX markups were approximately 2%–4.7% before explicit wire charges; fintech transfer costs were generally lower, while stablecoin paths still incurred conversion, trading, withdrawal, liquidity and fiat-access costs. This is a study result, not a universal quote or guarantee. https://www.bis.org/publications/working-paper-1385-competing-rails-crossborder-payments-banks-fintechs-and-stablecoins
- BIS Working Paper 1295 (2025-10-06) reviews fast-payment pricing and notes zero fees at the end-user level may be unsustainable without subsidies or alternative revenue. https://www.bis.org/publications/working-paper-1295-pricing-fast-payments-practical-and-theoretical-overview
- FSB G20 targets for relevant cross-border retail payments set an average cost no higher than 1% and no corridor above 3% by end-2027. Remittance targets are separate: no more than 3% on average to send $200 by 2030, with no corridor above 5%. These are sector targets—not Mycelix results or an SLA for every use case. https://www.fsb.org/work-of-the-fsb/financial-innovation-and-structural-change/cross-border-payments/g20-targets-for-enhancing-cross-border-payments-2/
- Reuters reported on 2026-10-16 that South Africa plans national payment-system access for eligible non-bank participants from 2027, after rulemaking, licensing and integration. Treat the exact path as contingent on final SARB rules and qualification. https://www.reuters.com/world/africa/south-africa-open-payments-system-retailers-telecoms-fintechs-2027-2026-10-16/
- BIS Nexus describes a model for connecting domestic instant-payment systems. Interoperability is a potential cost lever; it does not prove that an exact country pair, provider, asset, recipient or direction is live or free. https://www.bis.org/publications/nexus-enabling-instant-cross-border-payments

## Paradigm shifts

### 1. From “fee-free” to auditable cost truth

Compare total cost per successful delivered payment, not just the headline fee. Include explicit payer and recipient charges, embedded FX spread, intermediary and rail costs, quote slippage, failure/return costs, disputes, and amortized operational cost when evidence allows. Show denominators, sample sizes and unknown fields.

### 2. From a proprietary rail to a provider-neutral optimizer

Compare authorized routes for the exact amount, pair, direction, payer/recipient profile, purpose, timing, payout endpoint and settlement conditions. Optimize risk-adjusted delivered cost and the probability/time of funds availability—not just indicative FX rate. Banks and fintechs remain candidates, and Mycelix must be free to recommend the incumbent or abstain.

### 3. From duplicate controls to portable evidence—with accountability intact

Explore consent-based, scoped, revocable reuse of identity/business/recipient verification evidence to reduce repeated onboarding and exception costs. Reuse cannot waive each provider’s obligations, sanctions or currentness checks, or accountability. Avoid a universal, unbounded KYC dossier.

### 4. From transaction-fee dependence to explicit cost sharing

Possible sources that must be evaluated and disclosed: business/treasury APIs; reconciliation and workflow services; optional enterprise SLAs; disclosed merchant-side services only when total merchant cost remains competitive; grants/public-benefit subsidy; and verified provider-volume discounts passed through to users. Optional premium capabilities must not coerce users away from an essential basic service.

Explicitly reject hidden FX spread, undisclosed recipient fees, cross-subsidies that conceal total merchant cost, sale of identifiable transaction data, abusive overdraft/credit, and business models dependent on speculative float/interest income or unrealized volumes. Model who pays, who benefits, and failure/sensitivity cases.

### 5. From isolated instructions to a full payment lifecycle

Preserve distinct states and evidence for quote → consent → instruction → provider acceptance → execution → settlement → recipient availability → reconciliation → return/dispute. Automate reconciliation and predictable exceptions. Timeout means unknown until resolved, not automatically failed or successful; retries must be idempotent.

## Hard boundaries

- Begin with read-only fee/route comparison and a normalized fee receipt; then effects-disabled simulation; only then consider a narrowly scoped partner-operated domestic flow.
- No funds movement, custody, deposit-taking, credit, FX principal risk, settlement guarantee or live payment authorization is created by this document.
- SAP issuance remains governed by #5313/#5314 and the existing AC-102 (#4128), AC-154 (#4589), AC-176 (#4519), FIN-ECO-007 (#1157), interoperability (#3343) and compliance (#2437) work. No fee-reduction goal can bypass the fiat-to-SAP gate.
- A route must fail closed when authorization/eligibility, beneficiary reachability, quote freshness, liquidity, price, or settlement conditions are unknown or invalid.
- Before any live South African payment activity, verify the final SARB PEM rules and the exact licensing/partner requirements; use qualified, activity-specific legal/compliance review.
- No BRICS/African corridor is assumed enabled because of a declaration, membership, standard or provider’s global coverage claim.

## Benchmark protocol

Freeze the cohort, amount bands, exact pair/direction, timestamp window, payout endpoint, fee schedule, FX reference method, provider access, quote age, policy/code version, sample counts and outcome definitions. Match conditions across Mycelix, direct provider, and incumbent/bank alternatives. Count failed/returned/unknown cases. Do not compare indicative quotes to executed settlement outcomes.

### Cost measurements

- Total all-in cost per successful payment: absolute currency and basis points.
- Payer charge, recipient deduction, merchant/rail/provider cost where observed.
- FX spread against a named, timestamped reference; quote-to-execution slippage; intermediary deductions.
- Failure, return, reversal and unknown-outcome cost.
- Fully loaded cost per 1,000 successful payments: provider, integration, compliance operations, fraud/loss, support, dispute and reconciliation costs.
- Cost distribution by amount, route/corridor, provider and user segment; mean alone is insufficient.

### Outcome and safety measurements

- Quote itemization completeness, freshness, expiry and final-cost deviation.
- Instruction-to-beneficiary-available p50/p95/p99 where timestamps and sample sizes support it.
- Provider acceptance, completion, rejection, return/reversal, timeout and unknown-outcome rates.
- Duplicate instructions, payments or credits; unreconciled ledger deltas; median/p95 repair time.
- Provider outages, recovery and failover; fraud/loss and appeal outcomes; complaints and refund/dispute completion.
- Effective access: who cannot use the cheaper route and the evidence-backed reason.

## Proposed initial qualification goals (not achieved results)

1. **Full transparency:** 100% of quotes itemize explicit fees, disclose FX benchmark/spread and timestamp, expected delivery window, expiry, relevant exclusions, and expected recipient amount—or explicitly mark unknown fields.
2. **No hidden charge:** zero undisclosed Mycelix markup or undisclosed recipient/intermediary charge in the frozen conformance corpus.
3. **Zero platform fee:** R0 Mycelix platform fee on the specified domestic-P2P pilot path, covered by an explicit, audited budget. Pass-through fees stay visible.
4. **Demonstrate cost reduction:** use a paired comparison. A 30% reduction in all-in user cost versus a matched incumbent cohort is a stretch hypothesis for evaluation, not an asserted result or deployment requirement. If evidence shows no improvement, say so or recommend the incumbent.
5. **Safety:** zero duplicate side effects in the frozen retry/replay/unknown-outcome corpus; zero false claims of completion/finality; every planned case has a result or explicit invalid/unknown disposition.
6. **Reliability:** a provisional live-pilot threshold of at least 99.5% successful completion for eligible, non-fraudulent, in-profile instructions over a declared window, with rejected, user-caused, provider-caused and unknown outcomes reported separately. This is not a consumer guarantee until evidence supports it.
7. **Unit economics:** signed-off cost-coverage model, per-transaction marginal cost, subsidy source and cap, sensitivity/break-even analysis and stop conditions before a live pilot. No launch based on unreviewed reserve use, customer-fund float or hypothetical discounts.

Targets are gates to test and refine, not proof of current performance.

## Minimum synthetic corpus

Cover domestic standard and instant rails; micro and large amounts; stale/expired quotes; hidden FX markup; recipient/intermediary charges; failed payout; timeout after provider acceptance; duplicate callback; partial execution; reversal after apparent success; provider outage; price change; unavailable liquidity; eligibility mismatch; screening false positive; corrected identity/recipient evidence; dispute/chargeback; and a high-cost case where the optimizer must abstain.

Begin with synthetic fixtures. Live data must be lawful, minimized, purpose-limited and appropriately protected.

## Deliverables

1. Language-independent fee/cost component schema and quote/receipt semantics.
2. Frozen synthetic corpus with reference calculator.
3. Provider-neutral route-selection baseline with explicit constraints and abstention.
4. Quote conformance tests for hidden spread, fee shifting, stale rates and missing data.
5. Domestic-ZAR pilot unit-economics model and legal/product gate checklist.
6. Public benchmark report with source dates, conditions, counts, uncertainty, failure cases and exact claim ceiling.
7. Later corridor-by-corridor comparison of selected African/BRICS-related routes only once individually qualified.

## Acceptance criteria

- Never claim “zero cost” merely because the platform fee is zero.
- Never choose a route if authorization/eligibility, beneficiary reachability, quote freshness, liquidity or risk conditions fail or are unknown.
- Never optimize a single “cheapest” scalar without delivery, failure and risk constraints.
- Attribute every measured cost to a payer or say unknown.
- Make the matched inputs, cohort, versions, results and failure cases reproducible.
- Make no unverified claim of superiority to banks, fintechs, stablecoins, instant payment systems or a provider.
- No live movement of funds is authorized by this research.

## Related work

- Policy: #5310 / PR #5311.
- Fiat-to-SAP remains disabled pending proof: #5313 / PR #5314.
- Related blockers: #1157, #3343, #2437, #4128, #4519, #4589, #4460, #4472, #4475.

## Claim ceiling

This is a research and design proposal. It does not establish lower fees today, live rail availability, a licensed Mycelix payment service, sustainable subsidy economics or lawful operation of any specific activity. Those require evidence and relevant authorization.