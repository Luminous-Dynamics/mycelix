# SAP, Fiat Exchange, and BRICS Corridor Policy — Proposal

**Status:** proposed product/governance policy for review; not legal advice, not an authorization to handle customer funds, and not proof of implemented controls.  
**Prepared:** 2026-10-20  
**Owner:** Mycelix Finance/Economic Fabric maintainers, with qualified jurisdiction-specific legal/compliance review required before live deployment.  
**Decision record:** [FIN-EXCHANGE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5310)

## Executive recommendation

1. **Do not make Mycelix an FX dealer, principal market maker, customer-money custodian or multilateral FX venue in the initial release.** Build an interoperability, quote-comparison, compliance-evidence, payment-coordination and reconciliation layer. Actual FX and fiat movement should be performed by a provider authorized for the transaction and jurisdiction.
2. **Default launch policy: no direct fiat-to-SAP purchase and no SAP-to-fiat redemption.** Let users pay merchants in fiat through approved payment rails. Keep SAP issuance separate, with a typed and authorized program/issuer cause (for example, a qualifying grant, contribution, reward or bounded program allocation) and a clear statement of its actual rights.
3. **Do not confuse a one-way on-ramp with a safe or unregulated one.** One-way fiat-to-SAP may be reconsidered only after a defined product purpose, rights, transferability, issuer obligation, use scope, refund/chargeback rules, accounting/tax treatment and legal classification have been reviewed and the implementation gates below are satisfied.
4. **Treat BRICS as a set of country/currency/corridor opportunities, not as one currency or one universal operating payment rail.** Support exact local-currency corridors through qualified providers as they become legally and operationally available. Do not infer corridor availability from a BRICS label or political declaration.
5. **Keep SAP's possible future reserve-standard ambition separate from today's SAP semantics.** The existing [SAP reserve-standard constitution proposal (#4109)](https://github.com/Luminous-Dynamics/mycelix/issues/4109) is a research trajectory, not an approved promise of convertibility, market liquidity, redemption or reserve status.

These decisions aim to preserve user trust and optionality while reducing legal, liquidity, settlement, and implementation risk.

## 1. Separate the objects before deciding how they exchange

Mycelix must not model all of the following as interchangeable:

| Object | What it actually represents | Must not be silently promoted to |
|---|---|---|
| ZAR/USD/CNY/INR/BRL/etc. | A sovereign fiat unit under an exact currency/unit profile | A bank deposit, cash balance or settlement claim |
| Bank deposit | A claim against a particular bank under its legal and account terms | Physical cash or guaranteed immediate cross-border funds |
| SAP | A Mycelix-defined instrument whose exact rights depend on its approved profile | ZAR, a stablecoin, legal tender, investment, bank deposit or redeemable claim |
| FX observation | A market-data observation with origin and time | An executable offer |
| Indicative conversion rate | A price indication under stated assumptions | A firm quote or best-execution guarantee |
| Firm provider quote | A bounded provider offer for exact parties, asset pair, direction, amount and validity interval | A completed trade or final settlement |
| Accepted payment instruction | An instruction accepted by a payment provider | Cleared funds, a completed transfer or legal discharge |
| Provider trade confirmation | Evidence of the provider's reported execution outcome | Final interbank settlement unless its profile explicitly establishes that |
| Settlement receipt | Evidence about a named payment/settlement step under a named rail/profile | Universal finality across every underlying system |
| SAP issuance receipt | Evidence of a typed, authorized, one-time issuance cause | Evidence that a foreign fiat deposit exists unless it is bound to qualified external settlement evidence |

Compose existing semantic owners; do not invent a parallel asset, FX, quote, issuance, settlement or authority engine.

## 2. Should Mycelix operate forex?

### Initial answer: no; coordinate authorized providers

Mycelix should own the interoperability and evidence layer, not the principal FX risk, the liquidity book, or the customer's external funds.

In the initial scope Mycelix may, after the appropriate authorization and provider contracts exist:

- compare provider quotes on an apples-to-apples, time-bounded basis;
- show the reference rate, quoted rate, spread, explicit fees, provider/rail costs and estimated total delivered amount;
- bind a quote to the exact customer/order context, asset pair, amount, direction, provider, fees, expiry and policy profile;
- coordinate an explicitly authorized instruction through a provider API;
- preserve quote, provider execution, payment and settlement receipts as distinct states;
- reconcile expected versus observed legs and surface partial, returned, reversed, disputed and unknown outcomes;
- export a reviewable evidence packet and support a customer's ability to challenge the result.

The initial product should **not**:

- take the opposite side of the user's trade or hold FX inventory;
- promise a fixed or guaranteed exchange rate without the provider's qualifying quote and execution commitment;
- custody fiat or pool user balances to facilitate exchange;
- operate an FX order book, proprietary trading desk, AMM or liquidity pool;
- promise immediate liquidity, guaranteed convertibility or a universal route;
- infer execution authorization from a model recommendation, valid signature, user login or accepted payment message;
- report a provider's acceptance response as final settlement without evidence for that exact state.

A future decision to operate a venue, principal market maker, custody service, redemption facility or liquidity pool must be a separate programme with qualified legal advice, licensing/authorization determinations, capital/liquidity, counterparty and settlement limits, operational resilience, surveillance, conflicts-of-interest controls and independent governance.

### Why this is also a better engineering strategy

Provider neutrality should be proved by materially different provider profiles, not by a generic interface written around one integration. For any abstraction, preserve the provider-specific semantics when fields, states, quote firmess, fees, operating hours, cancellation, finality, or supported jurisdictions differ. A common type may narrow claims; it must never widen them.

## 3. Fiat -> SAP without SAP -> fiat?

### Recommended default: neither at initial launch

Keep the normal merchant payment in fiat. If a user qualifies for SAP under a separate programme, issue SAP through that programme's typed authorization and evidence. This avoids needing a general-purpose exchange ratio merely to let people pay for goods and services.

If a future use case requires fiat -> SAP, it must be a named, bounded product rather than an implicit property of every SAP balance. Before enabling it, review at least:

- what the user receives and whether SAP is transferable or spendable outside a bounded network;
- whether the issuer owes goods, services, repayment, redemption or other performance;
- whether any statement, UI or marketing could create an implied redemption or price-stability promise;
- whether balances expire, whether fees apply, and who bears value-loss, service-provider, insolvency and merchant acceptance risk;
- how refunds, chargebacks, mistaken payments, disputed receipts and reversed external payments are handled;
- consumer notices, accessibility, complaint handling and evidence that users understand non-redemption and transferability terms;
- accounting, taxation, safeguarding/reserve questions, and applicable payments/e-money/financial-services/exchange-control obligations for each jurisdiction.

A product cannot avoid legal classification simply by disabling the reverse button. Economic function, rights, use, transferability, issuer obligations and the transaction chain all matter. A lawyer/compliance owner must classify the exact design; a technical label is not that determination.

### Required semantic policy for any approved on-ramp

If the product-specific review later approves an on-ramp:

1. Use a **versioned, directional conversion profile** that names the exact fiat asset, SAP unit profile, issuer, purpose/program, jurisdiction, limits, fee/rate policy and effective interval.
2. Display whether the rate is indicative or firm, who sets it, what it is based on, quote expiry, fees, rounding policy, transferability, and whether refunds or redemption exist. Do not label a one-way issuance rate as a market FX rate unless it actually is one.
3. Require authenticated issuer authority, an immutable typed issuance cause, confirmed source settlement under a qualified profile, exact amount arithmetic, source/currency/amount linkage, idempotency and anti-replay.
4. Bind each issuance to the exact initiating payment/settlement receipt, user/issuer, policy revision, terms version, amount, time and unique use/claim identity.
5. Handle reversed or charged-back funding through explicit correction/compensation lineage. Never erase original history; never blindly mint twice on retry; never assume a later refund can be satisfied by destroying SAP if it may already have been transferred or spent.
6. State where a correction leaves an unresolved issuer/user/merchant obligation. Do not silently externalize that liability onto another participant.
7. Cap exposure and stop the feature when source settlement, issuer authority, amount identity, current policy or reconciliation status is unknown.

No fixed 1:1 SAP-to-fiat parity should be implied unless a separate legal, reserve/liquidity, redemption, accounting and settlement design explicitly establishes that promise.

## 4. Immediate source-code and authority gates

Existing open Mycelix work already identified important blockers. Their current exact-head status must be rechecked before any live feature is enabled; this document does not claim they are closed.

- [AC-102 (#4128)](https://github.com/Luminous-Dynamics/mycelix/issues/4128): documented floating-point conversion at fiat/collateral/SAP money boundaries and tolerance around recomputation. Monetary state requires canonical fixed-point/integer arithmetic, a named rounding rule, overflow rejection and shared cross-boundary test vectors.
- [AC-143 (#4386)](https://github.com/Luminous-Dynamics/mycelix/issues/4386): documented fail-open collateral oracle attestation when oracle consensus is unavailable, malformed, undecodable or invalid. Value-bearing paths must fail closed if their required authoritative valuation evidence is absent.
- [AC-176 (#4519)](https://github.com/Luminous-Dynamics/mycelix/issues/4519): documented a public SAP-credit entrypoint with arbitrary recipient, amount and caller-supplied reason that lacks sufficient issuance authority binding.
- [AC-154 (#4589)](https://github.com/Luminous-Dynamics/mycelix/issues/4589): requires typed external issuance/return causes, cross-domain provenance, source action/status/amount binding and anti-replay; unsupported cross-domain credits should remain fail-closed.
- [FIN-ECO-007 (#1157)](https://github.com/Luminous-Dynamics/mycelix/issues/1157): owns exact cross-asset quotes, conversion semantics, FX execution separation and adversarial tests.
- [MYC-ECO-002 (#3343)](https://github.com/Luminous-Dynamics/mycelix/issues/3343): owns broad external-instrument and fiat interoperability.
- [FIN-COMPLY-000 (#2437)](https://github.com/Luminous-Dynamics/mycelix/issues/2437): owns policy-profile, compliance-evidence and legal-authority boundaries.

**Gate:** no general fiat -> SAP on-ramp can enter even limited production while the exact applicable amount, external issuance-authority and provenance findings remain unresolved or explicitly disabled.

## 5. BRICS: compare corridors, not flags

### Current public-policy evidence

The 2025 Rio de Janeiro BRICS Leaders' Declaration instructed continued work on the BRICS Cross-Border Payments Initiative and acknowledged technical exploration of greater interoperability; it did not itself establish that every BRICS currency pair has an operational universal rail. The 2026 New Delhi declaration continues to discuss interoperability of payment and messaging channels and local-currency trade settlement while respecting national priorities and different country circumstances.

Useful source references:

- [2025 Rio de Janeiro Leaders' Declaration, paragraphs 49–50](https://brics.br/en/documents/presidency-documents/250705-brics-leaders-declaration-en.pdf/%40%40download/file)
- [2026 New Delhi Declaration — public copy](https://ebs.publicnow.com/view/322C608779C3EE82D80D8869B7D797096FDC352E)
- [BIS Project Rialto](https://www.bis.org/publications/project-rialto-improving-instant-cross-border-payments-using-central-bank-money-settlement-technical-report) — a technical experiment linking payment systems, tokenised FX and central-bank-money settlement.
- [BIS Project Mandala](https://www.bis.org/project/mandala) — research on encoding jurisdiction-specific compliance checks and privacy-preserving proofs.
- [BIS FX settlement risk analysis, June 2026](https://www.bis.org/publications/qr-202606/uncovering-fx-settlement-risk-new-measures-2025-bis-triennial-survey).

These sources support a strategy of interoperability and local-currency settlement exploration, not the assumption of one BRICS currency, common central bank, universal participant network, common settlement finality or shared legal regime.

### A corridor profile for each country pair and direction

Use one versioned profile per pair/direction/provider/use case. It should carry:

- source and destination currency/asset identities, including exact unit/decimal profile;
- customer and counterparty eligibility, residence and purpose class;
- permitted provider(s), licensing/authorization evidence and restrictions;
- source/destination payment rails, intermediary route and supported operating window;
- amount limits, price/quote source, expiry, spread, explicit fees and expected all-in cost;
- liquidity/funding responsibility, cut-offs, holidays and expected value date;
- AML/CFT, sanctions, exchange-control and reporting profile versions as determined by authorized owners;
- execution, payment and settlement state machine, including partial, rejected, reversed and unknown outcomes;
- settlement method (PvP where supported; otherwise the exact approved risk-mitigating approach), gross exposure and time-at-risk;
- source evidence, last review/expiry, owner, tested limits and explicit claim ceiling.

If the exact direction is unsupported, display **unsupported corridor** rather than falling back to an unreviewed correspondent, stablecoin, token, bridge, third currency or cash path.

### Prioritization

1. **Domestic ZAR workflows first:** validate local payments, identity/consent, fees, disputes and reconciliation before global routes.
2. **Existing regional African payment corridors next:** evaluate licensed providers and rails such as PAPSS where available for the exact corridor and use case; verify the actual participant, currency, payout and settlement coverage instead of assuming membership means that a route is enabled.
3. **BRICS-related trade routes afterward:** begin with a small number of routes supported by real trade demand, authorized provider access, local-currency liquidity, lawful settlement and measurable all-in savings. Add pair/direction one by one.
4. **No speculative universal pool:** do not invent SAP as a bridge currency merely to connect illiquid currency pairs. A multi-hop route must disclose every quote, spread, fee, provider, liquidity dependency and settlement risk, and must be compared to direct-provider and conventional correspondent alternatives.

The goal is to make lawful cross-border trade less expensive and more transparent where we can prove it—not to market “de-dollarisation” as an engineering achievement by itself.

## 6. What should we measure against industry?

Set up a pre-registered, paired benchmark for exact corridors. Compare Mycelix orchestration against direct provider checkout and the bank/business's existing correspondent or payment workflow. Use matched quote timestamps, amounts, beneficiary/settlement conditions and provider access.

| Dimension | Benchmark | Evidence required |
|---|---|---|
| Delivered cost | Total all-in cost: explicit fees + embedded spread + rail/provider cost + FX slippage + failed/returned-payment cost | Timestamped provider quotes and final execution/settlement receipts; never compare an indicative quote to an executed rate |
| Quote quality | Quote freshness, expiry, rejection rate, quote-to-execution drift | Provider-attributed quotes bound to exact amounts/pairs/directions |
| Speed | p50/p95/p99 instruction-to-beneficiary-available time where timestamps and sample size support the claim | Separate instructed, accepted, executed, cleared and available states |
| Reliability | Completion, return/reversal, timeout and unknown-outcome rates | Full scheduled-run/result census and transaction-level outcome receipts |
| Settlement risk | Exposure amount and time; PvP share where supported; other risk-mitigation method and its limitations | Exact settlement-method profile and settlement evidence |
| Reconciliation | Duplicate payment/issuance, unexplained balance delta, repair time | Independent ledger reconciliation and replay tests |
| Corridor coverage | Pair, direction, amount tier, provider, operating window, use case, payout endpoint | Source-pinned per-corridor eligibility register |
| User trust | Comprehension of transferability/redemption, complaints, refund/dispute outcomes | Carefully designed user testing and properly governed pilot data |
| Operational cost | Integration, support, legal/compliance operations, reconciliation and provider fees | Fully loaded cost accounting, not only API latency |
| Resilience | Provider outage, stale quote, local holiday, liquidity shortage and alternate-route behavior | Defined failure injection, approved failover rules and independent verification |

### Proposed initial qualification gates — not achieved results

- 100% of scheduled conversion/settlement tests have a recorded result or explicit invalid/unknown disposition.
- 100% of material quote/execution/settlement outputs bind the exact asset pair, direction, provider, quote validity, amount profile, policy revision and evidence identity.
- Zero duplicate SAP issuance or payment side effects in the frozen retry/replay/unknown-outcome corpus.
- Zero false claims of redemption, finality, legal compliance or available liquidity in the conformance corpus.
- Exact replay of all fields the implementation declares deterministic, given the same pinned inputs and versioned rules.
- For any public claim of lower cost or faster completion, paired confidence intervals must support the claimed improvement, including failures and full costs. Otherwise publish “no demonstrated improvement”.
- A live corridor is not promoted based on synthetic tests alone; require a qualified provider route, legal/control approval, effects-disabled tests, independent review and a tightly bounded pilot.

## 7. Paradigm shifts worth pursuing

### A. From a forex desk to a neutral corridor optimizer

Optimize total delivered value and risk-adjusted settlement, not the size of Mycelix's FX book. The product compares route options, makes spread and counterparty dependencies visible, and chooses only from authorized routes. It should be able to say that the incumbent route is better.

### B. From currency symbols to rights-aware instrument semantics

Use a typed instrument profile and exact asset identity. SAP, MYCEL reputation, TEND/time exchange, fiat, deposits, stablecoins and mutual-credit instruments are not interchangeable just because a screen can display amounts for them.

### C. From rates to executable, consented, settled transactions

A rate is not an offer. An offer is not authorization. Authorization is not execution. Execution is not settlement. Settlement is not necessarily legal discharge. Keep this chain visible from the first interface through accounting and support.

### D. From liquidity claims to measured, route-specific service

For each corridor, expose which party supplies liquidity, who absorbs rate movement, what happens after partial settlement, and what route is used on failure. Do not claim 24/7 or instant settlement unless the exact provider route substantiates it.

### E. From one-way conversion as a loophole to explicit consumer rights

One-way SAP issuance can be a legitimate design in some bounded contexts, but it should not be used as a shortcut around regulation. Either keep fiat payments and SAP issuance separate, or make a narrow, well-defined product whose legal and consumer obligations are explicit and tested.

### F. From BRICS branding to evidence-qualified interoperability

Build a provider-neutral, corridor-aware protocol that can serve BRICS local-currency trade and African regional commerce without depending on a presumed unified BRICS rail, token or reserve asset. The same protocol should admit other corridors on evidence, not political branding.

## 8. Staged implementation

- **Stage 0 — policy and audit:** agree SAP rights, issuer model, one-way/no-conversion default and jurisdictional review. Reverify AC-102, AC-176 and AC-154 at the exact current repository heads.
- **Stage 1 — read-only FX observation:** show normalized provider quotes and all-in price components with freshness/expiry; no customer funds, execution or SAP minting.
- **Stage 2 — effects-disabled simulation:** exact fixed-point arithmetic, provider quote mapping, corridor schemas, idempotency, expiry, failure/return/reversal and reconciliation golden vectors.
- **Stage 3 — partner-operated domestic flow:** ZAR merchant payments and reconciliation through an authorized partner, with no SAP purchase unless separately approved.
- **Stage 4 — bounded cross-border corridor:** one explicitly qualified currency pair and direction, a provider route, user-purpose/consent controls and reviewed settlement-risk limits.
- **Stage 5 — SAP on-ramp decision:** proceed only if the use case justifies it, all issuance blockers are closed, terms are understood by users, the legal classification is documented and the pilot passes the above gates.
- **Stage 6 — redemption/venue/reserve product:** separate board/governance decision with legal authority, real liquidity/reserve design, redemption obligations, capital/settlement risk and independent assurance. Nothing at earlier stages implies this stage is approved.

## References

- [Mycelix FIN-EXCHANGE-001 product-policy work item](https://github.com/Luminous-Dynamics/mycelix/issues/5310)
- [SARB Financial Surveillance FAQ](https://www.resbank.co.za/en/home/what-we-do/financial-surveillance/FinSurvFAQ)
- [SARB Authorised Dealers and Currency and Exchanges Manual](https://www.resbank.co.za/en/home/what-we-do/financial-surveillance/authorised-dealers)
- [SARB Payments Ecosystem Modernisation](https://resbank.co.za/en/home/what-we-do/payments-and-settlements/pem)
- [SARB QR+ Standard 1.2 (7 July 2026)](https://www.resbank.co.za/en/home/publications/publication-detail-pages/other-publications/2026/qr-1-2-overview)
- [SARB Governor Rashad Cassim — Money in motion: Payments, digital assets and the future of finance (2026)](https://www.resbank.co.za/en/home/publications/publication-detail-pages/speeches/speeches-by-governors/2026/cassim-gibs)
- [BIS — FX settlement risk from the 2025 Triennial Survey (June 2026)](https://www.bis.org/publications/qr-202606/uncovering-fx-settlement-risk-new-measures-2025-bis-triennial-survey)
- [BIS — Project Rialto](https://www.bis.org/publications/project-rialto-improving-instant-cross-border-payments-using-central-bank-money-settlement-technical-report)
- [BIS — Project Mandala](https://www.bis.org/project/mandala)
- [BRICS — Rio de Janeiro Leaders' Declaration (2025)](https://brics.br/en/documents/presidency-documents/250705-brics-leaders-declaration-en.pdf/%40%40download/file)
- [BRICS — New Delhi Declaration (2026, public copy)](https://ebs.publicnow.com/view/322C608779C3EE82D80D8869B7D797096FDC352E)

---

**Non-claim:** This policy proposal does not establish that SAP is or is not legally classified as e-money, stored value, a financial product, a payment instrument, legal tender or a redeemable claim in any jurisdiction. It does not establish that any FX provider/corridor is authorized or live, and it does not prove the underlying code findings are closed. Those questions require current implementation verification and qualified, scope-specific legal/compliance review.
