# Mycelix Regenerative Finance Reference Architecture

**Document ID:** MYC-REGEN-FIN-0.1  
**Status:** Proposed architecture and implementation sequence  
**Date:** 2026-11-22  
**Scope:** Banking infrastructure, savings/custody, payments, lending, investment, productive collateral and cooperative finance

## 1. Thesis

Mycelix should not become a single unlicensed "decentralized bank". It should provide interoperable infrastructure for separately accountable financial institutions, cooperatives, custodians, issuers, lenders, project owners and investors.

The goal is to connect **financial authority, verified evidence, productive capacity and fair access to capital** without confusing money, credit, reputation, collateral or investment.

A central design rule:

> Finance should be allocated against a truthful view of repayment capacity, enforceable rights and real outcomes—not merely against existing wealth or a universal reputation score.

This is not a promise to abolish credit risk. It is a proposal to measure risk more honestly, make decisions more contestable, reduce avoidable intermediary costs and finance projects that conventional systems may find difficult to assess.

## 2. Keep institutional roles separate

| Role | Accountable party | Mycelix role |
|---|---|---|
| Deposit-taking bank / cooperative | Licensed or registered institution | Accounts, consent, authorizations, audit receipts and customer workflows |
| Monetary issuer | Named legal issuer | Issuance/redemption state machines, reserve/liability evidence |
| Custodian | Qualified independent custodian | Signed custody statements, encumbrance and release records |
| Lender | Licensed/registered lender or appropriate partner | Underwriting evidence, loan contract, servicing and workout records |
| Investment manager / market operator | Authorized entity where required | Offer disclosures, subscriptions, investor rights, allocation and distributions |
| Project operator | Borrower/cooperative/company | Milestones, invoices, physical delivery and performance records |
| Independent verifier/auditor | Separate qualified provider | Verification attestations, exceptions and assurance scope |
| Symthaea | Software agent under delegated authority | Advisory analysis only; no self-authorized lending, minting or liquidation |

The technology operator is not automatically the issuer, bank, custodian, lender, asset owner or investment manager. Protocol deployment does not remove legal responsibilities.

For South Africa, treat payment, deposit-taking, cooperative, credit, advisory/intermediary and collective-investment activities as separate regulatory questions. Obtain specialist advice and partner with appropriately authorized institutions before offering services to the public.

## 3. Financial product taxonomy

### 3.1 Everyday accounts and payments

Provide:
- account and wallet views across supported institutions;
- verified balance and transaction history;
- payment initiation with purpose and authority limits;
- receipts and reconciliation state;
- scheduled payments, standing instructions and escrow;
- controlled offline intent capture with explicit pending/unconfirmed status;
- fees and FX shown before approval;
- dispute, refund and recovery workflows.

A balance display must identify whether it represents a bank deposit, safeguarded e-money claim, custodial asset, mutual-credit balance or investment value. Do not aggregate unlike liabilities into a single "cash" number.

### 3.2 Savings and custody

Offer distinct wrappers, not one blended account:

- **transaction balance:** immediately spendable claim;
- **protected savings:** legally defined deposit/custody product with stated access limits;
- **time-bound savings:** term product with disclosed early-withdrawal consequences;
- **project escrow:** funds restricted to a contract/milestone;
- **investment account:** securities or fund interests, carrying market and liquidity risk.

The interface must state the legal owner of funds, account/custody provider, insolvency treatment, guarantee eligibility (if any), withdrawal rules and fees.

## 4. Lending architecture

### 4.1 Underwrite repayment capacity, not social worth

Use a layered, consent-based evidence pack:

1. **Affordability and cash flow:** income, expenses, debt obligations, volatility, seasonality and household resilience.
2. **Contracted revenue:** verified purchase orders, invoices, receivables, customer concentration, payment history and assignment rights.
3. **Productive capacity:** equipment condition, utilization, skills, supply arrangements and operational capacity.
4. **Collateral:** independently verified title, custody, market depth, lien priority, valuation and liquidation cost.
5. **Repayment conduct:** actual settled payment records and restructurings, with context for disputed or hardship events.
6. **External constraints:** insurance, climate, commodity, regulatory, geographic and counterparty risk.

MYCEL can be an optional and purpose-limited trust signal, never a substitute for affordability or collateral validity. Do not use protected characteristics, sensitive personal data or opaque social network centrality as an unreviewable credit shortcut. A person's lack of history is not evidence of bad character.

Every use of external or sensitive data requires legal basis, consent/notice as applicable, purpose limitation, data minimization, correction, retention controls and access logging.

### 4.2 Risk model

Use established credit risk components rather than inventing a single "consciousness credit score":

`ExpectedCreditLoss = PD × LGD × EAD`

Where PD is probability of default, LGD is loss given default and EAD is exposure at default, with time horizon, discounting and portfolio dependence specified by the applicable accounting/risk framework.

Symthaea may help produce scenario analysis, feature explanations, cash-flow stress and anomaly alerts. It must show confidence, assumptions, model version and data freshness. It cannot approve, deny, reprice, liquidate or alter a contract without an authorized decision process.

### 4.3 Loan products

| Product | Primary repayment source | Key safeguards |
|---|---|---|
| Personal resilience loan | Verified disposable income | Affordability cap, hardship process, clear total cost |
| Equipment/productive loan | Incremental operating cash flow | Inspection, insurance, maintenance plan, conservative LTV |
| Invoice/receivables finance | Named debtor payment | Debtor confirmation, assignment, duplicate-financing check |
| Agricultural seasonal loan | Harvest/sales proceeds | Crop/warehouse evidence, climate stress, storage/insurance |
| Housing finance | Sustainable household income | Title, affordability, valuation, foreclosure safeguards |
| Energy-efficiency loan | Contracted energy savings/revenue | Baseline, tariff sensitivity, performance guarantee where available |
| TEND mutual credit | Reciprocal goods/services | Bounded zero-sum ledger, transparent balance limit, dispute process |

TEND is not a substitute for regulated consumer credit. Its purpose, limits, eligibility and settlement rules must be explicit.

### 4.4 Loan state machine

Proposed lifecycle:

`Draft → Underwriting → Offer → Accepted → Authorized → Funded → Active → RepaymentDue → Settled`

Exception states:

`Arrears | Hardship | Forbearance | Restructured | Disputed | Defaulted | Recovery | WrittenOff`

Required properties:
- A loan is not funded until the borrower has accepted the final disclosed terms and authorized funding.
- A repayment is not confirmed by a caller setting `status = Repaid`; it requires a reconciled payment receipt or legally valid discharge.
- Each payment uses a unique idempotency reference and must be reconciled against the external settlement rail.
- Concurrent payments must not over-credit, understate or duplicate principal/interest allocation.
- Interest, fees, penalties, allocation order and effective dates are deterministic and versioned.
- Restructuring is a new authorized contract event; old obligations remain historically traceable.
- A lender/servicer cannot unilaterally fabricate borrower acknowledgements.
- Collateral release follows verified discharge and a separate release receipt.
- Disputes and hardship do not automatically trigger liquidation.
- Unknown payment status freezes further mutation until external reconciliation.

### 4.5 Repayment and pricing transparency

Before acceptance, display:
- principal and net disbursement;
- nominal and effective annual rate where applicable;
- every fee and compulsory third-party cost;
- total repayment under base and stressed scenarios;
- payment schedule and allocation order;
- variable-rate and FX exposure;
- collateral and guarantee consequences;
- arrears, hardship, restructuring and early-settlement rules;
- complaints, appeal and ombud/regulatory routes.

Compare price and terms across participating lenders using a common schema. Never optimize solely for approval rate or loan volume.

## 5. Productive collateral network

Create a registry for assets that can support credit, with a strict difference between **recording**, **verification**, **pledging**, **custody** and **release**.

Supported classes may include:
- stored agricultural commodities;
- energy equipment and contracted generation;
- vehicles and machinery;
- property and land rights;
- inventory and receivables;
- construction materials;
- insured equipment and specialized productive assets.

For every asset record, capture legal owner, unique identifier, provenance, location, custodian, condition, valuation, insurance, prior liens, pledge status, expiry and recovery rights.

Collateral acceptance requires:
1. title/authority verification;
2. unique-asset and duplicate-pledge detection;
3. independent valuation and freshness;
4. custody/control or perfected security interest;
5. priority and lien checks;
6. class-specific haircut and stress test;
7. independent verifier attestation;
8. explicit pledge and release transaction.

An appraisal or token does not itself create a perfected security interest. A digital warehouse receipt is useful only when the warehouse operator, governing law, title transfer and enforcement process make it legally and operationally meaningful.

## 6. Investment architecture

Do not combine securities into a fungible SAP balance. Create distinct products and rights.

### 6.1 Permitted structures

- **Project debt / community bonds:** repayment waterfall, coupon, maturity, security and default terms.
- **Cooperative equity:** member rights, voting, distributions, transfer restrictions and exit rules.
- **Revenue participation:** defined share of specified revenue, caps, audit rights and loss conditions.
- **Asset/project funds:** fund governance, custody, NAV methodology, liquidity, valuation and investor eligibility.
- **Grants and public goods allocations:** non-investment funding with transparent criteria and outcome reporting.

Each offering must disclose whether it is a deposit, debt, equity, fund interest, revenue share, donation or other legal instrument.

### 6.2 Evidence-based project finance

A project dossier should include:
- sponsor identity and authority;
- project rights, permits and land/access agreements;
- independent engineering and cost assessment;
- market demand and signed/credible offtake;
- cash-flow model with sensitivity and downside cases;
- operating, maintenance, insurance and replacement costs;
- procurement and related-party interests;
- funding stack and security priority;
- environmental/social safeguards where relevant;
- milestones, evidence requirements and release conditions;
- exit, default and insolvency treatment.

Capital release follows a contract milestone and independently verified evidence, not an AI-generated confidence score.

Symtropy can validate engineering assumptions and observed physical performance. Sol-Atlas preserves design provenance and rights. Mycelix records the investment contract, authority, custody and distributions. Symthaea provides scenario analysis and monitoring.

## 7. Risk sharing and anti-exploitation design

- Losses must fall on the disclosed risk-bearing parties according to the contract, not be silently socialized to payment-token holders.
- A first-loss reserve must have an identified owner, funding source, cap, replenishment policy and eligible claims.
- Borrowers must not be forced to pledge unrelated essential assets for small productive loans.
- Do not offer automatic wage garnishment, health-data pricing or irreversible AI collection.
- No compounding penalty fees for hardship without legal review and borrower protections.
- A community guarantee must be voluntary, bounded and visible to guarantors; no hidden joint liability.
- Credit limits must consider correlated exposures and local economic shocks.
- Loan sales/participations must preserve borrower rights, notices and servicing accountability.
- Conflicts of interest between originator, verifier, custodian and investor must be disclosed and controlled.

## 8. Banking and investment interoperability

Each financial provider publishes:
- legal entity and jurisdiction;
- licensed/authorized activity scope;
- account/custody model;
- supported payment rails;
- settlement finality definition;
- currency and FX conventions;
- fees, limits and service hours;
- dispute and chargeback rules;
- reserve/insurance disclosures where applicable;
- API version and evidence endpoints.

Federated clearing must be explicit about which institution owes whom, in which currency, under what netting agreement, with what collateral and default waterfall. A DHT record is not a substitute for enforceable netting, settlement finality or insolvency law.

## 9. Web4 integration

Finance becomes a high-impact workload in the Luminous Web4 architecture:

- **Symthaea:** advisory underwriting, cash-flow stress and anomaly detection, within signed, time-bounded authority.
- **Mycelix:** identity, consent, contracts, issuer/lender accountability, permissions, settlement and audit.
- **Symtropy:** collateral condition, asset productivity, energy output, project milestones and physical verification.
- **Sol-Atlas:** asset provenance, licenses, beneficial claims, design rights, records and disclosures.

The shared execution sequence is:

`Borrower/Investor Intent → Authority → Contract → Funding → Observation → Independent Verification → Payment/Settlement → Reconciliation`

A missing receipt, network timeout or conflicting observation is indeterminate, not success.

## 10. South African regulatory design notes

This section identifies relevant starting points, not legal advice or a licensing determination.

- The Prudential Authority describes a CFI as a deposit-taking cooperative and states registration criteria including a cooperative with at least 200 members and R100,000 share capital, plus human, financial and operational requirements. A cooperative bank requires prior CFI registration with at least 200 members and member deposits of R5 million or more. Confirm the current rules and the precise activity before structuring an entity. https://www.sarb.co.za/en/home/what-we-do/Prudentialregulation/functions-of-the-prudential-authority
- The Co-operative Banks Act 40 of 2007 provides a legal framework for registered deposit-taking cooperatives and cooperative banks. https://www.gov.za/documents/co-operative-banks-act
- The Financial Advisory and Intermediary Services Act regulates specified advisory and intermediary services. https://www.gov.za/documents/financial-advisory-and-intermediary-services-act
- The IFWG's 2026 rand-pegged stablecoin discussion paper identifies reserve, redemption, governance, cyber, operational and market-conduct concerns. The scope is rand-pegged stablecoin arrangements; it is not a blanket authorization for non-rand or basket assets. https://www.ifwg.co.za/Reports/Functional%20analysis%20of%20existing%20regulatory%20frameworks%20and%20rand-pegged%20stablecoin%20arrangements.pdf
- The FSB's 2025 peer review found continuing gaps in global stablecoin implementation, including reserve, redemption, liquidity, disclosure and resolution issues. https://www.fsb.org/2025/10/fsb-finds-significant-gaps-and-inconsistencies-in-implementation-of-crypto-and-stablecoin-recommendations/

Before a live product, local counsel must map the exact flow to the Banks Act, National Payment System framework, Co-operative Banks Act, National Credit Act, FAIS, Collective Investment Schemes Control Act, FICA, POPIA and any applicable FSCA/PA/SARB rules. Do not infer a licence from the name "cooperative", "DAO", "protocol" or "token".

## 11. Implementation phases

**Phase 0 — Contain and reconcile**
- Re-audit exact current heads for SAP credit/debit/transfer, bridge deposits, reserve accounting, collateral lock/release, oracle fail-open paths, lending state transitions and finance/governance cross-cluster routes.
- Disable or keep off by default any loan/credit-score path that cannot prove authorization, conservation and repayment finality.
- Make the SAP-ZAR monetary contract explicit before new issuance features are added.

**Phase 1 — Conserved money and reserve proofs**
- Use the TEND conservation model as a reference, not an assumed pass.
- Integrity-level authorization and issuance-record validation.
- Liability-aware reserve ledger and custody receipts.
- Two-phase redemption and idempotent external settlement.
- End-to-end bank-rail tests.

**Phase 2 — Collateral integrity**
- Authoritative unique asset registry.
- Legal/custody evidence and lock protocol.
- Independent price and quality attestation.
- Class-specific haircuts and stress limits.
- Release only after verified discharge.

**Phase 3 — Loan lifecycle**
- Loan contract and repayment allocations.
- Cash-flow underwriting and affordability.
- Payment receipt and reconciliation.
- Arrears, hardship, restructuring, disputes and collateral release.
- Explainable decisions and human appeal.

**Phase 4 — Investment/project finance**
- Standardized project dossier and risk disclosures.
- Milestone escrow and independent observation.
- Distribution waterfall and investor reporting.
- Secondary transfers only where legally permitted and rights can be maintained.

**Phase 5 — Federated cooperative network**
- At least two independently operated financial institutions.
- Cross-institution identity and credential acceptance.
- FX, clearing, liquidity and default rules.
- Independent audit and regulatory reporting.

## 12. Non-negotiable launch conditions

No public monetary/financial product until:
- legal issuer, lender, custodian and service providers are identified;
- licensing/registration or compliant partner scope is confirmed;
- liability, reserve, collateral and investment accounts are distinct;
- every money-moving action has bounded authority and replay protection;
- all external outcomes can be reconciled;
- privacy, consumer protection, complaints and appeals are operational;
- adversarial tests and independent review are complete;
- disclosures match actual capabilities, not roadmap aspirations.

## 13. References

- Basel Committee, Principles for the Management of Credit Risk (30 April 2025): https://www.bis.org/publications/202504-guidelines-principles-management-credit-risk
- Basel consolidated General Credit Risk Management chapter (published 1 January 2026): https://www.bis.org/committees/bcbs/basel-consolidated-guidelines/module/cri/10
- South African Prudential Authority, functions and CFI registration: https://www.sarb.co.za/en/home/what-we-do/Prudentialregulation/functions-of-the-prudential-authority
- Co-operative Banks Act 40 of 2007: https://www.gov.za/documents/co-operative-banks-act
- FAIS Act 37 of 2002: https://www.gov.za/documents/financial-advisory-and-intermediary-services-act
- IFWG, rand-pegged stablecoin discussion paper: https://www.ifwg.co.za/Reports/Functional%20analysis%20of%20existing%20regulatory%20frameworks%20and%20rand-pegged%20stablecoin%20arrangements.pdf
- FSB, 2025 implementation review: https://www.fsb.org/2025/10/fsb-finds-significant-gaps-and-inconsistencies-in-implementation-of-crypto-and-stablecoin-recommendations/
