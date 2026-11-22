# SAP Monetary Constitution and Reserve Architecture

**Document ID:** MYC-SAP-RESERVE-0.1  
**Status:** Proposed design; no issuance authorization  
**Date:** 2026-11-22  
**Owner:** Mycelix Finance  
**Scope:** SAP monetary definition, issuance, reserve backing, redemption, collateral separation and evidence

> This document is a design proposal, not a claim that SAP currently has the reserves, legal structure, redemption service or production assurance described here. Do not mint, market or represent SAP as fully reserved or redeemable until the launch gates in Section 14 have been independently satisfied.

## 1. Executive decision

The current Mycelix documentation has described SAP as a transferable circulation medium and, in different documents, as a basket-backed stable unit. Those descriptions are not yet a single, complete monetary contract.

The first design decision must be to separate three concepts:

1. **Unit of account:** the published reference used to quote and compare prices.
2. **Redeemable monetary liability:** the issuer's enforceable obligation to redeem a unit under stated conditions.
3. **Productive collateral:** an asset that may secure a loan or investment but is not necessarily liquid enough to redeem money on demand.

They must not be collapsed into one number or one token claim.

### Proposed initial profile: SAP-ZAR

For an initial South African pilot, define **SAP-ZAR** as a redeemable payment claim denominated in rand:

- 1 SAP-ZAR represents a contractual claim to redeem for ZAR 1, subject to the published redemption terms and applicable law.
- Every outstanding redeemable SAP-ZAR liability is covered by eligible, unencumbered, independently verified liquid reserve assets after conservative haircuts.
- The reserve is segregated from operating capital, investment assets, loan receivables and community development funds.
- SAP-ZAR issuance is backed only by cleared and verified reserve funding or an explicitly approved, legally equivalent reserve contribution.
- Productive assets can support a separate credit facility. They do not automatically back SAP-ZAR.
- The essential-goods basket is initially a **purchasing-power benchmark**, not a promise of redemption into a physical basket.

This is the most honest and testable first step. It is not a claim that ZAR is the ideal long-term global unit of account. It creates a clear legal and accounting liability that can be qualified before broader ambitions are attempted.

### Future profile: basket-indexed unit

A basket-indexed monetary unit may eventually be useful for interregional accounting. It must be separately specified. It will have a changing fiat exchange value, a defined index methodology, regional price sources, rebase rules and explicit redemption or non-redemption terms.

Do not silently make SAP-ZAR both a fixed-ZAR par claim and a purchasing-power-indexed claim. Those promises can conflict during inflation, relative-price shocks and reserve stress.

For international deployment, use clearly identified issuer/currency instances (for example, SAP-ZAR, SAP-EUR) with explicit foreign-exchange conversion and settlement risk. A single ticker must not hide different issuer liabilities.

## 2. Monetary taxonomy

| Instrument | What it represents | Backing / risk | Permitted role |
|---|---|---|---|
| **SAP-ZAR** | Redeemable ZAR payment claim | Liquid segregated reserves and issuer/legal risk | Payments and ZAR-denominated settlement |
| **SAP Basket Reference (SBR)** | Published weighted essential-cost index | Price-data and methodology risk; not a liability | Purchasing-power reporting and contract benchmarking |
| **TEND** | Bounded bilateral/community mutual credit | Counterparty and community-network risk; aggregate net positions must conserve | Time-based exchange and local credit |
| **MYCEL** | Non-transferable reputation/recognition state | Measurement, governance and privacy risk | Scoped trust signals; never money or collateral |
| **Investment interests** | Debt, equity, revenue share or fund units | Project, market, liquidity and legal risk | Capital formation; not money |
| **Productive collateral** | Legally enforceable asset or receivable pledge | Valuation, title, custody, liquidity and recovery risk | Secured lending; not monetary reserves by default |

No asset or score can change category merely because it has been tokenized.

## 3. Reserve perimeter

### 3.1 Eligible reserve assets

Eligibility is determined by law, issuer structure, custody, liquidity, maturity, concentration and enforceability—not by an oracle price alone.

The initial policy should prefer:
- cash and settlement balances available within the published redemption window;
- eligible short-dated, high-quality sovereign instruments held with an independent qualified custodian, subject to issuer and sovereign concentration limits and conservative market-value haircuts;
- other instruments only after independent legal, credit, custody and liquidity review.

If the issuer cannot access central-bank money directly, the design must disclose the commercial-bank/custodian exposure and its resolution/insolvency treatment. Do not describe a commercial bank deposit as risk-free central-bank money.

### 3.2 Ineligible reserve assets

The following do not count toward the reserve ratio for immediately redeemable SAP-ZAR:

- land, buildings, property development projects and illiquid equipment;
- unharvested crops or inventory without perfected title, custody and a tested liquidation path;
- energy generation forecasts, unissued certificates or future production;
- carbon credits, biodiversity units or other credits without immediate cash redemption capacity;
- TEND balances, MYCEL scores, community promises or governance votes;
- SAP or other self-issued tokens;
- own-issued equity, project tokens or subordinated claims on the issuer;
- unsecured loans, ordinary receivables or assets already pledged elsewhere;
- assets subject to liens, covenants, disputes, withdrawal restrictions or uncertain ownership;
- assets that are only valued by the issuer's own oracle.

A productive asset can be valuable and still be unsuitable to back a money-like liability redeemable at par.

### 3.3 Legal and operational segregation

Reserve assets must be:
- legally ring-fenced from issuer operating assets to the maximum extent available under the applicable jurisdiction;
- held under a disclosed custody/trust arrangement with named accountable entities;
- unencumbered and not lent, rehypothecated or pledged to another creditor;
- reconciled against the full SAP liability ledger;
- recoverable through a documented insolvency, custodian failure and issuer wind-down procedure.

If counsel cannot establish effective segregation and holder rights, do not launch SAP-ZAR as a redeemable product.

## 4. Reserve coverage and liquidity

Let:

- (L_t) = all outstanding redeemable SAP-ZAR liabilities at time (t), including pending redemptions and any other unconditional redemption obligations;
- (A_i) = independently verified quantity/value of reserve asset (i);
- (h_i) = conservative haircut for price, liquidity, custody, settlement and legal risk;
- (E_i) = encumbrances or prior claims against asset (i).

Then:

[
ERV_t = sum_i max(0, A_i(1-h_i)-E_i)
]

[
ReserveCoverage_t = rac{ERV_t}{L_t}
]

**Hard rule:** `ReserveCoverage >= 1.00` at all times for a fully reserved par-redemption profile. An asset with uncertain title or unavailable custody evidence has zero eligible value until reconciled.

Reserve coverage is not the same as immediate liquidity. The issuer must also maintain a redemption liquidity ladder:

- immediately available settlement cash;
- same-day and next-business-day assets;
- assets maturing within the redemption horizon;
- stressed liquidation proceeds after market-impact haircuts.

The published redemption SLA defines the relevant horizon. The liquidity policy must demonstrate that stressed outflows can be met without relying on an optimistic asset sale, new issuance, credit lines that can be withdrawn, or emergency governance votes.

Do not promise instantaneous global redemption unless the reserve and settlement system can actually support it across time zones and banking cut-offs.

## 5. Reserve attestation and proof of liabilities

A proof-of-reserves screenshot is insufficient. It does not prove that all liabilities were disclosed, that assets are unencumbered, or that the same collateral was not pledged twice.

The reserve assurance package must combine:

1. independently confirmed custody and asset positions;
2. a complete, reconciled liability register, including pending, escrowed, frozen and redeemable claims;
3. encumbrance and related-party disclosures;
4. a published calculation methodology, asset haircuts and concentration schedule;
5. a cryptographic commitment to the liability snapshot, with privacy-preserving inclusion evidence for holders where feasible;
6. an independent auditor/custodian attestation with explicit scope, limitations, timestamp and exceptions.

Suggested cadence for a pilot:
- continuous internal reconciliation after each issuance/redemption;
- daily reserve/liability reconciliation and exception report;
- weekly independent custodian position confirmation;
- monthly independent assurance statement;
- annual full-scope audit, or more frequently where legally required.

These are proposed operating targets, not substitutes for a regulated audit standard. A cryptographic proof can establish consistency with committed data; it cannot independently establish that an off-ledger warehouse, bank account or asset title exists.

## 6. Issuance and redemption

### 6.1 Authorized issuance sources

For SAP-ZAR, mint only against cleared ZAR reserve funding or an approved, legally equivalent reserve asset delivered into eligible custody.

Every issuance must reference:
- a unique issuance intent and idempotency key;
- issuer and accountable author;
- verified deposit/custody receipt;
- applicable currency, value date and conversion terms;
- reserve asset identifier and haircut;
- policy version;
- approval/capacity evidence;
- a one-time consumed issuance authorization;
- resulting liability and balance delta.

An energy certificate, agricultural record, property appraisal or Symtropy prediction is not by itself an issuance authorization.

### 6.2 No credit-created SAP by default

A loan creates a loan receivable and a borrower obligation. It does not automatically create SAP-ZAR.

When an institution wants to lend, it should disburse existing SAP-ZAR or an explicitly regulated deposit/credit instrument with its own balance sheet, capital, liquidity and disclosure requirements.

This distinction prevents a secured loan book from being presented as liquid par reserves. Any future fractional-reserve or deposit-creating bank model must be separately licensed, capitalized and supervised; it is not smuggled into the SAP token contract.

### 6.3 Redemption state machine

Proposed lifecycle:

`Requested → Authenticated → Reserved → Approved → Paid → Burned → Reconciled`

Exception states:

`Rejected | Expired | Disputed | Indeterminate | RecoveryRequired`

Rules:
- Reserve the liability before paying out; do not burn the holder's claim before funds are irrevocably available unless the holder explicitly accepts an escrowed redemption workflow.
- Use a unique redemption idempotency key.
- If the external bank reports an unknown outcome, stop duplicate execution and reconcile using the external reference before any retry.
- A successful bank transfer receipt is not enough if the transfer can still be reversed; define finality by the actual rail and contract.
- On a failed or expired redemption, restore the exact liability through a compensating, auditable event.
- All events are author-bound and replay-protected.

### 6.4 Conservation invariant

For each SAP-ZAR issuer and currency:

[
Issued - Burned = OutstandingRedeemableLiabilities
]

Internal transfers must be net-zero. Escrow, frozen balances, pending redemptions and locked balances remain part of the total liability until legally and operationally extinguished.

The balance ledger should be event/delta-based and validated at the integrity layer. A mutable aggregate balance updated only in coordinator logic is not a sufficient financial invariant under concurrent distributed writes.

Use TEND's stronger conserved-value design as a reference, then qualify SAP independently under concurrent, replay and partition scenarios. Do not assume TEND's tests prove SAP.

## 7. Productive asset and credit pool — separate from reserves

Mycelix should absolutely support productive collateral. It should do so in a separate pool with distinct accounting and risk limits.

Potential collateral categories:
- warehouse-held agricultural commodities with verified receipts and insurance;
- energy equipment and contracted project cash flows;
- vehicles and industrial equipment with title and repossession rights;
- property with independently verified ownership and lien priority;
- inventory and receivables with enforceable assignment and debtor confirmation;
- other assets only where legal recovery, custody and liquidation have been tested.

Each collateral record must bind a unique asset identifier to the owner, custodian, location, title, inspection, valuation date, liens, insurance, expiry/condition, prior pledges and release conditions.

A collateral pledge must lock the rights/claim in the authoritative registry or a legally enforceable external custody system. Recording a string or a token does not lock the underlying asset.

### 7.1 Conservative collateral valuation

[
EligibleCollateralValue = IndependentMarketValue 	imes (1-h)
]

The haircut (h) includes:
- price volatility and stale-price risk;
- market depth and expected liquidation discount;
- legal-title uncertainty and time to enforce;
- custody, insurance and transport costs;
- spoilage, obsolescence and physical-condition risk;
- concentration and common-cause exposure;
- counterparty and settlement risk.

Loan-to-value limits must be calibrated per asset class using observed recovery data and stress scenarios. Do not grant a generic diversification bonus that mechanically reduces required collateral just because a position contains multiple asset labels. Diversification only helps when correlations, liquidation markets and custody risks are genuinely independent.

### 7.2 Credit pool rules

- Loan assets and reserve assets have separate legal ownership, accounts and reconciliations.
- Credit exposure has limits per borrower, group, sector, region, custodian and asset class.
- A collateral asset cannot secure two loans unless the priority and residual value are explicit and independently validated.
- Valuation disagreement or unavailable oracle evidence blocks new borrowing and triggers review; it does not cause an automatic confiscation.
- Liquidation requires documented contractual rights, notice, applicable law and an appeal/workout route.
- Expected credit loss and stress losses are absorbed by disclosed lender/investor capital, not silently by SAP holders.

## 8. Demurrage compatibility

The existing finance README describes 2% annual SAP demurrage. That policy is not automatically compatible with a claim promising redemption of 1 SAP-ZAR for ZAR 1.

A par claim with a declining holder balance is either a different monetary contract or a clearly disclosed custody/account fee. It should not be marketed as unconditional par redemption without explaining that deduction.

**Proposal:** do not apply demurrage to the reserve-backed SAP-ZAR liability. If a community wants an anti-hoarding circulation instrument, define a separate non-par community instrument with explicit supply, decay, exemption, redemption and tax/accounting semantics. Do not apply hidden demurrage to the reserve claim.

Re-run the existing agent-based monetary simulations before retaining any demurrage policy. The July 2026 economic review reported that the simulated rate had no measurable velocity effect under its then-current exempt-floor assumptions and that small exchanges could reset the decay clock. Those findings are a dated test result that should be reproduced at the current code/model head before policy is changed.

## 9. Oracle and evidence policy

An oracle is an evidence router, not a truth engine.

Every price/asset observation must include:
- independent reporter identity;
- source and methodology;
- event and publication times;
- confidence and known limitations;
- market depth/liquidity;
- sample freshness;
- reporter conflicts and concentration;
- outlier and collusion analysis;
- fallback/unknown behavior.

A quorum of two reporters is not automatically independent. Measure common providers, data sources, geography, beneficial ownership and infrastructure. When sources conflict or are stale, return `Unknown` and pause new issuance against that asset.

For physical assets, require suitable inspection, custody and title evidence. For fiat reserves, reconcile direct custodian statements and bank movement references. Never accept a self-declared collateral value as independent evidence.

## 10. Governance and control separation

Separate roles:
- issuer/legal obligor;
- reserve custodian;
- valuation/oracle provider;
- verifier/auditor;
- mint/redeem operator;
- governance authority;
- technical operator.

No one actor or agent should perform all roles for the same high-impact action.

Monetary policy changes require:
- advance publication;
- versioned rules and effective time;
- impact and distribution analysis;
- independent security and legal review;
- holder communication and a challenge window;
- no retroactive alteration of already-issued claims.

Governance can change future issuance parameters within disclosed law and contract. It cannot vote away a holder's existing redemption claim.

## 11. Failure and recovery policy

- **Reserve shortfall:** halt new issuance immediately; preserve redemption queue ordering and legal priority; notify holders; activate independent incident governance.
- **Custodian failure:** freeze related reserve eligibility, not the holders' record; invoke alternate custodian/insolvency plan.
- **Oracle outage:** no new asset-backed issuance; use only pre-authorized, time-bounded fallback sources for non-critical valuation.
- **Network partition:** no conflicting issuance or redemption; reconcile before resuming.
- **Stale proof:** assets with missing current confirmation become ineligible until reconciled.
- **Compromised key:** revoke at the authority layer; preserve historical evidence; rotate keys and reissue current authorization.
- **Issuer insolvency:** follow the legally established trust/segregation and creditor priority, not an improvised DAO vote.

Unknown transaction outcome means no new mutation until reconciliation.

## 12. What the current Mycelix code means for this proposal

The public finance surface already contains collateral registrations, energy certificates, agricultural assets, multi-collateral positions, a fiat bridge and redemption-related APIs. This is useful groundwork, not proof of reserve backing.

The public economic improvement plan dated 10 July 2026 identified material design and implementation risks at that snapshot, including collateral deposits that did not actually lock external assets, self-confirmation, fail-open oracle paths, incomplete SAP conservation and gaps in reserve/issuance specification. It also documented partial fixes in progress. These findings must be re-audited at the exact current source head before implementation; do not assume every finding is still present or already resolved.

Source material:
- Finance API: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/docs/COLLATERAL_API.md
- Finance types: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/types/src/lib.rs
- Finance coordinator: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/zomes/currency-mint/coordinator/src/lib.rs
- Dated improvement review: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-workspace/MYCELIX_ECONOMY_IMPROVEMENT_PLAN_2026-07-10.md

## 13. Launch gates

SAP-ZAR must not be represented as a redeemable, fully reserved payment instrument until all gates pass:

1. Monetary definition and legal issuer are unambiguous.
2. Holder redemption rights, insolvency treatment and dispute jurisdiction are documented and independently reviewed.
3. Segregated custody and reserve account(s) are operational.
4. Reserve/liability reconciliation is complete and independently attested.
5. Minting is integrity-authorized, replay-proof, conserved and bounded.
6. Redemption is tested end-to-end with the actual external payment rail, including unknown outcomes and duplicate requests.
7. Concurrent issuance, transfer, escrow, redemption, reversal and partition tests find zero conservation violations.
8. Oracle failures, stale sources and colluding reporters fail closed.
9. Reserve and credit pools are separate in code, accounting and legal custody.
10. Independent security assessment and incident exercises are complete.
11. Applicable financial regulatory permissions/partner arrangements are in place.
12. Public disclosures accurately describe fees, risks, redemption SLA and reserve composition.

Until then, label the product **experimental / non-redeemable / not reserve-qualified**, as applicable. Never use "fully backed", "stable", "guaranteed" or equivalent marketing language without current evidence.

## 14. References

- BIS, Annual Economic Report 2026, "Anchoring trust in money": https://www.bis.org/publications/aer-2026/anchoring-trust-money
- BIS, stablecoin and tokenization discussion, Annual Economic Report 2026: https://www.bis.org/publ/arpdf/ar2026e.pdf
- South African IFWG, functional analysis of existing regulatory frameworks and rand-pegged stablecoin arrangements: https://www.ifwg.co.za/Reports/Functional%20analysis%20of%20existing%20regulatory%20frameworks%20and%20rand-pegged%20stablecoin%20arrangements.pdf
- Mycelix public collateral API and dated economic review links in Section 12.
