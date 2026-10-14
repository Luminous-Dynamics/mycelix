# Monetary Authority and Issuance Boundary v1

Status: design contract; no new monetary authority is granted by this document.

## Purpose

Define a neutral, machine-auditable boundary for monetary and credit issuance inside the Mycelix Economic Fabric.

The system must distinguish:

```text
monetary authority
!= currency identity
!= credit creation
!= reserve/collateral
!= valuation evidence
!= treasury spending
!= settlement finality
!= economic outcome
```

The objective is not to select one universal currency model. Mycelix should remain able to interoperate with sovereign fiat, commercial-bank deposits, mutual credit, community currencies, resource/time credits, tokenised securities and other explicitly typed instruments.

## Research basis

Recent BIS work describes the next-generation monetary system as a tokenised two-tier architecture in which central-bank money remains the settlement anchor while commercial-bank money and other claims become programmable on interoperable or unified-ledger infrastructure. BIS explicitly retains the distinction between central-bank reserves, commercial-bank money and tokenised assets.

Current Bank of England material likewise describes bank deposits as the dominant transactional form of money and explains that commercial banks create deposits when lending, subject to capital, liquidity and settlement constraints.

Recent FSB work identifies leverage, liquidity transformation, opacity and cross-institution interconnections as important sources of systemic amplification, including between banks and non-bank financial intermediaries.

These sources motivate the architecture below but do not become Mycelix truth merely because an institution published them.

## Core model

### Monetary layers

Mycelix represents monetary systems as multiple typed layers:

| Layer | Example | Primary semantic role |
| --- | --- | --- |
| Base settlement money | central-bank reserves, notes | final settlement / monetary anchor |
| Deposit money | commercial-bank deposits | payment + credit intermediation |
| Mutual credit | TEND | reciprocal obligations with explicit limits |
| Community currency | SAP or other local unit | local circulation / allocation |
| Reserve instrument | government bond, approved reserve asset | liquidity / collateral |
| Credit claim | loan, invoice, receivable | financing obligation |
| Asset claim | security, tokenised real asset | ownership / financial claim |

An implementation MUST NOT infer that one layer is another merely because both have a numeric balance.

### Authority classes

An issuance or monetary mutation MUST identify its authority class:

- `CentralBank`: authority over base-money liabilities in its jurisdiction.
- `RegulatedDepositTaker`: authority to create deposit liabilities within its legal and prudential perimeter.
- `MutualCreditCommunity`: authority to extend bounded reciprocal credit under a community constitution.
- `CommunityCurrencyIssuer`: authority to issue a local/community instrument under an explicit monetary constitution.
- `AssetIssuer`: authority to create an asset/security claim according to its governing instrument.
- `ExternalAuthority`: an imported external claim whose origin remains outside Mycelix.
- `TreasurySpendingAuthority`: authority to commit or spend already-authorised public/commons resources; this class does not by itself imply base-money creation.

The class describes the claimed authority. It is not itself proof that the authority is legally valid.

## Issuance contract

A monetary issuance event SHOULD carry at least:

- `issuance_id`: globally unique deterministic identifier;
- `issuer_identity`: DID/agent or institutional identity;
- `authority_class`;
- `authority_reference`: governance/legal/constitutional evidence;
- `instrument_id`;
- `unit_of_account`;
- `quantity`;
- `effective_at`;
- `created_at`;
- `issuance_reason`;
- `backing_or_asset_reference`, when applicable;
- `policy_decision_reference`, when issuance is policy-governed;
- `settlement_scope`;
- `redemption_terms`, when applicable;
- `expiry_or_recall_terms`, when applicable;
- `source/provenance references`;
- `correction/supersession lineage`.

Issuance MUST be append-only at the event layer. A mutable balance is a projection, not the authoritative explanation of why supply changed.

## Double-entry / conservation boundary

Where an issuer operates a balance-sheet instrument, monetary creation SHOULD be represented by linked accounting entries rather than a naked balance increment.

For a conventional bank-credit example:

```text
bank asset:      +loan receivable
bank liability:  +customer deposit
```

The deposit is not required to be 100% reserve-backed to be meaningful as bank money. What matters for the ledger contract is that the claim's issuer, counterparties, settlement path, and balance-sheet effects remain explicit.

For a community mutual-credit instrument:

```text
member A: -TEND
member B: +TEND
system total: 0
```

For a reserve-backed claim:

```text
claim outstanding
<-> identified reserve/collateral
<-> valuation + haircut policy
<-> redemption/settlement rule
```

The exact economic meaning differs by instrument. Mycelix MUST preserve these distinctions.

## Physical evidence is not monetary value

A verified physical observation can justify a policy condition, allocation or issuance rule, but:

```text
physical observation
!= monetary value
!= purchasing power
!= legal tender
!= reserve adequacy
```

In particular, an authenticated energy-generation observation does not by itself establish that one unit of a currency should have stable purchasing power relative to goods, labor or other currencies.

Thermodynamic Genesis should therefore be treated as a typed issuance policy whose physical evidence, valuation rule and monetary effect are separately represented.

## Authorization boundary

The system MUST distinguish:

```text
identity
-> authority
-> policy decision
-> issuance event
-> accounting projection
-> settlement
-> outcome
```

No single client-provided DID, governance flag, balance, oracle value or proof blob is sufficient to imply the entire chain.

Authorization evidence MUST be independently addressable and bound to the exact monetary action.

Conflicting authority roots, missing authority, malformed authority evidence, or ambiguous issuer state MUST fail closed.

## Symthaea boundary

Symthaea SHOULD operate as an analytical and supervisory layer:

```text
qualified evidence
-> state reconstruction
-> causal/risk model
-> scenario analysis
-> recommendation
```

Symthaea MUST NOT silently become an issuer.

In particular:

- a Symthaea recommendation is not a monetary-policy decision;
- a forecast is not an authority grant;
- an anomaly score is not a fraud finding;
- a risk estimate is not a liquidation instruction;
- a governance proposal is not an executed issuance;
- an AI-generated value is not authoritative source truth.

Any action that changes monetary supply MUST pass through the explicit authority and governance path.

## Systemic-risk fabric

Mycelix can make banking risk legible by preserving the graph:

```text
issuer
 -> liabilities
 -> assets
 -> counterparties
 -> collateral
 -> funding
 -> settlement dependencies
 -> downstream outcomes
```

The graph SHOULD support queries such as:

- Which issuers share the same collateral?
- Which institutions are exposed to the same counterparty?
- Which deposits depend on short-term funding?
- Which assets would be jointly liquidated under one stress scenario?
- Which monetary claims depend on a single oracle, bridge, custodian or legal authority?
- Which policy actions could create correlated liquidity demands?

This directly targets opacity and hidden interconnection rather than assuming decentralisation alone removes systemic risk.

## Bank-run / liquidity boundary

A bank-run detector should not output a magical `solvent/insolvent` truth.

Instead it should construct evidence from:

- deposit-flow velocity;
- insured/uninsured concentration where legally observable;
- liquidity buffers;
- asset maturity;
- collateral eligibility;
- funding concentration;
- counterparty dependence;
- unrealised/realised losses;
- payment-system dependencies;
- correlated exposures.

The analytical result should be a versioned scenario with explicit inputs, assumptions and uncertainty.

Policy authorities may then choose among liquidity, resolution, guarantees, restrictions or other lawful interventions. Mycelix records the decision and evidence; it does not autonomously decide which public policy is correct.

## Current Mycelix composition

The existing Finance architecture already contains several useful owners:

- `currency-mint`: community currency lifecycle, minting, balances and issuance-adjacent flows;
- `payments`: SAP/TEND payment and settlement paths;
- `tend`: mutual-credit accounting and bilateral clearing;
- `treasury`: commons-pool resources and governance-sensitive allocation;
- `price-oracle`: valuation observations;
- `bridge`: external collateral/asset boundaries;
- `recognition`: MYCEL reputation;
- `mycelix-finance/types`: shared economic types;
- `docs/finance/FIN_SYS_SOURCE_PROFILE_V1.md`: macro-financial evidence and source/model airlock;
- `mycelix-identity` and governance systems: identity and authority dependencies.

The correct next step is composition, not collapsing these semantics into one universal "money" object.

## Immediate hardening frontiers discovered in the current source

The current Finance code also shows why the issuance boundary needs to be explicit:

1. `currency-mint` currently has a dedicated Thermodynamic Genesis path that records a proof byte vector and physical claim before crediting SAP. The coordinator source currently documents the real proof verifier as future work; a non-empty proof payload is therefore not yet sufficient evidence of physical-state truth.

2. The balance-proof endpoint currently validates structural fields but does not perform the promised cryptographic proof verification. It must remain non-authoritative until the actual verifier is wired and qualified.

3. Some historical coordinator paths contain permissive bootstrap patterns. Any path that can alter money, collateral, treasury resources or authorization state must use an explicit fail-closed mode once its authority dependency is expected to exist.

4. Currency update validation should never convert a failed predecessor lookup into successful validation. Dependency failure MUST be distinct from a valid predecessor.

5. Treasury contribution and allocation flows need the same exact authority, predecessor, settlement-effect and concurrency treatment already being applied to hardened TEND surfaces.

These are implementation observations, not claims of real-world exploitability without exact-head runtime qualification.

## Engineering tranche

The preferred implementation order is:

### A. Define typed issuance primitives

Add shared types for:

- `MonetaryInstrument`;
- `AuthorityClass`;
- `IssuanceEvent`;
- `RedemptionEvent`;
- `AccountingEvent`;
- `AuthorityEvidenceRef`;
- `BackingEvidenceRef`;
- `SettlementReference`;
- `PolicyDecisionRef`.

Keep them descriptive and instrument-neutral.

### B. Build an append-only issuance journal

All supply-changing paths should emit an issuance/redemption/retirement event with exact authority and provenance.

Balances become projections of the journal.

### C. Bind authority to the event

The exact issuer, authority evidence, governance decision and instrument definition should be hash-bound to the issuance event.

No "current governance state" lookup should be allowed to retroactively authorise a historical issuance.

### D. Complete proof-bearing issuance

For Thermodynamic Genesis and future proof-backed issuance, replace structural "proof present" checks with actual versioned proof verification.

The verifier result must itself be provenance-bound:

```text
proof bytes
-> circuit/version identity
-> verifier identity
-> verification result
-> issuance event
```

### E. Add bank-style accounting projections

Introduce a separate double-entry projection so the system can model:

- deposits;
- loans;
- reserves;
- collateral;
- capital/equity;
- fees;
- write-offs;
- redemptions.

This enables systemic simulation without pretending all instruments share one accounting semantics.

### F. Add systemic-stress simulation

A deterministic simulation harness should model:

- rapid deposit withdrawal;
- collateral haircut shock;
- counterparty default;
- oracle disagreement;
- settlement outage;
- bridge failure;
- correlated asset liquidation;
- mutual-credit limit exhaustion.

Symthaea can consume the qualified state and produce scenarios; deterministic fixtures establish reproducibility.

## Adversarial qualification corpus

At minimum, the monetary-issuance corpus should include:

1. forged issuer identity;
2. valid identity with wrong authority class;
3. missing authority evidence;
4. conflicting authority roots;
5. reused issuance ID;
6. duplicated issuance event;
7. historical governance decision replay;
8. future-dated issuance;
9. stale policy decision;
10. revised source evidence;
11. malformed proof;
12. non-verifying proof;
13. proof verified against wrong circuit version;
14. physical-yield observation replay;
15. source-unit mismatch;
16. currency/unit confusion;
17. balance projection detached from issuance journal;
18. redemption before settlement finality;
19. concurrent issuance race;
20. concurrent redemption race;
21. collateral double-use;
22. collateral released before claim consumption;
23. treasury authority unavailable;
24. oracle unavailable;
25. oracle conflict;
26. cross-hApp authority mismatch;
27. bank-run scenario with correlated counterparties;
28. resolution action with stale state;
29. duplicate recovery after crash;
30. unsupported policy intervention.

No vector is a PASS merely because a unit test compiles. Qualification must bind to the exact source head, substrate identity, executable corpus and evidence artifact.

## Source/model airlock

The macro-financial source profile already establishes the important distinction:

```text
source artifact
-> source-specific parser
-> normalized observation + receipt
-> derived metric
-> Symthaea analysis
-> scenario
```

The monetary authority layer should reuse the same airlock rather than allowing a model or external publication to become a hidden issuance authority.

## Design consequence

The deeper target is not "a decentralized bank."

It is:

> **an economic operating system in which every monetary claim has an explicit origin, authority, accounting meaning, settlement path, and correction history, while systemic analysis can traverse the resulting evidence graph.**

That lets Mycelix coexist with many monetary institutions without requiring all of them to use one currency.

## Claim ceiling

This document does not establish:

- legal authority to issue money;
- sovereign monetary recognition;
- reserve solvency;
- price stability;
- bank safety;
- economic superiority;
- successful systemic-risk prediction;
- cryptographic proof correctness;
- Holochain 0.7 substrate qualification;
- real-world financial effectiveness.

It establishes a candidate engineering boundary for making those questions testable rather than implicit.

## References

- BIS, Annual Economic Report 2025, Chapter III, "The next-generation monetary and financial system": https://www.bis.org/publications/aer-2025/next-generation-monetary-financial-system
- BIS, Annual Economic Report 2026: https://www.bis.org/publ/arpdf/ar2026e.pdf
- Bank of England, "How is money created?": https://www.bankofengland.co.uk/explainers/how-is-money-created
- FSB, "Leverage in Nonbank Financial Intermediation: Final report": https://www.fsb.org/2025/07/leverage-in-nonbank-financial-intermediation-final-report/
- FSB, "Report on Vulnerabilities in Private Credit": https://www.fsb.org/2026/05/report-on-vulnerabilities-in-private-credit/
- FSB, "Global Monitoring Report on Nonbank Financial Intermediation 2025": https://www.fsb.org/2025/12/global-monitoring-report-on-nonbank-financial-intermediation-2025/
- FDIC, "Dissecting Depositor Flight: An Analysis of the Spring 2023 Bank Failures" (2026 release): https://www.fdic.gov/news/press-releases/2026/fdic-releases-staff-study-deposit-flows-three-failed-banks-spring-2023
