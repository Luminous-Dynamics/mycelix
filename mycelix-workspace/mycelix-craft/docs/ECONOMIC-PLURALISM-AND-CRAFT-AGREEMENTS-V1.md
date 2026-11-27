# Economic Pluralism and Craft Agreements V1

- **Work item:** CRAFT-ECONOMIC-AGREEMENTS-001
- **Status:** Pure Rust agreement-reference validator added; no new currency or ledger implemented; owner integrations remain pending
- **Version:** 1.0-draft
- **Owner:** Mycelix Craft
- **License:** AGPL-3.0-or-later

## 1. Decision summary

**Yes: Craft should support multiple economic systems, including agreements that combine more than one arrangement.** It should not force every guild, employer, worker, apprenticeship or community project into one currency, token, payment rail, price theory or settlement process.

This is not a request to build another currency inside Craft. Mycelix already has a substantial economic architecture:

- Finance defines distinct MYCEL, SAP and TEND semantics.
- [FED-CUR-001 #5464](https://github.com/Luminous-Dynamics/mycelix/issues/5464) and [draft PR #5465](https://github.com/Luminous-Dynamics/mycelix/pull/5465) qualify independently governed community currencies and loss-bounded federation clearing.
- [FED-CUR-002 #5484](https://github.com/Luminous-Dynamics/mycelix/issues/5484) covers partition-safe, pre-allocated exposure reservations.
- [FIN-ECO-006 / Creditism #4771](https://github.com/Luminous-Dynamics/mycelix/issues/4771) explores distinct contribution, capacity, community-allocation and deletion semantics without inventing another generic Mycelix currency.
- [ECON-EFFECT-001 #6602](https://github.com/Luminous-Dynamics/mycelix/issues/6602), [ECON-INTEROP-001 #6610](https://github.com/Luminous-Dynamics/mycelix/issues/6610) and [ECON-EFFECT-002 #6612](https://github.com/Luminous-Dynamics/mycelix/issues/6612) define owner-scoped economic effects, semantic interoperability and reusable effect-role templates.
- [ECON-SAP-PROFILES-001 #6634](https://github.com/Luminous-Dynamics/mycelix/issues/6634) distinguishes local atomic transfer from owner-authenticated bilateral transfer.

Craft should reference those systems through typed, versioned agreement terms. Finance and other domain owners remain responsible for authorization, ledgers, resource movements and settlement.

### 1.1 Executable reference core

The companion crate `crates/economic-agreement-v1` now validates immutable issuer/profile/revision/digest references, owner and unit identity, exact integer quantities, party consent to the terms digest, duplicate owner-obligation pairs, profile lifecycle/validity, and explicit conversion-quote references accepted by all parties. It performs no arithmetic across units and has no payment, balance, journal, FX-execution, reserve, or settlement API. Profile signatures, consent signatures, quote authenticity, and owner-side effects remain external verification duties.

## 2. Why pluralism matters

Different kinds of work and communities need different coordination mechanisms. A small repair cooperative may prefer mutual credit. A salaried employer may pay in local fiat through a bank. A community care network may use time credits. An open-source guild may fund work through grants or commons budgets. A worker-owned enterprise may distribute patronage or surplus under its constitution. A project may combine wages, training grants, in-kind tools and a bounded timebank exchange.

These are not merely different payment rails. They can encode different institutional relationships:

- **Market exchange:** price, wage, contract, purchase order and payment.
- **Cooperative ownership:** member capital, patronage, surplus allocation and democratic member control.
- **Mutual credit:** reciprocal credit/debit positions under a community-specific risk and clearing policy.
- **Time exchange:** defined time units and service categories; an hour in one network is not automatically an hour of every other service or a legal wage.
- **Commons allocation:** shared resources, budgets, access rights and accountable stewardship.
- **Grant / public-benefit funding:** eligibility, award, tranche, evidence, recipient claim and recovery.
- **In-kind and reciprocal exchange:** tools, materials, space, accommodation, services or agreed resource transfers.
- **Contribution-based allocation:** explicitly defined contribution rights or non-transferable allocation units, with separate rules for capacity and spending.
- **Volunteer / unpaid commons contribution:** explicit and voluntary; never silently inferred from a missing price or a guild membership.
- **Hybrid arrangements:** two or more of the above with independently described components.

Economic pluralism does not mean every unit can be exchanged for every other unit. It means communities can choose different arrangements while preserving their own rules and making cross-system boundaries explicit.

## 3. Craft's responsibility boundary

Craft owns:
- the work opportunity and work agreement;
- the identity and consent of the contracting participants;
- the accepted work scope, milestones and contribution responsibilities;
- the agreed economic arrangement references and exact versions;
- the evidence required to claim that work or a milestone occurred;
- disputes about work acceptance and agreement terms.

Craft does **not** own:
- currency issuance or monetary policy;
- participant account balances;
- FX rates or asset valuation;
- escrow, collateral, reserves or credit limits;
- payment instructions, bank messages or settlement finality;
- accounting journals or tax determinations;
- physical resource inventory or consumption;
- public-benefit eligibility decisions.

Those remain with Finance, the relevant community currency issuer, Commons, Supply Chain, Manufacturing, the external financial institution, or other responsible authority.

A Craft agreement records intent and references obligations. It must not create a second mutable ledger.

## 4. Proposed agreement model

A future `CraftEconomicAgreementV1` should be an immutable, versioned envelope with one or more independently typed components.

Conceptually:

```text
WorkAgreement
  ├── parties + consent + jurisdiction
  ├── work scope + milestones + acceptance evidence
  ├── economic components[]
  │     ├── economic system profile ID + revision
  │     ├── issuer / owner authority reference
  │     ├── exact unit profile and quantity semantics
  │     ├── obligation reference + immutable intent digest
  │     ├── due condition / milestone
  │     ├── evidence and acceptance requirements
  │     └── domain-owned settlement / effect references
  ├── conversion or valuation references (optional)
  ├── dispute, correction, withdrawal and exit terms
  └── consent and signature references
```

The agreement should allow multiple components. For example, a single apprenticeship may contain:
- a fiat stipend under an employer contract;
- a community mutual-credit component for local services;
- a training grant paid to a provider;
- in-kind equipment or transport;
- a co-operative surplus/patronage rule after the member becomes eligible.

Each component remains independently typed and governed. Craft does not add unlike quantities or imply that these components are economically equivalent.

### 4.1 Economic system profile reference

Every component must name a canonical, issuer-scoped profile, not a display ticker or user-supplied label.

Minimum reference fields:

- immutable instrument/system ID;
- issuer or community DID;
- profile family and profile revision;
- exact unit/scale definition;
- issuance or allocation authority reference;
- policy digest and lifecycle state;
- applicable jurisdiction and effective period;
- owner/settlement authority;
- supported effect/claim template and schema version.

Unknown, stale, suspended or retired profiles fail closed for new obligations. Historical agreement records remain readable under their original profile version.

### 4.2 Component categories

The initial envelope should support references to, rather than reimplement, these categories:

| Category | Examples | Authority owner |
| --- | --- | --- |
| Monetary transfer | Fiat wages, invoice, SAP transfer, external tokenized instrument | Finance / external payment institution / instrument issuer |
| Mutual credit | TEND, local community mutual credit, CES/LETS-style network | Issuing community + Finance adapter |
| Time exchange | Timebank hours or service credits | Timebank/community issuer |
| Grant / public benefit | Training grant, project award, benefit claim | Grant or public authority |
| Commons allocation | Shared budget, project capacity, housing or equipment access | Commons / resource owner |
| Cooperative participation | Member capital, patronage, surplus allocation | Cooperative constitution and finance owner |
| In-kind / reciprocal | Parts, tools, meals, accommodation, service exchange | Resource owner / agreement parties |
| Contribution allocation | Non-transferable personal or community allocation rights | Exact contribution-system profile |
| Volunteer contribution | Explicit voluntary contribution without monetary entitlement | Participating parties and applicable legal rules |
| Mixed | Any explicitly accepted combination | Each component's own owner |

MYCEL must remain non-transferable reputation under its current Finance semantics. It must not silently become a wage, payment token, collateral or transferable reward. SAP, TEND and community currencies also retain their own separate rules.

### 4.3 Exact quantities and unit safety

- No floating-point quantity at authorization or settlement boundaries.
- Use exact typed units or integer/fixed-precision quantities declared by the instrument profile.
- Do not add raw numerals from unlike units.
- Do not infer time-credit parity with wages, mutual-credit parity with fiat, resource-credit backing, or legal equivalence.
- A conversion requires a separate explicit quote/valuation record with source, method, timestamp, validity, fees/spread, limits, uncertainty and parties' acceptance.
- A quote is not a payment, and a payment status is not settlement finality.

### 4.4 Failure and fallback behavior

- An unavailable settlement system does not authorize substitution into another currency or rail.
- An agreement may pre-authorize a fallback component, but the trigger and exact alternative terms must be visible and consented before work begins.
- A failed community currency or federation edge must not freeze unrelated local activity or seize unrelated balances.
- A contract may pause new obligations while already incurred obligations are reconciled under their original rules.
- Cross-community liabilities and clearing are governed by the explicit federation agreement, not by guild membership alone.
- Dispute, cancellation, reversal and recovery must be owner-specific and append-only.

## 5. Guild economic sovereignty

A guild should be able to choose its economic model in its charter and work agreements. It may also allow different models for different projects.

Examples:
- a worker cooperative that owns the platform and distributes surplus by its bylaws;
- a professional association funded by membership fees and grants;
- a mutual-aid guild using community currency and a timebank;
- an open-source guild funded by grants, sponsorship and commons budgets;
- a local manufacturing cooperative using fiat procurement plus mutual credit for member services;
- a multi-stakeholder guild combining worker, learner, user and community participation.

A guild may adopt the MYCELIX Finance family, a community-specific currency, a Creditism-compatible adapter, external fiat, in-kind arrangements, or no currency. It may join multiple economic federations or none.

The guild's governance role may define which economic policies its members adopt. That does not give it authority to mint currency, mutate accounts or settle payments. It must use the issuing community and domain-owner authorization.

## 6. Interoperability and standards

Craft should rely on the current Economic OS crosswalk and preserve the semantic differences between:

- **Valueflows:** plans, commitments, claims, events, resources and agents;
- **W3C PROV:** entity/activity/agent provenance and derivation;
- **GS1 EPCIS:** physical and digital visibility events;
- **ISO 20022:** versioned payment, clearing, status, return, reversal and reporting messages;
- **Peppol / EN 16931:** invoice syntax and jurisdiction-specific business rules;
- **W3C Verifiable Credentials:** portable issuer/holder/verifier claims.

Do not infer that:
- an offer or commitment means work was delivered;
- a work event means the recipient accepted it;
- a valid invoice means a payment occurred;
- a payment status means final settlement;
- a valid signature proves a physical resource or external reserve exists;
- an instrument symbol identifies an issuer or guarantees parity;
- a standards mapping makes unlike economic systems fungible.

The canonical source remains in the owning domain. Interchange is a loss-aware projection with explicit unsupported fields and claim ceilings.

## 7. Benchmarks and qualification targets

All targets are proposed, not measured results.

### Hard correctness gates

| Benchmark | Target |
| --- | ---: |
| Unknown or stale economic profile admitted for a new obligation | 0 |
| Cross-system conversion inferred without explicit quote/agreement | 0 |
| Unlike units combined arithmetically | 0 |
| Duplicate owner effect after replay/concurrency | 0 |
| Craft writing a Finance or issuer-owned balance/journal | 0 |
| Unconsented economic fallback or substitution | 0 |
| Unsupported semantic mapping silently dropped | 0 |
| Frozen conformance cases correctly classified | 100% |

### Economic pluralism quality metrics

Measure separately:
- profile coverage across supported economic arrangements;
- successful agreement completion per arrangement;
- time from offer to mutually accepted terms;
- time to settle and percentage of obligations held/indeterminate;
- fees, spreads and third-party costs;
- liquidity and capacity utilization within each instrument;
- defaults, loss exposure, dispute rate and recovery time;
- mentor compensation and worker income;
- accessibility for newcomers and people without conventional banking access;
- local retention of value and dependency on external rails;
- participant understanding of unit meaning, risks and fallback terms;
- privacy, data minimization and operator burden;
- concentration of issuance, treasury, clearing and governance power.

Do not optimize for the number of currencies or transaction volume. A community may rationally choose no currency. A successful system may deliberately keep some resources outside exchange.

### Comparative baselines

Compare the specific arrangement and effect boundary—not entire platforms:

- employer payroll and invoice/payment-provider workflows;
- platform-marketplace contracts and escrow;
- cooperative member-capital and patronage systems;
- TEND/timebank/community exchange systems;
- community mutual-credit and federated currency designs;
- grant and public-benefit workflows;
- commons resource-allocation systems;
- Valueflows/REA resource-flow representations and standard interchange profiles.

Use the same work tasks, agreement requirements, exact instrument profiles and failure scenarios. Report legal/jurisdiction constraints and operational costs rather than claiming a universal best model.

## 7.1 Industry baselines and the Craft differentiation

Craft should compare with actual market capabilities, not a straw-man résumé board.

| Baseline | Existing capability | What Craft should add or measure |
| --- | --- | --- |
| Upwork-style work marketplaces | Talent profiles, work history, proposals, contract terms, project workspaces, milestone/hourly protections and payment workflows | Portable evidence across marketplaces, independent physical-work verification, member-governed rules and multiple economic arrangements |
| Eightfold-style talent intelligence | Skills-based candidate matching, internal mobility, project staffing, skill-gap identification and development recommendations | Explainable evidence provenance, subject-controlled disclosure, explicit abstention, independent assessment and no opaque universal person score |
| Platform cooperatives | Shared ownership and democratic control of platform technology, work structures, algorithms and data | Reusable infrastructure for member-owned guilds while leaving ownership and policy with each cooperative |
| Community-currency / mutual-credit networks | Local issuance, exchange and community-specific credit rules | Versioned instrument identity, explicit federation agreements, bounded exposure and typed cross-system clearing |
| Commons and grant programmes | Shared resource allocation, public-purpose budgets, eligibility and milestone funding | Bind work acceptance to resource evidence without pretending an allocation is cash or physical capacity |

References:
- Upwork profile/work-history and protected-payment documentation: https://support.upwork.com/hc/en-us/articles/360016252373-How-to-build-your-freelancer-profile-the-essentials and https://www.upwork.com/pricing/client
- Eightfold product and skill insights: https://docs.eightfold.ai/ and https://docs.eightfold.ai/en_US/ta-core/skills-related-insights
- Platform Cooperativism Consortium: https://platform.coop/
- FED-CUR-001 / NCF-1 #5464 and PR #5465 for community-currency federation.

The realistic competitive claim is that Craft can compose evidence, work, governance and economic pluralism across communities. It should not claim that skill matching, portfolios, protected payments, token exchange or cooperative governance are individually novel.

## 8. Implementation sequence

1. **CRAFT-ECON-001:** publish this ownership boundary and map Craft terms to existing Economic OS references.
2. **CRAFT-ECON-002:** add a pure Rust agreement envelope with typed component references, exact units and version pins; no ledger mutation.
3. **CRAFT-ECON-003:** add the negative-first corpus for unknown profile, stale profile, unit mismatch, unauthorized issuer, replay, fallback substitution, cross-federation leakage and contradictory settlement observations.
4. **CRAFT-ECON-004:** bind apprenticeship and work agreements to owner-specific obligation and effect references.
5. **CRAFT-ECON-005:** implement Finance / community currency / timebank / grants / Commons adapters only after each owner contract is qualified.
6. **CRAFT-ECON-006:** run matched pilots and report outcomes by arrangement, including no-currency control groups where appropriate.

### Existing work to compose

- FED-CUR-001 #5464 / draft NCF-1 PR #5465;
- FED-CUR-002 #5484;
- FIN-ECO-006 #4771;
- ECON-EFFECT-001 #6602 and crosswalk PR #6603;
- ECON-INTEROP-001 #6610 and crosswalk PR #6611;
- ECON-EFFECT-002 #6612 and typed role/template PR #6613;
- ECON-SAP-PROFILES-001 #6634;
- external economic instrument / adapter algebra #3343 and #3344;
- exact quote/conversion/FX semantics #1157.

## 9. Research references

- Platform Cooperativism Consortium: https://platform.coop/
- OECD, *Platform Cooperatives and Employment*: https://www.oecd.org/content/dam/oecd/en/publications/reports/2023/09/platform-cooperatives-and-employment_8e8a1d61/3eab339f-en.pdf
- ILO Recommendation 208, Quality Apprenticeships: https://www.ilo.org/resource/other/r208-quality-apprenticeships-recommendation-2023
- W3C Verifiable Credentials Data Model 2.0: https://www.w3.org/TR/vc-data-model-2.0/
- Valueflows: https://valueflo.ws/
- GS1 EPCIS 2.0: https://www.gs1.org/standards/epcis
- ISO 20022: https://www.iso20022.org/
- Interledger Protocol and settlement engines: https://interledger.org/developers/rfcs/interledger-protocol/

## Claim boundary

This document is a proposed Craft integration contract. It does not create a currency, approve monetary policy, promise convertibility, establish legal tender, make tax or employment-law determinations, guarantee solvency, or authorize any economic effect. Each instrument issuer, community, Finance owner and jurisdiction remains responsible for its own policy and execution.
