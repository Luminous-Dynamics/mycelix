# Mycelix Capital-to-Commons v1

Status: normative design draft

Parent: MYC-CAP-001 / #880
Program issue: MYC-CAP-002 / #912

## 1. Purpose

Mycelix Capital-to-Commons defines a financing and ownership pattern for public-interest infrastructure in which private capital can earn a fair, pre-agreed return without acquiring perpetual extractive ownership or unilateral constitutional control.

The preferred architecture is not "private ownership now, community ownership later." It is:

```text
commons/public-benefit stewardship from inception
+
time-/return-bounded investor claims
+
contestable operating rights
+
progressively increasing community economics
+
machine-auditable transition state
```

The model is intended for assets with public-good, essential-service, local-monopoly, shared-network, or long-lived commons characteristics. It is not a default financing model for every Mycelix commercial software product.

## 2. Core constitutional invariant

```text
capital recovery right
!= permanent asset ownership
!= protocol authority
!= community governance authority
!= right to liquidate the commons
```

Capital may be rewarded for financing productive infrastructure. Capital does not thereby acquire an indefinite right to extract rents from, sell, or constitutionally redefine that infrastructure.

## 3. Unbundle the rights

Do not treat "ownership" as one indivisible property. A project should explicitly separate at least five rights:

1. **Legal title / asset custody** — who legally holds the infrastructure or controlling asset right.
2. **Constitutional stewardship** — who can defend the mission, asset lock, service obligations, and transfer restrictions.
3. **Economic claim** — who receives project cash flows and under what bounded return envelope.
4. **Operational right** — who may build, maintain, operate, or upgrade the asset, for what term and under what service-level obligations.
5. **Use / beneficiary right** — who is entitled to service, access, participation, tariff protections, or community benefit.

Where lawful and practical, legal title and constitutional stewardship SHOULD be community/public-benefit locked from inception. Investor economics and operating concessions may be temporary and transferable only within the constitutional constraints.

## 4. Preferred entity topology

```text
Community / Public-Benefit Steward
        │
        ├─ asset title / constitutional control
        ├─ asset lock / transfer restrictions
        └─ beneficiary governance
                 │
                 ▼
         Project / Infrastructure SPV
        ┌────────┼─────────┐
        │        │         │
   investors   operator   lenders
  bounded      concession  senior
  claims       / service   claims
```

The steward may be implemented, depending on jurisdiction and asset class, as a community/public-benefit trust, perpetual purpose trust, community benefit society, public authority, cooperative, foundation/commons entity, or equivalent structure with enforceable asset-lock and mission-protection mechanisms.

Exact legal form is outside this specification and requires qualified counsel.

## 5. Investor claims

Eligible financing instruments may include:

- senior/project debt;
- amortizing revenue participation;
- redeemable non-voting preferred interests;
- self-liquidating equity-like claims;
- availability-payment-backed claims;
- concession-linked cash-flow rights;
- benchmark- or inflation-indexed bounded returns;
- narrowly scoped protective covenants.

Investor protection MAY include information rights, default remedies, replacement of a failed operator, covenant enforcement, and agreed economic protections.

Investor protection MUST NOT imply unilateral authority to dissolve the asset lock, change the public-benefit purpose, seize protocol governance, or reset the community transition.

## 6. Return Envelope

"ROI achieved" is too ambiguous to be a constitutional trigger. The project MUST define an objective **Return Envelope** before capital is accepted.

A minimal conceptual envelope is:

```text
eligible contributed capital
+
agreed return entitlement
+
explicitly eligible unrecovered project costs
-
counted distributions / redemptions
=
remaining investor claim
```

The exact return profile may use a capped multiple, preferred return, bounded IRR, indexed yield, or other lawful instrument. The profile MUST define:

- what counts as contributed capital;
- what costs may increase the claim;
- whether and how return accrues over time;
- what distributions reduce the claim;
- maximum return / duration where applicable;
- treatment of grants, subsidies, tax credits, insurance proceeds, refinancing, and asset sales;
- rounding and accounting units;
- amendment authority;
- independent verification source.

No unspecified management fee, related-party payment, refinancing charge, transfer price, or affiliate service charge may silently increase the Return Envelope.

## 7. Capital Claim Balance

For machine-auditable implementations, use exact fixed-point or integer accounting units, never floating-point authority arithmetic.

A future executable profile should model:

```text
CapitalClaimBalance(t)
  = prior balance
  + allowed accrual
  + approved new qualifying capital
  + approved recoverable lifecycle expenditure
  - counted investor distributions
  - mandatory impairment / write-down events
```

The claim reaches the terminal state only when the profile-defined balance is exactly satisfied under the governing evidence set.

```text
remaining claim == 0
=> investor transition condition satisfied
```

This does not itself prove that every legal transfer or redemption step has completed; those are separate state transitions.

## 8. Progressive Commons Accrual

Do not wait until one cliff event to create community rights.

The project SHOULD define community/public-benefit rights from inception and may progressively increase residual economic rights as investor claims amortize.

A simple conceptual progress measure is:

```text
commons_accrual
= 1 - remaining_eligible_investor_claim / initial_eligible_return_envelope
```

This ratio is descriptive only unless explicitly adopted as the legal/economic allocation formula. The governing profile may instead use tranches, milestones, redeemed units, or scheduled vesting.

Community constitutional protections SHOULD NOT depend on this ratio; mission/asset-lock protections begin at inception.

## 9. Cash-flow waterfall

The default normative ordering SHOULD be:

```text
gross project revenue
  -> taxes / statutory obligations
  -> essential operating cost
  -> safety and compliance cost
  -> maintenance reserve
  -> lifecycle / replacement reserve
  -> resilience / disaster reserve
  -> senior secured debt service
  -> bounded investor return / redemption
  -> community dividend / tariff relief / commons reinvestment
```

A project MUST NOT accelerate investor returns by starving maintenance, resilience, safety, lifecycle replacement, or legally required service obligations.

Reserve policy and minimum coverage MUST be explicit and independently auditable.

## 10. Operating rights remain contestable

Community ownership does not require permanent operation by the community steward.

The asset steward SHOULD be able to procure or rebid operations separately from asset ownership.

```text
community owns/stewards infrastructure
        ↓
operator A receives bounded concession
        ↓
service term expires / performance fails
        ↓
operator A, B, C compete for next term
```

This preserves public-benefit ownership while retaining competition in construction, maintenance, operations, upgrades, software, insurance, and financing.

Operator replacement MUST NOT reset capital-to-commons transition state.

## 11. Transition state machine

A future executable implementation should distinguish at least:

```text
PROPOSED
  ↓
CAPITAL_COMMITTED
  ↓
CONSTRUCTION
  ↓
OPERATING_INVESTOR_CLAIM_ACTIVE
  ↓
PARTIALLY_AMORTIZED
  ↓
RETURN_ENVELOPE_SATISFIED
  ↓
REDEMPTION_PENDING
  ↓
COMMONS_ECONOMICS_COMPLETE
  ↓
STEADY_STATE_COMMONS
```

Additional orthogonal states are needed for:

- refinancing;
- operator default;
- lender enforcement;
- force majeure;
- impairment;
- project failure / early termination;
- community-governance dispute;
- asset replacement;
- approved constitutional amendment.

A financial PASS MUST NOT collapse these states into a single boolean "community owned" field.

## 12. Refinancing invariant

Refinancing may replace or restructure eligible financial claims, but MUST NOT silently reset accrued commons rights or the original extraction horizon.

```text
refinancing
!= new project genesis
!= community vesting reset
!= unlimited new return envelope
```

Any increase to the total Return Envelope requires the profile-defined amendment authority and a documented reason such as qualified new capital expenditure, emergency reconstruction, or community-approved expansion.

Pure refinancing of existing obligations does not qualify as new productive capital by default.

## 13. Change-of-control invariant

Sale of an investor, operator, lender position, or project company MUST carry forward:

- the asset lock;
- remaining Return Envelope;
- historical distributions;
- accrued community rights;
- service obligations;
- maintenance/resilience reserve obligations;
- transition state.

The purchaser acquires the remaining bounded claim, not a fresh perpetual claim.

## 14. Default and insolvency

Creditor protections must be real enough to finance projects, but default must not become a back door to permanent privatization.

A project profile SHOULD distinguish:

- right to cure;
- temporary step-in operational rights;
- right to replace operator;
- foreclosure against eligible financial/project-company interests;
- rights against non-locked collateral;
- prohibited seizure of constitutionally locked commons assets where law permits.

The constitutional recovery path should preserve essential service continuity and then restore compliant stewardship.

Exact insolvency priority is jurisdiction-specific and requires legal design.

## 15. Grants and public subsidy

Public grants, philanthropic grants, tax credits, or other non-repayable support reduce private capital requirements unless their governing instrument explicitly provides otherwise.

```text
public grant
!= investor contributed principal
```

An investor may not count a public grant as contributed private principal and then earn a return on that amount absent an explicit, independently approved structure.

## 16. Anti-extraction invariants

A compliant profile MUST address at least:

1. related-party management fees;
2. affiliate procurement and transfer pricing;
3. unnecessary refinancing;
4. artificial extension of concession terms;
5. excessive executive/operator compensation charged to the project;
6. monopoly tariff increases used solely to accelerate investor distributions;
7. deliberate under-maintenance;
8. deferred lifecycle replacement;
9. sale-and-leaseback or derivative structures that recreate permanent extraction;
10. off-balance-sheet claims that bypass the Return Envelope;
11. hidden side letters granting governance/control rights;
12. dilution of community economics after vesting;
13. reset of vesting after merger, bankruptcy, operator replacement, or change of jurisdiction.

## 17. Community governance

"Community owned" MUST identify the constituency.

A profile should identify relevant stakeholder classes such as:

- residents;
- service users;
- workers;
- local businesses;
- municipality/public authority;
- affected neighboring communities;
- independent public-benefit stewards;
- other materially affected parties.

Governance may be one-person/one-vote, cooperative, delegated, multi-stakeholder, chambered, or another bounded form. No single stakeholder class should receive capture rights merely because it supplied capital.

At minimum define:

- eligibility;
- voting/representation rules;
- conflict-of-interest rules;
- recall/rotation;
- minority protections;
- transparency;
- amendment thresholds;
- emergency authority;
- anti-corruption controls.

Community ownership is not itself proof of democratic legitimacy.

## 18. Service and tariff constitution

For essential or monopoly-like infrastructure, the project SHOULD separate return mechanics from unconstrained pricing.

Tariffs/prices should be bounded by transparent service and affordability rules. Investor return should come from the agreed project economics, not from an unlimited right to raise prices until the target return is reached.

Potential mechanisms include:

- regulated tariff bands;
- availability payments;
- indexed service fees;
- affordability floors/ceilings;
- community-approved extraordinary adjustments;
- independent benchmark review.

## 19. Evidence integration

Capital-to-Commons should integrate with MYC-EVID as a dedicated evidence plane/profile.

The system should be able to reconstruct:

- original capital contributions;
- Return Envelope revision;
- eligible new capital;
- approved lifecycle expenditure;
- counted distributions;
- remaining claim;
- reserve balances/status;
- current operator/concession;
- current legal/stewardship rights;
- community accrual state;
- refinancing/change-of-control history;
- amendments;
- terminal redemption/transition event.

Capital-provider self-report is not sufficient authority for final transition state.

## 20. Commons Transition Receipt

A future executable profile should emit an independently verifiable receipt describing one transition checkpoint without itself granting governance authority.

Conceptual fields:

```text
project_id
capital_profile_id
constitutional_profile_id
period_or_epoch
opening_claim_balance
eligible_additions
counted_distributions
closing_claim_balance
reserve_status
operator_id
steward_id
community_accrual_state
transition_state
evidence_root
verifier_profile
nonclaims
```

A receipt proves only the bounded accounting/governance facts established by its verifier profile.

## 21. Applicability

Strong candidates include:

- local/federated fiber and connectivity infrastructure;
- community compute/storage;
- energy generation, storage, microgrids, or distribution assets;
- water/waste systems;
- municipal digital infrastructure;
- public identity/credential infrastructure;
- local clearing/payment rails where lawful;
- logistics/market infrastructure;
- housing/land/community facilities;
- other durable public-interest assets.

Do not automatically apply this model to:

- ordinary SaaS products;
- short-lived commercial tools;
- highly speculative R&D;
- businesses where perpetual equity is the appropriate risk-capital instrument.

## 22. Digital public infrastructure

For protocols and standards, prefer commons stewardship from inception.

```text
open protocol / conformance semantics
  -> neutral steward / commons

managed implementation / hosting / support
  -> investable commercial operator

public deployment asset
  -> Capital-to-Commons project vehicle where appropriate
```

Investors finance implementations, operators, deployments, and services rather than purchasing permanent protocol sovereignty.

## 23. Worked conceptual example: community compute

A regional community requires a compute/storage facility.

- The public-benefit steward holds the land/asset-lock rights from inception.
- A project SPV raises debt and redeemable investment capital.
- A commercial operator runs the facility under a 10-year service agreement.
- Revenue first funds operating, safety, maintenance, replacement, and resilience reserves.
- Investor claims amortize under a pre-agreed Return Envelope.
- Community residual economics increase as claims are redeemed.
- The operator may be rebid without changing the asset lock or vesting history.
- After the final eligible investor claim is redeemed, residual economics flow to tariff relief, expansion, reserves, and community benefit under the constitution.

## 24. Worked conceptual example: municipal fiber

A municipality/community steward grants a project vehicle rights to finance and build fiber infrastructure while retaining constitutional/public-benefit control.

The operator receives a bounded concession and earns service revenue. Investors receive debt and/or redeemable revenue-participation claims. Network maintenance and replacement reserves rank ahead of discretionary investor distributions.

When the Return Envelope is satisfied, investor economic claims expire or redeem. The fiber remains public/community-benefit infrastructure; operation may continue under the existing contract or be competitively rebid.

## 25. External precedents informing the design

This specification is not a copy of any one legal model. It composes several established ideas:

- World Bank PPP guidance recognizes concession/BOT structures where private parties finance/operate infrastructure for a defined project period while public ownership may remain in place or assets revert at the end.
- World Bank PPP guidance also emphasizes that legal title and economic exploitation rights can be separated.
- UK Community Interest Companies use a compulsory asset lock intended to keep assets and retained value for community benefit.
- UK community benefit societies can adopt statutory asset locks restricting private appropriation of residual/community assets.
- Steward-ownership financing practice demonstrates redeemable/self-liquidating capital and separation of voting/control rights from investor economic rights.

These precedents establish feasibility of the component ideas, not legal validity of this exact Mycelix structure in any jurisdiction.

## 26. Promotion gates

Before a project may call itself `Capital-to-Commons Qualified`, require separately named gates for:

```text
constitutional / asset-lock validity
capital-rights schedule
return-envelope specification
reserve policy
service / tariff policy
community constituency/governance
operator/concession contract
independent accounting/evidence profile
refinancing/default/change-of-control rules
legal review for the target jurisdiction
```

A financing close alone is not qualification.

## 27. Nonclaims

This document does not establish:

- securities-law compliance;
- tax treatment;
- trust/cooperative validity;
- municipal/procurement authority;
- utility regulation compliance;
- lender enforceability;
- accounting treatment;
- investor suitability;
- guaranteed returns;
- democratic legitimacy;
- project feasibility.

It freezes the intended economic architecture so those questions can be evaluated against a clear target.

## 28. Constitutional summary

```text
Public infrastructure should be investable without being permanently acquirable.

Capital receives a bounded right to participate in value creation.
The commons retains the durable mission and asset lock.
Operations remain professional and contestable.
Community economics grow as private risk is retired.
The transition is governed by explicit evidence, not promises.
```
