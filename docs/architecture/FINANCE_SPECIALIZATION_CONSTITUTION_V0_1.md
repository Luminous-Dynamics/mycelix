# Mycelix Finance Specialization Constitution v0.1

Status: architecture contract. This document defines the ownership, evidence, settlement, and integration boundaries for the Finance domain beneath the Mycelix Business Fabric. It does not establish new runtime behavior, qualification, legal compliance, settlement finality, or autonomous financial authority.

Issue: FIN-ECO-000 / #913.

## Purpose

Mycelix already has separate authoritative domains for business coordination, commerce, finance, identity, governance, physical resources, work, justice, and evidence. Finance therefore must not become a universal economic database or a second business-authority stack.

The Finance domain exists to own and qualify financial state that is genuinely financial: exact quantities, balances, ledgers, treasury state, financial reservations, exposure, finance-specific accounting treatment, settlement observations, and finance-specific settlement qualification.

The Business Fabric remains the coordination layer for decision lineage, action contracts, authorization bindings, prepared actions, aggregate policy, execution attempts, provider acknowledgements, and economic reconciliation.

This constitution makes the intersection explicit.

## Upstream constitutional dependencies

This document is subordinate to and must preserve the existing Business Fabric architecture, including the principles established by the Business Safety Constitution and its executable contract stack.

In particular:

- Commerce owns commercial exchange, agreements, commitments, claims, and commercial settlement semantics.
- Finance owns ledgers, balances, treasury, accounting treatment, and financial reservations.
- Governance/Xenia owns policy authority, capabilities, delegation, revocation, and institutional authorization.
- Identity/Lawful Identity owns identity and legal-entity bindings.
- Supply Chain / Property / Commons own physical-resource, custody, provenance, and rights state appropriate to their domains.
- Justice owns dispute and adjudication state.
- Symthaea owns no institutional authority merely by reasoning about any of these domains.

Finance may hold typed references to authoritative records in other domains. A reference or projection MUST NOT silently become authoritative ownership of the referenced state.

## Finance is a specialization, not a parallel fabric

The relationship is:

```text
Business Fabric
  decision / contract / authorization / coordination / attempt / reconciliation
                         |
                         v
Finance specialization
  exact amounts / balances / reservations / exposure / settlement qualification
                         |
                         v
Finance-owned state and evidence
```

Finance does not redefine the generic decision pipeline.

Finance consumes or binds to existing Business Fabric concepts such as:

- Action Contract identity;
- Decision Capsule lineage;
- Prepared Action identity and lifetime;
- authorization binding;
- authority epoch and fencing context;
- reservation references;
- aggregate-policy requirements;
- idempotency identity;
- execution-attempt identity;
- provider acknowledgement;
- economic reconciliation;
- execution-time material revalidation.

Finance adds only the domain-specific checks and state transitions that Finance itself is authoritative to perform.

## Core separations

The following are normative distinctions.

```text
commercial agreement
!= financial ledger state
```

A contract or commercial obligation may justify a financial effect, but it is not itself a ledger entry or balance mutation.

```text
business authorization
!= financial reservation
```

Authorization answers whether an action is institutionally permitted. A financial reservation answers whether Finance has currently reserved the required scarce financial capacity. One cannot substitute for the other.

```text
financial reservation
!= financial execution
```

A reservation is a bounded claim on capacity. It is not evidence that a financial effect occurred.

```text
execution attempt
!= transport success
!= provider acknowledgement
!= settlement observation
!= qualified settlement
!= commercial obligation satisfaction
!= legal discharge
```

Each promotion requires its own authority and evidence.

```text
DHT visibility
!= settlement finality
```

The presence of an entry, action, link, receipt, or countersigned record on a Holochain DHT is not by itself generic evidence that a scarce financial effect is final.

```text
countersigning
!= generic double-spend prevention
```

Countersigning may be useful evidence or coordination for a specific profile, but it is not a universal finality theorem.

```text
finance result
!= accounting/legal/tax truth
```

A Finance result may be an input into qualified accounting, legal, or tax processes. It does not automatically establish those external semantics.

```text
Symthaea proposal
!= authority
!= financial reservation
!= financial execution
```

Reasoning capability cannot manufacture permission or scarce financial capacity.

```text
capital contribution
!= protocol authority
```

Financial investment does not implicitly grant protocol, evidence, federation, identity, or technical-qualification authority.

## Finance authority surface

Finance may own or qualify the following classes of state when explicitly implemented and qualified:

### Exact financial quantities

- asset/currency-bound amounts;
- signed mutual-credit/exposure quantities;
- exact rates and ratios;
- deterministic rounding and scale rules;
- conversion results only under an explicit conversion authority/profile.

Authoritative financial quantities SHOULD use exact integer, rational, decimal, or bounded fixed-point semantics. Binary floating-point values are suitable for analysis, forecasting, optimization, and diagnostics, but SHOULD NOT be canonical persisted representations for authoritative amounts, debts, reservations, exposures, or settlement amounts.

### Ledger and balance state

- balances;
- ledger mutations;
- issuance/burn/redemption state where Finance is the named authority;
- provenance required by the ledger model;
- internal conservation evidence.

### Treasury state

- treasury balances;
- available and reserved capacity;
- liquidity floors where Finance owns their numeric state;
- finance-side budget consumption.

The policy authorizing a treasury action may be owned elsewhere. Finance enforces the Finance-owned state consequences of that policy.

### Financial reservations

Finance may issue bounded reservation records or references for scarce financial capacity, including:

- spend capacity;
- liquidity capacity;
- credit exposure;
- collateral-backed borrowing capacity;
- project capital envelopes;
- other explicitly defined financial limits.

A Finance-issued reservation MUST be scoped, amount/asset-bound where applicable, time-bounded where appropriate, replay-safe, and attributable to the exact action or intent it reserves capacity for.

### Credit exposure

Finance may own:

- current exposure;
- authorized credit limit projections;
- utilization;
- finance-specific collateral valuation projections where explicitly qualified;
- concentration and portfolio quantities;
- finance-owned default/reserve state.

Finance does not thereby own the underlying identity, contract, receivable, property, or commercial relationship.

### Settlement observations and qualification

Finance may own typed records that represent:

- observations from internal or external settlement rails;
- the subject financial effect being evaluated;
- the selected finality profile;
- the evidence frontier used;
- qualification results under that profile;
- later reversal/reorg/chargeback/invalidation lineage where relevant;
- unknown or unresolved settlement state.

A qualified settlement result means only what its exact profile proves.

## State triad

Finance SHOULD avoid collapsing all financial truth into one status enum.

For consequential financial effects, model three independently meaningful state dimensions when applicable.

### 1. Financial state

Examples:

- amount owed/outstanding;
- balance;
- reservation amount;
- exposure;
- ledger effect;
- treasury availability.

### 2. Settlement state

Examples:

- not attempted;
- attempted;
- observation received;
- finality pending;
- qualified under profile X;
- reversed/reorganized/charged back;
- unknown.

### 3. Evidence / reconciliation state

Examples:

- required evidence incomplete;
- evidence qualified;
- conflicting evidence;
- reconciliation pending;
- reconciled as applied/not-applied/compensated/still-unknown.

A higher-level business domain may use these dimensions when determining whether a commercial obligation is satisfied, but Finance MUST NOT make that commercial determination merely from its own state.

## Settlement finality profiles

There is no universal `is_final` boolean independent of a profile.

A Finance finality profile SHOULD identify at minimum:

- profile ID and revision/digest;
- settlement rail/network/domain;
- exact subject identity rules;
- amount/asset matching rules;
- required observations/evidence;
- freshness requirements;
- confirmation/witness/notary requirements where relevant;
- conflict rules;
- reversal/reorg/chargeback semantics;
- timeout/unknown semantics;
- qualification and invalidation behavior;
- replay/domain-separation requirements.

Potential future profiles may cover internal SAP conservation, TEND mutual credit, external banking/payment providers, EVM networks, Unyt, or witness/notary arrangements. Naming a potential profile does not claim it is implemented or qualified.

## Unknown is a first-class financial state

When a provider, network, or counterparty outcome cannot be established, Finance MUST preserve uncertainty.

```text
unknown
!= failure
!= success
```

A retry that could duplicate an economic effect requires reconciliation or an idempotency guarantee sufficient for the selected action contract and settlement profile.

Finance SHOULD preserve the same stable idempotency identity across retries when the underlying rail supports it.

## Reversal and compensation lineage

A later reversal, reorg, chargeback, cancellation, compensation, or corrective ledger entry MUST NOT erase the original observation, qualification, authorization, attempt, or result lineage.

Finance SHOULD represent the new effect as a new lineage event referencing the predecessor it modifies or compensates.

```text
compensation
!= erasure
```

## Business-to-Finance boundary

A future executable Business-to-Finance request SHOULD carry only the information Finance needs to validate and apply its own state, while binding to the upstream business/authority lineage by reference or digest.

A request may need to bind:

- Action Contract identity/digest;
- Decision Capsule or Prepared Action reference;
- authorization binding;
- authority epoch and fencing token;
- financial subject/account/treasury scope;
- exact amount and asset;
- financial effect class;
- aggregate-policy key(s);
- idempotency identity;
- preparation and expiry time;
- required finality profile.

Finance MUST re-check Finance-owned current state rather than trusting a stale projection supplied by the caller.

## Finance-to-Business boundary

Finance returns typed evidence/results rather than a generic success boolean.

A result may include references to:

- reservation state;
- finance execution attempt/effect;
- ledger mutation;
- settlement observation;
- qualified settlement;
- finance reconciliation state;
- unknown state;
- compensating/reversal lineage.

The Business Fabric then uses those finance-domain results as inputs to its own economic reconciliation and outcome lineage.

```text
Finance success
!= Business success
```

## Reservation semantics

A Finance-owned reservation SHOULD satisfy the following properties where applicable:

- issued by Finance or a qualified Finance authority;
- bound to one subject/scope;
- bound to exact amount and asset for quantity-limited actions;
- bound to an action/intent digest;
- bound to relevant authority/fencing context;
- finite lifetime or explicit renewal semantics;
- non-transferable unless the contract explicitly permits transfer;
- replay-safe;
- releasable/consumable under explicit transitions;
- invalidated by relevant revocation, expiry, state conflict, or fencing advance.

A Business Fabric reservation reference is evidence that a reservation was named, not evidence that the reservation is still valid. Execution-time Finance validation remains required.

## Aggregate and anti-structuring integration

The Business Fabric owns the Action Contract and the policy requirement that aggregate constraints be evaluated. Finance owns the authoritative numeric projection for finance-state dimensions it controls.

Finance SHOULD be able to expose or validate bounded aggregate quantities such as:

- total spend;
- counterparty exposure;
- credit utilization;
- asset/issuer/project concentration;
- transaction velocity;
- treasury liquidity floor;
- delegated shared budget consumption;
- project capital envelope utilization.

Splitting one intended economic effect into smaller requests MUST NOT permit bypass of an aggregate Finance constraint required by the Action Contract.

## Capital constitution compatibility

Finance may model financing positions and economic rights, but must preserve the capital constitution's anti-capture separation.

At minimum:

```text
capital contribution
!= protocol governance right unless explicitly granted
!= evidence truth
!= technical qualification
!= federation sovereignty
!= user-data ownership
```

A financing position may include explicit economic participation, repayment priority, capped return, redemption conditions, information rights, and other finance-specific rights.

Protocol-governance or institutional authority, if any, must be separately explicit and traverses the ordinary authority architecture.

Finance code MUST NOT infer governance power merely from invested amount, token balance, repayment priority, or economic ownership.

## Symthaea boundary

The safe default integration mode is:

```text
Finance state/evidence
        |
        v
read-only projection
        |
        v
Symthaea
  observe / estimate / forecast / simulate / explain / recommend
        |
        v
Proposal
        |
        v
Business Fabric authority + coordination
        |
        v
Finance reservation/execution
```

Symthaea may propose actions, risk limits, liquidity moves, financing structures, or other financial decisions. A proposal cannot issue a Finance reservation or mutate Finance state.

Any future bounded autonomous financial capability must use an explicit Action Contract, current authority, Finance-owned reservation/execution checks, and exact evidence lineage. Intelligence must not create its own authority envelope.

## Interoperability posture

### ValueFlows / hREA

Mycelix SHOULD prefer interoperable economic vocabulary over creating redundant generic commercial ontologies.

Generic Intent, Commitment, Claim, Agreement, and EconomicEvent semantics belong in the commerce/economic coordination domain where appropriate, not in Finance merely because Finance consumes them.

Future ValueFlows/hREA compatibility work SHOULD classify each mapping as:

- lossless;
- lossy;
- unsupported;
- Mycelix extension.

No adapter may silently promote an external semantic object into Finance authority.

### Unyt

Unyt or another Holochain-native accounting/mutual-credit system may become a settlement/accounting adapter if separately evaluated and qualified.

The integration question is not whether Finance can duplicate an external system. It is whether a defined Finance effect can delegate accounting/settlement mechanics while preserving Mycelix authority, evidence, reconciliation, privacy, and finality semantics.

No Unyt behavior is qualified by this document.

## Exact quantity rule

New authoritative Finance APIs SHOULD NOT introduce binary floating-point representations for canonical monetary amounts, debt principals, reservation amounts, exposure, settlement amounts, or exact contractual rates.

Existing Finance code may contain historical floating-point representations. Migration of those fields requires its own compatibility and qualification work and MUST NOT be silently bundled into this architecture tranche.

Analytical and model-facing values may remain floating point when they are explicitly non-authoritative.

## Qualification and evidence discipline

Finance follows the repository's evidence discipline.

At minimum:

```text
implementation
!= qualification
```

```text
ancestor PASS
!= child PASS
```

```text
queued
!= PASS
```

```text
draft
!= PASS
```

```text
provider acknowledgement
!= qualified settlement
```

```text
commercial adoption
!= security assurance
```

Each executable FIN-ECO tranche must state the exact subject, dependency lineage, test/qualification surface, and nonclaims appropriate to that tranche.

## Proposed tranche sequence

The intended first implementation sequence is:

1. FIN-ECO-000 — this Finance specialization constitution;
2. FIN-ECO-001 — exact financial quantity/rate primitives and migration census;
3. FIN-ECO-002 — settlement observation/finality/qualified-settlement model;
4. FIN-ECO-003 — typed Business Fabric to Finance reservation/execution binding;
5. later credit, capital, interoperability, and Symthaea-finance tranches only after the lower layers are stable.

A Holochain platform-version migration SHOULD remain a separate compatibility lineage rather than being bundled into these semantic tranches.

## Constitutional invariants

The following identifiers are normative names for later qualification tests.

- FIN-ECO-I01 Domain Ownership: Finance references but does not seize authority from Commerce, Governance, Identity, Property, Justice, or other source domains.
- FIN-ECO-I02 Exact Quantity Boundary: new authoritative financial amounts/rates do not rely on binary floating-point semantics.
- FIN-ECO-I03 Authorization Separation: business/institutional authorization does not imply available financial capacity.
- FIN-ECO-I04 Reservation Separation: reservation does not imply execution.
- FIN-ECO-I05 Observation Non-Finality: settlement observation cannot promote itself to qualified finality.
- FIN-ECO-I06 DHT Non-Finality: DHT presence/countersigning is not generic settlement finality.
- FIN-ECO-I07 Profile Binding: qualified settlement binds the exact finality profile revision and evidence frontier.
- FIN-ECO-I08 Unknown Preservation: unresolved financial outcomes remain unknown until reconciled.
- FIN-ECO-I09 Compensation Is Not Erasure: reversals/compensation preserve predecessor lineage.
- FIN-ECO-I10 Business/Finance Non-Collapse: Finance success does not self-assert commercial obligation satisfaction.
- FIN-ECO-I11 Authority Conservation: Finance results, balances, investments, or model outputs cannot mint unrelated institutional authority.
- FIN-ECO-I12 Reservation Currentness: named reservation references must be revalidated against Finance-owned current state before use.
- FIN-ECO-I13 Replay Safety: repeated execution identity causes at most the permitted financial effect under the selected contract/rail semantics.
- FIN-ECO-I14 Aggregate Preservation: action splitting cannot bypass required finance-owned aggregate constraints.
- FIN-ECO-I15 Evidence Binding: amount, asset, subject, rail/network, and evidence domain are bound against replay/substitution.
- FIN-ECO-I16 Capital Non-Capture: economic participation cannot imply protocol/evidence/federation authority absent an explicit separate grant.
- FIN-ECO-I17 AI Non-Authority: Symthaea analysis/proposal cannot directly become Finance reservation or execution authority.
- FIN-ECO-I18 Qualification Non-Inheritance: semantic/runtime changes do not inherit prior FIN-SAFE or FIN-ECO qualification by ancestry alone.

## Nonclaims

This document does not establish:

- runtime Finance behavior;
- Holochain, bank, payment-provider, EVM, Unyt, or other rail finality;
- legal settlement or discharge;
- tax, accounting, banking, securities, consumer-credit, or other regulatory compliance;
- creditworthiness or underwriting quality;
- hREA/ValueFlows conformance;
- Unyt interoperability;
- autonomous financial authority;
- Symthaea execution authority;
- capital-structure legal enforceability;
- any technical PASS beyond exact existing qualification evidence.

It freezes the intended Finance ownership and integration boundary so subsequent implementation can be reviewed against one explicit target.