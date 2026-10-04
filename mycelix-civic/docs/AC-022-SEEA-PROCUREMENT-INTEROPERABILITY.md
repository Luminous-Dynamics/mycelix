# AC-022 — SEEA / Procurement Interoperability

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-022 makes the AC-017 substrate model usable with established environmental-accounting evidence and public-procurement decisions.

The design has two strict boundaries:

1. SEEA data is evidence, not a universal Mycelix score.
2. Procurement price comparison occurs only after declared substrate policy gates are evaluated.

## SEEA interoperability

The UN System of Environmental-Economic Accounting (SEEA) is the international statistical framework for organizing environmental information alongside economic information.

SEEA Ecosystem Accounting explicitly distinguishes:

- ecosystem extent;
- ecosystem condition;
- physical ecosystem-service flows;
- monetary ecosystem-service flows;
- ecosystem monetary assets.

It also supports accounting areas such as nations, provinces, river basins, protected areas, and other defined spatial territories.

AC-022 mirrors these account families as typed observations rather than collapsing them into one metric.

### Evidence envelope

Every imported observation carries:

- stable observation ID;
- account family;
- accounting area;
- ecosystem type;
- optional economic-unit reference;
- native measurement unit;
- exact integer value;
- optional reference value;
- change semantics;
- accounting period;
- source reference;
- source schema version;
- optional content hash;
- optional source timestamp.

The adapter validates structural integrity and provenance but does not claim that the observed value is scientifically correct. That remains an epistemic question handled by the evidence/provenance layer.

### Safe projection rule

Only a SEEA ecosystem-condition observation maps directly to the AC-017 ecological substrate account.

This is intentional.

An extent observation is not automatically equivalent to condition.

A physical ecosystem-service flow is not automatically equivalent to substrate stock.

A monetary ecosystem-service flow does not become a physical ecological state merely because it has a currency value.

An ecosystem asset account is not silently converted into a policy threshold.

Those transformations require explicit local policy.


## Freshness qualification

A valid published observation is not automatically current decision evidence.

SEEA accounting periods may differ across data sources, and explicit adjustments or assumptions may be needed when integrating sources with different reference periods. AC-022 therefore separates structural validation from temporal qualification.

The reference SDK exposes a freshness policy with:

- a decision timestamp;
- a maximum permitted age measured from the observation period end.

It rejects observations that are:

- from the future;
- accompanied by a future publisher timestamp;
- older than the permitted policy window when current evidence is required.

This prevents a real and correctly sourced ecosystem account from becoming a stale authorization token.

The policy is deliberately caller-owned. Different procurement classes may require different freshness windows, and the protocol does not pretend that an annual ecosystem account must satisfy a universal freshness constant.

## Procurement interoperability

Open Contracting Data Standard (OCDS) represents procurement as a lifecycle including planning, tendering, awarding, contracting and implementation.

OCDS also emphasizes that value for money cannot be reduced to price alone; quality, efficiency, implementation, competition and other non-price attributes can matter.

AC-022 adds one further explicit policy dimension:

> substrate eligibility.

A procurement option therefore contains:

- price;
- price unit;
- policy reference;
- required substrate dimensions.

The protocol then evaluates substrate eligibility before comparing prices.

## Adversarial municipal fixture

The core test is:

- Project A is cheaper.
- Project B is more expensive.
- The declared ecological substrate boundary is breached.

Both options are therefore blocked for discretionary procurement under the declared policy.

A second fixture uses a healthy ecological account:

- both projects are eligible;
- Project A is cheaper;
- Project A is selected.

This proves that the protocol is not secretly optimizing for ecological preference. It is applying a declared safety constraint and then retaining ordinary price comparison inside the eligible set.

## Important distinction

The system is not:

price + ecological score = single ranking.

It is:

policy gate -> eligible set -> ordinary decision rule.

This distinction is important for legitimacy.

A municipality can later choose a different policy:

- hard exclusion;
- conditional approval;
- restoration bond;
- insurance requirement;
- compensation fund;
- emergency override;
- community consent;
- additional lifecycle criteria.

Those are governance decisions.

The protocol should make the consequences explicit rather than embedding one universal morality function.

## SEEA + OCDS composition

The practical composition is:

SEEA observation
-> provenance validation
-> local substrate mapping
-> boundary evaluation
-> procurement eligibility
-> price/value comparison
-> auditable decision lineage

OCDS already provides immutable releases for events in a contracting process, with new releases used for later changes. That aligns naturally with Mycelix's append-only evidence and provenance approach.

## South African adoption path

Stats SA's National Natural Capital Accounting Strategy explicitly aims to develop priority natural-capital accounts and effective statistical/institutional mechanisms to inform integrated planning and decision-making.

That makes South Africa a particularly useful reference environment for a future Mycelix adapter:

Stats SA / national natural-capital accounting
-> SEEA-shaped observations
-> Mycelix provenance and qualification
-> municipal policy boundary
-> procurement / treasury action
-> restoration obligation

Mycelix would therefore sit at the operational decision layer instead of attempting to replace official statistics.

## Security properties

Future qualification should establish:

1. Invalid or provenance-less SEEA observations cannot be projected.
2. Non-condition account types cannot silently gain ecological policy semantics.
3. Missing ecological substrate evidence prevents discretionary price selection when the policy requires it.
4. A breached hard boundary blocks all candidate options that depend on that boundary.
5. Healthy substrate does not prevent normal market price selection.
6. Processing order cannot affect substrate state.
7. Source schema changes are explicit rather than inferred.
8. Historical observations remain distinct from current policy revisions.
9. Price never repairs a breached substrate boundary merely by being lower.
10. A procurement policy can be changed only through the separate boundary-governance machinery.

## Research boundaries

AC-022 does not establish:

- that SEEA observations are true merely because they are published;
- that an ecological boundary value is universally correct;
- that the cheaper compliant project is socially optimal;
- that environmental impacts have a correct universal monetary price;
- that a procurement process is corrupt;
- that a municipality must adopt this policy.

It establishes only a machine-checkable composition in which environmental/accounting evidence can become operationally relevant without being transformed into an opaque score.

## Research grounding

UN SEEA introduction and account structure:
https://seea.un.org/en/Introduction-to-Ecosystem-Accounting

UN SEEA methodology:
https://seea.un.org/en/methodology/ecosystem-accounting

Stats SA National Natural Capital Accounting Strategy:
https://www.statssa.gov.za/?page_id=14714

Open Contracting Data Standard:
https://standard.open-contracting.org/latest/en/

OCDS release semantics:
https://standard.open-contracting.org/latest/en/schema/reference/

OCDS value-for-money guidance:
https://standard.open-contracting.org/latest/en/guidance/design/user_needs/

## Thesis

The interoperability goal is simple:

> authoritative statistics should not have to become a new centralized economic authority in order to influence economic decisions.

They can remain evidence.

Mycelix can add provenance, context, policy boundaries, obligations, and auditability around them.

That is the safer division of labor.
