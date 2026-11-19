# Housing Amenity Commons Benchmark v0.1

**Tracking issue:** [HOUSING-AMENITY-COMMONS-001](https://github.com/Luminous-Dynamics/mycelix/issues/6328)  
**Status:** Design fixture only — synthetic, not executed, not qualified  
**Version:** 0.1.0

## Why this benchmark exists

Mycelix already has housing-oriented zomes for community land trusts, units, membership, governance, finances, and maintenance. The current Leptos housing/property pages are static example views. The missing product-level seam is a shared facility represented not merely as a building or a common-property label, but as a legally and operationally accountable service.

A community hall, greenhouse, workshop, guest suite, pool, laundry, gym, or microgrid can look like a benefit in a masterplan while being unaffordable to operate, inaccessible to some residents, overbooked, under-maintained, poorly insured, or captured by a small group. This benchmark makes those failure modes explicit before a UI or production workflow is built around them.

This is a **cross-owner evaluation envelope**, not a new canonical property, finance, governance, maintenance, booking, or physical simulation system.

## Strategic thesis

The unit of value is not "an amenity exists." It is:

> an eligible resident can obtain a declared service, at a known quality and availability, at an affordable and transparent cost, with lawful authority, a funded maintenance path, and continuity over the asset lifecycle.

Compare facilities only within the same service-specific functional unit. Examples:

| Amenity | Candidate functional unit | Do not compare as |
|---|---|---|
| Workshop | usable, equipped workshop-hour | square metres of workshop |
| Community room | accessible community-room-hour | room count |
| Guest suite | accepted guest-suite night | bedroom count |
| Pool | qualified pool-lane-hour or access-hour | water volume |
| Greenhouse | kg of defined edible output meeting a declared quality class | greenhouse floor area |
| Water asset | m³ delivered at a declared quality and service boundary | tank capacity alone |
| Energy asset | kWh delivered at a declared availability/quality boundary | installed kW or battery nameplate alone |

Never combine these into a synthetic "commons score". Publish an outcome vector and show the distribution of access and burden.

## Existing Mycelix implementation boundary

The repository currently contains these housing zomes:

- **housing-clt:** land trust, ground lease, resale formula, affordability report.
- **housing-units:** building and dwelling-unit records.
- **housing-membership:** cooperative membership, applications, waitlist and rent-to-own records.
- **housing-governance:** governance, proposals and elections.
- **housing-finances:** member charges, payments, budgets and reserve funds.
- **housing-maintenance:** maintenance requests, work orders and building inspections.

The current housing finance model includes member/unit-level monthly charges and generic reserve funds; maintenance models requests and inspections at building scope. The present UI pages contain hard-coded demonstration data. This is a useful base, but it does not yet establish the complete cross-domain chain for a shared amenity: asset identity → service definition/capacity → eligible beneficiaries → access event → operating cost → maintenance/inspection → replacement liability → handback.

The first implementation should therefore **bind to owner-native records**, not duplicate them in a parallel amenity database.

## Ownership and authority split

| Fact / decision | Owner or authority |
|---|---|
| Parcel, tenure, common-property, lease and statutory property rights | Qualified land/property source and relevant jurisdiction; Sol-Atlas may project source-backed references |
| Cooperative members and membership status | Mycelix housing-membership |
| Land trust / ground lease terms | Mycelix housing-clt, subject to the underlying legal instruments |
| Governance rules, proposals, approvals and challenges | Mycelix housing-governance and the applicable signed constitution |
| Charges, budgets, payments and reserves | Mycelix housing-finances |
| Requests, work orders and inspections | Mycelix housing-maintenance |
| Physical service capacity, commissioning, safety, access and environmental performance | Responsible qualified professionals/operators and source systems; Symtropy only within a qualified model/test envelope |
| Resident requirements and design alternatives | Symthaea / resident-authored brief |
| Spatial site/neighbourhood context and public-interest projections | Sol-Atlas, with exact source, scope and rights |
| Cross-domain audit and evidence envelope | This benchmark; it does not supersede any source owner |

A Mycelix record, public key, token, membership share, vote, booking, or digest is not a deed, title search, building approval, pool operating permit, water-quality certificate, or professional inspection.

## Asset roles must remain distinct

Each asset should bind the following as separate references with explicit validity periods:

1. **Legal interest / rights source:** owner, trustee, sectional-title body corporate, lease, CPA, HOA, municipality or other legally relevant party, as applicable.
2. **Purpose guardian:** protects the declared public/collective purpose and any asset lock.
3. **Representative steward:** exercises resident/community decisions within the constitution.
4. **Operator:** maintains and delivers the service under a defined operating agreement.
5. **Decision authority:** approves the particular budget, access-policy, closure, disposal or change.
6. **Beneficiary constituency:** who is entitled to use or receive the service, including any externally agreed public beneficiaries.
7. **Independent reviewer / enforcer:** where the legal or constitutional arrangement requires one.

One organization may hold multiple roles only where the constitution and conflicts are explicit. Do not infer one role from another.

## Service and access contract

For each amenity, bind:

- stable amenity ID, design/asset revision and location reference;
- facility class and use restrictions;
- service-specific functional-unit definition;
- maximum safe/service capacity and the exact qualified source receipt;
- service availability window and outage/closure states;
- eligibility policy and the authorized policy revision;
- reservation/usage events, with privacy limits;
- accessibility requirements selected by residents and the evidence for the declared scope;
- emergency closure, manual fallback, queue and dispute process;
- upstream dependencies (energy, water, network, staff, consumables, insurance, supplier, route);
- current inspection, commissioning, condition and maintenance evidence.

A booking is not proof that capacity exists. A capacity model is not a certificate of safe occupancy. A facility shown on a map is not proof that it is open to the public.

## Whole-life finance and affordability

Separate these line items; never roll them into one opaque monthly levy:

- acquisition / land allocation / construction / fit-out;
- finance charges and repayment;
- operator and staffing cost;
- energy, water, connectivity and consumables;
- cleaning, inspection, insurance and compliance;
- preventive and corrective maintenance;
- planned component replacement and renewal;
- lifecycle reserve contributions;
- end-of-life, removal and restoration;
- administration and governance;
- hardship support, subsidy and public-service obligations.

Report at least:

- fully burdened cost per **available** service unit;
- fully burdened cost per **accepted, used** service unit;
- cost per eligible household;
- cost per household actually served;
- service utilization and wait-time distributions;
- operating cash coverage and reserve funding ratio against a dated, independently scoped replacement-liability estimate;
- arrears, hardship support and distributional effects;
- deferred maintenance and residual useful life.

The denominator differs by amenity. For a greenhouse, report accepted produce output and water/energy/labour per kg, for example; for a guest suite, report cleaned/accepted nights and cost per occupied night; for a workshop, report safe, usable, equipped hours. Do not use one normalization to hide unlike services.

A positive operating cash balance does not prove that a replacement reserve is adequate. A reserve balance cannot be counted twice as both operating cash and capital backing. No economic claim is made from the synthetic fixtures.

## Governance and anti-capture

The benchmark checks the project-specific constitution rather than prescribing a universal governance model.

Hard questions:
- Who is included in the affected constituency, and who can challenge its definition?
- Can resident beneficiaries challenge a steward or operator?
- Are ordinary operating decisions distinct from constitutional changes?
- Can one large contributor buy superior access, vote weight, or veto power contrary to the charter?
- Can an operator change access policy, charge structure, or closure conditions without authority?
- Can the asset be sold, encumbered, or converted to private use?
- What happens when an operator fails, a member leaves, a household changes, or a steward body is replaced?
- Is the constitution enforceable outside the software?

Use the existing Mycelix anti-capture and handback work (#987 and #954). Ordinary majority, token balance or member-share balance must not silently authorize disposal of an asset-locked commons.

## Water-, energy- and climate-aware amenities

Do not optimize a pool or greenhouse in isolation from its service inputs and local conditions.

For Johannesburg and any other water-constrained jurisdiction:
- bind the current local restriction profile and exact applicability period;
- distinguish potable and non-potable circuits physically and in the evidence;
- do not assume treated wastewater, greywater or rainwater is suitable for an arbitrary use;
- model seasonal supply, storage, demand, leakage, losses, reject streams, cleaning and maintenance;
- keep water quality, plumbing, health and municipal approvals with the proper authorities;
- treat pool evaporation, cover, filtration, backwash and replacement water as lifecycle flows;
- treat greenhouse yield, irrigation, nutrients, heat, ventilation and labour as an integrated productive service;
- make external restrictions and outages explicit scenario events, not footnotes.

A municipal restriction does not become permission to use a different source. The exact source, treatment, use class and applicable approvals still matter.

## Synthetic corpus and dispositions

The accompanying corpus has 20 synthetic cases. It is a design fixture and expected-outcome proposal, not an executed benchmark.

The disposition precedence is:

1. **INVALID** — the submitted records are internally contradictory or malformed (e.g. duplicate reservation identity or double-counted reserve backing).
2. **BLOCK** — a known hard constraint or authorization fails (e.g. over-capacity booking, prohibited use, asset sale without constitutional approval).
3. **UNKNOWN** — required evidence is missing, stale, disputed, revoked or out of scope.
4. **ACCEPT** — all required inputs and hard gates for the declared, narrow service claim are valid.

Precedence is for a single case with multiple seeded defects; the report must retain the full reason-code set, not only the top-level disposition. A benchmark verifier must not infer that an ACCEPT is engineering approval, legal title, or general operational permission.

## Proposed performance targets

These are **targets for future E1/E2/E3/E4 qualification**, not current results:

- 100% exact agreement with an independently hand-reviewed expected disposition across the frozen corpus.
- 100% of seeded missing/stale/wrong-scope/revoked/contradictory mandatory evidence remains UNKNOWN, BLOCK or INVALID as prescribed.
- Zero over-capacity reservations accepted against an upstream service receipt.
- Zero unauthorized sale, encumbrance, purpose change, reserve withdrawal, levy change or access-policy change accepted.
- 100% financial reconciliation across the included cost buckets; no reserve double counting.
- 100% of operational service claims bind a facility-specific functional unit, beneficiary scope, time interval, capacity receipt and current condition/inspection evidence.
- 100% of critical maintenance and handback gaps remain visible; no "completed" work order silently proves service restoration.
- Deterministic normalized reports bind exact corpus, schema, policy, verifier and owner-artifact revisions.

When a real pilot exists, compare P50 and P90 availability, wait time, service cost, repair duration and reserve adequacy—not just average utilization. Report outcomes by eligible beneficiary groups where lawful and privacy-preserving.

## Evidence ladder

| Tier | Meaning | What it does not establish |
|---|---|---|
| E0 | Concept / requirement | Implementation |
| E1 | Frozen synthetic fixture + hand-reviewed expected result | Real-world truth or operation |
| E2 | Executable independent verifier + exact-head receipt | Physical performance |
| E3 | Independent legal/governance/accounting/access audit of a real structure | Engineering certification |
| E4 | Instrumented amenity test, qualified installation and accepted commissioning | 12-month operating performance |
| E5 | At least 12 months of measured service, finance, repair and resident access | Replication beyond that scope |
| E6 | Multiple independent sites and governance forms with matched comparisons | Universal applicability |

## First implementation sequence

1. Review current zome schemas/validation and the issue's owner-seam table.
2. Freeze the contract/schema and synthetic corpus.
3. Independently review the expected dispositions.
4. Implement a small standalone Rust verifier, separate from the DHT coordinator zomes; make it consume frozen bytes and emit an evidence receipt.
5. Integrate owner-native record references and scope envelopes.
6. Extend the Leptos housing/property pages only after the service model is stable; replace mock examples with clearly identified synthetic data.
7. Instrument one low-risk, unoccupied amenity test cell before any occupied deployment.

## Claim boundary

The benchmark does not claim the housing zomes already implement a complete shared-amenity system. It does not make legal, structural, fire, electrical, plumbing, pool, water-quality, health, investment or statutory decisions. Synthetic outcomes must never be presented as measured facility performance.
