# Maritime Stay Protocol v0.1 — research specification

**Status:** research draft; not a production protocol, booking service, legal instrument, or safety authority.  
**Parent:** [MARITIME-STAY-COMMONS-001 #5916](https://github.com/Luminous-Dynamics/mycelix/issues/5916)  
**Implementation slice:** [MARITIME-STAY-PROTOCOL-001 #5918](https://github.com/Luminous-Dynamics/mycelix/issues/5918)

## 1. Problem and thesis

A maritime journey often composes independent providers: a marina, accommodation host, repair yard, fuel or charging provider, provisioning business, and local transport operator. Existing systems generally manage parts of that journey. This specification explores a federated coordination protocol that can compose those parts without requiring providers to surrender their authoritative inventory, customer relationship, price book, or local governance.

The central invariant is:

> A composed stay is a set of provider-authoritative commitments, not one global transaction.

A Mycelix itinerary is a verifiable view over independently issued commitments. It is not the source of truth for a provider's inventory, and it cannot grant permission to navigate, enter a port, depart, berth, cross a border, or occupy property.

## 2. Scope and non-goals

### In scope

- Berth, shore accommodation, and optional local-service offers.
- Provider-owned availability snapshots and expiring holds.
- Provider-issued commitments and effect receipts.
- Composition into a stay manifest with explicit partial-success states.
- Idempotent retry, expiry, cancellation, compensation and reconciliation semantics.
- Scoped reciprocal-access entitlements.
- Destination policy and ecological-capacity references.
- Synthetic conformance fixtures and an independent verifier.

### Explicitly out of scope

- A new navigation, weather-forecast, AIS, chart, or route-authority system.
- Provider inventory databases, marina management software, accommodation property management, or payment-card processing.
- Global identity, credential issuance, legal title, insurance, or a universal reputation score.
- Port clearance, customs/immigration, vessel safety inspection, pilotage, or statutory authorization.
- A coastal hydrodynamics solver, city feasibility engine, GIS, or engineering certification.
- A claim that the current Mycelix Commons implementation is production-ready.

## 3. Ownership and composition boundaries

| Information or decision | Canonical owner | This protocol may do |
|---|---|---|
| Provider inventory and capacity | Provider's system | Carry a time-bounded snapshot reference |
| Reservation hold and commitment | Provider | Verify provider-issued receipts and state transitions |
| Price, tax, fees and refund terms | Provider / authorized payment system | Carry disclosed amounts and immutable quote references |
| Vessel configuration and engineering history | Mobility Commons and relevant issuer | Reference narrowly scoped evidence |
| Person or organization identity | Mycelix Identity / recognized issuer | Reference selective-disclosure credentials |
| Destination rules and community commitments | Duly constituted local authority/community process | Reference adopted, versioned policy |
| Ecological measurements and limits | Qualified source / competent authority | Carry provenance, coverage, freshness and claim ceiling |
| Navigation, safety, port entry and border clearance | Competent operator / statutory authority | Display external references only |
| Physical scenario results | Symtropy / qualified external model | Carry exact run and qualification references |
| Geospatial assets and source lineage | Sol-Atlas | Project evidence without changing its authority |

Do not create a second identity scheme, provenance graph, payments ledger, inventory authority, destination governance system, or navigation authority.

## 4. Core domain objects

The JSON Schema in [schema-v0.1.json](schema-v0.1.json) is a first structural contract. It is deliberately not a full semantic validator. Cross-field, temporal, authority and state-transition rules belong in an independent Rust verifier.

### StayIntent

A request to coordinate one or more resources over explicit time intervals. A stay intent can be submitted by a person, crew, or authorized agent. It describes required and optional components, vessel compatibility requirements where relevant, accessibility requirements, destination preferences, and references to policy constraints.

A stay intent is a request, not a reservation or offer acceptance.

### ProviderOffer and AvailabilitySnapshot

A provider-owned offer describes one resource class and its terms. An availability snapshot must identify:

- provider and resource references;
- source-system revision or event reference;
- observed-at time;
- valid-until time;
- capacity and capacity unit;
- applicable constraints and terms;
- source provenance and disclosure scope.

A snapshot may be cached for discovery but cannot itself create a hold or commitment. The provider must re-check current capacity when processing a hold request.

### ReservationHold

A temporary, provider-owned capacity reservation with a unique hold reference and explicit expiry. Holds are not transferable unless the provider explicitly permits it. A hold must have an idempotency key scoped to the provider and operation.

An expired hold cannot be revived by a client. A new hold request is required.

### ProviderCommitment

A provider-issued confirmation for a resource and time interval. It references the originating hold or another provider-defined reservation workflow, a current provider revision, terms/price quote, and an effect receipt.

Only the provider can create, amend, release, or cancel its commitment.

### EffectReceipt

A durable reference to a provider-side effect. It identifies the provider, operation, idempotency scope, outcome, source revision, event time, resource, and digest or immutable source-record reference. A signature or authenticated transport can support issuer attribution, but a signature alone does not prove the underlying physical claim is true.

### StayManifest

A composition of component references and provider commitments. The manifest reports the state of each component and an aggregate state derived from provider receipts.

Aggregate state must never upgrade a component's authority. A manifest marked Confirmed means every mandatory component has a valid, current provider commitment. It does not mean the journey is safe, the vessel may depart, a port will admit it, or all services are operational.

### ReciprocityEntitlement

A scoped, revocable agreement such as reciprocal berthing, member rates, shared facilities, or home exchange. It must declare issuer, beneficiary class, eligible resources, destination or provider scope, dates, limits, exclusions, settlement terms, and revocation/effectivity rules. It is not a universal discount promise.

## 5. Time, money, capacity and identity semantics

### Time

- Wire timestamps use RFC 3339 with explicit UTC offset; canonical persisted times normalize to UTC.
- Availability and commitment periods are half-open intervals: [start, end).
- Keep event time, observation time, retrieval time, valid time, expiry time and local receipt time separate.
- Provider clock and time-source limitations must remain visible where they affect expiry.
- A stale or missing time value is Unknown, not current.

### Money

- Never use binary floating point for money.
- Represent amounts as integer minor units plus ISO 4217 currency.
- Represent tax, mandatory fees, optional services, discounts and refunds as explicit line items.
- Preserve quote revision, validity and total arithmetic; the independent verifier recomputes totals.
- Payment authorization, capture and settlement remain owned by a payment provider or authoritative finance system.

### Capacity

Capacity has a declared unit and resource identity. A berth's capacity is not interchangeable with a guest-night, an electrical-power quota, or a workshop appointment. Concurrent reservation tests must be performed at the provider's canonical capacity boundary.

Resource dimensions must remain explicit. A berth slot, accommodation unit-night, repair appointment, shore-power connection count, instantaneous feeder capacity (W/kW), cumulative shore-energy quota (Wh/kWh), potable-water volume and waste-reception mass/volume are different resource dimensions. Do not represent both instantaneous power and cumulative energy as one generic "shore power" quota. Symtropy already tracks this distinction in [MARITIME-POWER-CAPACITY-001 #3734](https://github.com/Luminous-Dynamics/symtropy/issues/3734) and joint arrival / berth / shore-energy planning in [MARITIME-JIT-001 #1858](https://github.com/Luminous-Dynamics/symtropy/issues/1858). Voluntary arrival-window changes must reuse participant authorization in Mycelix [LOG-FLEX-001 #5185](https://github.com/Luminous-Dynamics/mycelix/issues/5185) and Symtropy [MARITIME-FLEX-001 #1903](https://github.com/Luminous-Dynamics/symtropy/issues/1903); an optimizer cannot change a participant's commitment.

### Identifiers

Identifiers are stable, opaque protocol identifiers. Holochain ActionHash or EntryHash may be used as provenance references where appropriate, but must not be repurposed as universal engineering, vessel, property, or reservation identity.

## 6. State model

### Provider resource lifecycle

The proposed logical lifecycle is:

1. OfferPublished
2. AvailabilityObserved
3. HoldRequested
4. Held
5. CommitRequested
6. Committed
7. Released, Expired, Cancelled, Failed, or Disputed

The provider's underlying system remains authoritative. The protocol records receipts and projections; it does not force providers to use an identical internal state machine.

Allowed transitions, event authority, and compensation rules must be explicit. Unknown or contradictory provider state is not silently resolved in favor of confirmation.

### Aggregate stay lifecycle

The itinerary may be:

- Draft — no provider commitments requested.
- Pending — one or more provider operations outstanding.
- PartiallyConfirmed — some components are committed, but at least one mandatory component is not.
- Confirmed — every mandatory component has a current provider commitment.
- PartiallyCancelled — one or more components cancelled while others remain committed.
- Cancelled — all applicable components are cancelled or released.
- Expired — all relevant holds expired without sufficient commitments.
- Disputed — an unresolved provider or participant dispute affects the manifest.
- Unknown — required state cannot be established from current evidence.

These are derived view states, not new provider authorities.

## 7. Distributed transaction model: saga, not global ACID

Providers are independent legal and technical actors. A cross-provider berth + room + repair appointment cannot be assumed to support a single atomic database transaction.

The orchestrator should:

1. Discover provider offers and their source/validity metadata.
2. Ask each provider to re-check current availability.
3. Request expiring holds using idempotent operation identifiers.
4. Obtain explicit provider-issued hold receipts.
5. Request commitments from providers in a declared order.
6. Re-check each mandatory component's current commitment and terms.
7. Publish a Confirmed manifest only after all mandatory commitments validate.
8. On failure, preserve partial success and invoke only the cancellation/release/compensation actions authorized by each provider's terms.
9. Reconcile provider state after timeout, restart or network partition using provider receipts and source revisions.

A failed accommodation confirmation must not make a still-valid berth disappear from history. The traveler must see the partial state and the available remedies. The system must not imply that the marina automatically refunds a berth merely because another provider failed.

### Idempotency and retries

- Idempotency keys are scoped by provider, authenticated actor, operation kind and intended resource.
- A repeated request with the same key and same payload returns the original effect result.
- Reuse of a key with a different payload is a conflict.
- A timeout is an Unknown outcome until reconciled; it is not proof that the operation failed or succeeded.
- No client retry may create a second charge or duplicate hold.
- A provider must own the final idempotency decision for its own effects.

## 8. Safety, privacy, rights and ecological constraints

- A confirmed berth is not permission to enter a port or depart in current weather.
- Vessel length, beam, draft, air draft, propulsion, shore-power compatibility and access restrictions must be explicit where applicable.
- Weather and navigation systems supply advisory evidence only. Forecast source, issue time, validity, uncertainty and vessel applicability remain visible.
- Never expose precise live vessel location, personal travel plans, home addresses, household details, or sensitive provider inventory on a public DHT by default.
- Use purpose limitation, data minimization, selective disclosure, revocation, correction, retention limits, appeal and dispute procedures.
- Community housing protections, resident access, visitor limits, protected-area rules, water/waste capacity and adopted ecological limits are constraints—not preferences an optimizer may trade away for revenue.
- A token vote, member rating, Mycelix receipt or AI recommendation cannot override statutory authority, property rights, consumer protection, safety or environmental law.
- No universal reputation scalar. Credentials and fulfillment evidence must be relevant to the particular claim.
- Low-bandwidth and offline-capable providers must have an explicit reconciliation path. Offline does not mean stale inventory may be confirmed.

## 9. Standards and interoperability

Prefer existing standards and provider APIs where their scope fits. The protocol is an integration layer, not a replacement standard.

- IMO Maritime Single Window and the IMO Compendium govern relevant ship/port/public-authority data exchanges. The stay protocol must not become an alternative clearance channel.
- At FAL 50 in March 2026, the IMO approved amendments concerning Advance Passenger Information (API) and Booking and Reservation Information (BRI), with submission for adoption expected at FAL 51 in 2027 and anticipated entry into force on 1 January 2029. If adopted and applicable, those statutory reporting flows remain with competent authorities and national Maritime Single Windows. A private stay manifest is not an API/BRI payload, passenger-name record, port clearance, or substitute for statutory reporting. Do not copy complete passenger/crew or private booking details into a public DHT; any legally required disclosure must be separately scoped, minimized, authorized and delivered to the competent recipient.
- IHO S-100 product profiles provide versioned hydrographic and marine information. S-101, S-102, S-104, S-111, S-124 and S-129 have distinct semantics and intended uses.
- The IHO's newer S-127 Marine Traffic Management and S-131 Marine Harbour Infrastructure editions should be evaluated as external evidence profiles. Their data do not, by themselves, establish live berth availability or a provider commitment.
- Accommodation interoperability can be investigated through provider-approved APIs and applicable OpenTravel Alliance / property-management interfaces; do not scrape or redistribute unlicensed inventory.
- Payment handling should integrate through authorized payment service providers; do not store payment-card data in Holochain.
- Environmental programme criteria such as Blue Flag can be referenced as an external certification claim, not treated as a substitute for law or continuous ecological measurement.

## 10. Current industry reference systems

These are component-level competitors and comparison baselines, not one directly comparable market. The scale figures below are provider-reported and should be rechecked before external publication.

| Reference system | Current demonstrated capability / scale | What Maritime Commons should do differently |
|---|---|---|
| Dockwa | Advertises nearly 4,000 marinas and more than 500,000 boaters; reservations, inventory, contracts, billing, fuel, POS, dry stack, service work orders and marina operations. | Do not rip and replace marina management. Build provider-approved interoperability, cross-provider stay composition, reciprocal access and independent commitment verification. |
| Navily | App Store listing reports 1.2 million+ boaters, 35,000+ marinas/anchorages and 700+ bookable marinas. Premium includes offline use, 72-hour weather forecasts, protection scores and automatic itineraries. | Do not recreate the cruising guide. Add accommodation, local services, longer stays, portable access and provider-authoritative commitments to the route/destination workflow. |
| PredictWind | Weather routing compares multiple forecast models; its published 17 September 2026 help article describes high-resolution AROME 1.3 km for some first-two-day routing, ECMWF 9 km for days 2–10 and ensemble products for longer horizons. | Do not claim a better forecast without matched hindcast evidence. Integrate forecast provenance/uncertainty with refuge, berth, shore-energy and accommodation availability; preserve human command. |
| TransEurope Marinas | 2026 membership rules provide reciprocal visitor-berthing discounts for eligible long-term berth holders, with explicit scope and annual limits. | Make reciprocity portable across more provider types, versioned, revocable and community-governed; do not imply discounts exist without a live provider agreement. |
| Airbnb | Its 2026 Q2 SEC filing reports over 5.5 million hosts and over 2.5 billion guest arrivals. | Do not compete on accommodation listing volume at launch. Focus on vessel-compatible stays, berth + home + service composition, maritime-specific credentials and local ownership/value-sharing. |
| Blue Flag | External criteria cover marina environmental management, education, biodiversity, climate, safety, services and accessibility, with evaluation/monitoring during the award period. | Reference certifications as scoped evidence; complement them with continuous, source-qualified water/energy/waste/ecology observations. Do not invent a single green or regeneration score. |

Sources: Dockwa (https://marinas.dockwa.com/); Navily App Store listing (https://apps.apple.com/us/app/navily/id881103205) and Premium features (https://www.navily.com/navily-premium); PredictWind routing-model update (https://help.predictwind.com/en/articles/4685434-forecast-models-used-in-weather-routing); TransEurope Marinas 2026 rules (https://www.transeuropemarinas.com/how-to-join/transeurope-marinas-joining-rules-and-principles-2026/); Airbnb Q2 2026 SEC filing (https://www.sec.gov/Archives/edgar/data/1559720/000155972026000027/abnb-20260630.htm); Blue Flag criteria (https://www.blueflag.global/criteria).

### Competitive thesis

The first market wedge is not "one app with more listings." It is a **federated stay manifest** that composes independent provider-authoritative resources and preserves partial success, evidence, privacy, local rules and settlement semantics. The first competitive test is successful end-to-end completion for a matched set of real provider workflows, not downloads, listings, or a composite score.

## 11. Comparative benchmark methodology

The frozen synthetic corpus is in [benchmark-v0.1.json](benchmark-v0.1.json).

Compare identical requests, capacity, provider latency, outage timing, policy constraints and random seeds across:

1. Manual / siloed workflow: separate provider calls, emails and independent booking operations.
2. Berth-only workflow: a marina reservation system without integrated accommodation.
3. Accommodation-only workflow: an accommodation marketplace without berth coordination.
4. Centralized aggregator: one coordination operator holding a consolidated inventory projection.
5. Federated protocol: provider-owned inventory and commitments with a composed stay manifest.

Report at minimum:

- end-to-end completed-stay rate;
- false-confirmed manifest count;
- duplicate holds, commitments or charges;
- double-booking count;
- stale-snapshot commit count;
- partial-success frequency and time-to-resolution;
- median and p95 coordination latency;
- provider adapter implementation effort and conformance;
- final-price variance and fee-disclosure completeness;
- cancellation/refund correctness;
- offline reconciliation success and unresolved-conflict rate;
- accessibility match rate and unmet demand;
- local provider revenue retained and coordination fees;
- privacy leakage and unauthorized disclosure;
- community/ecological policy bypasses.

Safety and integrity metrics are hard gates, not terms in a weighted utility function. A system cannot compensate for one false-confirmed booking with a faster median booking time.

For every performance comparison, publish workload definitions, source data rights, software commit, build profile, hardware, run count, confidence intervals, evaluator version, exact fixture digest and raw result receipts. Separate synthetic results from field observations. No SOTA superiority claim is permitted without matched baselines and independent reproduction.

## 12. Proposed initial targets (not measured results)

- Zero false Confirmed manifests in the frozen fault-injection corpus.
- Zero duplicate provider effects under same-key retries and injected timeouts.
- Zero commits from stale or expired availability snapshots.
- 100% of Confirmed manifests have a valid provider commitment for every mandatory component.
- 100% of mandatory fees and taxes visible before final acceptance.
- Zero silent policy, access-control, or ecological-limit bypasses.
- At least 30% lower median coordination time than the measured manual/siloed baseline, without regressions in hard-gate metrics.
- Offline recovery must preserve explicit Pending / Conflict / Expired states; no optimistic confirmation.
- 100% of displayed external evidence carries issuer/source, revision, observed/valid time, scope and use limitations where applicable.

These are hypotheses and acceptance goals. They are not current performance claims.

## 13. Integration sequence

1. Freeze ownership map and semantic glossary.
2. Validate the wire schema and synthetic fixtures.
3. Implement provider transition verifier and idempotency checks in a standalone Rust test harness.
4. Implement saga/reconciliation tests and independent oracle.
5. Integrate existing Mycelix Commons housing, property, transport, identity, governance and finance owners only where their canonical contracts fit.
6. Add one provider adapter using a willing pilot partner and documented API permissions.
7. Conduct privacy, consumer-protection, community and jurisdictional review.
8. Only then consider a deployed pilot.

Do not add new Commons DNA zomes or change production schemas until the contract and ownership map have independent review.

## 14. Research references

- IMO Maritime Digitalization Strategy and Maritime Single Window: https://www.imo.org/en/ourwork/facilitation/pages/maritimesinglewindow-default.aspx
- IHO S-100 standards in force: https://iho.int/en/standards-in-force-0
- IHO Standards and Specifications, including S-127 and S-131: https://iho.int/standards-and-specifications
- Dockwa marina management capabilities: https://marinas.dockwa.com/
- TransEurope Marinas 2026 joining rules: https://www.transeuropemarinas.com/wp-content/uploads/2026/04/TransEurope-Marinas-Joining-Rules-and-Principles-2026.pdf
- Blue Flag marina and tourism-boat criteria: https://www.blueflag.global/criteria
- World Bank, Nature-Based Solutions for Ports: https://www.worldbank.org/en/topic/disasterriskmanagement/publication/how-ports-work-with-nature-to-build-climate-resilience
- South Africa National Coastal Management Programme 2026–2031: https://www.dffe.gov.za/legislation/gazetted_notices_ncmp2026.2031adoption

External references inform the design. They do not establish production readiness, regulatory compliance, safe navigation, actual ecological benefit or market superiority.
