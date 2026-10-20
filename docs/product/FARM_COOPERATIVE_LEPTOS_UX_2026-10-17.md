# Mycelix Farm Commons — Leptos Product Design and Acceptance Plan

**Status:** UX and integration design proposal; not a production-ready Commons UI  
**Date:** 2026-10-17  
**Frontend direction:** Rust + Leptos, composing with existing Mycelix Commons zomes  
**Related canonical work:** [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001)

## Product principle

Mycelix should feel like the farmer's cooperative workbench, not a blockchain interface and not a second Sol Atlas.

Sol Atlas helps answer “what is happening at this place, and what evidence describes it?” Mycelix helps answer “who is working together, what have we agreed to share or provide, what resource is reserved, what was actually handed over, and how can we resolve a correction or dispute?”

A solo producer should gain value without joining a co-op, issuing a token, publishing exact location, or allowing model training. Cooperative features appear when the producer explicitly chooses to collaborate.

The public Mycelix README describes Commons as pre-alpha with no real UI deployed. This is a design contract and usability plan, not a claim of a live product.

## Paradigm shifts

| Common farm/co-op software | Proposed Mycelix shift |
|---|---|
| Listings and order forms first | Producer workflow, available resources, commitments and fulfillment first |
| Every product is measured in kilograms | Quantity and sale unit explicit: count, dozen, litre, kg, tray, crate, hour or task |
| Forecast supply shown as inventory | Planned, forecast, observed, reserved, offered, handed-off and rejected quantities are distinct states |
| Upload everything to one platform | Private by default; share scoped information for an explicit purpose |
| Reputation/token score stands in for trust | Show the actual agreement, delivery receipt, inspection evidence, dispute state and correction history |
| Everyone must understand governance | Simple direct action path, transparent rules only where they affect the action |
| One app replaces everything | Interoperate with Sol Atlas, existing records and open exports; no forced migration before value |

## Audiences and views

### Producer / farm manager
A private Today view for tasks and commitments, observations, production batches, costs, available inventory and decisions due. No public map pin or yield claim without explicit choice.

### Worker / field operator
A fast mobile workflow for assigned jobs, offline observations, equipment check-out, completion evidence, time spent and a clear “needs review” route. The worker sees only records necessary for assigned work.

### Cooperative coordinator
A role-scoped overview of member-approved supply commitments, shared equipment reservations, distribution windows, shortages, substitutions and disputes. Surface exceptions instead of exposing every farm's private operations.

### Technician / fabricator / equipment steward
A consented equipment catalogue with design revision, availability, maintenance state, compatible attachments, booking, inspection/repair receipts and known limitations. A machine cannot be labelled “safe” only because it has a credential or passed simulation.

### Household / buyer / recipient
A simple order/reservation view for products actually offered by producers, with units, price basis, pickup/delivery window, substitution/shortage policy and fulfillment state. Expose only the location precision needed for fulfillment.

## Proposed Leptos information architecture

1. **Today** — assigned work, decisions due, commitments, sync state and urgent exceptions.
2. **My farm** — fields/paddocks/mobs, crop plans, observations, production batches, inventory and assets; private by default.
3. **Work & equipment** — jobs, service requests, shared tools, reservations, operator roles and maintenance history.
4. **Food & fulfillment** — producer offers, typed quantities, lot/batch records, cooperative demand, reservations, pickup/delivery, partial fulfillment, spoilage and recall.
5. **Open designs** — candidate design briefs, source/license status, CAD/drawing/BOM readiness, inspection and test evidence, manufacturing steps, repairability and independent build replication. Candidate cards are intake-only until evidence exists.
6. **Co-op & sharing** — members, scoped commitments, shared resources, mutual-aid requests, governance as needed, dispute resolution, evidence labels and controlled sharing.

Navigation is task-led with progressive detail rather than six dense dashboards. Default to Today. Include map/context when location materially changes the action, not as the universal root screen.

## Key workflows

### A. Record an observation offline
1. Choose a field/paddock/mob or identifier.
2. Record a short observation, unit, time, author, optional media and source classification.
3. Save locally immediately and show a stable receipt ID and “pending sync” when offline.
4. On reconnect, deduplicate retries and surface conflicts; never drop an edit silently.
5. Allow corrections with reason/history rather than destroying prior records.

### B. Create a producer offer without fabricating inventory
1. Select product profile and lot/batch.
2. Enter amount and explicit unit; select price basis/currency when selling.
3. Distinguish available-now from planned supply with expected date and uncertainty.
4. Set recipients, pickup/delivery scope, substitution and shortage policy.
5. Confirm offer as a distinct object. Offer != stock; order != fulfillment.

### C. Coordinate shared equipment
1. Search by service/task, tool/attachment compatibility, location at producer-controlled precision, capacity, date, availability, repair state and training requirements.
2. Compare borrowing, sharing, rental, contractor/manual work and purchase without assuming ownership is best.
3. Reserve with clear terms, pickup/return and responsibility.
4. Record checkout/return condition, operating hours and issue reports.
5. Record maintenance and inspection as attributed evidence with expiry and reviewer; never overstate what an inspection proves.

### D. Fulfill a cooperative food commitment
1. Distinguish planned production from available lot.
2. Reserve unit-compatible inventory without overbooking.
3. Report shortage or substitution before the delivery/pickup window.
4. Capture actual accepted/rejected quantity and handoff time.
5. Append receipts and correction/recall lineage; avoid exposing household identity in public traceability.

### E. Share selected information
1. Preview exact fields, time range, location precision and artifacts.
2. Select recipient, purpose and expiry/review time where supported.
3. Record consent and grant minimum access.
4. Show future revocation separately from deletion; revocation cannot erase a copy already received.
5. Offer export and a plain-language share/access log within the guarantees supported by the actual system.

## Integration boundaries

- **Mycelix Commons owns** producer, product-profile, lot/offer/order, fulfillment, cooperative membership, permissions and dispute records using canonical zomes and [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001). Do not duplicate domain models in the frontend.
- **Sol Atlas** supplies optional spatial context and source-bound site/environment data. Preserve scale and show missing/stale status.
- **Symthaea** may summarize source-grounded exceptions, identify missing prerequisites, or suggest observations. It must not silently create inventory, certify food safety, guarantee yield, diagnose animals or authorize a physical action.
- **Symtropy** supplies bounded scenario/resource/economic comparisons only where assumptions and inputs are adequate. Output remains a scenario unless validated.
- **Identity/authorization** features may be used only where the deployed hApp supports the exact capability. Do not invent access-revocation guarantees unsupported by transport or application semantics.
- **Interoperability:** evaluate [ADAPT Standard 2.0](https://adaptstandard.org/) for agricultural production/work-operation data and open exports. It is a data-exchange schema, not a replacement for coop governance, privacy or fulfillment semantics.

The UI degrades gracefully if integrations are missing. Routine offline capture, task completion and export should not require a live Symthaea service.

## Trust and rights model

- Farm identity, full boundary geometry, yield history, animal data, worker records, costs and household orders are private by default.
- Sharing a summary does not imply permission to share raw records. Preview and explicitly scope.
- “Observed”, “estimated”, “planned”, “synthetic”, “stale”, “conflicting” and “unverified” are distinct states.
- A credential reports an issuer's claim and scope; it does not automatically prove food safety, machine safety, competence or truth.
- Food categories can have jurisdiction-specific handling and legal requirements. Provenance is not a food-safety certificate. South African pilots need qualified local review of applicable food, dairy, premises and transport rules before sales flows are enabled.
- Do not reuse community knowledge or farm data for training/analytics without clear purpose, explicit terms, a withdrawal process and producer-visible benefit.
- Keep tokens/reputation mechanics out of the critical path of food, water, work or equipment access.

## Proposed benchmark gates

These are pilot targets, not claims about existing Mycelix code.

### Usability
- At least 8/10 first-time participants complete critical tasks unassisted: record/correct an observation, create an offer with a non-kg unit, reserve equipment, report a shortage and export records.
- Median simple observation capture <= 30 seconds after onboarding, with immediate local save.
- Median time to find order/commitment status <= 60 seconds for the responsible role.
- At least 70% of participating operators voluntarily use the selected workflow weekly by week 4; publish denominator and reasons for non-use.
- Test owners, workers, coordinators and people with low connectivity/digital confidence.

### Reliability and invariants
- Zero silent observation loss in offline/reconnect/retry tests; deterministic duplicate-event and conflict handling.
- No negative, non-finite or unit-incompatible quantities accepted; currency and price basis explicit.
- 100% exact agreement with independent expected outcomes for frozen synthetic cases: planned != available, offer != stock, order != fulfillment, capacity, unit mismatch, partial delivery and recall/correction lineage.
- Zero unauthorized reads/writes in the frozen permission adversarial suite; unknown or revoked scope fails closed.
- Complete export/import preserves IDs, units, timestamps, source, correction history, lot/order/fulfillment relationships and permissions metadata.

### Open-design qualification
- 100% of required release artifacts identify exact revision and license; no unknown source/license is treated as open or reusable.
- Complete BOM coverage for every required item before a package may claim complete-cost coverage; unknown quantity/cost remains explicitly blocked.
- No design is labeled build-ready without editable source, dimensioned drawings, build/inspection instructions, relevant tests, maintenance/repair documentation and stated operating limits.
- At least one selected design is independently reproduced by a second fabricator before claiming reproducibility; record rework, missing instructions, actual build time and deviations.
- Zero false accepts of hard constraints in the frozen qualification corpus; simulation or catalogue presence is never treated as physical qualification.

### Cooperative outcomes
- Choose one cooperative metric before the pilot: coordination time per fulfilled order, avoidable spoilage, fulfillment rate, equipment idle time/cost per use, producer share of value, or unpaid administrative effort.
- Establish baseline and report direct costs, operator time, failed fulfillment, substitutions and disputes. An order is not impact until physical fulfillment evidence exists.
- Include cost of devices, connectivity, training and coordination, especially for small producers.

### Security, privacy and interoperability
- Export and future-access revocation discoverable without contacting maintainers.
- Every share shows recipient, purpose, scope and time; explain immutable copies.
- 100% externally shown quantities carry units and state; no silent unit coercion.
- Import/export a documented fixture without data loss or changed field meanings.
- Zero unresolved critical privacy, access-control or silent-loss findings before real cooperative pilot.

## Leptos implementation principles

- Compose existing design tokens/components; avoid a second one-off design system.
- Progressive disclosure: simple task path first, evidence/detail view when relevant.
- Touch targets >= 44 CSS px; meaningful keyboard order, visible focus, screen-reader labels, strong outdoor contrast and reduced motion.
- Explicit loading, empty, offline, error, retry, denied, stale and conflict states.
- Local save with queue/receipt ID; a button must not imply remote success before acknowledgement.
- Keep forms/display types separate from canonical Holochain entry types; adapt through explicit versioned boundaries.
- Use mock/synthetic fixtures for component tests, marked conspicuously and never as live records.
- Keep core workflows testable without cloud/model dependencies.

## Research and references

- [Mycelix Commons README](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-commons/README.md) — existing food-production, food-distribution, food-preservation, food-knowledge and water zomes.
- [MYC-FOOD-001 #5001](https://github.com/Luminous-Dynamics/mycelix/issues/5001) — typed units, lot/offer/order/fulfillment boundaries and synthetic acceptance corpus.
- [Sol Atlas AGRO-CLIMATE-001 #107](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/107) — local spatial/seasonal profile and evidence gates.
- [ADAPT Standard 2.0](https://adaptstandard.org/) — JSON Schema, defined units, UTC timestamps and geospatial exchange.
- [FAO precision-agriculture adoption study](https://www.fao.org/agrifood-economics/publications/detail/en/c/1618844/) — 22 case studies; cost, skills, connectivity/infrastructure and evidence of impact matter.
- [FAO digital agriculture in lower-income countries](https://www.fao.org/agrifood-economics/publications/detail/en/c/1618845/) — ten cases spanning sub-Saharan Africa, Latin America/Caribbean and Asia.

## Acceptance state

- [x] Product boundary and audiences defined.
- [x] Core producer, worker, cooperative, equipment and fulfillment workflows specified.
- [x] Canonical Mycelix and cross-project ownership included.
- [x] Proposed benchmark and privacy gates documented.
- [x] Add the concept route to the existing public Commons Leptos app at /farm.
- [ ] Verify route rendering in a built/browser deployment.
- [x] Add a path-scoped standalone Leptos formatting/WASM-check workflow for draft PRs; its first exact-head run is not yet verified.
- [ ] Check in the app workspace's resolved Cargo.lock so builds use a committed dependency graph instead of a run-generated lock.
- [ ] Verify dependencies/permissions against the exact Holochain client.
- [ ] Add fixture-backed component tests for offline/conflict and unit invariants.
- [ ] Run accessibility and low-end/mobile review.
- [ ] Conduct producer/worker/coordinator usability tests.
- [ ] Integrate real hApp data only after contracts and authorization are verified.
