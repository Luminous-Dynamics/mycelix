# Munch Adapter Discovery Matrix — October 2026

Status: **research and integration plan; not a claim of vendor integration or tested API access**

## 1. Decision summary

Do not build a second Munch POS and do not assume a public Munch API is available to arbitrary third parties. The next commercial/technical step is to establish a permissioned, read-only data path and document what evidence that path can actually provide.

The near-term product candidate is the **Luminous Restaurant Intelligence Layer**: vendor-neutral source ingestion, evidence qualification, forecast/replenishment evaluation, waste-measurement quality, and owner-facing explanations. It should sit beside an existing POS before any effort to replace checkout, payments, kitchen operations, or franchise controls.

A successful Munch adapter would be one connector among several—not a provider enum embedded in Mycelix Business Core. Keep connector identity, schema, query semantics, rights, and evidence lineage outside the brand-agnostic business profile.

## 2. Publicly documented baseline

Research checked on 13 October 2026. These are vendor-published capability claims, not independent product tests.

| Capability | Public evidence | What that establishes | What remains unknown |
|---|---|---|---|
| POS and order lifecycle | [Munch official site](https://www.munch.cloud/) | Munch markets POS, order management, mobile/table-side and digital ordering, payments, kitchen displays and cloud back office | Exportable raw event history, stable source-event IDs, correction semantics and third-party API rights |
| Inventory and recipes | [Official site](https://www.munch.cloud/) | Inventory control, recipes/menus, multiple units of measure, mobile stock counts and stock movement reporting are advertised | Full item-level movement histories and immutable effective-time semantics through an authorised external interface |
| AI and reporting | [Official site](https://www.munch.cloud/) | Munch markets machine-learning demand forecasting and advanced reporting | Actual error metrics, test design, calibration, uncertainty coverage, or independent causal waste/savings results |
| Stock movement reports | [10 May 2026 API release note](https://releases.munch.cloud/API-Release-Notes-10-May-2026-efe90bbba3df40f68df85d3c773bdf44?pvs=21) | Official release notes discuss stock-movement reporting, adjustment handling, and actual/theoretical stock views | Whether a third-party receives all underlying rows, query snapshots and report lineage rather than a rendered report |
| Goods receipt | [Munch release-note hub](https://releases.munch.cloud/) | Munch documents Goods Receipt Voucher workflows and reporting/downloads | Machine-readable supplier/line-item history, expected-vs-received discrepancies, amendments and source-issued coverage proof |
| Data import | [Official use cases / FAQ](https://www.munch.cloud/use-cases) | Munch says menus, inventory and stock levels can be imported by CSV | This is an import statement; it is **not evidence** of a corresponding complete export API |
| Software evolution | [Official release-note hub](https://releases.munch.cloud/) | Munch continues to publish dated API/product changes. Examples in 2026 include report fixes, units of measure, supplier portal pagination, refund timestamps, and transaction/report semantics | Stability guarantees, deprecation policy, versioned schemas and availability of a developer sandbox |
| Franchise scale/strategic context | [Famous Brands — Consumer technology investments, IAR 2025](https://famousbrands.co.za/iar2025/strategy/consumer-technology-investments.php) | Famous Brands reported 318 restaurants had implemented Munch POS at reporting year-end, with lower franchisee fees, cloud deployment, standardisation and potential supply-chain/e-commerce integration cited as benefits | Current October 2026 active-install base and operational performance are not established by that 2025 reporting snapshot |
| Pricing reference | [Munch official site](https://www.munch.cloud/) | Published pages list Starter R1,500, Standard R3,250 and Premium R5,500 per location/month | Current negotiated terms, per-module fees, integration fees, taxes, hardware/payment costs and enterprise contracts |

**Competitive implication:** forecasting, stock visibility, inventory/recipes, reports, digital ordering and kitchen displays are not credible standalone differentiation claims for Luminous. The plausible differentiator is cross-vendor, evidence-first qualification and measurement: proving what the source did and did not provide, keeping missing data visible, separately scoring prediction quality and physical measurement quality, and evaluating operational outcomes without overstated causal claims.

## 3. Key discovery gap: data rights and source semantics

The official public pages reviewed do not establish whether an independent developer can obtain the full source records needed for the proposed qualification flow. Do not infer API availability from an API release-notes page, nor infer export completeness from CSV import support.

The integration must first answer:

- Is there a documented, supported third-party API or partner data-export path available for a customer-authorised read-only integration?
- Does it cover all relevant business/site scopes, registers/channels, order items/modifiers, discounts, tax, payments, refunds, voids, cancellations, stock movements, stock counts, recipes/UOMs, supplier purchase orders and goods receipts?
- Does each event have a stable immutable source identity and a way to distinguish create, update, reversal and correction?
- Can queries be bound to a stable snapshot or export session? Do cursor chains guarantee exhaustion for the declared query?
- Are event-time and recorded-time semantics specified, including late-arriving and backdated records?
- Is the full report/export row set available, or only aggregate/PDF/Excel output?
- Can Munch provide a source-issued row/count/coverage statement for a declared query and interval, and can that statement be independently authenticated?
- What are the permitted retention, processing, onward sharing, data residency, rate limit, service support, sandbox and contract requirements?

Until answered in writing and proven using a small real export, record the relevant connector capabilities as **unknown** rather than “supported”.

## 4. Adapter qualification ladder

Qualification is per connector + schema + mapping + query semantics + business scope + time window. Passing one tier does not imply another tier.

| Tier | Minimum evidence | Allowed conclusion | Still prohibited |
|---|---|---|---|
| **D0 — Unqualified / exploratory** | Sample CSV, screenshot, or undocumented report | Format exploration only | Forecast qualification; completeness claims; operational decisions |
| **D1 — Reproducible import** | Original bytes, file digest, schema/mapping digest, diagnostic manifest, exact row denominator, rejection records, scope and timestamps | Import is reproducible for those supplied bytes | Claiming the export contains every source event |
| **D2 — Source query bound** | Documented query/filter semantics, source identity, immutable or well-defined snapshot/export session, exact interval, complete cursor/terminal evidence or documented equivalent, source-event identities and corrections | Structural coverage evidence for the **declared query** | Authenticity or legal/accounting authority; proof every real-world event was recorded |
| **D3 — Authenticated coverage + independent reconciliation** | D2 plus an authenticated provider coverage statement, revalidated issuer/verifier/credential/revocation context, separately issued control totals, full-interval coverage and an independently approved metric contract | Stronger source-coverage and control-reconciliation evidence for the registered scope/window | Universal completeness, physical truth beyond the controls, causal savings, or business-write authority |

The current Mycelix export-completeness crate implements a structural slice of D2 and deliberately calls its result StructurallyConsistentExport. It does not implement Munch authentication or make D3 claims. The separate hospitality field-pilot and waste-measurement contracts cover later evaluation boundaries.

Manual CSVs can be useful at D1 and possibly in a restricted D1 shadow experiment. Missing source-level completeness evidence must remain an explicit limitation. Do not try to reverse-engineer completeness by guessing a row count, treating file end as a provider terminal cursor, or declaring a digest to be a signature.

## 5. First data contract

Create a provider-neutral adapter that maps the vendor's exact semantics into stable domain events without hiding the original identifiers.

### Sales / order event

Minimum candidate fields, if the permitted source supports them:

- source system + site/business scope + stable source event ID;
- order ID, line ID, version/revision and parent/reversal reference when provided;
- event type/status (created, accepted, paid, fulfilled, voided, refunded, corrected);
- event time and recorded/updated time, preserving their distinct meanings;
- item/product/variant/modifier identities, quantity, unit of measure, recipe mapping version;
- gross, discount, tax, refund, net and currency fields as separate fixed-point values;
- originating channel/register and documented inclusion/exclusion semantics;
- source file/page/event digest and adapter/schema/mapping digests.

A void or refund should not be silently overwritten as if the earlier sale never existed. Represent source corrections according to the documented contract; determine the qualifying target metric in a preregistered projection.

### Inventory and purchasing events

When available, collect source event identity, site, product/variant/UOM, quantity delta, movement type, effective/recorded time, reason/status, recipe/BOM mapping version, supplier and purchase-order/receipt reference, source payload digest, and exact stock-count/receipt window. Preserve adjustments and backdated receipts as events instead of flattening them away.

### Privacy minimisation

Start with aggregated item/time/site information. Do not ingest customer names, phone numbers, payment credentials, biometrics, or employee-level attendance when those fields are not necessary for the preregistered question. The initial intelligence layer does not need customer identity to compare item-demand forecasts or ingredient consumption.

## 6. Questions to send through Munch's official channel

Use [Munch's contact and partnership page](https://www.munch.cloud/contact). Request a technical discovery session and sandbox/data-export terms; do not begin by asking for production credentials.

1. Which supported third-party API, partner endpoint, scheduled export or customer-authorised report path can provide read-only access to sales line-items, refunds/voids, recipes/UOMs, stock movements/counts, purchase orders and goods receipts?
2. For each object, what are the versioned schema, unique IDs, timestamp semantics, pagination/snapshot guarantees, rate limits, retention period and correction/backfill policy?
3. Are deleted, voided, refunded, partially refunded, reopened, corrected and backdated records exposed as separate events, revisions or snapshots?
4. How are transaction counts and report totals calculated, including discounts, tax, delivery, tips, service charges, fully discounted orders and non-stock items?
5. Is there a source-signed/otherwise independently verifiable completeness or control statement for the full result set of a declared query/window? If not, what independent control exports can be supplied?
6. Are a sandbox, synthetic example dataset and API/schema version/deprecation policy available?
7. What customer consent, data-processing agreement, purposes, retention, security, geographic hosting, support/SLA and commercial approval are required for an independent analytics integration?

Ask for a small anonymised sample bundle with expected counts and a reconciliation total before committing to a large implementation.

## 7. Suggested first connector experiment

1. Obtain explicit permission, documented access and a representative anonymised dataset from a single site.
2. Freeze the question, scope, event semantics, source/schema/mapping digests and forecast target schedule before evaluation begins.
3. Preserve the original export bytes; run the D1 import diagnostic and reject malformed, duplicate, stale, contradictory and future-dated rows visibly.
4. Attempt to qualify the source extraction as D2; if pagination/snapshot/coverage evidence is absent, retain that unresolved limitation instead of inventing a pass.
5. Build transaction-derived item-demand actuals only from the frozen event-status projection. Compare Symthaea to a seasonal-naive same-weekday/daypart baseline on identical targets and slices.
6. Run waste measurement as its own lane: physical weighing/count/visual/model-imputed/missing remain separate. Never use model-imputed waste as direct physical ground truth.
7. Publish a reproducible, source-backed report of accuracy, coverage, missingness, stockouts, and costs. Only a subsequent controlled field comparison can support a causal/net-savings claim.

## 8. Go / no-go gate

Proceed with a Munch production connector only when the owner/operator authorises the data path and answers enough of the schema/semantic questions to support reproducible extraction.

If only manual CSV exports are permitted, proceed with a low-cost D1-limited shadow trial where useful, but keep the completeness limitation and do not position it as fully verified. If the terms do not permit the intended processing, do not ingest the data.

If the partner path is delayed, build the first adapter against a public, synthetic or explicitly licensed reference dataset and demonstrate the generic contract independently. Do not couple the generic Business profile or core intelligence to a single vendor while access remains unknown.

## 9. Non-claims

This research artifact does not assert that:

- Munch lacks any API or feature that has not been documented in the public sources reviewed;
- a third-party API or source-issued completeness attestation is available;
- the proposed Mycelix crates have passed CI or qualified on live Munch data;
- Symthaea outperforms Munch's forecasting;
- Luminous has reduced waste, improved margin, or produced causal savings; or
- a passing structural evidence digest proves authentic source data or physical reality.

The goal is a disciplined integration decision—not a guessed connector.
