# Business Launchpad: Cross-Ecosystem Architecture RFC

**Status:** Proposal — not implemented; not a compliance or production-readiness claim  
**Date:** 2026-10-12  
**Scope:** Mycelix + Symthaea + Luminous Platform  
**Working name:** Business Launchpad  
**Decision sought:** Approve a thin, evidence-first business-starting and operations layer that reuses existing operational software instead of rebuilding a complete ERP.

## 1. Executive proposal

Build a product that helps an individual or group move from a business idea to a lawful first sale, then operate and improve the business with low administrative burden. Support food, services, retail, and later additional verticals through shared primitives and versioned jurisdiction/industry packs.

The product should be a **guided business workflow and coordination layer**, not an AI chatbot with a directory attached and not a new monolithic ERP.

The central experience is:

1. Describe the proposed business and its constraints.
2. Evaluate demand, costs, legal prerequisites, available resources, and downside scenarios.
3. Produce a staged launch plan with evidence, cost estimates, unknowns, and clear go/no-go gates.
4. Configure a minimum viable operating workflow or connect an existing operational system.
5. Record sales, expenses, stock or service delivery, payment status, and obligations.
6. Improve the business using measured outcomes.
7. Optionally connect with suppliers, other businesses, co-ops, shared equipment, or lawful community-finance mechanisms through Mycelix.

The platform must not promise that it can make every business legal, funded, profitable, or available in every country. Global reach is an architecture goal; verified capability is delivered jurisdiction by jurisdiction.

## 2. Repository fit and honest maturity boundary

The current Mycelix README describes food as part of Commons, plus finance, identity, marketplace, and supply-chain domains. It also explicitly distinguishes unit-test coverage from multi-agent tests and deployed interfaces, and identifies several domains as pre-alpha or backend-only. Those existing components are starting points, not proof that a complete business product exists.

Symthaea is an experimental cognitive architecture. Business forecasts and recommendations must be validated independently against deterministic baselines and observed outcomes; general architecture claims do not establish useful accuracy for a particular business task.

The Luminous Platform repository currently focuses on the NixOS/sovereign runtime layer. Keep that boundary: use it for secure deployment, updates, identity integration and operational reliability where those capabilities actually exist. Do not place the whole business domain model inside the host-management platform.

This RFC adds no claim that code, APIs, UIs, jurisdiction data, or integrations described below already exist.

## 3. Product principles

1. **Owner sovereignty.** The owner controls their business data, connected accounts, permissions, exports, and decisions. The platform must not make data portability contingent on staying subscribed.
2. **Evidence before confidence.** Display the source, observation time, jurisdiction, assumptions, uncertainty, and expiry of consequential claims and recommendations.
3. **Advice is not authority.** Symthaea can recommend, forecast, detect anomalies, and explain. It does not independently commit money, submit legal filings, issue credit, hire or dismiss workers, or change prices.
4. **A proposal is not an effect.** Proposed, approved, submitted, accepted, settled, reconciled, and failed are distinct states. A timeout after submission can mean an indeterminate outcome; it is not proof that nothing happened.
5. **Local-first resilience.** Core workflows should remain useful on low-end devices and weak connections. Offline actions are provisional until synchronization and conflict resolution are complete.
6. **Privacy by default.** Keep customer identities, detailed transactions, payroll, credentials, and private supplier terms in authorized private stores. Publish only intentionally shared claims or offers to Mycelix.
7. **Reuse mature components.** Prefer adapters to proven POS, inventory, accounting, payments, booking, and invoicing systems. Rebuild only where a measurable unmet need justifies the burden.
8. **Inclusive without hidden exclusion.** No single trust score, credit score, smartphone model, bank, identity provider, or connectivity level should become a universal eligibility gate.
9. **Progressive capability.** A sole operator can begin with a catalog, simple quotes/orders, expenses and cash tracking. More complex accounting, inventory, payroll, multi-site operations and cooperative finance can be enabled later.
10. **Fail closed at consequential boundaries; fail soft in guidance.** If payment status, policy version, authorization, or legal prerequisite is unclear, stop the consequential action and explain the blocker. Continue to provide non-consequential guidance where safe.

## 4. Responsibility boundaries

### Luminous Platform / application shell

Own the business-facing workflow, setup wizard, local configuration, deployment, integration adapters, secrets management, access control, backups, synchronization, and operational monitoring. Provide a consistent API and interface without requiring users to understand the underlying repositories.

### Symthaea — advisory and decision-support service

Provide optional capabilities with scoped contracts:

- Forecast demand, sales, stock use, and cash runway from explicitly identified inputs.
- Compare business models, price scenarios, and staged spending plans.
- Detect anomalies or missing records and explain the evidence that triggered a warning.
- Generate an action proposal containing its objective, assumptions, confidence/uncertainty, data window, model/version, constraints, and expected measurable outcome.
- Learn from actual results only under a documented retention/consent policy.

Any forecast must include a baseline and evaluation record. It must be safe to disable Symthaea without breaking essential point-of-sale, accounting, order, or audit workflows. It may not silently replace a deterministic calculation with a model guess.

### Mycelix — optional shared trust and coordination

Use Mycelix where multi-party coordination provides a real benefit:

- Business identity and verifiable credentials with disclosed issuer, scope, expiry, and revocation status.
- Public or permissioned supplier and service listings, including provenance and explicit terms.
- Consent-based business-to-business introductions and agreements.
- Supply-chain provenance claims, deliveries, disputes, and counterparty attestations.
- Cooperative purchasing, shared resources, and carefully bounded mutual-credit experiments if the legal, governance, liquidity, and default models are validated.
- Receipt references proving what each party agreed to or attested to, without publishing the underlying private ledger.

A Mycelix credential is evidence of a particular claim, not a guarantee that a business is honest, solvent, safe, or suitable. Reputation must be appealable and context-scoped. Avoid a single universal trust score as an access-control decision.

### External operational systems

Accounting, POS, inventory, bookings, payroll, banking, and payments remain external systems of record when an adapter is used. An adapter must not claim that an operation completed until it obtains and verifies the relevant external evidence. The product must export data in documented, usable formats.

## 5. Initial architecture

```text
Owner / cooperative members
        |
Business Launchpad UI (mobile-first, offline-aware)
        |
Domain API + authorization + policy engine
        |-------------------|--------------------|
        |                   |                    |
Business workflow       Symthaea adapter     Mycelix adapter
and state machine       advisory only        consented coordination
        |
Operational adapters
        |---------------------|---------------------|
        POS / ERP             Payments / banking    Local rule packs
        |                     |                     |
External facts, receipts, reconciliation and audit trail
```

The domain API must remain independent of any one AI model, payment provider, ERP, or national jurisdiction. Integrations are versioned adapters with explicit capability declarations and contract tests.

## 6. Common business primitives

Start with a small, reusable domain model. The exact persistence and API schemas should be proposed separately after inspecting existing crate and frontend boundaries.

- **BusinessProfile:** jurisdiction(s), legal form, operating locations, owners/authorized operators, sector, currency, timezone, language, and data-retention settings.
- **Capability/Prerequisite:** an action or permission the business needs, authority/source, applicability conditions, verification status, dependencies, evidence required, and re-check date.
- **Offer:** product or service, unit, description, price, taxes/fees treatment, availability, and terms.
- **Party:** customer or supplier with minimal necessary identifying data and consent/provenance metadata.
- **Order/Job:** requested work or goods, quoted terms, acceptance, schedule, delivery evidence, cancellation/refund path, and responsible party.
- **StockMovement:** item, quantity, unit, location, source document, reason, observation time, and valuation reference where supported.
- **Expense/Obligation:** amount, currency, counterparty, due date, evidence, tax classification state, and reconciliation state.
- **PaymentIntent / PaymentEvidence:** provider, idempotency key, amount/currency, external reference, verified provider event, settlement state, and reconciliation linkage. Never store raw card data or banking secrets in the business domain.
- **BusinessEvent:** append-only record of a meaningful state transition, actor/capability, prior state, resulting state, timestamp, source, evidence references, and correlation identifier.
- **Recommendation:** goal, data provenance, assumptions, model/rule version, uncertainty, alternatives, expected effects, constraints, and evaluation follow-up.
- **JurisdictionPack:** machine-readable rules and workflow steps, official sources, jurisdiction/industry scope, effective dates, last checked, reviewer, test cases, unsupported cases, and package version.

Do not put every private business event on a public DHT. Store private operational records in an authorized store; publish only consented claims, listings, or proofs that are necessary for cross-party coordination.

## 7. Consequential operation protocol

Every side effect must use a single explicit lifecycle:

`Draft -> Validated -> AwaitingApproval -> Approved -> Submitted -> Confirmed -> Reconciled`

Terminal or exception states include `Rejected`, `Failed`, `Cancelled`, and `Indeterminate`. Transitions are guarded and evidence-bearing.

Required controls:

- Validate the exact business, actor, permission, amount, currency, target, policy version and pre-state immediately before submission.
- Give each logical effect a stable idempotency key. A retry must not duplicate a charge, order, supplier payment, invoice, or filing.
- Record the request digest and external correlation ID; never treat a queued job or HTTP success alone as proof of settlement.
- If the network fails after a request may have reached a provider, enter `Indeterminate`, query authoritative status, and reconcile before any retry that could duplicate an effect.
- Require explicit owner approval for payments, credit commitments, tax/legal submissions, employment actions, and other material commitments. Approval must bind to the exact reviewed payload; material changes invalidate approval.
- Keep a human-readable receipt and machine-verifiable event for each material transition.
- Preserve rollback/compensation paths where supported; do not claim a financial transaction can always be rolled back.

The business UI must make uncertainty visible. It must never label a prepared, queued, submitted, or provider-accepted operation as paid, delivered, legally registered, or reconciled without authoritative evidence.

## 8. First release: narrow but complete

The first release should validate the common business lifecycle before adding complex finance or on-chain-like economic flows.

### Required v0.1 workflows

1. Create a local business profile and choose sole trader, partnership, cooperative, or company as applicable to the jurisdiction pack. If the user is unsure, explain options and direct them to authoritative or qualified advice; do not silently choose a legal structure.
2. Define products/services, prices, supplier costs, opening costs, and operating assumptions.
3. Produce a transparent viability worksheet: startup cash, fixed/variable costs, unit contribution, break-even volume, cash runway, scenarios, missing evidence, and sensitivity to price/demand/cost changes.
4. Create and track a quote/order/job from draft through customer acceptance and recorded fulfillment/cancellation.
5. Record income and expenses, attach/export receipts, and reconcile payment status using manual evidence or a verified adapter.
6. Produce a daily/weekly summary that ties to source records and distinguishes actuals, estimates, unpaid invoices, and unsettled payments.
7. Export all business data and a chronological audit trail.
8. Show a localized checklist of prerequisites and evidence, with an explicit status per item: `verified`, `owner-attested`, `reported-by-third-party`, `unknown`, `expired`, or `not-applicable`.
9. Provide a clear route for errors, corrections, disputes, data deletion where applicable, and support escalation.

### Deferred from v0.1

- Holding customer funds, issuing stored-value balances, lending, underwriting, or acting as a regulated payment intermediary.
- Automatic legal/tax filing or authoritative compliance decisions.
- Automated purchasing, pricing, hiring, or credit decisions without specific owner approval.
- Global rules presented as complete before each jurisdiction is researched and tested.
- Public business rankings based on an opaque or universal trust score.
- Rebuilding a complete general ledger, payroll, tax engine, ecommerce platform, or POS before adapter analysis establishes a need.

## 9. Integration strategy: adapter first

ERPNext already documents integrated POS, stock and accounting workflows. Odoo also offers broad business applications. These are baseline candidates, not mandated dependencies.

Proposed sequence:

1. Define a vendor-neutral adapter contract and canonical exchange formats.
2. Assess ERPNext as the first operational-system integration candidate because its documented scope includes accounting, stock, purchasing, orders, and POS.
3. Build a small compatibility spike: create/read a product or service, record a quote/order, ingest a sale or invoice, read stock, import an expense, and reconcile external payment evidence in a disposable test instance.
4. Measure setup complexity, licensing obligations, data export/import quality, offline behavior, API stability, permission granularity, and maintenance burden.
5. Keep a native lightweight mode for the simplest businesses if integration setup itself becomes a prohibitive burden. Do not maintain dual writable ledgers accidentally.
6. Add other adapters only after contract tests prove equivalent semantics or explicitly document weaker capabilities.

The adapter must declare which capabilities it supports and which states it can independently verify. A provider's missing capability remains visible; do not simulate it by silently lowering a safety requirement.

## 10. Jurisdiction packs and first market

Treat laws and official administrative requirements as versioned source data, not model memory.

Each jurisdiction pack needs:

- Country, state/province, municipality, industry, entity form, activity, and premises-specific applicability keys.
- Canonical official source URL, issuing authority, relevant section, effective-from/to dates, retrieval/verification timestamp, and content digest where feasible.
- A distinction between a legal obligation, an official explanation, an optional good practice, and platform-generated advice.
- A named review state: draft, machine-extracted, source-checked, qualified-review-needed, verified for stated scope, superseded, or withdrawn.
- Tests for applicability, date boundaries, missing inputs, stale sources, and conflicting sources.
- A fail-closed response for consequential steps when a required source is outdated, ambiguous, or absent.

### South Africa pilot notes — validate before release

- SARS's current small-business material notes 2026 changes to VAT and turnover-tax thresholds effective 1 April 2026. The application must link to current SARS sources and avoid embedding a threshold without its effective date and verification metadata. Individual taxpayer circumstances and eligibility still matter.
- Food businesses need a distinct food-premises workflow. National Department of Health Regulation R638 governs general hygiene requirements for food premises and food transport; municipal authorities may administer Certificate of Acceptability processes and premises checks. This is not replaceable by a generic company-registration checklist.
- POPIA controls are required wherever personal information is processed. Implement data minimization, purpose limitation, access controls, appropriate security safeguards, retention/deletion controls, breach workflows, and a process for data-subject requests. Never present a privacy checklist as legal certification.
- Business registration is not the same as permission to conduct every activity at every premises. Zoning, municipal by-laws, sector licenses, tax, labor and consumer obligations can remain independently applicable.
- Payments must use providers and rails lawfully available to the merchant in its country and business category. Keep the merchant's provider relationship and secrets under the merchant's control.

Official source starting points (verify on every pack update):
- SARS small businesses and taxpayers: https://www.sars.gov.za/businesses-and-employers/small-businesses-taxpayers/
- SARS VAT information: https://www.sars.gov.za/types-of-tax/value-added-tax/register-for-vat/
- SARS starting a business and tax: https://www.sars.gov.za/businesses-and-employers/small-businesses-taxpayers/starting-a-business-and-tax/
- National Department of Health food-control regulations: https://www.health.gov.za/food-control-regulations/
- City of Cape Town Certificate of Acceptability process as a municipal example, not a national substitute: https://www.capetown.gov.za/city-connect/apply/licences-and-permits/business-and-trade/apply-for-a-certificate-of-acceptability
- Information Regulator: https://inforegulator.org.za/
- CIPC BizPortal: https://www.bizportal.gov.za/

## 11. Security, privacy, and operational requirements

- Tenant and business-boundary authorization must be checked server-side on every read and write, not only hidden in the UI.
- Separate owner, operator, accountant, bookkeeper, supplier, auditor, and advisor capabilities. Use least privilege and revocation.
- Encrypt credentials and sensitive data in transit and at rest using supported deployment primitives; never log secrets, raw payment instruments, or unnecessary personal data.
- Use append-only state transition evidence with correction events; do not silently rewrite previously relied-upon records. Corrections must preserve both the original event reference and the correction reason.
- Backups must be encrypted, tested for restoration, and exportable independently of a proprietary control plane.
- Offline mutations need unique IDs, causal metadata, clear pending status and deterministic conflict handling. Financial effects must not be replayed just because an offline queue resynchronizes.
- Webhooks must verify provider signatures, reject replay where applicable, deduplicate event IDs, validate amounts/currency/references, and reconcile against provider state.
- Threat-model malicious counterparties, spoofed invoices, duplicate charges, account takeover, supplier substitution, collusion, poisoned recommendations, false credentials, stale legal guidance, and compromised endpoints.
- Do not use hidden consciousness/Φ values, reputation, or inferred psychological traits for business eligibility or risk scoring.
- Make model use optional for core business operations and disclose when data is sent to an external model or service.

## 12. Evaluation gates

Tests and metrics must make failures observable and must not report implementation as successful before the evidence exists.

### Deterministic correctness

- Unit tests for business-state transitions, money/currency rounding, taxes as adapter-provided rules, permissions, idempotency, reconciliation, exports, and localization boundaries.
- Property tests: total amounts reconcile; rejected transitions produce no effect; duplicate provider events do not duplicate postings; unauthorized actors cannot read/write another business; export/import preserves declared fields.
- Fault injection around timeouts after submit, provider webhook reordering, duplicate/replayed events, clock skew, stale rule packs, offline edits, partial synchronization, and restart recovery.
- Golden tests for every jurisdiction pack's effective dates and applicability. Outdated rules must fail the relevant launch gate.
- Contract tests against disposable integration instances; no production credentials in CI.

### Intelligence evaluation

- Compare every forecast with a declared deterministic baseline.
- Use time-ordered splits to avoid leakage; report coverage, calibration/error, performance by business type and data sparsity, and the cost of false positives/negatives.
- Report abstention when data is insufficient. No model recommendation may authorize a consequential operation.
- Maintain a versioned corpus, exact input/output digests where appropriate, and reproducible evaluation commands.

### Product outcomes

Establish baselines before claiming improvement:

- Time and direct cost from onboarding to first lawful sale.
- Weekly owner time spent on administration.
- Cash-flow forecast error and stockout/waste rates where applicable.
- Missing/duplicated transaction rate and payment-reconciliation time.
- Data export completeness and backup restore success.
- Completion rates across low-end mobile, poor connectivity, and supported languages.
- Supplier delivery success, dispute resolution time, defaults, and participant diversity for network features.
- False compliance guidance rate, stale-source detection, and correction latency.

Simulations are useful evidence for state and failure behavior, not evidence of market fit or survival improvements. Begin with deterministic simulations, then controlled demonstrations, then opt-in real usage and measured results.

## 13. Incremental delivery plan

### Phase 0 — Evidence and fit

- Map the current Mycelix finance, Commons/food, identity, marketplace and supply-chain APIs, tests, license obligations, and UI maturity.
- Map available Luminous Platform APIs and deployment/secret/back-up primitives.
- Compare ERPNext/Odoo operational coverage and licensing; build no adapter until the contract is agreed.
- Produce a repository-specific capability matrix with links to exact files and passing test evidence.

**Exit gate:** no duplicate domain service is started before the capability and licensing map is reviewed.

### Phase 1 — Vertical slice without money movement

- Business profile, catalog, cost/price worksheet, quote/order/job lifecycle, expense records, exports and audit events.
- Local-first UI with explicit pending/synced state.
- One mock adapter and deterministic fixtures; no real customer funds or automated legal filings.
- A reference service-business journey, with extension points for food and retail.

**Exit gate:** one test business can move from setup to recorded fulfillment and reconciled sample accounts end-to-end, including injected failure cases.

### Phase 2 — Operational integration

- Select one accounting/POS adapter following the compatibility spike.
- Import real-format data into a sandbox; verify totals and record provenance.
- Implement payment evidence verification and reconciliation only for a supported provider sandbox.
- Add a reviewed South African service-business jurisdiction pack, then a food-premises pack with municipal applicability.

**Exit gate:** financial totals reconcile against source records; failed or indeterminate operations are visible, and no duplicate side effects occur under retries.

### Phase 3 — Advisory intelligence

- Add forecasting and scenario comparison behind an interface and feature flag.
- Measure against deterministic baselines; record model/data versions and uncertainty.
- Provide correction and feedback mechanisms that do not silently poison training data.

**Exit gate:** reproducible evaluation supports each released recommendation class; features can be disabled without breaking core operations.

### Phase 4 — Mycelix network features

- Consent-based business listing, credentials, supplier discovery, provenance and bilateral agreements.
- Add shared purchasing before mutual credit because it tests coordination without introducing credit issuance and liquidity risk.
- Treat mutual credit as a separately reviewed experiment with legal scope, issuance/redemption rules, caps, default handling, dispute resolution, and clear non-cash representation where applicable.

**Exit gate:** multi-agent tests and threat models cover collusion, revocation, duplicate claims, and disagreement; no public trust score becomes an unreviewable single point of failure.

### Phase 5 — Repeatable localization

- Add a jurisdiction pack development kit, test harness, source-review workflow, and explicit support matrix.
- Expand by measured demand and source-review capacity, not by translating an unverified checklist.
- Version API and data contracts so existing businesses can safely upgrade or pin a supported pack.

**Exit gate:** a second jurisdiction can be added without changing shared business semantics or weakening existing tests.

## 14. Open design decisions

Before implementation beyond Phase 0, resolve with evidence:

1. Where does the user-facing application live — an existing Mycelix app, a separate public product repository, or a thin client within Luminous Platform? Current platform ownership suggests a separate business-facing app/client consuming stable Mycelix and Symthaea interfaces; avoid putting business-domain logic into the OS runtime.
2. Which current Mycelix role/zome is the proper home for each shared claim, listing, and agreement? Prefer existing maintained domains where semantics match; do not wire a new UI to pre-alpha zomes just because they compile.
3. Is ERPNext the first system of record for operations or should v0.1 use a lightweight local ledger? Decide only after the integration spike and low-end deployment test.
4. Which municipality and initial business activity are in pilot scope? The answer determines real compliance evidence; it cannot be generalized from a country-level list.
5. What is the data-controller/processor model, retention policy, service boundary, and support process for each deployment option?
6. What exact licenses and third-party obligations apply to the chosen app, adapters, Mycelix dependencies, and Symthaea components? AGPL and third-party licenses must be assessed for the actual distribution model.

## 15. Decision recommendation

Approve the direction as an architecture proposal and proceed with Phase 0. Do not launch mutual credit, automated regulatory actions, a new general ledger, or AI-driven transaction automation as the first milestone.

The smallest credible product is an **evidence-first business launch and operating workflow** that gives owners a transparent viability model, required-step tracker, order/job tracking, simple financial records, exportability, and a clear source of truth. The wider Mycelix network and Symthaea intelligence should deepen that product only when their incremental value is measured and their failure modes are contained.

## References

- Mycelix repository and maturity notes: https://github.com/Luminous-Dynamics/mycelix
- Symthaea status and evidence documentation: https://github.com/Luminous-Dynamics/symthaea
- Luminous Platform architecture: https://github.com/Luminous-Dynamics/luminous-platform
- ERPNext documentation: https://docs.frappe.io/erpnext/introduction
- ERPNext POS: https://docs.frappe.io/erpnext/point-of-sale
- ERPNext inventory accounting: https://docs.frappe.io/erpnext/accounting-of-inventory-stock
- Odoo: https://www.odoo.com/
- Paystack payment channels (availability varies by country/merchant): https://paystack.com/docs/payments/payment-channels/
- Current South African primary-source links are listed in Section 10 and require verification on each update.
