# PCOE Product Passport and Asset Lifecycle Addendum v0.1

**Status:** proposed interoperability and assurance profile; not legal advice, a registry, product certification, or a claim of regulatory conformance.  
**Date:** 22 November 2026.

## 1. Why this is a PCOE domain

A community energy project does not procure an abstract solar or battery capacity number. It procures specific physical products: modules, inverters, battery packs, control systems, cabling, transformers and safety equipment. Those products have identities, manufacturers, conformity evidence, software/firmware, warranties, maintenance events, repairs, replacements, ownership history and end-of-life obligations.

A procurement workflow that proves only the invoice and installation milestone loses the lifecycle evidence needed to assess safety, warranty, repairability, circularity, residual value, recalls and eventual recycling.

PCOE should therefore extend the proof-carrying obligation graph from the project and contract to each material asset.

**Paradigm shift:** treat an asset as a persistent, versioned lifecycle subject, not a static SKU, certificate PDF or inventory row. Each consequential change is a new linked event with an accountable issuer, authority, evidence, time, scope and supersession relationship.

## 2. Integrate with the EU Digital Product Passport ecosystem

The European Commission's Digital Product Passport Registry became operational on 20 July 2026. The Commission describes decentralised product data with central registration of unique product identifiers and associated metadata, secure UI/API access, user verification, and machine-readable data models and vocabularies. Certain battery categories are scheduled to require passports from 18 February 2027.

The Battery Passport guidance includes specific data points and public/restricted access distinctions. The Commission's battery FAQ states that a responsible economic operator may authorize another operator in writing to create or update passport information on its behalf. The FAQ also explains that repurposing or remanufacturing requires a new passport linked to the previous one; a simple repair does not automatically require a new passport.

These requirements are product- and jurisdiction-specific. They must be pinned to the applicable legal instrument, delegated/implementing act, data-point list and effective date. Do not assume all PV modules, inverters, batteries or energy equipment currently have the same mandatory passport obligation.

**PCOE must not create a shadow DPP registry.** It should maintain a privacy-aware evidence projection referencing the authoritative passport identifier, registry, issuer, schema/profile version and source evidence.

## 3. Product and asset evidence graph

Model at least the following linked records:

1. **Product identity:** product model/type, manufacturer, serial or batch identifier, product passport identifier, applicable registry and product category.
2. **Responsible operator and authority:** responsible economic operator, authorised delegate, scope of written authority, validity and revocation evidence.
3. **Conformity and provenance:** declarations/certificates, critical components, origin/material claims, safety notices, firmware/software configuration and their issuers.
4. **Procurement and custody:** contract line, supplier, buyer, invoice, delivery receipt, chain-of-custody events and unique installed-asset identifier.
5. **Commissioning:** installation location, installer authority, commissioning procedure, inspection, test results, configuration, warranty commencement and exact installed-product identity.
6. **In-service events:** operating configuration, maintenance, inspection, sensor/condition history, repair, component replacement and software updates.
7. **Lifecycle transitions:** transfer of ownership/custody, refurbishment, repurposing, remanufacturing, decommissioning, waste handling and recycling.
8. **Passport status:** created, registered, current, restricted, suspended, superseded, indeterminate or retired/closed as defined by the applicable programme.

A passport registration receipt is evidence of registration, not proof that every product claim is true. A manufacturer declaration is an issuer assertion, not independent conformity assessment. A maintenance event is not proof that a repair restored safety. A digital twin's estimated state of health is not a legally valid battery test.

## 4. Versioned lifecycle transitions

Never overwrite the product's identity or erase its history to represent a repair or change of responsible operator.

- A normal service event appends a maintenance record linked to the same product identity.
- A simple repair appends the intervention, changed components, authority, method, evidence and inspection disposition.
- Component replacement links the removed and installed component identities and preserves both histories.
- Repurposing/remanufacturing follows the applicable legal and registry rules. Where a new passport is required, it must link to the previous passport and the predecessor must remain discoverable.
- Transfer of ownership/custody is a new, authorised event; it does not rewrite the manufacturer's identity or erase safety/recall history.
- Decommissioning, waste transfer and recycling preserve chain-of-custody and accountable operator evidence.
- Corrections and registry updates append superseding evidence and re-evaluate dependent contracts, guarantees and financial claims.

A current status must be derived from authoritative registry events and the pinned policy version. A locally cached passport cannot silently remain current after a recall, suspension, transfer, correction or revocation.

## 5. Privacy and access control

Do not equate transparency with public disclosure of every product record.

The profile must distinguish public product information from commercially confidential, personal, security-sensitive and regulator-restricted information. Require:

- purpose-bound, role-specific access;
- a record of which authority permitted a delegate to create/update the passport;
- selective disclosure for downstream buyers, installers, financiers, recyclers and inspectors;
- access decisions bound to data-point category and applicable policy version;
- least-privilege API integration and auditable disclosure;
- a fail-closed response when registry access rights or responsible-operator authority cannot be verified.

Do not put personal data, private facility security details, credentials or unrestricted operational telemetry into public transparency logs. A transparency receipt should bind the statement digest and registration context, while the underlying record remains under its lawful access policy.

## 6. PCOE integration with the four systems

| System | Contribution | Boundary |
|---|---|---|
| Symthaea | Advisory supplier/product comparisons, lifecycle-risk analysis, repair/replacement planning and anomaly triage | Cannot assert conformity or authorize procurement |
| Mycelix | Identity, supplier/operator mandates, procurement contracts, custody events, guarantees, asset ownership and financial obligations | Cannot replace the legally authoritative DPP registry |
| Symtropy | Physical installation twin, commissioning, performance/condition observations, maintenance and decommissioning evidence | Simulation/estimated state remains distinct from inspections and certified tests |
| Sol-Atlas | Geographic/site context, provenance, rights, facility history, contextual asset knowledge | Context does not prove product identity, conformity or title |
| External issuer/registry/assessor | Authoritative passport registration, product claims, conformity assessment and registry lifecycle | Must be independently authenticated and scoped; do not trust a local status projection as authority |

## 7. Adversarial scenarios

The companion `pcoe-product-passport-scenarios.v0.1.json` defines 12 scenarios. Passing requires assertions on both the resulting disposition and any forbidden side effects or disclosures.

- Product serial or passport identifier duplicated across unrelated assets.
- Manufacturer, model, serial or batch swapped after procurement acceptance.
- A delegate updates a passport without valid written authority or beyond its delegated scope.
- A passport is locally shown as registered although registry registration is missing, pending or rejected.
- Restricted battery/product data is disclosed to an unauthorized buyer, agent or public transparency service.
- A component substitution is made after conformity and commissioning evidence was accepted.
- Maintenance is falsely promoted to certified repair or successful safety inspection.
- Repurposing/remanufacturing creates a new passport without linking the prior passport and lifecycle history.
- Recall, suspension or safety notice is omitted from the current local projection.
- A stale/offline cache is accepted as current after registry state has changed.
- Correction or deletion rewrites predecessor history instead of appending a superseding event.
- Recycled-content, origin or material claims lack the relevant supplier/issuer evidence chain.

## 8. Proposed benchmarks

Targets are research goals, not legal thresholds or achieved results.

| Measure | Proposed target | How to qualify |
|---|---:|---|
| Duplicate product/passport identity accepted | 0 | Cross-supplier serial/batch collision tests |
| Unauthorized passport updates accepted | 0 | Delegation, expiry, scope and revocation tests |
| Pending or rejected registry state promoted to registered/current | 0 | Registry outage and negative response fixtures |
| Unauthorized restricted-data disclosures | 0 | Access-control matrix and data-minimization adversarial tests |
| Mandatory lineage links omitted after repurposing/remanufacturing | 0 | Predecessor/successor lifecycle tests |
| Asset commissioning evidence bound to wrong physical item | 0 | Serial, photograph/inspection and component substitution controls |
| Recall/suspension freshness | 100% of tested affected assets blocked before consequential acceptance | Event propagation and stale-cache tests |
| Independent lifecycle reconstruction | 100% of published test bundles reproducible | Clean-room verification from exported evidence |
| Passport retrieval and verification | Report p50/p95/p99 | Versioned API workload with registry availability declared |
| Lifecycle evidence completion | Report by data point and product category | No average may hide a mandatory missing field |

Report missing-data rates, stale-cache duration, correction propagation, time-to-revoke, operator effort, API costs, false blocks and false acceptance. Do not optimize only for schema completeness: a syntactically complete passport may still contain a false claim.

## 9. Industry comparison and strategic advantage

| Existing capability | PCOE should do |
|---|---|
| EU DPP Registry and product-specific registries | Integrate; do not fork identity or registry status |
| Manufacturer/ERP product systems | Reconcile supplier assertions against independent delivery, commissioning, inspection and custody evidence |
| Asset management / CMMS | Bind work orders, technician authority, replacement components and evidence to the durable asset identity |
| SCITT / transparency services | Register signed statements and verify receipts; do not confuse inclusion with product truth |
| Digital twins / predictive maintenance | Link predicted degradation to measured observations and qualified inspections without collapsing their epistemic status |
| Warranty and finance systems | Bind warranty obligations, service levels, maintenance duties, residual value and payment milestones to the exact asset lineage |

The differentiator is not a prettier passport UI. It is **asset lifecycle accountability across institutions**: the same verified product identity should connect supplier evidence, procurement, installation, operation, service, ownership, finance, repurposing and recycling—while remaining interoperable with the relevant authoritative systems.

## 10. References and current status

- European Commission, Digital Product Passport Registry live (20 July 2026): https://single-market-economy.ec.europa.eu/news/digital-product-passport-registry-now-live-2026-07-20_en
- European Commission, Digital Product Passport overview and indicative timeline: https://single-market-economy.ec.europa.eu/single-market/digital-product-passport_en
- European Commission, Battery Passport data points and implementation timeline: https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/batteries_en
- European Commission, Battery Passport FAQ: https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/eu-digital-product-passport-faq-batteries_en
- IETF RFC 9943, SCITT transparency architecture: https://datatracker.ietf.org/doc/rfc9943/

The EU Registry is operational, but product-specific requirements and access rights continue to evolve. The 18 February 2027 battery deadline applies to the relevant categories, not every energy product. Pin all applicable acts, data-point versions and effective dates at integration time.

---

**Interpretation rule:** PCOE asset-lifecycle success means the tested asset's identity, authority, provenance, custody, commissioning and lifecycle events were reconciled under a declared product/jurisdiction profile. It does not confer product conformity, legal title, warranty coverage, registry authority or permission to publish restricted passport data.
