# Beverage Batch Evidence and Provenance V1

Status: proposed evidence envelope and synthetic fixtures. This is not a certification, a legally sufficient traceability system, or an implemented Holochain zome.

Tracking issue: https://github.com/Luminous-Dynamics/mycelix/issues/5330  
Scientific benchmark: https://github.com/Luminous-Dynamics/symthaea/issues/8232  
Process twin: https://github.com/Luminous-Dynamics/symtropy/issues/2387  
Regional input evidence: https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/226

## Problem

A beverage's final batch may depend on multiple ingredient lots, fermentation batches, blends, distillation/maturation events, packaging runs, labs, transport legs and custody changes. A single QR code or immutable record does not make all those claims true. The system needs to distinguish:
- identity of a batch or product;
- an event a party declares occurred;
- a measurement and its method/calibration context;
- a model output or simulation;
- who attested to which bytes and for what scope;
- whether a consumer or partner is authorized to see a claim;
- whether an evidence-backed claim has actually passed an independent, claim-specific rule.

## Paradigm shift: attest the claim, not the entire producer

The product label should be able to prove a specific permissible statement—such as the identity of a lot or the existence of a valid lab report—without exposing a recipe, formula, supplier pricing, complete telemetry or personal information. Keep process secrets outside public DHT entries; publish or share only the minimum disclosure necessary for the particular claim.

No generic verified badge should collapse all evidence into one binary label. Claim eligibility is computed from the requested claim, evidence class, data rights, artifact digests, method, signer scope and validity period. Missing evidence is unknown, not a pass. A hash proves byte identity relative to that hash—not the truth of a sensor reading or laboratory method.

## Interoperability

Use standard identifiers and event semantics where applicable instead of inventing a closed supply-chain dialect:
- GS1 EPCIS/CBV 2.0 is the candidate interoperable shape for timestamped events, batch transformations, sensor observations and custody movement.
- GS1 Digital Link is a candidate product identifier-to-information pointer, not a truth oracle or an authenticity certificate.
- Producer-specific internal batch IDs can be pseudonymous in shared records, with a permissioned resolver mapping them to operational data.
- A jurisdiction-specific wine-labeling or food-information adapter must pin the applicable legal source, effective date, product/market scope and rule version. This schema does not make a regulatory-compliance determination.

## Record semantics

1. **Batch graph** — each output batch references its input batches, and each transformation event records a time, actor, location where applicable, and supporting evidence references. Independent validation must reject cycles, broken references and impossible event ordering.
2. **Evidence record** — identifies subject, category, evidence class, method, units, timestamp, calibration context, exact artifact digest and data-rights state. Values from measured, modeled, simulated and declared sources must not be silently mixed.
3. **Attestation** — binds an issuer and a declared scope to exact artifact bytes, issuance time, expiry and revocation information. Signature verification, issuer authorization, key state and certificate trust need a separate verifier; a schema field cannot prove them.
4. **Claim request** — names a target batch, claim type, value and required evidence references. A verifier computes the disposition; a producer-supplied status field cannot force a claim to be verified.
5. **Disclosure** — the record names a disclosure purpose/audience while the authorization layer decides which fields may be revealed. Never put recipe contents into this public evidence envelope.

## Acceptance

- Unique IDs across relevant namespaces; all references resolve to a record of the expected type.
- Transformation lineage is acyclic, with coherent event ordering and explicit inputs/outputs for blending, distillation and packaging.
- Any content digest is recomputed from exact bytes; algorithm, size and source reference are explicit.
- Unknown, revoked or expired attestations cannot pass a claim requiring current valid attestation.
- Data rights and permission-to-use are evaluated separately from availability; unknown rights means ineligible for reuse/disclosure, not open by default.
- A SIMULATED or MODELLED value cannot satisfy a claim that explicitly requires MEASURED evidence.
- Correction is append-only through explicit supersession; prior bytes and attestations remain auditable.
- Public claim disclosure is minimized; access logs and consent are separate from evidence authenticity.
- All hard gates are non-compensatory: low cost, high sensory rating or sustainability score cannot override invalid evidence or absent authorization.
- The receipt distinguishes schema validity, evidence-integrity checks, signature/issuer verification, claim disposition, and legal/regulatory review.
- Negative controls cover tampered digest, wrong subject binding, duplicate IDs, cycle in lineage, stale calibration, expired/revoked attestation, unauthorized disclosure and simulated-as-measured records.

## Claim ceiling

This proposal defines a data contract and tests for future verifiers. It does not demonstrate that a beverage's origin, ingredients, quality, safety, sustainability, alcohol content or regulatory label are factually correct. No attestation can substitute for the relevant accredited laboratory, audit, competent authority or applicable law.

## Research anchors

- GS1 EPCIS / CBV 2.0: https://www.gs1.org/standards/epcis
- GS1 Digital Link: https://www.gs1.org/standards/gs1-digital-link
- GS1 Global Traceability Standard, Critical Tracking Events and Key Data Elements: https://www.gs1.org/standards/gs1-global-traceability-standard/current-standard
- European Commission wine labelling explainer (rules applied from 8 December 2023 to wines obtained from 2024 harvest, with transition boundaries): https://agriculture.ec.europa.eu/media/news/new-rules-wine-labelling-enter-application-2023-12-07_en
- BIER 2025 Water, Energy, and Emissions Benchmarking Summary: https://www.bieroundtable.com/2025-water-energy-and-emissions-benchmarking-summary-bier
