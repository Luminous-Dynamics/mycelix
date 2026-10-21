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

## Resource-meter evidence must carry its accounting boundary

Resource and sustainability claims need stricter evidence than a generic statement plus a hash. The evidence schema now has a dedicated `RESOURCE_METER` category. A record in that category must declare a numeric reading, units, meter/instrument reference, measurement period, functional unit, versioned meter-boundary reference and status, production-output basis, and policy for missing intervals. Optional fields can record nonnegative measurement uncertainty and a production-mix reference.

This makes a key cross-project invariant explicit: resource evidence only supports a comparison when the numerator boundary and denominator are aligned. For a beverage plant, a preferred denominator is saleable, quality-conforming output; total output and input volume remain representable, but must not be silently compared as if equivalent. Partial or unknown meter coverage and estimated data need to remain visible to the claim-specific verifier. The schema is a contract, not an implemented comparator; it cannot resolve boundary references or itself decide whether a resource-saving claim passes.

Two additional synthetic negative controls are now included:
- `BP-011 resource_claim_with_mismatched_functional_unit`
- `BP-012 resource_savings_claim_from_partial_unreported_meter_coverage`

Their purpose is to reject unqualified comparisons, not to simulate measured resource performance.

## Abnormal-process outputs must not be promoted into safety facts

Model-based fermentation monitoring may flag a trajectory as abnormal, but a residual or anomaly score alone cannot establish contamination, identify a microorganism, certify food safety, or justify a regulatory statement. The evidence envelope now has four additional synthetic negative controls:
- `BP-013`: an anomaly score is presented as contamination confirmation;
- `BP-014`: sensor drift mimics a process disturbance;
- `BP-015`: an estimator switches without an auditable trigger, data-prefix digest or uncertainty record;
- `BP-016`: a paper that uses confidential industrial data is presented as reusable batch evidence.

A model estimate should retain its evidence class, model/version, observation cutoff, input-prefix digest and uncertainty. Laboratory claims still require the relevant sample identity, method and actual result. A sensor calibration record is separate evidence, not an optional annotation that makes a suspect measurement valid. An unknown state remains unknown; it must not default to either normal or contaminated.

This maps to the Symtropy abnormal-adaptation benchmark and the 2026 literature comparator at https://doi.org/10.1016/j.ceja.2026.101157. That paper's underlying industrial data are confidential, so its published metrics can motivate test design but cannot be imported as measured evidence for a new batch or reproduced as if the rows were public.

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

## Adaptive-sensing decision provenance

The evidence envelope now includes a typed `SAMPLING_DECISION` category and a `SAMPLING_DECISION_RECORDED` event. The intent is to preserve the chain from a sampling recommendation to the measurement (if obtained), the resulting process inference, and any later claim. A policy recommendation and a completed measurement are different evidence objects; a request must not be serialized as though the sample result already exists.

For a sampling-decision evidence record, retain the policy version and configuration digest, decision timestamp, declared input-prefix digest, available channel inventory, requested action, reason code, calibration state, cost estimate, and the required tests due versus required tests still scheduled. SAMPLE records name the available channel and request time; DEFER records name a future review time and require calibrated uncertainty; ESCALATE and ABSTAIN records cannot simultaneously carry a sample request or pretend to defer a required check.

The Draft 2020-12 schema checks required fields and action-specific shape. Cross-field temporal order (for example, observation availability not later than decision time), actual subset preservation of due required tests, digest recomputation, channel-inventory truth, and policy reproducibility still require an executing verifier. The companion Symthaea contract runner checks several of these internal relationships for a single decision record. Neither schema nor the runner authenticates the underlying measurement or proves that the declared minimum testing schedule is complete.

New synthetic policy/provenance cases BP-017–BP-020 cover missing decision lineage, unavailable channels, deferral that drops a required test, and unsupported claims that fewer samples preserved event detection. The 20-case corpus and synthetic decision-evidence envelope are test fixtures only, not a measured batch record or a plant authorization.

### Decision-value and deadline provenance

The sampling-decision record now keeps the expected loss without a discretionary measurement, expected loss when its result is used, estimated sampling cost, estimated delay cost, and expected result-availability time alongside the decision deadline. These terms use one preregistered loss/utility unit; do not add money, time, and probability directly without a documented conversion.

For a SAMPLE recommendation, Symthaea's executable contract checks that:

`expected_loss_without_sample - expected_loss_with_sample - sampling_cost - delay_cost > 0`

and that the expected result arrives by the decision deadline. This check means the recommendation is internally consistent with its own stated rationale; it does not mean the estimates are calibrated or the sample will actually improve a batch outcome. Retrospective replay should score forecasted net value against held-out outcomes, and a result arriving after the current decision deadline cannot be counted as improving that already-due decision.

The Mycelix schema records these terms and the synthetic fixture carries an internally positive example. A production provenance verifier still needs cross-field arithmetic and chronological checks, digest recomputation, signature and authorization validation, and independent quality/process review. New cases BP-021 and BP-022 reject non-positive expected net value and a sample result arriving after the claimed decision deadline.

The companion paired replay evaluator in Symthaea distinguishes policy-record validity from cohort performance. It compares a declared policy with a fixed baseline on paired trajectories or matched independent units; it reports cost-reduction and decision-loss intervals, worst-pair loss, high-severity event misses, false alarms, deadline misses and required/hard-constraint violations. A synthetic replay receipt remains a contract result, not an empirical or causal finding.
