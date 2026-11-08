# Maritime Commons Interoperability Benchmark v0.1

**Status:** Draft research specification; not executed  
**Owner:** Mycelix / Maritime Commons  
**Tracking:** [#5932](https://github.com/Luminous-Dynamics/mycelix/issues/5932)  
**Parents:** [#5916](https://github.com/Luminous-Dynamics/mycelix/issues/5916), [#5920](https://github.com/Luminous-Dynamics/mycelix/issues/5920), [#5921](https://github.com/Luminous-Dynamics/mycelix/issues/5921), [#5925](https://github.com/Luminous-Dynamics/mycelix/issues/5925)

## 1. What this benchmark must falsify

The benchmark tests whether a thin, provider-neutral coordination layer can compose independent maritime service providers while preserving each provider's authority over its own inventory, customer relationship, terms and commitments.

It must compare equal tasks, not feature lists or vendor marketing. It must be possible for the benchmark to show that federation is slower, more expensive to integrate, more difficult to operate, or less reliable than an incumbent workflow. A benchmark that cannot falsify the thesis is not useful.

The v0.1 benchmark is a protocol and evaluation contract. It is not a product qualification, market study, operator endorsement, certification, or claim of superiority.

## 2. Comparison arms

Use the same synthetic task, resources, policy constraints, fault schedule and outcome definitions for each applicable arm.

| Arm | Description | Evidence boundary |
| --- | --- | --- |
| MANUAL | Current siloed process: separate marina, accommodation and local-service requests, with human handoffs and reconciliation | Measure participant/operator actions and active labor separately from waiting time |
| NATIVE | Each provider uses its own existing system and remains the source of truth | Requires provider-authorized access or an operator-observed workflow |
| NETWORK | An incumbent booking/discovery network or marketplace is used where the required capability is actually offered | Compare only available, region-relevant, permissioned functionality |
| FEDERATED | Maritime Commons coordinates provider-owned offers, commitments and receipts through replaceable adapters | No central authoritative inventory mirror and no hidden write authority |

Not every case is comparable across every arm. Record "not applicable" with a reason rather than manufacturing parity. A public marketing page is not a runtime trial. Lack of public API documentation is "unknown", not proof that a vendor has no integration capability.

## 3. Evidence grades

Every asserted capability and result must carry one of these grades:

1. public_documented: published by the responsible organization; not independently reproduced.
2. vendor_reported: a claim or metric published by the vendor; retain the original attribution.
3. authorized_sandbox_observed: tested with permission in an explicitly identified sandbox.
4. operator_observed: directly observed with the operator's permission; sample and procedure recorded.
5. independently_reproduced: repeated by an independent evaluator from frozen inputs and documented procedure.
6. synthetic_only: demonstrated only by generated fixtures or a local simulation.
7. unknown: not established. This must not be converted into a negative capability claim.

Separate each result's evidence grade from its confidence interval and from the protocol's own conformance status.

## 4. Workload model

The v0.1 workload composes:

- one berth resource with time interval and vessel-compatibility constraints;
- one independently operated accommodation resource;
- one optional local service resource;
- provider-specific fee, cancellation and refund terms;
- optional community-recognized eligibility;
- an independent port/statutory clearance condition that is never implied by a marina booking.

Each provider is the authority for its own availability and commitment. The benchmark may compare a coordination layer's representation of evidence, but it cannot grant a provider authority or infer a physical or statutory fact from a message.

The machine-readable workload manifest is interoperability-workload-v0.1.json. It references the existing semantic and webhook corpora; it does not duplicate them.

## 5. Scenario matrix

The initial frozen matrix includes:

- complete mandatory stay across three independent providers;
- mandatory provider rejection;
- optional provider rejection;
- provider commits durably, then the response is lost;
- repeated and out-of-order event delivery;
- two competing claims for one capacity-one resource;
- stale offer or superseded provider revision;
- provider adapter replacement;
- partition during a multi-provider stay;
- key rotation or revocation before an attempted action;
- an entitlement not recognized by the destination community;
- fee mutation, partial cancellation and refund ambiguity;
- overbroad disclosure versus purpose-minimized disclosure;
- marina booking presented incorrectly as port-entry or border clearance.

The matrix is not exhaustive. New cases must be versioned rather than silently replacing a failed case.

## 6. Measures — no composite score

### 6.1 Correctness and safety

Report counts and rates for:
- valid completed stays;
- partial stays and unresolved stays;
- false-confirmed stays;
- double-committed scarce resources;
- duplicate financial effects;
- unauthorized actions or disclosures;
- stale/revoked observations promoted to current commitments;
- unresolved contradictions and manual reconciliation.

A zero in a synthetic corpus means only that the implementation passed those exact cases. It does not establish a zero real-world failure rate.

### 6.2 Coordination effort

Record separately:
- wall-clock end-to-end elapsed time;
- active participant minutes;
- active provider/operator minutes;
- machine processing latency;
- number of human handoffs;
- number of systems/channels touched;
- manual reconciliation interventions;
- successful completion and abandonment.

Do not merge machine latency, human waiting time and active labor into one headline number.

### 6.3 Integration and lifecycle cost

Record:
- engineering/person-hours to first successful adapter conformance;
- operator/admin time and permissions required;
- number of provider-specific mapping rules;
- number of custom code paths;
- number and sensitivity of fields disclosed;
- time and code changes to replace a provider adapter;
- maintenance effort after provider schema/profile revision;
- rollback time and behavior after adapter revocation.

Count the ongoing operator burden as well as the one-time developer effort.

### 6.4 Provider autonomy and local governance

Verify:
- who owns authoritative availability and allocation;
- who can create, alter, cancel and refund a provider effect;
- whether the provider can replace or revoke its adapter;
- whether provider operations continue if the federation is unavailable;
- whether a community can reject an otherwise valid credential or entitlement;
- whether local resident-access, accessibility and ecological rules remain enforceable.

### 6.5 Privacy and economic transparency

Measure purpose-specific data fields, retention, unnecessary copies, fee disclosure before acceptance, invoice arithmetic, refund evidence and reconciliation outcomes.

A valid signature does not establish that a payment settled. A payment provider's finality reference must be independently recognized under the relevant payment/accounting contract.

### 6.6 Interoperability and recovery

Inject duplicate, missing, delayed, out-of-order and contradictory events; stale cache entries; endpoint replacement; key rotation; credential revocation; partitions; process restarts; and failed reconciliation.

Report the exact time and evidence required to move from Unknown or NeedsReconciliation to a justified terminal disposition.

## 7. Non-compensatory safety gates

A run fails the safety comparison if any of these occurs:

- false-confirmed stay;
- double-committed capacity-one resource;
- duplicate financial effect;
- unauthorized or wrong-purpose action;
- acceptance of revoked or locally unrecognized authority;
- disclosure outside the declared purpose/data-minimization policy;
- stale/offline observation represented as a current provider commitment;
- marina booking represented as port-entry, customs, immigration, navigation, pilotage or safety authorization.

Do not average a safety failure away with faster completion or better usability.

## 8. Reproducibility and statistics

Every run must record:

- benchmark and case-set version;
- exact repository commit/tree identity for software under test;
- fixture and input digests;
- provider adapter and profile version;
- evaluator version and independent-oracle identity;
- environment and machine configuration;
- fault schedule and random seed, if any;
- all attempts, including timeouts, failed and excluded runs;
- raw per-case outcomes and reasons for exclusions.

Use paired task runs where possible. Report sample counts, median, p90/p95 and confidence intervals. Pre-register the human study procedure and participant roles. Keep synthetic runs, sandbox runs and field observations in separate result tables.

A practical initial design is at least 30 paired observations per benchmark arm for exploratory timing distributions, followed by a power analysis using observed variance before confirmatory human-performance claims. This is a planning floor, not a guarantee of statistical power.

No result is reproducible if the evaluator silently changes fixtures, drops failures, relaxes a gate or uses a different software head without a new benchmark version.

## 9. Proposed hypotheses — not measured targets

Test these hypotheses against the frozen baselines:

- at least 30% lower median end-to-end coordination time than the matched manual baseline;
- lower active human coordination time without increasing provider/operator effort;
- adapter replacement without transferring provider inventory or customer databases;
- lower unnecessary personal-data disclosure for the same task and purpose;
- adapter onboarding effort decreases as reviewed profile reuse increases;
- ambiguity after a lost provider response remains explicit until provider-authoritative reconciliation.

The 30% target is a hypothesis only. Report the measured baseline first. Do not tune the workload after seeing results.

## 10. Industry and standards comparison boundaries

Dockwa, BerthApp, Navily and other products have overlapping but different booking, discovery, operational and marketplace functions. A comparison must use the same region, user task, provider permissions and actual feature availability. Preserve vendor attribution for vendor-reported metrics.

### Current public product baseline (retrieved 8 November 2026)

| Product | Publicly described strengths | What this means for the experiment | Evidence limits |
| --- | --- | --- | --- |
| Dockwa | Marina management plus boater marketplace; reservations, contracts, billing, fuel, POS, space occupancy, dry stack, service/work orders, dynamic pricing, payments and accounting integrations. Its company page reports 4,000+ marinas, 500,000+ boaters, 900,000+ boats and 100M+ nights booked, last updated 1 October 2026. | Treat Dockwa as a mature vertically integrated incumbent. Do not compete by rebuilding marina CRM, occupancy, payments or work orders. Test whether a third-party federation composes Dockwa with unlike non-Dockwa providers without moving the marina's source of truth. | Features and network totals are Dockwa-published/vendor-reported, not independently audited. |
| Navily | Cruising guide, community discovery of anchorages/marinas, reviews/photos, itinerary/distance planning and bookings in partner marinas. Its homepage reports 1.2M users, 40,500 marinas/anchorages and 750,000 contributions. | Treat discovery and cruising community features as existing strengths. Test cross-provider service composition and local entitlement/obligation reconciliation, not another anchorage directory. | Usage and inventory figures are Navily-published; a listing is not live availability, a confirmed berth or navigation authority. |
| Berth / BerthApp | South African marina-management/booking platform. Its terms (updated 2 October 2025) describe digital berth booking/payment while stating the marina operator remains the berth-service provider and direct contractual counterparty; cancellation/refund arrangements depend on provider payment configuration, and cancellation is initiated with the provider. | The platform/operator contract split is already present here. Do not claim that contractual separation alone is novel. Test whether BerthApp-connected operators can interoperate with providers using different systems, share bounded evidence and participate in locally governed service compositions. | Terms are Berth-published and not a technical API contract, security assessment, proof of current API availability or integration permission. |
| Manual/siloed process | Separate booking channels, human handoffs, document/email/WhatsApp follow-up and manual reconciliation. | Measure active human effort, elapsed waiting, missed handoffs, partial failures and dispute resolution using matched tasks. | Requires operator-observed workflow study; synthetic simulation is not a field baseline. |

Do not infer that a product lacks an API, integration surface or capability simply because its public marketing page does not mention it. Record unknowns as unknown and ask for authorized documentation or test access. Do not compare Dockwa's North American network totals to a South African pilot as though geographic coverage or operational scale were equivalent.

### Standards and transport boundaries

DCSA Port Call 2.0 is a structured exchange profile for commercial port-call service events. Its OpenAPI contract uses publisher and subscriber roles and exposes GET /events and POST /events patterns; subscriber registration and bilateral authentication/authorization are out of scope. The DCSA API Design and Implementation Guidelines 2025.Q4 additionally say that new DCSA APIs use consumer-implemented notification endpoints instead of callback/webhook patterns and avoid CloudEvents for notifications. That is a DCSA design choice, not a mandate for every maritime provider.

Dockwa, by contrast, publicly documents admin-enabled HTTPS webhook POST delivery. The first Dockwa adapter must preserve that provider-native model and its actual authorized header/payload semantics. Do not translate between webhook, DCSA notification endpoint and CloudEvents transports as if they were identical.

CloudEvents 1.0.2 is an optional event-context profile for systems that actually use it; it is not the canonical Maritime Commons wire format. Source-native data, identity, schema and timestamp semantics remain evidence. Any normalization emits an explicit translation/loss record.

DCSA Port Call is not a recreational marina booking, cross-provider stay contract or statutory clearance. Maritime Commons may adapt applicable event semantics, but must implement its own reviewed identity, authorization, subscriber onboarding, purpose and local-recognition boundaries. Use the DCSA conformance sandbox and scenario model where relevant, without claiming DCSA certification for the stay protocol.

IHO S-100 product specifications describe defined maritime information products. They are not live berth inventory, booking confirmations or port-entry permissions. Pin exact product edition, producer, source dataset/cell, coverage, datum, validity, validation profile and transformation lineage; never use one generic S-100 compliant flag.

A standards adapter proves only the declared mapping/conformance subset. It does not prove operational acceptance by a port, marina, authority or ECDIS equipment.

## 11. Stop conditions and field-study permissions

No live booking, reservation hold, payment, cancellation, customer-data export, webhook activation or production endpoint connection is part of this benchmark without separate written authorization from the responsible provider/operator and appropriate legal/security review.

A provider sandbox or sample payload must be clearly identified as authorized. Synthetic payloads must never be represented as real provider traffic.

A South African candidate register is not a partnership or pilot selection. Keep ZYC/BerthApp and Durban Marina as screening candidates until operators explicitly agree to participate.

## 12. Core acceptance

- Comparison arms use matched tasks and explicit applicability rules.
- Claims are assigned evidence grades and vendor attribution.
- The workload manifest is machine-readable and references the existing semantic/webhook corpora.
- Safety gates are non-compensatory and have negative controls.
- All failures, timeouts and missing observations remain visible.
- The evaluator does not import production orchestration or capacity-transition helpers.
- Synthetic, sandbox, operator-observed and independently reproduced results remain separate.
- No SOTA, performance, certification, partnership or production-readiness claim is made without the evidence required for that claim.




## 13. Global interoperability: MCB-GLOBAL

The global compatibility corpus is defined in the separate Global Interoperability Profile workstream: Mycelix #5945, with 30 declarative cases in `global/global-compatibility-cases-v0.1.json`.

Globality is not measured by the number of country flags, translated screens or provider logos. It is measured by whether the protocol preserves meaning across different identifier schemes, currency metadata, time zones, languages/scripts, physical units, jurisdictional authority, data-governance policies, registry maturity and connectivity conditions.

### Comparison dimensions

Report independently:
- exact profile coverage and correctness, with NotTested/Unsupported/NotApplicable visible;
- jurisdiction-pack source freshness and review/approval status;
- identifier subject and issuer linkage correctness;
- monetary scale, rounding and FX evidence correctness;
- time-zone/calendar conversion determinism and historical reconstruction;
- unit dimension and conversion correctness;
- language/script rendering, translation provenance and comprehension;
- public-sector authority boundary (private stay versus MSW/PCS submission/acknowledgement/clearance);
- data-residency/retention policy enforcement;
- low-bandwidth and offline provider continuity;
- time/person-hours to add a country pack, language, identifier scheme, currency profile and standards adapter.

### Global hard gates

These seven additional gates are non-compensatory:
- zero identifier subject misbindings;
- zero currency scale errors;
- zero unit dimension errors;
- zero silent daylight-saving gap/fold resolution;
- zero unreviewed translation promoted to authoritative terms;
- zero local data-governance or cross-border-transfer policy bypass;
- zero stale or pre-release registry record promoted to production/current.

They compose with the existing safety gates. A global profile PASS is not a national legal compliance certificate, regulatory acceptance, government adoption or field interoperability claim.

### Starter regional research coverage

Use Southern Africa, Europe/EEA, North America, Gulf/Middle East, South/East/Southeast Asia, Latin America, Caribbean/SIDS and Pacific/Oceania as research coverage groupings only. They are not single legal jurisdictions. Country/port packs remain unapproved until exact sources, effective dates, authority scope, translations and local review evidence are recorded.

## 14. Standards-profile conformance: coverage is not correctness

Use a separate conformance family inspired by DCSA's public conformance framework. Measure two different things:

- **Coverage:** which requirements in the exact pinned profile, edition and implementation role were exercised.
- **Correctness:** whether each tested requirement is correct, incorrect or inconsistent.

Use explicit per-requirement statuses: Correct, Incorrect, Inconsistent, NotTested, Unsupported and NotApplicable. NotTested is not PASS. NotApplicable requires a reason. A result binds exact profile/version, role/direction, requirement ID, scenario and evaluator.

DCSA Port Call 2.0 is a useful reference because its conformance tooling exercises publisher GET and subscriber POST roles against synthetic counterparts. Its framework reports coverage and correctness separately. Maritime Commons may reuse that *method*, but must not claim DCSA certification or confuse commercial port-call coordination with recreational berth/stay coordination.

Apply the same precision to IHO adapters: pin product-spec edition, S-100 dependency, product validation profile, source dataset and operational applicability independently. Never collapse these into a generic compliant boolean.

This benchmark adds three synthetic cases: complete DCSA-profile conformance, intentionally incomplete/untested profile coverage, and semantic-equivalence checks between native and normalized event adapters.

### Ratio and evidence rules

For one exact pinned profile/role, define:

- Applicable requirements = total requirements minus justified NotApplicable requirements.
- Tested requirements = Correct + Incorrect + Inconsistent.
- Coverage = Tested / Applicable.
- Correctness = Correct / Tested.

NotTested and Unsupported are not correct and remain visible as uncovered requirements. If Tested is zero, correctness is undefined, not 100%. NotApplicable requires a reason and must be excluded before calculating coverage. Report raw counts alongside ratios. Do not publish a single scalar "compliant" score.

### Primary public sources

- Dockwa product and vendor-reported network metrics: https://marinas.dockwa.com/about-us (page states last updated 1 October 2026).
- Navily product and vendor-reported community metrics: https://www.navily.com/ (retrieved 8 November 2026).
- Berth / BerthApp platform terms and provider-contract boundary: https://www.berthapp.com/legal/terms (terms updated 2 October 2025).
- Dockwa administrator-enabled webhook integration: https://mayday.dockwa.com/en/articles/17710897-set-up-webhooks-in-dockwa.
- DCSA Port Call 2.0 OpenAPI: https://reference.dcsa.org/content/standards/releases/port-call/v2/pc-v2-openapi-specification.
- DCSA API Design and Implementation Guidelines 2025.Q4: https://reference.dcsa.org/content/standards/guidelines/api-design-and-implementation-guidelines.
- DCSA conformance overview and scenario tooling: https://reference.dcsa.org/content/standards/guidelines/conformance and https://reference.dcsa.org/content/standards/releases/port-call/v2/pc-v2-conformance-scenarios.
- CloudEvents 1.0.2 specification: https://github.com/cloudevents/spec.


## 15. Monetary interoperability: MCB-MONEY

The monetary profile and fixture corpus are tracked in [MARITIME-MONEY-INTEROP-001 #5948](https://github.com/Luminous-Dynamics/mycelix/issues/5948), with the draft profile in [PR #5949](https://github.com/Luminous-Dynamics/mycelix/pull/5949).

The 18 MCB-MONEY cases test:
- currency exponent zero, two and three as positive controls;
- historic currency codes and metadata revision;
- same integer under different scale metadata;
- locale presentation that must not mutate canonical values;
- quote versus settlement currency;
- FX direction, source and freshness;
- credits/discounts and signed line-item semantics;
- refund request/attempt/finality distinction;
- cash rounding versus accounting scale;
- overflow and unsupported scale;
- unknown currency codes;
- mandatory fee mutation after acceptance;
- tax-inclusive/exclusive mapping loss.

### Money hard gates

Four additional non-compensatory gates apply:
- zero authoritative monetary operations using binary floating-point;
- zero implicit FX conversions;
- zero silent monetary rounding or overflow;
- zero refund finality inferred without recognized external evidence.

The ISO 4217 metadata snapshot and digest must be pinned for every execution. The exponent-zero/two/three fixtures are selected from that exact artifact rather than a remembered table. CLDR locale formatting is not accounting metadata.

This is a compatibility review, not a change to Stay Protocol v0.1. MCB-MONEY does not qualify tax treatment, financial licensing, bank settlement, refund completion or an FX execution service.
