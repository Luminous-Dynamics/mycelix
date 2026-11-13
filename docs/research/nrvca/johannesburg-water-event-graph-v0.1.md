# Johannesburg Water Evidence Event Graph v0.1

## Purpose

This package adds an append-only identity and temporal-lineage layer above the metadata-only source register. The unit of ingestion is a **source assertion** (one publisher statement or image-backed notice), not a presumed physical incident. Event groups are explicit, provisional or qualified interpretations whose memberships and relationships remain inspectable.

The v0.1 fixture is deliberately small and auditable. It covers:
- two 5 November 2026 Lonehill burst notices at 12:20 and 20:50, grouped as one provisional reported incident with a directional `updates` edge;
- the 2 November High Level / Hospital Hill notice as one recurring operating schedule, not a manufactured series of observed outages;
- three separate system-capacity snapshots on 3, 4 and 5 November, kept as separate source assertions with metric comparability explicitly unestablished.

The fixture references the existing source register by its exact Git blob SHA-1. That is a Git object identifier, **not** a SHA-256 digest of source bytes. The existing register's source media SHA-256 values remain null.

## Paradigm shift: from outage rows to evidence graph

A conventional flat incident table often conflates four different things:

1. a physical condition in the network;
2. the time at which an operator reported it;
3. a later notice that updates or continues the report;
4. a measured customer-service effect.

This contract keeps them separate. It models:
- **assertions**: immutable publisher-specific claims with exact source URI, notice class and raw time wording;
- **events**: provisional groupings of one or more assertions about a reported incident, recurring schedule, system snapshot or security activity;
- **relations**: typed, directional edges such as `updates`, `continuation_of`, `contradicts` and `supersedes`;
- **valid-time semantics**: unknown, report time, snapshot time, planned interval or recurring schedule;
- **scope**: raw operator geography/asset labels until geometry and topology are independently qualified;
- **impact/restoration status**: reported-only unless instrument or independent observations support a stronger claim.

No string-matching model is allowed to erase provenance or force a merge. A model may propose a relation; a human or independent evidence process must adjudicate it.

## Exact example interpretations

### Lonehill: two notices, one provisional incident

The 12:20 and 20:50 notices both identify the burst at Winnie Mandela Drive and Mulbarton Road and name the same affected areas. The source register explicitly records that the later notice is an update, not a second incident. The fixture therefore has:
- two distinct assertion IDs and two preserved media URIs;
- one provisional event ID;
- one directional relation from the 20:50 assertion to the 12:20 assertion;
- unknown physical onset, restoration and verified footprint.

This is not a claim that customers experienced one continuous outage. That requires independent service evidence.

### High Level / Hospital Hill: one rule, not repeated observed incidents

The 2 November notice announces daily 14:00–04:00 closures from 3 November until further notice. The graph stores one `scheduled_operating_rule`. It does not generate an outage row for each day, infer that the closure executed, or infer the rule's termination date.

### City capacity: three snapshots, not a calibrated trend

The notices report 45% (3 November), 52% (4 November) and 50% (5 November). Their metric definition, denominator, method and underlying telemetry are not supplied in the source register. Each is retained as a separate snapshot with `comparability_status=not_established`. The values must not be plotted as a calibrated time series or used to estimate service recovery.

## Interoperability: reuse standards, do not replace them

- **OGC WaterML 2.0 / ISO 19156 Observations & Measurements**: use for qualified hydrological observations and time-series exchange. Keep publisher-assertion metadata separate until an observation meets measurement requirements.
- **OGC HY_Features**: use for authoritative hydro-feature identity and network relationships when the asset owner or qualified spatial authority supplies them.
- **W3C PROV-O**: map assertion derivation, attribution, revisions and provenance edges to standard provenance concepts.
- **ETSI NGSI-LD / SAREF4WATR / Smart Data Models**: consider for context-graph exchange once local meanings, identity rules and governance are mapped. Do not treat a broker as proof of measurement quality.
- **Existing EPANET/WNTR and owner hydraulic models**: remain the hydraulic computation engines. This package qualifies the evidence that enters them; it does not calculate pressure, flow, tank level, service area or hydraulic state.

Reference points:
- OGC WaterML: https://www.ogc.org/standards/waterml/
- EU Rolling Plan 2026, Water Management Digitalisation: https://interoperable-europe.ec.europa.eu/collection/rolling-plan-ict-standardisation/water-management-digitalisation-rp-2026
- W3C PROV-O: https://www.w3.org/TR/prov-o/
- AWWA 2026 survey, *Model or Digital Twin?*: https://doi.org/10.1002/awwa.70075
- OGC 26-050, *Building Blocks for Interoperable, AI-Ready Geospatial Data* (12 August 2026): https://docs.ogc.org/techpaper/26-050/26-050.html

## Industry and SOTA comparison: compete at the missing layer

### What established platforms already do well

Bentley describes OpenFlows WaterSight as integrating SCADA, GIS, hydraulic models, IoT, AMI/CRM and customer analytics into a live water-network environment. Its stated scope includes anomaly monitoring, burst/leak geolocation, pump optimization, water balances and emergency-response scenario simulation. OpenFlows Water provides steady/extended-period hydraulic analysis, calibration against measured field data, scenario comparisons and transient analysis.

That is a substantial operational-engineering stack. We should not claim to outperform it on live utility integrations, hydraulic-model maturity, asset coverage, calibrated forecasts or deployed response capability. The correct approach is interoperability and qualification around existing owner/vendor systems.

### Capability boundary

| Capability | Established utility / commercial stack | Luminous target layer |
|---|---|---|
| Hydraulic computation | EPANET/WNTR and commercial solvers; owner models | Reuse the qualified solver; no duplicate hydraulic engine |
| Live SCADA/GIS/AMI integration | Mature commercial capability; depends on utility configuration and data access | Permissioned source adapters with explicit source/time/unit/provenance contracts |
| Leak/burst analytics | Commercial monitoring and anomaly/geolocation features | Do not replace leak models; qualify the evidence linking notice, event, asset, observation and repair |
| Event identity | Often embedded in vendor/operator workflows; exact semantics vary by implementation | Explicit append-only assertions, typed relations, versioned identity decisions and conflict preservation |
| Temporal semantics | Operational time-series and event records available where instrumented | Keep publication time, physical valid time, observation time and ingestion time separate |
| Evidence quality | Calibration/model governance available in mature deployments; varies by source | Explicit evidence class, calibration/timebase/coverage/rights status and fail-closed abstention |
| Community service floors | Usually requires local utility policy and separate service-equity analysis | Non-compensatory household/community service vectors and no hidden averaging |
| Governance / sovereignty | Tenant and vendor controls; utility-specific data agreements | Open, federated and rights-aware evidence exchange; no automated allocation or actuation |

### What current field research says

- The AWWA *Model or Digital Twin?* article (first published 15 April 2026) summarizes a 2024 water-utility survey. It reports interest in AI, digital twins and predictive analytics, while many utilities still lacked well-defined implementation plans. This supports a staged, interoperable adoption path rather than declaring a full city twin from public notices.
- The European Commission's 2026 Water Management Digitalisation plan identifies open semantic models and linked-data approaches, including WaterML 2.0, HY_Features, SAREF4WATR, Smart Data Models and ETSI NGSI-LD. It also identifies technological, economic, social and regulatory barriers, including cybersecurity and citizen empowerment.
- Bentley's product descriptions establish a real commercial comparator for connected hydraulic operations; they do not establish independent accuracy or guarantee performance for a particular utility. Vendor capability statements are not independent benchmark results.

### Defensible differentiation hypothesis

Luminous can attempt to outperform incumbent stacks on *evidence integrity per unit of integration cost*, not by pretending to have more telemetry or a better hydraulic solver. Candidate differentiators are:
1. a cross-vendor, source-role-preserving evidence graph that never rewrites the operator's original assertion;
2. deterministic event lineage and temporal reasoning with measured false-merge / false-split tradeoffs;
3. explicit coverage, rights, calibration, method, topology and uncertainty receipts at every boundary;
4. safe abstention when identity, geometry, event time or service outcome is unresolved;
5. non-compensatory service floors, so city-wide averages cannot hide a community's water, sanitation or energy shortfall;
6. low-cost staged adoption that starts from public/permissioned evidence and only progresses to live shadow mode after independent validation.

This is a hypothesis to test, not a present product advantage. Demonstration requires matched source populations, measured integration cost, adjudicated event links, independent observations and external review.

Primary references:
- Bentley OpenFlows WaterSight: https://www.bentley.com/en/products/openflows-watersight/
- Bentley OpenFlows Water: https://www.bentley.com/products/openflows-water/
- Chenevey & Cooper (2026), *Model or Digital Twin? AWWA Survey on Water Distribution Technology*: https://doi.org/10.1002/awwa.70075
- European Commission, *Water Management Digitalisation — Rolling Plan 2026*: https://interoperable-europe.ec.europa.eu/collection/rolling-plan-ict-standardisation/water-management-digitalisation-rp-2026

## Benchmark design

Do not benchmark this layer with hydraulic-model RMSE or leak-detection accuracy. Its outputs are different. Use the following qualification tracks:

| Track | Measure | Proposed gate |
|---|---|---|
| Event identity | Pairwise precision/recall and B-cubed F1 against a blinded, adjudicated event-linkage set | Report all metrics; target precision >= 0.995 before automatic grouping; no automatic merge for high-consequence ambiguous events |
| False merges | Distinct physical incidents incorrectly merged | Zero on the critical adjudication set |
| False splits | One incident represented as several event groups | Report separately; optimize only after false-merge constraint is met |
| Temporal lineage | Publication/report, valid/event and observation times silently conflated | Zero |
| Schedule semantics | Announced recurrence converted into unobserved actual events | Zero |
| Evidence ceiling | Synthetic, publisher-only, stale or unknown evidence promoted to measured/independently verified | Zero |
| Spatial claims | False-positive assignment to a service polygon/asset | Zero on the manually adjudicated reference set; report coverage and abstention separately |
| Append-only history | Earlier assertion overwritten or relation history lost | Zero |
| Reproducibility | Same pinned inputs + code revision produce different canonical graph output | Zero unexplained differences |

The 0.995 precision value is a proposed engineering target, not an observed result or an industry standard. Require a sufficiently large, representative adjudication set and confidence intervals before claiming that it has been met. Publish abstention and coverage beside accuracy; a system that refuses to guess on ambiguous links is preferable to a confidently incorrect merge.

## Benchmark populations and baseline comparisons

1. **Rule-only baseline**: exact publisher/media ID plus explicit source identifiers; no probabilistic matching.
2. **Conservative deterministic linkage**: exact reported asset/location plus compatible notice class and publication window; ambiguous candidates abstain.
3. **Probabilistic record linkage**: evaluated only as a candidate proposer, with calibrated probability and human adjudication.
4. **Temporal graph approach**: typed events, bitemporal semantics, relation graph and conflict preservation.

Use event-level, source-publication-level and geographic holdouts. Prevent near-duplicate image pages or continuation notices from crossing train/test splits. Compare cost per correctly linked event, false merges per 1,000 assertions, abstention rate and adjudication time. No performance claim is valid until a source-pinned, rights-cleared, manually adjudicated corpus exists.

## Phased benchmark targets

- **T0 — contract correctness**: 100% of frozen known-answer fixtures preserve exact source IDs, URIs, raw timestamps, event memberships and directed relations; all negative controls fail closed.
- **T1 — adjudicated notices**: create a rights-reviewed, manually adjudicated event-linkage set from at least 500 assertions across multiple notice families and at least two independent reviewers. Measure inter-rater agreement before computing model accuracy.
- **T2 — cross-source linkage**: only after T1, evaluate operator notices against authorized CMMS/work-order, SCADA and customer-contact evidence. Keep source-role-specific claims visible.
- **T3 — physical observation**: only instrumented/field-approved data can qualify actual onset, service interruption, restoration or hydraulic impact. Preserve calibration, timebase, topology, sampling and consent receipts.
- **T4 — federated deployment**: separate utility tenants, source-specific access controls, redaction and audit logs; no cross-tenant entity resolution without explicit governance.

## Safety and governance invariants

- No household-level inference from a suburb or supply-system label.
- No automated service disconnection, valve/pump actuation, water allocation or enforcement.
- No inference that migration, relocation or immobility is preferred from a risk or capacity score.
- No redistribution of source images or full notices until rights are assessed.
- No promotion from publisher statement to measured/verified service without the required independent evidence.
- No destructive "latest update wins" merge. Conflicts and corrections remain queryable.

## Known limitations and next work

The current source register contains nine publication records and 25 image-backed notice reviews, but media bytes are not pinned, rights are not assessed, suburb/street labels are not verified geometry, and the dataset is not an exhaustive incident feed. The six assertion fixture rows are a demonstrator, not a statistical benchmark.

Next implementation steps:
1. Add parser + graph validator in the existing Mycelix evidence-ledger Rust package; preserve v0.1 raw source register.
2. Add negative controls: same suburb/different asset, same asset/different date, continuation without new event, contradictory ETR, recurring schedule, stale update, duplicate media, missing source bytes and conflicting reports.
3. Build two-reviewer adjudication protocol and a rights-aware assertion corpus.
4. Add source adapters only for authorized work orders, telemetry and customer-service observations.
5. Map qualified observations into WaterML 2.0 and qualified assets into HY_Features; retain original source assertions and provenance.

## Evidence status

- JSON syntax and graph referential invariants are checked during authoring.
- Full JSON Schema validation, Rust compilation, tests, Clippy, formatting and hosted CI have not been run in this environment.
- No hydraulic, sensor, customer-service or restoration result is qualified by this artifact.


## Negative-control corpus added

The branch also contains `johannesburg-water-event-graph-negative-controls-v0.1.json` with 13 deterministic test vectors: three source-register-backed cases and ten synthetic adversarial controls. They cover update-vs-new-incident linkage, same-suburb/different-asset false merges, stale and ambiguous identity, recurring schedules, non-comparable snapshots, unknown event onset, contradictions, append-only supersession, security notices, unknown restoration, operator-reported restoration and unverified geography.

Custom authoring checks confirm all three observed-fixture assertion references resolve and all 13 case IDs are unique. These are test specifications only; no implementation test has been run or passed.
