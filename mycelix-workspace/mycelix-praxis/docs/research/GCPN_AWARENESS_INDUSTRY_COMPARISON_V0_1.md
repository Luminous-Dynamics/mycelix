# GCPN — Matched civilian drone-awareness comparison v0.1

**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child:** [GCPN-AWARENESS-INDUSTRY-COMPARISON-001 #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Related awareness contract:** [#5934](https://github.com/Luminous-Dynamics/mycelix/issues/5934)  
**Status:** synthetic comparison protocol; no vendor performance result claimed  
**Review date:** 8 November 2026  
**Claim ceiling:** comparison-manifest integrity and synthetic track semantics only

## 1. Purpose

A credible comparison must distinguish four things that vendors and buyers can otherwise blur:

1. A physical sensor's detection capability.
2. A multi-sensor fusion or command-and-control (C2) platform.
3. A downstream evidence/provenance and human-review workflow.
4. An active mitigation/effector capability.

This benchmark compares only the first three, and keeps sensor performance separate from software evidence handling. Active mitigation, effectors, operator geolocation, third-party control, targeting and engagement are excluded.

## 2. Paradigm shift: one “leaderboard” is the wrong abstraction

A single leaderboard would be misleading because an RF-only sensor, multi-sensor C2 platform and provenance/triage layer do not receive the same input or solve the same task.

We therefore define two independent tracks:

- **Track A — common-event-envelope replay:** every downstream evidence/triage workflow receives the exact same frozen, privacy-minimized event envelopes. This evaluates evidence provenance, freshness, disagreement, source dependence, abstention, operator-review output and privacy. It does not assess physical sensor detection.
- **Track B — physical sensor performance:** each sensor/C2 system is evaluated against the same independently adjudicated ground-truth event set and declared observation windows in an authorized, benign, controlled environment. Sensor inputs are modality-specific and are not described as identical. This track is blocked until site, aviation, radio, vendor, privacy, professional and insurance approvals are complete.

Never combine the results into one score. A better event-envelope consumer cannot claim to have a better radar/RF/optical sensor; a sensor with a higher detection score does not automatically provide better provenance, privacy or human decision support.

## 3. External evaluation reference: CWA 18150:2024

CWA 18150:2024, *Unmanned aircraft systems — Counter UAS — Testing methodology*, is a directly relevant external methodology reference. It is a CEN Workshop Agreement, not an official CEN standard: its own foreword states that it cannot be held as an official standard developed by CEN and its Members. Its published scope describes user-defined scenarios and qualitative/quantitative comparison of detection, tracking and identification systems integrated in a C2 system; the NEN scope summary says defeat/disable aspects are not covered. We use it as a method reference only, not as a certification, harmonized-standard or compliance claim. Source: https://www.cencenelec.eu/media/CEN-CENELEC/CWAs/ICT/cwa-18150.pdf (foreword, pp. 1–2) and https://www.nen.nl/en/cwa-18150-2024-en-330385.

- Current status and scope reference: https://www.nen.nl/en/cwa-18150-2024-en-330385
- CEN's explanation of CEN/CENELEC Workshop Agreements and their role/status: https://www.cencenelec.eu/get-involved/research-and-innovation/workshop-agreements-cwa/
- Adoption/status reference: https://dps.gov.al/en/project/show/dps%3Aproj%3A76336

The full CWA text is copyrighted and is not reproduced. Obtain authorized access and a standards specialist's review before claiming any conformity. This Luminous protocol is **not** represented as CWA-conformant.

Additional research context:
- Han & Jung (10 February 2026), time-synchronized multi-sensor radar/RF drone dataset: https://www.nature.com/articles/s41597-026-06802-6
- Johnson, Moore & Porter (September 2026), EO/IR, radar, acoustic, LiDAR and RF systems survey: https://doi.org/10.1016/j.ijcip.2026.100870
- ISO/TR 23267:2024 concerns UAS detect-and-avoid testing; it is relevant to aviation DAA where applicable, not a substitute for a ground-site awareness evaluation: https://www.iso.org/standard/87386.html

## 4. Public capability baselines (not measured results)

This initial set is a capability map, not an exhaustive market survey. Every statement is labelled a public vendor claim. Current product releases, firmware, options, exact configuration and data-export semantics must be pinned before a scored trial.

### Baseline D1 — Dedrone RF-360

Vendor describes RF-360/RF-360T as passive, network-attached RF sensors for drone detection, classification and localization. Its public page advertises remote-control/drone location and up-to-5-km direction finding. The advertised distance is a vendor claim with its own line-of-sight/condition caveats; it is not a result in this benchmark.

- Product: https://www.dedrone.com/sensors/rf-360
- Vendor/brand reference: https://www.axon.com/resources/dedrone-rf-360-sensor
- Scope for this benchmark: redacted, passive awareness event only.
- Excluded from benchmark data: operator identity/location, precise drone/remote-control geolocation, private communications and any active effectors.
- Important: the vendor's “no legal authorization required” marketing text is not a South African legal opinion.

### Baseline D2 — DroneShield DroneSentry-C2

Vendor describes DroneSentry-C2 as a multi-sensor C2 platform that combines RF, radar and optical feeds, with SensorFusion and third-party sensor integrations. Its 2026 releases describe interoperability, alerting and operator workflow improvements. The platform ecosystem also supports disruptors/effectors; those capabilities are outside this benchmark.

The Q2 2026 vendor release describes automatic Friendly/Neutral/Hostile/Unknown labels using serial-number/Remote ID inputs. A vendor-provided Hostile label is **not** ground truth, verified intent or authority in this civilian comparison. Our event model separates observation, identity claim, authentication, authorization and intent. A newer Q3 2026 vendor release dated 6 July 2026 reports claimed RF-sensing improvements versus Q2 (with results said to depend on drone type and operating frequency), plus air-gapped updates, offline Cloud Optimized GeoTIFF maps, expanded third-party sensor interoperability and additional language support. These remain vendor-reported claims; exact customer release/configuration and underlying test data must be pinned before independent evaluation.

- Product: https://www.droneshield.com/products-software
- Q1 2026 release: https://www.droneshield.com/media/press-releases/droneshield-announces-q1-2026-software-enhancements
- Q2 2026 release: https://www.droneshield.com/media/press-releases/droneshield-advances-decision-advantage-with-q2-2026-software-release-as-drone-threats-scale-globally
- Q3 2026 release (6 July 2026): https://www.droneshield.com/media/press-releases/droneshield-q3-software-release-sharpens-counter-uas-performance-against-faster-more-evasive-drone-threats
- Scope: passive detection/awareness event feed and evidence presentation only.
- Excluded: disruptor state/command, effector APIs, hostile labels as truth, target/engagement records and exact flight/operator tracks.

### Baseline D3 — Sentrycs DTI

Sentrycs describes its DTI module as passive RF-based detection, tracking and identification. Its product page separately describes optional mitigation functions. It also advertises operator last-known location and flight parameters; those data are outside this civilian benchmark.

- Official technology page: https://sentrycs.com/our-technology/
- Scope: permissioned, redacted awareness event feed only.
- Excluded: operator identity/location, exact tracks/flight parameters, all mitigation/takeover functions and private communications.

### Baseline D0 — existing manual/site workflow

Each participating site must document its current lawful manual observation, reporting and review process before any comparison. Do not assume this is free: include staff time, training, communication, recordkeeping, maintenance and incident review.

**No vendor has been independently tested by this package. No detection rate, range, false-alert rate, latency, cost or superiority result is asserted.**

## 5. Track A — common-event-envelope replay

### Question

Given exactly the same frozen, privacy-minimized event envelopes, does a downstream workflow improve evidence quality, reduce duplicate/unsubstantiated alerts and review burden without losing relevant events or violating privacy/authority constraints?

### Baselines

- A0: schema-only parser and deterministic rule baseline.
- A1: existing site/incumbent event-review workflow, if the site permits export of equivalent minimized event envelopes.
- A2: Luminous Mycelix provenance + Symthaea advisory triage.
- A3: another vendor-neutral reference consumer, if available.

A vendor-specific adapter is not a vendor endorsement. If a system cannot export a comparable event envelope, mark it unavailable/incomparable rather than manufacturing a substitute.

### Metrics

- schema and provenance completeness;
- identity claim vs authentication vs authorization separation;
- calibration and source-health validity;
- event time vs receipt time and stale-source detection;
- missingness and disagreement preservation;
- common-mode dependency inventory completeness;
- correct abstention / selective risk;
- duplicate and unsubstantiated alert burden;
- human-review latency p50/p95 and review minutes per case;
- privacy minimization and protected-data violations;
- offline reconciliation and correction lineage;
- deterministic replay reproducibility.

### Candidate improvement hypothesis

After establishing the incumbent baseline and preregistering a held-out set, test whether the evidence layer can reduce duplicate/unsubstantiated alert burden and human-review workload by at least **20% relative** without degrading semantic-state conformance, provenance completeness or required contradiction preservation, and without increasing privacy, safety or authority violations. Track A does not measure physical event recall; that belongs exclusively to Track B and its separate independent ground-truth protocol.

This is a falsifiable research target, not a result. If the baseline performs better, report that.

## 6. Track B — physical sensor performance (blocked)

This is a future, separately authorized test. It must use the same independent ground-truth event set and observation windows, with each physical sensor receiving its own modality-appropriate input. Do not claim identical raw inputs across RF, radar, optical and acoustic systems.

Required reporting:
- event-level precision/recall with an independent ground-truth denominator;
- false alerts per monitored hour and site-day;
- unknown/ambiguous and missed cases;
- correct abstention;
- per-modality/environmental-stratum performance;
- sensor uptime, calibration and stale-input rate;
- detection-to-human-review latency p50/p95;
- operator workload and independent adjudication time;
- privacy incidents and retention violations;
- fully loaded lifecycle cost per monitored site-hour.

No universal numeric sensor-performance target is set before the site, product/configuration, legal scope, ground-truth protocol and professional review are fixed.

## 7. Matching contract

Every scored comparison must bind:
- exact site/jurisdiction profile, generalized in public outputs;
- exact product, hardware, firmware, software, subscription and configuration;
- event-set version, ground-truth adjudicator and eligible denominator;
- modality coverage and missingness;
- observation window and environmental/context strata;
- operator training, staffing and permitted human-review budget;
- event matching/deduplication policy;
- source rights, privacy and retention policy;
- lifecycle cost, currency and base year.

If these cannot be matched, disposition is INCOMPARABLE. Do not create a blended leader board from incomparable tracks.

## 8. Data minimization and public reporting

Do not collect or export:
- operator identity or last-known location;
- exact flight paths, coordinates, bearings, altitude/speed trajectories or tactical geometry;
- raw audio or intelligible speech;
- identifiable bystander video, face/biometric data or private communications;
- sensitive facility coordinates or restricted airspace details without explicit rights.

Track A receives only approved event IDs, generalized context, source provenance, declared uncertainty and authorized review outcome. Track B ground truth remains in a controlled, access-approved environment with documented retention/deletion.

## 9. South African legal and professional gates

ICASA's published pages state that equipment type approval may apply to radio equipment, and that section 31(1) of the Electronic Communications Act addresses use of radio apparatus to transmit or receive radio signals subject to spectrum licence, except as legally provided. “Passive” and “receive-only” are not blanket exemptions. Obtain an equipment/frequency-specific written applicability determination from ICASA and qualified South African counsel before import, testing or operation.

- ICASA type approval: https://www.icasa.org.za/pages/type-approval
- ICASA spectrum licensing: https://www.icasa.org.za/pages/spectrum-licensing

South African UAS work must be checked against the operative Civil Aviation Regulations and SACAA category. Government indexes identify GoN 7844 (33rd Amendment) and GoN 7845 (36th Amendment), both in Gazette 55226 dated 21 August 2026. This is source navigation only, not a consolidated legal opinion. Confirm the applicable Part 101 text, commencement, exclusions and site permissions directly with SACAA/aviation counsel.

- Government Gazette index: https://www.gov.za/documents/notices?0=&1=&page=3
- SACAA UAS information: https://www.caa.co.za/industry-information/uas/

Track B requires, at minimum:
- aviation/SACAA specialist and site/airport/port operator;
- ICASA radio/spectrum/type-approval counsel;
- independent CWA 18150 standards/test-method reviewer with authorized access to the text;
- privacy/POPIA counsel and community review;
- independent ground-truth adjudicator;
- site safety, insurance and emergency-management reviewers;
- quantity surveyor/economist for full lifecycle costs;
- vendor/OEM permissions for each participating system.

## 10. Atomic metric contract and denominator discipline

The machine-readable metric dictionary is `GCPN_CIVIL_ATOMIC_METRIC_CATALOG_V0_1.json`. It defines 27 atomic measures spanning physical detection, tracking, identification, observability, common-mode dependence, evidence reconciliation, selective prediction, human workflow and lifecycle economics. It contains definitions only; it contains no vendor scores or results.

Each measure pins its unit, numerator, denominator, admissible truth source, required strata, NOT_EVALUABLE condition and comparison rule. This is necessary because the same headline metric can mean different things when the eligible exposure, target class, coverage, truth source or event-deduplication rule changes.

### Reporting rules

- Detection probability, event precision, false alerts per monitor-hour and false alerts per site-day are separate. Do not substitute wall-clock uptime for qualified observation exposure.
- Track completeness and continuity are based on independent truth intervals and qualified observable windows. A target-time interval outside known coverage is not silently counted as a missed detection.
- Position/velocity error is computed only for matched samples with compatible independent truth, CRS/datum/epoch and time quality. Unmatched samples, gaps and false associations remain separate.
- Identification metrics are per class under a pinned, non-intent ontology. Unknown and abstention remain visible; hostile/friendly intent labels are excluded.
- Coverage-qualified non-observation, false corroboration and dependency correctness are measured separately from sensor recall.
- Human review time, false-review burden and handoff completeness require a controlled/blinded workflow and independent adjudication.
- Lifecycle cost uses the same base currency/year and includes rights, integration, staffing, calibration, maintenance, connectivity, downtime and decommissioning.

### Hard gates and aggregates

Default output is atomic metrics only. No weighted single-number leaderboard is permitted in v0.1. Rights, privacy, authority, false-independence, false-negative-evidence and actuation-boundary violations are non-compensatory: a high detection score cannot offset them. Any future composite would require a separately preregistered protocol with public weights and still could not compensate for a nonzero hard gate.

Track A may compare downstream workflows with identical frozen event-envelope bytes; Track B requires independently adjudicated physical ground truth and remains blocked until legal, site, vendor, privacy, safety and insurance gates are met. Cross-domain summaries must retain modality and domain strata rather than pool incompatible observations.

## 11. Hard acceptance gates

- Track A and Track B results remain separate.
- Every vendor feature is labelled as vendor-public claim until independently measured.
- Same denominators, event frames and operator budgets for paired Track A comparisons.
- Zero active mitigation/effectors, third-party control or autonomous neutralization.
- Zero vendor “Hostile” label promotion to ground truth or action authority.
- Zero operator geolocation or private content in the benchmark data.
- Zero privacy/retention-policy violations.
- No numeric sensor performance, CWA conformity or industry superiority claim without authorized, independent evidence.
- Missing or incomparable evidence remains UNKNOWN/INCOMPARABLE.

The comparison-manifest verifier only establishes manifest integrity and declared synthetic semantics. It does not establish physical detection, intent, legal permission, operational protection or industry superiority.
