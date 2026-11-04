# Industry benchmark alignment and domain scorecards (v0)

**Purpose:** map Evidence Plane's proposed evaluation protocol to credible external practice. This document is research guidance; it is not a certification claim, a completed benchmark, or evidence of superior performance.

## Executive conclusion

There is no single cross-domain SOTA number for this work. Microgrid controller testing, machine-learning systems, spatial-data pipelines and scientific assessment-table transcription measure different things. The infrastructure should adopt each domain's strongest relevant methods and enforce an evidence/receipt layer consistently across them.

A serious comparison must show a **frontier**, not just one scalar: outcome quality versus cost, reliability versus resource use, and usefulness versus constraint violations. Constraint violations are hard gates, not tradeable penalties.

## Industry references verified 2026-10-23

### Microgrid controller evaluation

- [IEEE 2030.8-2018: Standard for the Testing of Microgrid Controllers](https://standards.ieee.org/ieee/2030.8/6169/) defines test procedures to verify and quantify controller functions and compare performance against requirements.
- [IEEE P2030.8](https://standards.ieee.org/ieee/2030.8/12543/) is an active revision project, approved on 2026-03-26, superseding 2030.8-2018. Track this revision, but do not claim compliance with a draft or a standard we have not actually tested against.
- [NREL/NLR, Hardware-in-the-Loop Test Bed and Test Methodology for Microgrid Controller Evaluation](https://www.nrel.gov/docs/fy20osti/71338.pdf) describes controller- and power-hardware-in-the-loop evaluation with grid-connected, separation, islanding and reconnection scenarios.
- [NLR Agora large-load test bed](https://nrel.sitefinity.cloud/grid/facilities/large-load-test-bed) demonstrates industry practice of emulating facility/grid interactions and evaluating equipment and controls before field deployment.

**Implication for Symtropy:** start with the published controller test families where conceptually applicable, then add Antarctic-specific contingencies rather than treating generic microgrid tests as sufficient: polar-night resource scarcity, storm derating, generator/ inverter loss, low battery state of charge, cold-related derating, delayed resupply, uncertain fuel delivery, sensor drift, controller restarts and communication loss. Separate software simulation from controller-HIL or field results.

Do not claim IEEE 2030.8 compliance or equivalent safety from an offline simulator. A useful staged evidence progression is: deterministic model tests → Monte Carlo/robust simulation → SIL/PIL/CHIL/HIL as available → witnessed site data and independent domain review. The applicable test environment and required acceptance limits must be justified by the engineering owner.

### Benchmark governance and systems performance

- [MLCommons benchmark programme](https://mlcommons.org/benchmarks/) explicitly emphasizes fair comparisons, reproducibility, commercial/research usefulness and affordable participation.
- [MLPerf Training rules](https://github.com/mlcommons/training_policies/blob/master/training_rules.adoc) describe fixed benchmark definitions and rules for fair runs, consistent systems and multiple trials.
- [MLPerf Endpoints](https://mlcommons.org/benchmarks/endpoints/) publishes a performance/quality frontier with multiple configuration points; each configuration has quality requirements and status labels such as verified, provisional or unverified. The details are endpoint-specific and should not be transplanted as if they were energy-system standards.

**Implication for Symthaea and the shared verifier:** build small, reproducible public baselines; disclose tuning and compute; retain per-run raw outputs and failures; report uncertainty; require a minimum quality/safety gate before comparing speed/cost; and label every published result by verification state. Do not optimize a benchmark-only path or compare a heavily tuned candidate to an untuned baseline.

### Research packaging and provenance

- [W3C PROV-O](https://www.w3.org/TR/prov-o/) is the W3C provenance ontology for interoperable entity/activity/agent descriptions.
- [RO-Crate 1.3 specification](https://www.researchobject.org/ro-crate/specification) was released in 2026 and defines portable packaging/metadata for data, workflows, software and contextual provenance.
- [SLSA provenance](https://slsa.dev/spec/v1.0/provenance) and [in-toto](https://in-toto.io/) can inform software/build-step attestations; they do not certify the truth or reuse rights of scientific inputs.

**Implication:** design receipt fields and export adapters around established concepts. The Evidence Plane schema is an application-specific minimum profile, not a replacement for these standards, and conformance must be tested before claimed.

## Proposed scorecards

### A. Antarctic energy resilience (Symtropy)

| Scorecard dimension | Measure | Required baselines/evidence |
|---|---|---|
| Critical-service delivery | Weighted served critical load; unserved critical-service hours and severity | incumbent/control policy; documented load priorities |
| Fuel/logistics | Fuel per delivered critical service, total fuel, resupply mass/volume, transport assumptions | current practice; diesel-only/reference policy |
| Reliability | outage count/duration, loss-of-load probability where model supports it, recovery time | deterministic and stress-test baselines |
| Resilience tails | worst-case and CVaR-like tail outcome across locked episodes | adversarial weather/outage/resupply scenarios |
| Renewable/storage | curtailment, SOC margin, cycles/degradation proxy, reserve margin | transparent dispatch rules and LP/MPC |
| Safety/operations | independent hard-gate vector, reserve violations, ramp/frequency/voltage criteria where modeled | engineering-approved limits; controller test plan |
| Cost of control | wall time, memory, maintenance complexity, computation/platform power | matched budgets |
| Evidence quality | calibration error, source completeness, forecast error, sensitivity to unknown inputs | holdout sites/seasons and negative controls |

Minimum comparison arms: incumbent/current operation; transparent dispatch rules; deterministic LP/MILP or MPC; robust/stochastic optimization; candidate controller/policy; ablations; and deliberately defective controls. Include a black-start/recovery test when the model and data support one.

Proposed hypotheses (not achieved results):
- ≥10% lower fuel per delivered critical service on locked held-out episodes.
- No increase in critical-service shortfall or safety-gate violations.
- Improvements remain under storm, resupply-delay, forecast-error and component-failure shifts.
- Report the full outcome-cost frontier and uncertainty, not only the best point.

The numerical target must be revised or rejected if source evidence, calibration and sample size cannot support a defensible estimate. A synthetic four-period fixture is not a field-performance test.

### B. Spatial evidence pipeline (Sol Atlas)

| Dimension | Measure |
|---|---|
| Source integrity | exact file-level hash verification, source/media mismatch detection, ingestion state accuracy |
| Metadata fidelity | no unresolved date/unit/citation conflict silently collapsed; explicit source-attributed resolution |
| Spatial semantics | CRS correctness, geometry validity, resolution, temporal support and aggregation scope |
| Claim correctness | reject regional aggregate → station-point promotion and reanalysis → direct-measurement relabeling |
| Reproducibility | identical normalized output digest under pinned inputs/toolchain, or declared tolerance for non-determinism |
| Fault handling | mutation controls for dates, units, CRS, station identity, missingness and source version |
| Performance | ingestion/normalization throughput, peak memory and query/render latency by dataset-size tier |
| User-facing calibration | uncertainty/provenance displayed and accessible; missing data distinguished from zero |

Compare the normalizer/rendering pipeline against a straightforward transparent baseline and at least one mature geospatial toolchain for selected transformations. Validate against independent reference points/known geometries and explicit known-invalid files. Measure each transformation separately; do not infer overall accuracy from visual plausibility.

### C. Scientific assessment transcription and model claims (Symthaea)

| Dimension | Measure |
|---|---|
| Transcription accuracy | exact field/value/unit match against source tables; per-field mismatch class |
| Semantic fidelity | correct F95/F50/F5/mean definitions, risk semantics and assessment-unit identity |
| Byte-level provenance | source and parsed artifact hashes; version and citation lineage |
| Reproducibility level | table transcription vs. input parsing vs. full algorithm/model reproduction explicitly distinguished |
| Independent review | blinded second transcription or independent parser; inter-reviewer disagreement resolved by evidence |
| Adversarial integrity | wrong quantile/unit/total/probability/source version reliably rejected |
| Claim discipline | no promotion to discovery, reserves, economic viability or authorization without separate evidence |

For the Larsen Basin thread, use the [official 2026 USGS release](https://www.usgs.gov/data/usgs-national-and-global-oil-and-gas-assessment-project-larsen-basin-antarctica-assessment), which supplies assessment-unit boundaries, input data and machine-readable fact-sheet result tables, with declared CC0 status. The current work remains a transcription audit until raw artifacts are acquired and validated; it is not a reproduction of the geological assessment model.

## Shared evidence-plane gates (all domains)

1. Schema and receipt structural validity.
2. Actual artifact SHA-256 and byte-count verification.
3. Exact code commit and transform/toolchain identity.
4. Source stewardship/terms/contact state resolved for the intended use.
5. Data split and benchmark protocol locked before test outcome inspection.
6. Matched budgets/information with baseline version pinned.
7. Mutation suite passes known material positive/negative controls.
8. Independent verification has a distinct identity and scope.
9. Safety/domain gate is satisfied for the specific claim and operating envelope.
10. Output claims do not exceed the evidence tier.

A higher numerical score cannot compensate for any failed non-tradeable gate. Missing, pending, skipped or unknown is not pass.

## What would qualify as a real SOTA result?

- Name the task, operating envelope and primary outcome before measuring.
- Select current strongest relevant baseline(s), cite versions and freeze the protocol.
- Give candidate and baselines the same allowed information, data cutoff, tuning/compute budget and failure rules, or narrow the claim accordingly.
- Evaluate independent site/time/event holdouts, distribution shifts and adversarial cases.
- Publish raw per-run outcomes, failures, uncertainty and the outcome-cost frontier.
- Reproduce results independently and disclose material limits.
- For safety-critical energy control, use appropriate controller-HIL/field evidence before making deployment or safety claims.

Until those conditions are met, call the result exploratory, a software-conformance improvement or an internal benchmark result—not state-of-the-art.
