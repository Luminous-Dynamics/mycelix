# GCPN Integrated Synthetic Evaluation v0.6

**Status:** integrated synthetic benchmark composition; non-operational.  
**Parent:** issue #5639; predecessor work: forecast integration PR #5634, human-outcomes PR #5636 and forecast-uncertainty PR #5638.  
**Package:** benchmarks/gcpn-integrated-evaluation.  
**Version:** 0.6.0.  
**Dependencies:** eight local Rust benchmark packages; no third-party Cargo dependencies.

## 1. What changed from v0.5

v0.5 composed six independent modules and upgraded probabilistic forecast verification to exact Murphy Brier decomposition plus chronological drift diagnostics.

v0.6 retains the original six gates and adds two separately evaluated local components:
- **Human outcomes:** accessibility after rendering, core-message comprehension, action-intent identification, uncertainty understanding, false-certainty interpretation and worst-profile response-time diagnostics.
- **Forecast uncertainty:** paired event-level bootstrap intervals for candidate-versus-climatology and candidate-versus-lead-reference Brier differences.

Neither module introduces a blended quality score. Human comprehension is not inferred from channel receipts or accessibility alone. Bootstrap intervals are descriptive under an explicit synthetic event_id resampling assumption; they do not prove real forecast superiority or real meteorological independence.

## 2. Eight independent evaluation gates

| Module | Frozen test population | Primary checks |
|---|---:|---|
| Civil record conformance v0.1.1 | 9 synthetic cases | Authority binding, source independence, state preservation, uncertainty disclosure, provenance privacy |
| Topology resilience v0.1 | 11 faults × 3 architectures | Common denominators, competent-authority gate, local-first continuity |
| Last-mile multi-channel v0.1 | 5 receiver cohorts × 10 channel states | Unique reach, accessibility, uncovered cohorts, duplicate channel receipts |
| Warning-to-action v0.1.1 | 11 synthetic cases | Authorized decisions, conditional financing, action chronology, population/service accounting, grievances and impact evidence |
| Forecast verification v0.2 | 65 scoring rows + 90 chronological drift rows | Brier/log loss, exact Murphy decomposition, calibration, subgroup score, rolling windows, CUSUM drift, missing/late/invalid/out-of-domain and partition leakage |
| Lifecycle cost v0.1 | 24 cost rows, 21 matched outcomes | Five-year present value, low/base/high ranges, staffing, restoration, cost per accessible unique receiver |
| Human outcomes v0.1 | 18 synthetic cells; 6 interaction profiles × 3 presentations | Accessible denominator, effective comprehension/action intent, uncertainty understanding, false certainty, worst profile and p90 latency |
| Forecast uncertainty v0.1 | 65 source rows; 12 complete paired event IDs | Deterministic 20,000-replicate paired event bootstrap, two 95% intervals, dependence caveat |

The final integrated reproduction status is a strict conjunction of eight frozen fixture invariants. It is not a weighted quality score, a ranking or a real-world readiness certification.

## 3. Lifecycle-cost module

### Reference assumptions

The cost fixture declares:
- five-year service horizon;
- 5% real discount rate;
- one-time capital and channel setup at year zero;
- annual operations, staffing and maintenance over years one through five;
- scheduled replacement in year three;
- restoration/contingency cost only in the scenario to which it is bound.

Costs are invented synthetic units, not currency. They are not market prices, vendor quotes, procurement estimates or national budgets.

### Cost components

Each architecture has separate line items for:
- capital and integration;
- channel infrastructure/setup;
- operations and connectivity;
- staffing (with annual staff hours recorded separately);
- maintenance;
- scheduled replacement;
- scenario-specific restoration.

Low/base/high values are kept for each row, with parser checks that all ranges are non-negative and monotonic.

### Cost calculation

For amount C at year t, discount rate r, and horizon T:

PV = sum from t=0 to T of C(t) / (1+r)^t

The evaluator applies each component according to its cost pattern:
- one-time: discounted once at the declared year;
- annual: repeated once per service year;
- scheduled: included once in its scheduled year if inside the horizon;
- scenario restoration: included only for the scenario to which it is bound.

Equivalent annualized cost is:

EAC = PV × r / (1 - (1+r)^(-T))

At zero discount rate, the evaluator uses PV/T.

### Staffing units

Annual staff hours and total staff hours over the horizon are reported separately. The all-available baseline profiles use 1,600 / 1,950 / 2,400 staff hours per year for centralized / regional / local-first architectures, respectively. Over the five-year service horizon, the corresponding totals are 8,000 / 9,750 / 12,000 hours. No wage rate or salary is inferred from these hours.

### Accessible-reach denominators

Cost-per-receiver metrics use:
- intended receivers;
- unique receivers reached;
- unique receivers reached through an accessible format.

Duplicate channel receipts never inflate unique reach. When the accessible-receiver denominator is zero, the unit cost is undefined (None), not zero or infinity.

## 4. Synthetic reference results

All services available, five-year horizon, 5% discount rate, base assumptions:

| Architecture | Lifecycle PV | Annualized cost | Accessible unique receivers | Cost per accessible receiver |
|---|---:|---:|---:|---:|
| Centralized | 28,026.38 | 6,473.39 | 1,800 | 15.57 |
| Regional federation | 31,443.19 | 7,262.58 | 1,850 | 17.00 |
| Local-first federation | 38,847.47 | 8,972.79 | 1,900 | 20.45 |

These are synthetic cost units and invented outcomes. The result is intentionally not a recommendation:
- centralized has the lowest base lifecycle cost in this invented profile;
- local-first serves more receivers in an accessible format in the same baseline;
- fault cases change reach and may add restoration cost;
- the authority-unavailable case has positive lifecycle cost and zero reach, so unit cost per accessible recipient is undefined.

The scenario matrix demonstrates why a single baseline cost ratio cannot choose an architecture. A real decision would require locally measured costs, service obligations, geography, channel availability, institutional capability, community preference, procurement constraints and uncertainty analysis.

## 5. Cross-module invariants

### Civil record integrity and authority

No forecast, message signature or channel receipt substitutes for a competent authority. Contradictory, stale, missing and unverified records remain distinct; provenance privacy is preserved.

### Topology

Centralized, regional and local-first references see the same intended receiver denominator for each scenario. Authority unavailability blocks all architectures. Local-first continuity is only counted when the local path and local authority are available.

### Last mile

Unique delivery is separate from channel-receipt volume. Accessible-format reach and uncovered cohorts remain independently visible.

### Warning-to-action

The chain distinguishes authorized warning, protocol decision, conditional finance, action start/completion, unique population served, grievances and independent outcome evidence. GCPN does not activate actions or release funds.

### Forecast verification

Probabilistic forecasts are scored using proper scores, exact Murphy Brier decomposition (reliability, resolution, uncertainty and residual), fixed-bin calibration diagnostics and hazard/lead subgroup results. A second fixture adds stable, gradual-drift and abrupt-shift chronological profiles with rolling five-record windows and a fixed one-sided CUSUM diagnostic. These diagnostics do not set alert thresholds, invalidate live forecasts or authorize warnings. The synthetic drift flags are not operational decisions.

### Lifecycle cost

Costs, staffing, reach and accessibility remain separate. Lower cost cannot offset an authority or accessibility failure; higher reach cannot conceal a non-sustainable operating burden.

### Forecast drift parameters

The integrated forecast module also verifies 90 chronological synthetic rows across three profiles. With window size 5, CUSUM allowance 0.1, threshold 3.0 and minimum observation count 5, the frozen first-crossing oracle is: stable control = no crossing; gradual calibration drift = sequence 28; abrupt calibration shift = sequence 19. These are software regression settings only, not an operational trigger.

## 6. No composite or automatic winner

The integration predicate checks eight module oracles independently. Separate mutation tests make human-outcomes and forecast-uncertainty failures visible while the other seven module summaries remain passing.

No weighted score is produced. In particular:
- cost per message is not substituted for cost per unique accessible receiver;
- a high receiver total does not conceal inaccessible formats;
- a low lifecycle cost does not compensate for failed local continuity;
- a favourable forecast score does not authorize public warning;
- a completed action record does not establish causal impact.

## 7. Fixture provenance

All values are synthetic. FNV-1a values detect accidental fixture changes; they are not cryptographic provenance or signatures.

| Fixture | Rows | FNV-1a 64-bit |
|---|---:|---|
| Civil record v0.1.1 | 9 | 0x8e5118bf74fc7b13 |
| Topology v0.1 | 11 | 0xe6f6679f94c93e31 |
| Last-mile receiver cohorts v0.1 | 5 | 0x59f956045ca16c3d |
| Last-mile channel faults v0.1 | 10 | 0x2a7269d9a7d4caee |
| Warning-to-action v0.1.1 | 11 | 0x4084062a6c04598f |
| Forecast verification v0.1 | 65 | 0x81af0b6779196770 |
| Lifecycle cost profiles v0.1 | 24 | 0x52072da4d6fb1ba6 |
| Lifecycle service outcomes v0.1 | 21 | 0x376c727b26d887aa |

## 8. Reproduction

From repository root:

    cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml
    cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml

The executable prints eight independent module sections. A PASS means the frozen synthetic fixture oracles reproduce, not that a live service is safe, affordable, calibrated or operationally ready.

## 9. Industry and SOTA comparison protocol

Compare layer by layer and preserve compatible denominators:
- CAP 1.2 semantics and provenance sidecars;
- national/regional interoperability and local continuity;
- unique accessible receiver reach under channel and topology faults;
- warning-to-action timeliness, reach, grievances, outcome design and cost;
- forecast skill and calibration across hazard, region and lead time;
- lifecycle cost per accessible recipient with matched service scope.

Real-world cost comparison requires locally measured capital and lifecycle data, equivalent service obligations, staff/training assumptions, replacement cycles, operating horizon and discounting conventions. Report ranges and sensitivities. Do not rank national or regional programmes without permissioned, independently reviewed cost and outcome evidence.

WMO's 29 October 2026 analysis of 35 EW4All roadmaps emphasizes costed, time-bound national implementation. SADC's current MHEWS consultancy seeks capacity-building and interoperability between national systems, SHOC and AMHEWAS, including CAP and responsible AI/ML. Those are ecosystem needs; this repository is not an endorsement, procurement response or claim of eligibility.

References:
- WMO, 35 EW4All national roadmaps (29 Oct 2026): https://wmo.int/media/update/from-global-ambition-national-action-take-aways-from-35-ew4all-roadmaps
- SADC MHEWS capacity/interoperability consultancy: https://sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
- ITU Common Alerting Protocol: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Common-Alerting-Protocol-and-Call-to-Action.aspx
- UNDRR AMHEWAS: https://www.undrr.org/about-undrr/where-we-work/regional-office-africa/amhewas

## 10. NRVDEF remains separate

The nuclear risk-reduction and verified-disarmament evidence programme is intentionally separate. Its legal and verification authority boundaries differ from civil warning. The NRVDEF custody evaluator must not be interpreted as another GCPN quality dimension or as a way to decide physical truth or treaty compliance.

## 11. Validation status and non-claims

Exact integrated source, local manifests and fixtures were retrieved and structurally reviewed. Fixture counts, field widths, FNV fingerprints, lifecycle present-value arithmetic, human-outcomes denominators and paired-bootstrap event/interval arithmetic were independently recalculated. The human-outcomes component adds 18 synthetic cells with a pinned FNV-1a hash; the forecast-uncertainty component independently reconstructs 12 paired event records and its deterministic bootstrap intervals.

The authoring environment does not provide cargo or rustc. Compilation and Rust test execution are not claimed. Independent Cargo execution and CI remain necessary.

This package does not create forecasts, set thresholds, issue alerts, activate actions, release funds, allocate assistance, connect to operational warning systems, infer real population coverage or recommend national procurement. It does not claim actual human comprehension, protective behaviour, trust, lives saved, forecast superiority or validated meteorological dependence clusters. All examples are synthetic and non-operational.

## 12. Human outcomes and message comprehension

The new human-outcomes component is a communication usability contract, not a packet-delivery extension. It distinguishes intended test exposures from accessible representations and measures core-message comprehension, message-stated action-intent identification, uncertainty understanding, false certainty and response-time descriptors separately.

The frozen v0.1 fixture crosses six functional profiles (standard smartphone, low-bandwidth text, screen reader, sign-language video, audio-only radio and language mismatch) with three equivalent message variants (dense technical, plain language and accessible multimodal). It contains 18 cells × 100 invented trials and has FNV-1a fingerprint 0x019e77cec7c2246c.

For the accessible-multimodal synthetic variant:
- 600 intended trials, of which 530 have an accessible representation;
- 484 correct comprehension responses, 452 correct action-intent identifications and 422 correct uncertainty interpretations;
- 87 false-certainty interpretations;
- effective comprehension per intended trial: 484/600 = 80.67%;
- comprehension conditional on accessibility: 484/530 = 91.32%;
- worst-profile effective comprehension: 55%; profile gap: 39 percentage points;
- maximum profile p90: 34 seconds.

The difference between 80.67% effective comprehension and 91.32% conditional comprehension is intentional: a conditional denominator alone would conceal the 70 inaccessible trials. The values are synthetic test vectors, not human-subject observations. A comprehension response is not a claim that a person acted protectively.

The component document proposes future research targets (e.g. 95% accessibility per profile, 90% effective comprehension, <=5% false certainty, <=10-point comprehension gap and p90 <=10 seconds). These are hypotheses for later ethical field studies, not standards, live acceptance gates or achieved performance.

## 13. Paired forecast uncertainty

The new uncertainty component uses the forecast v0.2 parser and scoring fixture. It requires complete paired candidate, climatology and lead-reference forecasts per accepted evaluation event. All three Brier losses are resampled together; individual rows or models are never sampled independently.

Reproducibility parameters:
- 12 complete event IDs;
- 20,000 paired percentile-bootstrap replicates;
- deterministic seed 0x6a09e667f3bcc909;
- candidate minus climatology Brier difference: -0.078317, interval [-0.240000, 0.164208];
- candidate minus lead-reference Brier difference: +0.131683, interval [-0.030000, 0.374208].

Both intervals cross zero. The synthetic point estimates do not establish superiority over either baseline. The source fixture has event_id but no independently verified dependence_cluster_id. It is therefore only a synthetic demonstration of paired resampling, not a claim that the event IDs are independent weather systems. Real evaluation must resample defensible dependence clusters (e.g. linked storm/spatiotemporal systems), not assume independent station-time rows.

The uncertainty package is a standalone component and is not yet a live forecast model-selection or calibration policy. No p-value or automated winner is produced. The 12-event example cannot validate interval coverage.

Component methodology:
- GCPN human outcomes and message comprehension v0.1: docs/research/GCPN_HUMAN_OUTCOMES_AND_MESSAGE_COMPREHENSION_BENCHMARK_V0_1.md
- GCPN forecast event-cluster bootstrap uncertainty v0.1: docs/research/GCPN_FORECAST_EVENT_CLUSTER_BOOTSTRAP_UNCERTAINTY_V0_1.md
