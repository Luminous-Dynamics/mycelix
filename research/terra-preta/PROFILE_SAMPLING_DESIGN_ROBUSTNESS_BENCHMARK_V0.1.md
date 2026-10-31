# Profile Sampling-Design Robustness Benchmark v0.1

**Status:** executable orchestration harness and synthetic stress fixture are committed. The NEON and profile-level runs remain to be executed on acquired raw profiles. No cadence is declared universally adequate and no field accuracy result is claimed.

## 1. Why this is a scientific benchmark, not a preprocessing detail

Preferential-flow signatures are affected by the sampling design itself. The 2025 review by Nimmo et al. reports that NSR studies commonly use 5–30 minute sampling intervals, identifies finite cadence as a source of false negatives and ambiguous response ordering, and notes that maximum-rate-of-rise metrics are sensitive to temporal resolution. It recommends high practical frequency and a greater number of instrumented depths when designing monitoring networks.

The 2026 NEON event-model paper resamples 1-minute soil-moisture observations to 10-minute medians and requires at least seven good measurements within each 10-minute bin; therefore, 10-minute is the correct paper-reproduction track, but a 10-minute score alone does not show how much the chosen grid alters profile evidence.

The design question is: **for the same soil, event, raw sensor trace and QA rules, how much do the inferred response intervals, NSR pairs and profile classification change as the sampling interval and monitored depth set change?**

## 2. Executable pipeline

The script [run-profile-design-sensitivity.sh](./tools/run-profile-design-sensitivity.sh) orchestrates the existing tools; it does not introduce a second response or NSR algorithm.

```sh
TERRA_PRETA_CADENCES_SECONDS="300 600 900 1800" \
TERRA_PRETA_RESPONSE_THRESHOLDS="0.01 0.02 0.03 0.05" \
bash research/terra-preta/tools/run-profile-design-sensitivity.sh \
  research/terra-preta/tools/neon-swc-normalizer/fixtures/synthetic-cadence-config.json \
  research/terra-preta/tools/neon-swc-normalizer/fixtures/synthetic-cadence-one-minute.csv \
  /path/to/new-cadence-sensitivity-run
```

For every cadence × response threshold combination it:
1. changes only the target aggregation interval and declared response threshold in a copy of the pinned normalizer config;
2. runs the 1-minute NEON soil-water normalizer with the same raw CSV, sensor roster, QA filters, porosity cap and median aggregation;
3. feeds the raw-normalized profile (not the interpolated paper-analysis clone) to the response extractor;
4. runs the interval-censored profile-flow auditor;
5. records normalization coverage, profile class, sensor counts, NSR/sequential/tied/ambiguous pairs, onset intervals and SHA-256 digests.

It then performs a leave-one-sensor-out ablation at the selected reference cadence and threshold. The default reference is 5 minutes, the highest-resolution comparator allowed by the current extractor. The report explicitly labels this as a *reference*, not as ground truth.

The run fails closed if a source/config/output is missing, the output directory already exists, a reference cadence is not in the cadence grid, or the depth-ablation threshold is not in the threshold grid. Generated raw-normalized profiles and intermediate reports are retained in the output directory so the run can be independently inspected.

## 3. Required outputs and interpretation

The machine-readable report contains:
- one row per cadence × threshold run;
- source CSV SHA-256, base config SHA-256, per-cadence config SHA-256 and normalized-profile SHA-256;
- observed, missing, interpolated and remaining-missing bin counts;
- response threshold, sensor count and indeterminate count;
- profile classification and every pairwise onset interval;
- classification-change and same-NSR-pair persistence comparisons against the 5-minute reference;
- leave-one-sensor-out classification and NSR-pair loss.

A transition from `mixed_response_ordering` at 5 minutes to an ambiguous or sequential classification at 15/30 minutes is not a bug to smooth away. It is evidence that the measurement design cannot resolve the underlying response ordering at that cadence. Conversely, unchanged classification across cadences is only algorithmic stability; it does not prove the signature is physically correct.

### Synthetic fixture's analytical expectation

The committed 1-minute fixture encodes response changes at 35, 45 and 65 minutes after midnight for the 30-, 10- and 60-cm sensors, respectively; event onset is 00:30 UTC. At 5-minute bins the 30-cm response interval (00:30, 00:35] is strictly earlier than the 10-cm interval (00:40, 00:45], so one deeper-first pair is resolvable. At 10-, 15- and 30-minute aggregation the relevant intervals touch or overlap under the profile auditor's strict interval-ordering rule; they must not be counted as robust NSR. A leave-one-sensor-out design should lose that pair when either 10- or 30-cm sensor is removed, but preserve it when the 60-cm sensor is removed. These are analytically predicted fixture outcomes, not a successful Rust or CI run.

For the fixture's 0.01 m³/m³ threshold, independent median-bin arithmetic gives the following first response-bin start times:

| Target cadence | 10 cm sensor | 30 cm sensor | 60 cm sensor | Expected profile-level implication |
|---|---:|---:|---:|---|
| 5 min | 00:45 | 00:35 | 01:05 | 30 cm is robustly earlier than 10 cm at this grid |
| 10 min | 00:40 | 00:30 | 01:00 | 30/10 onset intervals touch; not a robust NSR pair |
| 15 min | 00:45 | 00:30 | 01:00 | 30/10 onset intervals touch; not a robust NSR pair |
| 30 min | 00:30 | 00:30 | 01:00 | 10/30 are simultaneous within the 30-minute sampling resolution |

Times are UTC on 1 June 2026. The table predicts the bin timestamp that crosses the declared threshold; it does not estimate true physical flow arrival time. The profile auditor must use the entire onset interval `(last below threshold, first above threshold]`; different reported bin starts do not automatically imply a strictly ordered pair.

## 4. Benchmark matrix

| Dimension | Required strata | Reported measure |
|---|---|---|
| Temporal cadence | 5, 10, 15, 30 min | Response-onset interval width, event response detection, NSR/sequential/tied/ambiguous pairs, classification transition |
| Absolute response threshold | 0.01, 0.02, 0.03, 0.05 m³/m³ | Threshold-dependent detection coverage, response interval, profile class and pair persistence |
| Sensor depth layout | Full roster and leave-one-depth-out | NSR-pair loss, classification transition and indeterminate coverage |
| Sensor condition | Usable, saturated, frozen, fault, unknown | Evaluability and abstention; never relabel unusable as a valid negative |
| Data coverage | Complete, short gap, long gap, edge gap | Missing/interpolated coverage and onset-censoring changes |
| Soil/event regime | Antecedent wetness, event intensity, soil texture/structure, event size | Conditional cadence/layout sensitivity; do not pool across strata without reporting |

Run cadence and depth sweeps on the same event and frozen sensor roster. For field work, stratify by event and site and preserve all failures/abstentions. Use the independent experimental unit/site for uncertainty estimation; sensor samples and repeated events are nested observations.

## 5. Proposed targets (not achieved)

- **Coverage:** every normalized bin has a declared observed, missing or explicitly interpolated status; no missing bin silently becomes zero.
- **Auditability:** 100% of cadence runs bind to identical raw source bytes and a config that differs only in target cadence/declared threshold.
- **Pair stability:** publish the fraction of 5-minute NSR pairs that remain robustly deeper-first at 10/15/30 minutes; never hide a pair becoming interval-ambiguous.
- **Layout sensitivity:** publish every leave-one-sensor-out result and identify the marginal contribution of each depth.
- **Independent truth:** on tracer-adjudicated profiles, estimate sensitivity, false-positive rate and onset-interval coverage by cadence; do not use cross-cadence agreement as a truth label.
- **Adaptive design:** choose the lowest-cost sensor/cadence design that meets a preregistered external detection/uncertainty target, validated against a fixed-layout control.

A target design must meet the declared false-positive operating point on matrix-only controls *and* maintain sensitivity on high-signal tracer-positive cases. Average agreement with a 5-minute reference alone is not a pass criterion.

## 6. How to improve beyond current practice

The longer-term move is to replace a universal fixed sensor layout with **decision-aware adaptive measurement design**:

1. use existing high-frequency sensors to identify uncertainty about mechanism (NSR, VT, trajectory and spatial-heterogeneity disagreement);
2. select the additional depth, cadence, plot replication or tracer sampling time expected to distinguish the competing physical explanations;
3. compare adaptive selection against a fixed-layout baseline on independent events;
4. measure the information gained per instrument/lab cost and publish where the adaptive design abstains.

This is a sampling-policy research claim, not an automatic field instrumentation prescription. The value-of-information selector should be optimized against tracer/flux uncertainty, not agreement among proxies.

## 7. Scientific boundaries

- Five-minute is the highest-resolution reference available in this pipeline, but it is still aggregated data and is not tracer ground truth.
- The paper's 10-minute preprocessing remains a separate historical reproduction track; cadence sweeps are a robustness extension and must not replace it.
- The synthetic fixture is a software control with known response timings, not observed soil or a physical flow simulation.
- A deeper-first sensor response remains a profile signature, not direct measurement of connected macropore geometry or nutrient flux.
- Any instrument-adaptive detector must calibrate its false-alarm rate from sensor noise/calibration records and independent reference observations.

## References

- Nimmo, J. R. et al. (2025). Identifying preferential flow from soil moisture time series: Review of methodologies. *Vadose Zone Journal*, 24(2), e70017. https://doi.org/10.1002/vzj2.70017
- Li, B. et al. (2026). Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes. *Earth's Future*. https://doi.org/10.1029/2026EF008296


## Next research track: decision-aware placement

The cadence and leave-one-sensor-out runner diagnoses fragility in an existing layout; it does not optimize new depths or spatial positions. The separate [PF sensor-placement benchmark](./PREFERENTIAL_FLOW_SENSOR_PLACEMENT_BENCHMARK_V0.1.md) compares storage-oriented industry layouts with PF-specific mechanism-discrimination designs and requires decision-risk reduction per fully loaded lifecycle cost. Implementation is tracked in [PF-DESIGN-001](https://github.com/Luminous-Dynamics/mycelix/issues/5579).
