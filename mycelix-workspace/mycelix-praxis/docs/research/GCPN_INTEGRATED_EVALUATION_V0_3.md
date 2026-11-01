# GCPN Integrated Synthetic Evaluation v0.3

**Status:** integrated synthetic benchmark composition; non-operational.  
**Parent:** issue #5621; predecessor PR #5619.  
**Package:** `benchmarks/gcpn-integrated-evaluation`.  
**Version:** 0.3.0.  
**Dependencies:** five local Rust benchmark packages; no third-party Cargo dependencies.

## 1. What changed from v0.2

v0.2 composed four independent modules:
1. civil warning-record integrity;
2. topology resilience;
3. last-mile delivery/accessibility;
4. warning-to-action evidence.

v0.3 adds the fifth module: **probabilistic hazard-forecast verification**. It evaluates forecast quality and uncertainty with proper scoring rules, calibration diagnostics and per-hazard/horizon breakdowns.

The fifth module remains separate. A forecast with a favourable overall score cannot compensate for an authority failure, inaccessible last-mile delivery or an unsupported action/impact claim. Conversely, successful delivery and action records cannot compensate for a bad forecast subgroup.

## 2. Five independent evaluation gates

| Module | Frozen test population | Primary checks |
|---|---:|---|
| Civil record conformance v0.1.1 | 9 synthetic cases | Authority binding, source independence, state preservation, uncertainty disclosure, provenance privacy |
| Topology resilience v0.1 | 11 faults × 3 architectures | Common denominators, competent-authority gate, local-first continuity |
| Last-mile multi-channel v0.1 | 5 receiver cohorts × 10 channel states | Unique reach, accessibility, uncovered cohorts, duplicate channel receipts |
| Warning-to-action v0.1.1 | 11 synthetic cases | Authorized decisions, conditional financing, action chronology, population/service accounting, grievances and impact evidence |
| Forecast verification v0.1 | 65 forecast rows, 54 scored records | Brier/log loss, Brier skill, calibration, subgroup score, missing/late/invalid/out-of-domain and partition leakage |

The integrated reproduction status is a non-compensatory conjunction of the frozen fixture oracles. It is not a weighted quality score, a ranking or a real-world readiness certification.

## 3. Forecast-verification module

### Data-quality controls

The synthetic forecast fixture has:
- 65 raw forecast records;
- 54 scored records;
- 1 duplicate forecast record;
- 1 late forecast;
- 1 unverified observation record;
- 1 missing outcome record;
- 1 invalid forecast probability;
- 0 invalid reference probabilities;
- 1 out-of-domain forecast;
- 1 abstention;
- 1 event leaking between calibration and evaluation partitions;
- 1 event with inconsistent metadata.

Records affected by event-level partition leakage or event metadata mismatch are excluded from scoring. Abstention, out-of-domain, late, invalid and missing data remain visible through separate counters.

### Proper scores and calibration diagnostics

For the twelve synthetic evaluation records per model, the candidate and two references produce:

| Model | Brier score | Brier skill vs climatology |
|---|---:|---:|
| Climatology | 0.250000 | 0.000000 |
| Lead-conditioned reference | 0.040000 | 0.840000 |
| Candidate | 0.171683 | 0.313267 |

For the candidate:
- calibration-partition Brier score: 0.010000 across 6 synthetic records;
- held-out evaluation Brier score: 0.171683 across 12 records;
- held-out log loss, calibration-in-the-large, four-bin ECE and reliability bins are reported independently;
- candidate overall Brier skill is positive against climatology, but its aggregate Brier score is worse than the lead-conditioned reference.

The candidate's heat / 72-hour evaluation subgroup has:
- sample count: 2;
- Brier score: 0.980100;
- Brier skill versus climatology: -2.920400.

This is an intentionally adversarial synthetic example. It tests the principle that aggregate improvement must not hide a severe hazard/horizon regression. Two subgroup samples are far too few for scientific conclusions; this is an exact code-level regression, not evidence about a real forecasting service or hazard.

### Metrics

The integrated report preserves:
- raw and scored record counts;
- duplicate, late, missing, unverified, invalid, out-of-domain, abstention and leakage counters;
- candidate calibration and held-out sample counts;
- candidate Brier, log loss, Brier skill and calibration diagnostics;
- lead-conditioned reference score and skill;
- heat/72-hour subgroup sample count, Brier and skill.

The full component CLI also reports per-model, hazard, region, lead-time and partition results. No forecast accuracy or calibration metric is converted into alert authorization.

## 4. Cross-module invariants

### Civil record and authority

A valid record signature or source reference is not physical truth. Every public alert remains bound to a competent authority assertion. Contradictory, stale, missing and unverified states are not silently promoted.

### Topology

Centralized, regional and local-first references see identical receiver denominators per scenario. The competent-authority gate blocks all architectures. Local-first continuity is counted only when the relevant local path and local authority are available.

### Last mile

Unique recipient reach is separate from summed channel receipts. An individual reached by several channels is counted once. Accessible-format reach is separate from basic delivery, and a cohort with no accessible message is never hidden by aggregate coverage.

### Warning-to-action

The chain keeps authorized warning, protocol decision, conditional financing, action start, action completion, unique service, grievances and independent outcome evidence distinct. Some action packages may not need a separate funding-release event; v0.1.1 encodes that explicitly.

### Forecast verification

The forecast module evaluates probabilities; it does not set a public warning threshold or decide whether to alert. Forecasts are an evidence input for competent forecasters and authorities, not a replacement for them.

## 5. No compensating score

A global weighted score would let high delivery hide an inaccessible cohort, let a strong average forecast hide a hazard failure, or let an action-completion count hide unauthorized action or suppressed grievances.

The integrated predicate instead checks each declared fixture invariant independently. The tests deliberately mutate one module's invariant and require the overall reproduction predicate to fail while the other module summaries remain intact.

## 6. Fixture provenance

All values are synthetic. FNV-1a hashes are regression identifiers only; they are not cryptographic provenance or signatures.

| Fixture | Rows | FNV-1a 64-bit |
|---|---:|---|
| Civil record v0.1.1 | 9 | `0x8e5118bf74fc7b13` |
| Topology v0.1 | 11 | `0xe6f6679f94c93e31` |
| Last-mile receiver cohorts v0.1 | 5 | `0x59f956045ca16c3d` |
| Last-mile channel faults v0.1 | 10 | `0x2a7269d9a7d4caee` |
| Warning-to-action v0.1.1 | 11 | `0x4084062a6c04598f` |
| Forecast verification v0.1 | 65 | `0x81af0b6779196770` |

## 7. Reproduction

From repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-integrated-evaluation/Cargo.toml
```

The executable prints five module sections and a final fixture-reproduction status. A PASS means the local synthetic fixture oracles reproduce; it does not mean a live civil-warning service is safe, calibrated, reliable or operationally ready.

## 8. Industry / SOTA comparison protocol

The correct comparison is layer-specific, with equivalent task definitions and denominators:
- CAP 1.2 semantics and optional provenance sidecars;
- interoperability with national/regional systems such as SADC SHOC and AMHEWAS;
- topology reachability under matched faults;
- receiver-level unique and accessible reach by channel/cohort;
- warning-to-action process quality, time, equity, grievances and cost;
- forecast quality by hazard/horizon/region with held-out events, proper scores and calibration.

Forecast comparisons should use common event windows and authoritative outcome data, paired model denominators, climatology and suitable lead-conditioned references, held-out or rolling-origin partitions, event-clustered uncertainty intervals, and complete subgroup reports. Forecast quality, data coverage, latency, compute and cost should be reported separately. Do not rank named national or regional services without permissioned adapters and independent evaluation.

The synthetic v0.1 forecast fixture is not itself a SOTA comparison. Its 12 held-out cases per model and two-case subgroup are regression vectors, not representative samples.

## 9. Relationship to existing African early-warning institutions

AMHEWAS and SADC SHOC already have continental and regional coordination roles. GCPN should complement national warning systems and regional programmes through optional provenance, conformance and independent evaluation tooling; it must not create a competing situation room or supranational warning authority.

SADC's MHEWS consultancy notice issued on 28 October 2026 seeks national capacity-building and interoperability between national warning systems, SHOC and AMHEWAS, including CAP and responsible AI/ML. The notice closes 17 November 2026 at 12:00 PM. It is an ecosystem demand signal, not evidence of GCPN endorsement, eligibility or selection.

References:
- SADC MHEWS consultancy: https://sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and
- WMO, analysis of 35 EW4All roadmaps (29 Oct 2026): https://wmo.int/media/update/from-global-ambition-national-action-take-aways-from-35-ew4all-roadmaps
- ITU Common Alerting Protocol: https://www.itu.int/en/ITU-D/Emergency-Telecommunications/Pages/Common-Alerting-Protocol-and-Call-to-Action.aspx
- UNDRR AMHEWAS: https://www.undrr.org/about-undrr/where-we-work/regional-office-africa/amhewas
- ECMWF probabilistic forecast verification: https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
- WMO hydrological forecast verification guidelines (2025): https://primarynewssource.org/wp-content/uploads/WMO-1364-2025_en.pdf

## 10. NRVDEF remains separate

The Nuclear Risk Reduction and Verified Disarmament Evidence Framework is deliberately not a sixth GCPN score. Its legal, diplomatic and verification boundaries differ from civil warning and forecasting. Its evidence-custody benchmark remains a separate research line.

## 11. Validation status and non-claims

The exact integrated source, component fixtures and manifests were retrieved and structurally reviewed. Fixture row counts and fingerprints were checked, and the candidate/reference scoring values were independently recomputed from the synthetic CSV.

The authoring environment does not provide `cargo` or `rustc`. Rust compilation and tests are not claimed. Independent Cargo execution and CI remain necessary.

This package does not produce forecasts, select warning thresholds, issue public alerts, activate actions, release funds, allocate assistance, connect to operational warning systems or certify impact. All examples are synthetic and non-operational.
