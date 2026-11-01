# GCPN Integrated Synthetic Evaluation v0.4

**Status:** integrated synthetic benchmark composition; non-operational.  
**Parent:** issue #5627; predecessor PR #5622.  
**Package:** benchmarks/gcpn-integrated-evaluation.  
**Version:** 0.4.0.  
**Dependencies:** six local Rust benchmark packages; no third-party Cargo dependencies.

## 1. What changed from v0.3

v0.3 composed five independent modules:
1. civil warning-record integrity;
2. topology resilience;
3. last-mile delivery/accessibility;
4. warning-to-action evidence;
5. probabilistic forecast verification.

v0.4 adds **lifecycle cost and accessible-reach efficiency** as a sixth module. It accounts for capital, channel setup, operations, staff, maintenance, replacement and scenario-specific restoration, and relates these costs to intended, unique and accessible receiver counts.

Cost is not a quality score. A cheaper architecture is not necessarily better if it reaches fewer people accessibly or cannot preserve local service through a fault. Equally, greater reach does not erase unaffordable staffing or replacement obligations.

## 2. Six independent evaluation gates

| Module | Frozen test population | Primary checks |
|---|---:|---|
| Civil record conformance v0.1.1 | 9 synthetic cases | Authority binding, source independence, state preservation, uncertainty disclosure, provenance privacy |
| Topology resilience v0.1 | 11 faults × 3 architectures | Common denominators, competent-authority gate, local-first continuity |
| Last-mile multi-channel v0.1 | 5 receiver cohorts × 10 channel states | Unique reach, accessibility, uncovered cohorts, duplicate channel receipts |
| Warning-to-action v0.1.1 | 11 synthetic cases | Authorized decisions, conditional financing, action chronology, population/service accounting, grievances and impact evidence |
| Forecast verification v0.1 | 65 forecast rows, 54 scored records | Brier/log loss, Brier skill, calibration, subgroup score, missing/late/invalid/out-of-domain and partition leakage |
| Lifecycle cost v0.1 | 24 cost rows, 21 matched outcomes | Five-year present value, low/base/high ranges, staffing, restoration, cost per accessible unique receiver |

The final integrated reproduction status remains a strict conjunction of frozen fixture invariants. It is not a weighted quality score, a ranking or a real-world readiness certification.

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

Probabilistic forecasts are scored using proper scores and calibration/subgroup diagnostics. They do not set alert thresholds or authorize warnings.

### Lifecycle cost

Costs, staffing, reach and accessibility remain separate. Lower cost cannot offset an authority or accessibility failure; higher reach cannot conceal a non-sustainable operating burden.

## 6. No composite or automatic winner

The integration predicate checks six module oracles independently. Tests mutate a cost invariant while the other five module summaries remain passing and require the integrated reproduction predicate to fail.

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

The executable prints six independent module sections. A PASS means the frozen synthetic fixture oracles reproduce, not that a live service is safe, affordable, calibrated or operationally ready.

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

Exact integrated source, local manifests and fixtures were retrieved and structurally reviewed. Fixture counts, field widths, FNV fingerprints and lifecycle present-value arithmetic were independently recalculated.

The authoring environment does not provide cargo or rustc. Compilation and Rust test execution are not claimed. Independent Cargo execution and CI remain necessary.

This package does not create forecasts, set thresholds, issue alerts, activate actions, release funds, allocate assistance, connect to operational warning systems, infer real population coverage or recommend national procurement. All examples are synthetic and non-operational.
