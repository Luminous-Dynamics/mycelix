# Building metrology and resilience benchmarks v0.1

Research/benchmark proposal for Mycelix #6025/#6034, Symtropy #4507/#4531 and Sol-Atlas #704/#700. It separates implemented synthetic checks, proposed engineering targets and physical evidence. It does not claim any physical result.

## 1. The paradigm shift

Most building data stacks start with a point name, time series and dashboard. A robust decision system should start with a **purpose-bound measurement claim** and construct the evidence chain needed to support it:

`measurand/channel profile → instrument identity and capability → calibration certificate and scope → installation and geometry → raw sample and timebase → processing lineage → uncertainty components/correlation → decision-purpose profile → independent review → owner-native service outcome`

That changes the unit of trust. The unit is no longer a device or a `calibrated=true` flag; it is a *specific observation under a specific purpose with its full, current applicability chain*. Calibration date and due date are checked against the sample's observation date; a state flag cannot override an expired interval.

### Paradigm shifts to pursue

1. **Calibration is a scoped relation, not a device property.** A certificate applies to an instrument, channel, measurand, method, range, configuration and conditions. Accreditation applies only to the exact scope. A device can be valid for trend monitoring but insufficient for a threshold-adjacent decision.
2. **Every sample is an evidence object.** Preserve source payload identity, event time and acquisition time separately, firmware/profile, raw/derived state, QC, installation and calibration references. A vendor dashboard export is not a substitute for the underlying traceable record.
3. **Uncertainty is a dependency graph, not a confidence badge.** Retain uncertainty components, covariance/correlation groups, source records, method and coverage convention. Never silently treat common-reference components as independent or collapse them to an opaque score.
4. **Change is a first-class event.** Firmware, relocation, recalibration, component replacement, geometry/BIM revision, source correction and rights revocation trigger dependency-aware invalidation. Invalidate only affected samples and claims; preserve unrelated evidence.
5. **Qualification is purpose-specific and fail-closed.** The result is review eligibility, not automatic admission. Unknown, stale, contradictory, out-of-envelope, unauthorized or non-comparable data cannot become favourable numeric values.
6. **One physical reality, multiple owner authorities.** Mycelix qualifies measurement evidence; Symtropy evaluates event availability and physical model/trajectory evidence; Sol-Atlas maps geometry and evaluates read-only in-use outcomes. A command, an event or a semantic mapping cannot impersonate a measured service outcome.
7. **Resilience is a trajectory, not an annual average.** Keep threshold exceedance, time-to-breach, cumulative exposure, degradation, recovery, resource use and resident-defined service outcomes separate. No single scalar may hide loss of an essential service.

## 2. Industry and state-of-the-art comparison

| Industry / research lane | Existing strength | Do not claim as new | Luminous testable differentiator |
|---|---|---|---|
| Building automation / BMS / IoT platforms | Live points, alarming, control, trend history, equipment hierarchies, commissioning and analytics | Building dashboards, sensor networks, automation, point tags and digital twins | Can an exact sample be reconstructed from instrument/certificate/scope/method/installation/firmware/uncertainty, with owner-scoped evidence and revision invalidation? |
| Brick, Project Haystack, IFC, ASHRAE 223P and NIST semantic tooling | Building/equipment/space/point identity, topology, machine-readable integration, validation and analytics portability | Semantic interoperability itself | Preserve physical measurand, units, method, calibration applicability and evidence class during graph translation and round-trip; unresolved semantics must abstain |
| ISO/IEC 17025 / SANAS / ILAC calibration ecosystem | Competent labs, method-specific scopes, traceability, calibration statements and uncertainty | Calibration traceability or accreditation itself | Machine-check the *exact* current certificate and scope against the instrument/channel/method/range, then bind it to samples and downstream claims |
| BOPTEST and building-control benchmark ecosystems | Standardized virtual buildings, controls APIs, KPI reports and matched controller comparison | Model-predictive control, FDD, building control benchmarking or virtual twins | Pair virtual performance with physical measurement qualification and owner-native, measured service outcomes; never treat virtual success as field evidence |
| Thermal-resilience research | Threshold/exceedance, survival/time-to-breach, degree-hour accumulation, dynamic response/recovery and composite metrics | Outage simulation, thermal time constants, thermal inertia or resilience indices | Standardize event trace, instrument provenance, multi-metric trajectory and service-floor comparison across climates and real buildings |
| Household appliance test methods | Repeatability, reproducibility, uncertainty and round-robin comparison | Laboratory performance measurement | Keep lab repeatability, inter-lab reproducibility, production-unit variation and household ecological validity as four separate evidence strata |

**Research status correction:** NIST's 2 November 2026 update says ASHRAE 223P comments are being addressed and a committee vote for a second public review is awaited. It remains a proposed/in-development lane at this cut, not a finalized standard. Brick and Haystack remain important incumbent ecosystems; Sol-Atlas #704 owns the actual versioned translation experiment.

## 3. Benchmark suite

### A. Synthetic contract correctness (current software gate)

- Frozen v0.1 channels: 14.
- Instrument passport corpus: 51 synthetic known-answer/adversarial cases.
- Current synthetic target: 100% expected-disposition agreement; **zero false admission**; zero unknown fields or duplicate JSON member acceptance; zero silent missing-as-zero, stale, saturated or out-of-envelope readings; 100% of channel IDs/units/raw-derived semantics pinned independently of the fixture.
- A complete synthetic chain may reach only `ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY`.
- No physical tolerance or calibration range is invented by the software profile.

### B. Evidence-chain completeness

Measure the percentage of downstream claims for which the following are independently reconstructable: instrument/channel identity; exact capability envelope; calibration certificate/digest; lab and accreditation scope; method and revision; installation/geometry revision; sample payload/digest; timebase and sequence; QC state; processing/source lineage; uncertainty components/correlation; purpose profile; source rights.

**Target:** 100% of admitted-for-review observations have all mandatory references. Missing fields stay explicit `UNKNOWN`/`UNBOUND`; no imputation.

### C. Mutation and adversarial robustness

Expand from 51 to at least 100 cases before physical use. Add: forged/revoked certificate; wrong standard/reference artifact; a calibration point outside claimed scope; calibration correction applied to only some channels; certificate issued after sample date; NTP rollback and reboot sequence; daylight-saving ambiguity; duplicated/reordered packets; firmware scaling change; correlated drift; out-of-tolerance as-found review; missing raw data behind a derived value; invalid measurement rights; false assay sample ID; and selective deletion of inconvenient observations.

**Target:** 100% frozen known answers; zero seeded false admissions; mutation controls must prove each mutation changes the input and is rejected for the intended reason.

### D. Change invalidation precision

Construct a dependency graph from instrument/configuration → calibration → installation → sample → transform → aggregate → service/outcome claim. Mutate each upstream item and measure:

- true-positive invalidation: affected descendants revoked or marked stale;
- false-positive invalidation: unrelated samples and claims remain current;
- propagation latency: time from a source revision/revocation receipt to every affected dependent claim being blocked;
- recovery: exact new evidence needed to re-establish currentness.

**Target:** 100% seeded affected descendants invalidated; zero unrelated descendants invalidated; zero new dependent mutation while prior state is unresolved.

### E. Measurement-method performance (physical phase, not yet run)

Use reference methods and appropriately scoped calibration laboratories. For each selected channel, define a decision-specific error budget *before* hardware selection. Compare instruments at multiple points across their declared range, environmental conditions and response dynamics. Preserve raw results, as-found/as-left data, residuals, uncertainty and method revisions.

Report bias, repeatability, hysteresis, linearity, drift, response/time constant, cross-sensitivity, saturation/recovery, measurement uncertainty and coverage. Do not collapse them into one accuracy score. Instrument accuracy claims must be tied to their basis (absolute, % of reading, % full scale, range/conditions).

### F. Reproducibility and real-world representativeness

Follow the boundaries in IEC TR 63250:2021 for appliance method repeatability/reproducibility and round-robin uncertainty, but do not confuse that with product population or household representativeness. Keep four strata separate:

1. within-unit repeatability;
2. between-laboratory reproducibility;
3. between-unit/production-lot variation;
4. household/site ecological validity across climates, occupancy, operation, maintenance and user behaviour.

Set study size from pilot variance components and a preregistered precision/power analysis. Do not blindly copy an IID sample count to nested/crossed laboratory, unit and household effects.

### G. Semantic interoperability and physical truth preservation

Run IFC/BIM and building-point mappings through the Sol-Atlas #704 baseline (Brick stable lane, Haystack/XETO, and ASHRAE 223P only when an exact artifact is pinned). Evaluate:

- exact semantic equivalence rate and false-equivalence rate;
- correct abstention and declared-loss rate for unresolved/narrower/broader/derived mappings;
- unit/timebase/evidence-class and instrument/certificate binding preservation;
- round-trip graph loss, query-result disagreement and topology/flow-direction mismatch;
- downstream invalidation when ontology, BIM, firmware or profile revisions change.

**Hard safety metric:** false exact-equivalence rate must be zero on the frozen adversarial corpus. Syntactic schema/SHACL validity alone is not physical truth.

### H. Thermal resilience and household service

Do not use only annual kWh or one resilience ratio. For matched outage/weather traces report:

- threshold-specific hours above/below declared band and first time-to-breach;
- cold and hot degree-hour accumulation separately;
- degradation rate, peak/minimum value and time-to-peak/minimum;
- stable recovery time, repeat disturbance and time within the service floor;
- energy, water, ventilation, pollutant-specific IAQ, noise, maintenance and operator effort;
- uncertainty, data coverage, worst-zone/worst-stratum and resident-defined service floor;
- passive-only vs powered vs hybrid/local autonomy arms on exact same event inputs.

Calibrate on one period and validate on separate held-out periods/events. Report CV(RMSE), NMBE or other method metrics only with their exact denominator, time basis and method; report residual distributions and uncertainty too. A model calibration pass does not qualify real-world resilience.

## 4. Proposed superiority targets (hypotheses, not achieved results)

- Zero false instrument admission or false semantic equivalence on 100+ adversarial known-answer cases.
- 100% reconstruction of every sample used in an admitted decision from its exact instrument/channel, method, calibration, scope, installation, firmware, timebase, raw/derived lineage and uncertainty record.
- 100% change-impact coverage with zero unrelated evidence invalidation.
- At least 50% lower *human evidence-reconciliation time* than a manual certificate/CSV/BMS workflow on matched cases, without worsening false-accept/false-reject rates. This is a research target to test, not a current result.
- At least 30% lower bespoke point-to-measurement mapping effort when consuming existing Brick/Haystack/IFC/223-compatible assets, with the same or better semantic equivalence and no invented identity. This is also a proposed target.
- For physical resilience/service trials, demonstrate at least one preregistered, practically material service/recovery or lifecycle advantage against the best matched incumbent while preserving all hard service/safety/accessibility/consent floors. Report incumbent wins and ties.

These targets must be costed end-to-end: instrumentation, accredited calibration, commissioning, semantic modelling, data infrastructure, compute, operator time, maintenance, spares and lifecycle burden. A performance improvement that requires materially greater hidden labour or maintenance is not a clean superiority claim.

## 5. Stop/go gates

- **M0:** qualified design basis, hazard review, local standards applicability and instrument/method plan; no purchase/build authority from this document.
- **M1:** individual instrument and method verification with actual certificates, calibration scopes, reference instruments and installation records.
- **M2:** one service at a time in a controlled unoccupied test cell; prove raw data, uncertainty, safe isolation and service outcome.
- **M3:** coupled service and common-mode stress tests using matched traces; retain independent owner review.
- **M4:** independently reviewed field/household validation and any required professional/statutory approvals. Occupied testing requires a separate ethics, privacy, consent and safety protocol.

## 6. Current software slice

- Mycelix #6028: 14 v0.1 channel state-gating oracle, 36 synthetic cases.
- Mycelix #6036: seven missing-channel gap groups and 14 proposed v0.2 channel IDs; not admitted.
- Mycelix #6034: numerical capability/calibration/installation/sample passport phase; current branch adds 45 synthetic passport cases and a typed independent-assay record.
- Symtropy #4507/#4531: thermal-resilience measurement/trajectory and unoccupied service-cell design.
- Sol-Atlas #704/#700: semantic interoperability and read-only in-use outcome binding.

All exact-head software workflows must pass before a software PASS is claimed. Queued/skipped/pending is not PASS. All physical gates remain unrun.


## 8. Two research frontiers beyond passport completeness

### Digital calibration certificate interoperability

PTB's Digital Calibration Certificate (DCC) is a machine-readable, XML/XSD-based certificate model. The Luminous adapter should consume and preserve the original signed certificate, not invent a replacement certificate format. The research test is whether it can validate the exact DCC schema/version, signature/trust material, certificate issuer and accreditation-scope binding, then map each result to the precise instrument/channel/measurand/method/range/unit/uncertainty/sample relationship without losing semantics.

Benchmark against:
- valid, correctly signed and schema-valid DCC;
- schema-valid but unsigned or invalidly signed certificate;
- revoked/expired certificate and stale trust material;
- valid certificate for wrong measurand, unit, method or range;
- DCC from a laboratory whose current accreditation schedule does not cover the method/range;
- certificate correction or replacement with dependency-aware invalidation;
- lossless round-trip of original XML bytes/digest and semantically normalized fields.

Targets: zero false-authenticity acceptance; 100% exact issuer/scope/result binding; zero silent field loss; 100% reproducible verification receipts. The parser's capability is not the same as the laboratory's accreditation or the truth of a physical reading.

### Uncertainty and decision-fit computation

The 2026 GUM amendment explicitly addresses nonlinearity in measurement models. A next-generation measurement layer should not stop at storing a list of uncertainty components. It should represent the measurement equation, input distributions, sensitivity coefficients, covariance/correlation structure, model revision, environmental conditions, and output distribution/coverage interval. It should compare a linearized GUM calculation with Monte Carlo propagation where nonlinearities, non-normal distributions or asymmetric output make that appropriate.

For decision fitness, keep measurement uncertainty separate from the decision threshold and loss model. A downstream owner must select the conformity/guard-band rule and acceptable false-accept/false-reject risk. The passport should never silently choose the threshold or decision rule.

Benchmarks:
- analytically solvable linear measurement models with known covariance;
- correlated common-reference inputs that defeat naive root-sum-square treatment;
- nonlinear thermal-energy calculation under varying flow and temperature;
- non-normal/asymmetric uncertainty propagation with Monte Carlo reference;
- near-threshold cases under explicitly declared guard-band rules;
- changed model/correlation/calibration inputs with targeted descendant invalidation.

Targets: reproduce reference results within the preregistered numerical tolerance for each exact method; zero covariance omissions; zero unsupported independence assumptions; complete traceability of every propagated input; calibrated coverage on held-out reference cases; zero silent change of conformity rule. Tolerances are set per method and use case, not universally.
