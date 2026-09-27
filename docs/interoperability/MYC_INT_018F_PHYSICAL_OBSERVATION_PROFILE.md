# MYC-INT-018F — Domain-Safe Physical Observation Profile

Status: architecture / adapter profile only. Tracks #3231. Child of MYC-INT-018E / PR #3230.

Observed design date: 2026-09-27.

## 1. Purpose

Define the smallest reusable binding needed to carry physical measurements from productive-node domains into shared evidence/analysis workflows without creating a second telemetry bus, second evidence identity system, second currentness theorem, or generic safety verdict.

The productive-node showcase needs to combine measurements such as:

- hydroponic pH / EC / reservoir level / flow;
- greenhouse temperature / humidity / light;
- water capture / treatment / quality observations;
- energy production / consumption / subsystem metering;
- soil reference-bed measurements;
- compost temperature / moisture / oxygen / pH;
- later fabrication-machine or storage/environment readings.

Those measurements may share generic concepts such as subject identity, source identity, observation time, units, provenance and currentness evidence, but their domain meaning must remain owned by their native profiles.

## 2. Governing theorem

```text
measurement recorded
!= measurement calibrated
!= source authenticated
!= measurement accurate
!= domain state current
!= derived finding valid
!= standards/safety assessment
!= actuation authority
```

A productive-node observation profile may make those distinctions explicit. It may not collapse them.

## 3. Reuse existing semantic/evidence owners

The profile should consume existing owners rather than invent parallel primitives:

- namespace-safe `SchemaRef` / `SemanticRef` for semantic identity;
- EPI role-safe `ObservationId` / `SourceId` semantics;
- EPI provenance/derivation relations;
- existing temporal-evidence architecture for timestamps/currentness evidence;
- qualified `EvidenceLease` for bounded evidence-reuse horizons where its theorem applies;
- domain-specific currentness semantics;
- translation receipts when projecting a native source record.

Architecture rule:

```text
018F adapter
= composition layer
not foundational identity/evidence/currentness replacement
```

## 4. Candidate binding shape

Conceptually:

```text
PhysicalObservationBindingV1 {
    observation: ObservationId,
    subject: SemanticRef,
    source: SourceId,
    native_record: SemanticRef,
    measurement_profile: SchemaRef,
    temporal_evidence_ref: SemanticRef,
    provenance_ref: SemanticRef,
    evidence_lease_ref: Option<SemanticRef>,
}
```

Exact Rust ownership and field names are intentionally not frozen here until reconciliation against the qualified semantic/EPI primitives.

The native value/payload remains in the native source record or a profile-specific fixture.

Do not create:

```text
TelemetryEvent { name, value, timestamp }
```

as a universal substitute for source-domain semantics.

## 5. Observation identity

Each semantic observation must have a stable role-safe identity distinct from transport delivery and storage identity.

```text
ObservationId
!= Holochain ActionHash
!= MQTT message ID
!= HTTP request ID
!= delivery attempt ID
!= database row position
```

The native runtime record can be referenced, but semantic observation identity must remain namespace/profile aware.

## 6. Subject identity

Every physical observation binds an exact subject.

Examples:

```text
reservoir-volume observation
-> reservoir subject

nutrient-solution pH observation
-> solution/reservoir context subject

greenhouse ambient humidity observation
-> climate-zone subject

energy consumption observation
-> exact node/subsystem/metered-load subject

compost oxygen observation
-> compost batch subject
```

Two identical numeric values with identical units but different subjects are not the same observation.

```text
6.2 pH of nutrient solution
!= 6.2 pH of compost extract
```

## 7. Measurement profile

The measurement profile carries the semantics needed to interpret the native payload.

A profile should define, as applicable:

- phenomenon / quantity being observed;
- unit/profile;
- scalar/vector/categorical representation;
- sampling method;
- aggregation interval;
- time semantics;
- instrument/method class;
- precision/resolution semantics;
- uncertainty/error representation;
- missing/out-of-range semantics;
- whether the value is direct or derived;
- profile version.

Example conceptual profiles:

```text
hydroponic.solution.ph.direct/v1
hydroponic.solution.ec.direct/v1
water.volume.interval-metered/v1
energy.consumption.interval-kwh/v1
compost.core-temperature.direct/v1
soil.volumetric-water-content/v1
```

Do not use a shared display label such as `temperature` or `pH` as semantic identity.

## 8. Units

A unit is necessary but not sufficient for semantic equivalence.

```text
same unit
!= same phenomenon
!= same subject
!= same measurement method
```

For H0 fixtures, explicit `{ value, unit }` objects remain acceptable when paired with a versioned measurement profile.

Avoid unlabeled naked floats in cross-domain fixture data.

Where a qualified reusable quantity/unit type already exists, reuse it rather than defining another one here.

## 9. Source identity

`SourceId` identifies the source role, but source identity alone does not establish quality or authenticity.

```text
known sensor source
!= authenticated sensor
!= calibrated sensor
!= trusted sensor
```

The binding should preserve whether the source was:

- physical sensor/instrument;
- human observation;
- meter/provider record;
- imported external dataset;
- synthetic fixture;
- derived software output.

This source-class information belongs in an explicit profile/evidence path, not inferred from identifier spelling.

## 10. Synthetic fixtures

H0 data must remain unmistakably synthetic.

A synthetic fixture may still exercise:

- identity;
- unit semantics;
- currentness logic;
- duplicate delivery;
- conflicting observations;
- translation receipts;
- recommendation/authority boundaries.

But:

```text
synthetic physically plausible value
!= measured physical-world fact
```

No adapter may strip the synthetic-fixture provenance marker and promote the observation into real-world evidence.

## 11. Temporal evidence

A source timestamp is evidence supplied under a source/profile, not universal trusted time.

Preserve:

```text
source-declared observed time
transport/provider observed time
capture/import time
currentness evaluation time
```

as separate coordinates where they differ.

Do not collapse them into one `timestamp` whose authority is unclear.

Reuse the current temporal-evidence architecture instead of defining a new trusted clock in 018F.

## 12. EvidenceLease

Where already-qualified evidence may be reused for a bounded interval, reference the qualified `EvidenceLease` primitive.

Its ceiling remains:

```text
EvidenceLease
!= source authenticity
!= semantic currentness
!= physical accuracy
!= action authority
```

A valid lease means only that the already-qualified evidence may be reused within its bounded verification horizon under the relevant theorem.

## 13. Domain currentness

There is intentionally no universal:

```text
current: true
```

for all physical observations.

Different domains can define currentness differently:

- a fast-changing pump state may stale quickly;
- a calibration certificate may remain usable for a longer interval;
- a crop-cycle identity may remain historically valid indefinitely;
- an energy interval record is a historical interval fact rather than a live state;
- a water-quality sample does not automatically represent the present water state.

The 018F binding may reference currentness evidence/disposition, but the positive theorem remains domain/profile owned.

## 14. Direct versus derived observations

Preserve a strict boundary between direct source values and derived metrics.

Examples:

```text
raw meter interval kWh
!= calculated energy intensity per kg output

raw water meter liters
!= calculated water-use efficiency

raw temperature series
!= heat-stress finding

raw pH reading
!= potability assessment
```

Derived metrics receive their own derivation/provenance identity and cannot overwrite the source observations.

## 15. Water purity boundary

Current `water-purity::QualityReading` co-locates raw source-like fields with derived suitability fields.

For 018F:

```text
temperature / turbidity / pH / TDS / contaminant measurements
-> physical observation candidates

potability_score
-> derived assessment candidate

meets_who_standards / meets_epa_standards
-> source-domain assessment assertions unless independently qualified
```

Never map those booleans to a generic verification bit.

## 16. Process suitability boundary

Even a qualified drinking-water assessment is not a universal process admission.

```text
PotableAssessment
!= HydroponicWaterAdmission
!= AquaponicWaterAdmission
!= IndustrialProcessAdmission
!= DischargeAdmission
```

Each process profile defines its own requirements/evidence.

## 17. Energy boundary

Current `EnergyProduction` and `EnergyConsumption` provide useful amount/interval semantics, but meter provenance is limited.

For 018F:

```text
EnergyProduction.amount_kwh / interval
-> source assertion / source observation candidate under explicit profile

EnergyProduction.verified
-> source-domain assertion only
```

until a separate verification receipt proves what verification means.

A `verified: true` field may not satisfy 018F's calibration/source/verification requirements by itself.

## 18. Meter readings

A numeric meter reading without meter identity/profile is incomplete for qualification-grade telemetry.

A stronger profile may reference:

```text
meter identity
meter model/profile
calibration evidence
channel/register identity
reading type
interval/start/end
unit
counter rollover/reset semantics
source authentication evidence
```

018F does not require every current domain to possess those fields. Absence is represented as absence/limitation.

## 19. Hydroponic measurements

First hydroponic profile candidates:

- reservoir volume;
- make-up water volume;
- recirculation/flow rate;
- pump observed state;
- nutrient-solution pH;
- electrical conductivity;
- solution temperature;
- dissolved oxygen where used;
- nutrient dosing amount;
- leakage/discharge observation.

Each binds an exact subject and measurement profile.

Do not overinterpret EC:

```text
EC observation
!= exact nutrient composition
```

unless a separately qualified method/model establishes that relationship.

## 20. Greenhouse climate measurements

Candidates:

- ambient temperature;
- relative humidity;
- light / PAR / DLI profile where instrumentation supports it;
- canopy temperature;
- CO2 concentration when measured;
- fan/heater/cooling observed state.

A controller setpoint is not an observation of physical conditions.

```text
setpoint
!= measured environment
```

## 21. Soil reference-bed measurements

Candidates:

- soil moisture under an exact measurement profile;
- soil temperature;
- pH under a defined sampling method;
- nutrient/chemistry assay refs;
- irrigation input;
- ambient conditions.

Do not equate the soil profile with hydroponic root-zone profiles merely because similar labels exist.

## 22. Compost measurements

Reuse native compost-control semantics wherever possible.

Observed current fields include:

- core temperature;
- moisture percentage;
- oxygen percentage;
- pH;
- sensor identity;
- timestamp.

018F should wrap/project those facts without changing compost-control ownership.

The useful native separation remains:

```text
CompostReading
!= CompostAction recommendation
!= executed CompostAction
```

## 23. Calibration evidence

Calibration is evidence about an instrument/method, not a boolean field on the measurement.

Conceptually:

```text
measurement observation
+ calibration evidence ref
```

may support stronger use under a domain profile.

But:

```text
calibration evidence current
!= measurement physically correct
```

It narrows one source of uncertainty; it does not eliminate all error.

018F should not define a calibration authority unless a reusable qualified owner already exists or multiple domains demonstrate identical requirements.

## 24. Precision and uncertainty

Where instrumentation supports it, preserve:

- resolution;
- stated accuracy;
- uncertainty/error bounds;
- detection limit;
- confidence interval for derived metrics;
- sensor status/quality flags.

Absence of uncertainty metadata must not become implied zero uncertainty.

```text
uncertainty unspecified
!= uncertainty zero
```

## 25. Missing observations

Missing is a first-class state.

```text
missing
!= zero
!= nominal
!= unchanged
!= safe
```

The H0 corpus must include missing-after-restart and unavailable-source cases.

Current-state reconstruction must represent incompleteness explicitly.

## 26. Stale observations

A stale source value remains historical evidence but must not silently fill a live state view.

```text
historically valid observation
!= current observation
```

Later currentness logic may select/qualify an applicable observation according to its domain profile.

018F itself does not choose the current source head.

## 27. Conflicting observations

Two same-kind sources may disagree.

The default rule is:

```text
conflict
!= average-as-truth
```

Preserve:

- both observation identities;
- both source identities;
- timing/currentness evidence;
- instrument/profile evidence;
- explicit conflict/uncertainty state.

A later domain-specific fusion method may derive an estimate, but that estimate is a new derived object with method/provenance.

## 28. Duplicate delivery

Semantic observation identity is independent of delivery attempts.

```text
same observation delivered twice
!= two physical observations
```

The interface/seam layer should deduplicate by semantic/delivery identities under the active profile while retaining delivery receipts as needed.

## 29. Delayed and reordered delivery

Arrival order is not observation-time order and neither automatically determines domain currentness.

```text
received later
!= observed later
!= more current
```

Adapters must preserve observed-time and delivery-time evidence separately where material.

## 30. Restart/recovery

After restart, absence of a source must remain explicit.

Do not reconstruct:

```text
last known sensor value
```

as:

```text
current sensor value
```

unless the domain currentness theorem explicitly permits it within its evidence horizon.

## 31. Partition/reconnect

During network partition:

- local observations retain local/source identity;
- they do not become globally current by virtue of later delivery;
- reconnect must not rewrite historical observed times;
- stale derived summaries do not become authoritative because source data later arrived.

Federation/admission semantics remain separate.

## 32. Observation versus analysis

Symthaea consumes observations through the typed analysis boundary.

It may emit:

- diagnostic candidates;
- anomaly findings;
- predictions;
- forecasts;
- counterfactuals;
- recommendations.

But:

```text
AnalysisArtifact
!= SourceObservation
```

unless a separate source/admission process explicitly creates a new source-owned observation from a real measurement process.

## 33. Observation versus decision/authority

No physical observation authorizes an effect by itself.

```text
low reservoir observed
!= permission to open valve

high temperature observed
!= permission to alter HVAC

pump-stop finding
!= permission to restart pump
```

Decision/authorization/effect paths remain separate.

## 34. Threshold rules

A declared threshold may produce a derived alert/finding:

```text
source observation
+ threshold profile
-> threshold finding
```

The threshold profile must be identified and versioned.

The resulting finding is derived, not a mutation of the source observation.

## 35. Standards assessment

Likewise:

```text
source measurements
+ standards profile/revision
+ assessment method
-> DerivedAssessment
```

The assessment must bind the exact standard/profile revision and evidence used.

Later revisions do not retroactively rewrite the original assessment.

## 36. Cross-domain aggregation

A productive-node dashboard may combine observations into a view such as:

```text
food output
water input
energy input
waste output
labor
uptime
```

but the view is derived.

```text
cross-domain dashboard row
!= source record
```

The view must retain links to exact source observations and expose incomplete/stale/conflicting states.

## 37. Multi-objective metrics

Derived efficiency metrics may include:

```text
water input / accepted output mass
energy / accepted output mass
nutrient input / accepted output mass
labor duration / accepted output mass
```

Each metric must state:

- numerator source refs;
- denominator source refs;
- interval/cycle basis;
- units;
- inclusion/exclusion rules;
- derivation profile.

Do not produce a scalar winner by default.

## 38. Source authentication

018F does not assume a sensor is authentic because its identifier is well formed.

Future source authentication may use:

- local device identity;
- signed measurements;
- secure gateway attestation;
- Xenia transport/authentication;
- physical inspection/calibration processes;
- provider receipts.

These are evidence inputs, not properties of `ObservationId` itself.

## 39. Security boundary

A signed sensor reading can prove a signer/source statement under its crypto profile. It does not prove the physical value is correct.

```text
valid signature
!= correct calibration
!= correct installation
!= non-tampered physical process
!= physical truth
```

This distinction should be present in qualification/nonclaims.

## 40. Data minimization

Physical telemetry may reveal sensitive household, facility, labor or operational patterns.

The shared binding should reference only what is needed for interoperability.

Do not require all raw high-frequency telemetry to be globally replicated merely because the semantic envelope is federated.

Profiles may support:

- local retention;
- aggregated export;
- redacted views;
- evidence commitments;
- purpose-scoped disclosure.

## 41. Export / replaceability

Exports must preserve enough data to distinguish:

- native source record;
- semantic observation identity;
- subject;
- source;
- measurement profile;
- unit/profile;
- observed-time evidence;
- provenance;
- currentness/lease evidence where used;
- derived assessments separately.

Replacing Holochain, a sensor vendor, database or dashboard must not reinterpret historical observations.

## 42. Candidate profile registry

The first showcase should define only the profiles it actually needs.

Suggested initial families:

```text
CEA.water
CEA.solution-chemistry
CEA.flow
CEA.climate
CEA.energy
CEA.output
CEA.soil-reference
CEA.compost
```

These are organizational families, not protocol namespaces until reconciled with the existing schema registry.

Avoid trying to standardize every possible physical measurement before the H0/H1 fixtures demonstrate repeated need.

## 43. Required negative corpus

At minimum:

1. missing sensor becomes zero -> reject;
2. missing sensor becomes nominal -> reject;
3. stale reading populates current state -> reject;
4. duplicate delivery creates duplicate semantic observation -> reject;
5. delayed arrival treated as newer observation -> reject;
6. identical value/unit from different subjects collapses identity -> reject;
7. identical value/subject under different measurement profile collapses identity -> reject;
8. raw provider timestamp treated as trusted time -> reject;
9. valid EvidenceLease treated as domain currentness -> reject;
10. uncalibrated source labeled calibrated -> reject;
11. absent uncertainty interpreted as zero -> reject;
12. conflicting sensors automatically averaged into source truth -> reject;
13. water standards boolean becomes independent verification -> reject;
14. potable assessment becomes hydroponic process admission -> reject;
15. `EnergyProduction.verified` becomes verification receipt -> reject;
16. meter value without meter/profile provenance becomes qualification-grade telemetry -> reject;
17. compost pH becomes hydroponic pH -> reject;
18. setpoint becomes physical observation -> reject;
19. Symthaea prediction becomes source observation -> reject;
20. observation attempts direct actuation -> reject;
21. signature validity becomes physical truth -> reject;
22. cross-domain summary overwrites source records -> reject;
23. unknown profile coerced into familiar profile -> reject;
24. synthetic fixture loses synthetic provenance -> reject.

## 44. Positive corpus

At minimum:

1. native compost reading projects through 018F while compost-control remains source owner;
2. synthetic hydroponic pH observation carries explicit subject, unit/profile and synthetic provenance;
3. energy interval assertion remains usable as bounded source evidence while `verified` stays non-authoritative;
4. two conflicting pH sensors remain separate observations and produce explicit conflict state;
5. stale reading remains historical but is excluded from a live current-state projection;
6. duplicate delivery reuses one semantic observation identity;
7. derived efficiency metric links exact numerator/denominator source observations;
8. unknown future measurement profile remains opaque/round-trippable without semantic guessing.

## 45. Qualification ladder

Recommended progression:

```text
018F architecture/profile
-> H0 synthetic cross-domain fixtures
-> deterministic adapter evaluator
-> exact qualification
-> H1 physical wet-bench sensors
-> authenticated/calibrated-source profiles as separate theorems
```

Do not begin with autonomous control.

## 46. Nonclaims

MYC-INT-018F does not establish:

- universal telemetry semantics;
- sensor authenticity;
- sensor calibration;
- measurement accuracy;
- trusted time;
- generic currentness;
- standards compliance;
- food/water safety;
- IoT security;
- physical-process safety;
- actuation authority;
- Symthaea correctness.

It defines the observation boundary needed so those stronger claims can be made, qualified, or refused explicitly by the correct owners.
