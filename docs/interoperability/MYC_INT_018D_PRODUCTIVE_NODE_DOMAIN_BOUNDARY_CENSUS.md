# MYC-INT-018D — Productive-Node Domain Boundary Census

Status: architecture / semantic census only. Tracks the productive-node follow-on to MYC-INT-018C.

Observed repository date: 2026-09-27.

## 1. Purpose

Before composing the productive-node showcase into executable adapters, audit the current Mycelix domain models for fields that are safe to reuse directly, fields that require translation, and fields that must **not** be promoted into evidence or authority merely because they already exist in a domain entry.

The central rule is:

```text
existing field
!= interoperable semantic identity
!= verified evidence
!= authorization
```

This census intentionally prefers a narrow adapter over changing mature domain models prematurely.

## 2. Classification vocabulary

Every relevant current model is classified as one of:

- `ReuseAsSourceCandidate` — may serve as a domain-owned source fact after normal source/currentness qualification;
- `ReuseWithAdapter` — useful existing semantics, but requires an explicit projection/translation profile;
- `DerivedClaimBoundary` — field expresses an interpretation/assessment and must not be treated as raw source truth;
- `AuthorityBoundary` — field or status cannot confer effect/governance authority;
- `SemanticSplitRequired` — current type combines concepts that the productive-node profile must keep separate;
- `Gap` — current model does not carry enough identity/provenance/measurement semantics for the showcase requirement;
- `Deferred` — not required for the first physical profile.

These are architecture classifications, not qualification status.

## 3. Food production — current soil-first model

Current owner:

`mycelix-workspace/mycelix-commons/zomes/food-production`

Observed current types include:

```text
Plot {
  area_sqm,
  soil_type: SoilType,
  plot_type: PlotType,
  location_lat,
  location_lon,
  steward,
  status
}

Crop {
  plot_hash,
  name,
  variety,
  planted_at,
  expected_harvest,
  status,
  ...
}

YieldRecord {
  crop_hash,
  quantity_kg,
  quality_grade,
  harvested_at,
  ...
}
```

### Finding F1 — greenhouse currently still requires soil type

`PlotType` includes `Greenhouse`, but every `Plot` also requires `soil_type: SoilType`.

Therefore:

```text
PlotType::Greenhouse
!= hydroponic production environment
```

and a hydroponic greenhouse must not invent a fake soil classification solely to satisfy this type.

Classification: `SemanticSplitRequired`.

### Finding F2 — crop lifecycle is reusable with an environment adapter

The existing `Crop` and `YieldRecord` concepts are useful for planting/growing/harvest/output lineage, but `Crop` points to `plot_hash` and inherits the soil-first production context.

Classification: `ReuseWithAdapter`.

Candidate direction:

```text
ProductionEnvironmentRef
  -> SoilPlot(ActionHash)
  -> HydroponicSystem(SemanticRef)
  -> OtherQualifiedProfile(SemanticRef)
```

Do not replace the existing `Plot` type until a separate migration theorem proves that is desirable.

### Finding F3 — soil resource inputs are not hydroponic nutrient inputs

Current `ResourceInput` is explicitly a contribution applied to a soil/plot and its `ResourceType` taxonomy includes compost/manure/mulch/etc.

Therefore:

```text
ResourceInput
!= HydroponicNutrientDose
!= ProcessWaterTreatment
```

Classification: `SemanticSplitRequired`.

## 4. Water flow — allocation/economics is not physical flow telemetry

Current owner:

`mycelix-workspace/mycelix-commons/zomes/water-flow`

Observed current concepts include:

- `WaterSource`;
- `WaterShare`;
- `H2OCredit`;
- `WaterTransaction`;
- `UsageRecord`.

The module documentation explicitly frames the FLOW pillar around water allocation, credits and water economics.

### Finding W1 — UsageRecord is consumption/allocation evidence, not a hydraulic time series

`UsageRecord` carries liters used and an optional `meter_reference`, but does not itself model:

- instantaneous/interval flow rate;
- pressure;
- pump state;
- reservoir level;
- meter identity/profile;
- calibration/currentness;
- source observation identity beyond the Holochain action.

Classification: `ReuseWithAdapter` for aggregate use; `Gap` for hydroponic hydraulic telemetry.

Do not overload `UsageRecord` to mean `FlowObservation`.

### Finding W2 — water allocation authority is separate from physical availability

```text
WaterShare / H2OCredit
!= liters physically present
!= treatment quality
!= authorization to actuate a valve/pump
```

Classification: `AuthorityBoundary`.

## 5. Water purity — raw observations and derived suitability claims are currently co-located

Current owner:

`mycelix-workspace/mycelix-commons/zomes/water-purity`

Observed `QualityReading` includes raw measurement fields such as:

- temperature;
- turbidity;
- pH;
- TDS;
- dissolved oxygen;
- nitrates;
- arsenic;
- lead;
- coliform / E. coli;
- chlorine.

It also includes:

```text
potability_score: f32
meets_who_standards: bool
meets_epa_standards: bool
```

Current validation binds `sampler` to the committing agent and validates some physical ranges, but the observed validator does not derive the potability score or standards flags from a frozen assessment profile.

### Finding W3 — standards flags are derived claims, not raw source measurements

For productive-node interoperability:

```text
pH / turbidity / contaminant measurement
= source-observation candidate

potability_score
= derived assessment candidate

meets_*_standards
= policy/profile-bound assessment candidate
```

Classification: `DerivedClaimBoundary`.

A productive-node adapter must never map the booleans directly to a generic `Verified`, `Safe`, `Potable` or `Admitted` state.

### Finding W4 — drinking-water suitability is not process-water suitability

Even a qualified potable-water assessment would not automatically prove suitability for every crop, hydroponic recipe, industrial process, aquaculture system or discharge destination.

```text
PotableAssessment
!= HydroponicProcessAdmission
!= DischargeAdmission
```

Classification: `SemanticSplitRequired`.

## 6. Energy — useful immutable production/consumption records, insufficient verification provenance

Current owner:

`mycelix-workspace/mycelix-energy/zomes/grid`

Observed source-shaped entries:

```text
EnergyProduction {
  id,
  producer_did,
  project_id,
  amount_kwh,
  timestamp,
  period_hours,
  meter_reading,
  verified: bool
}

EnergyConsumption {
  id,
  consumer_did,
  amount_kwh,
  timestamp,
  period_hours,
  meter_reading
}
```

Production and consumption records are currently immutable after creation, which is useful for source-history semantics.

### Finding E1 — `verified: bool` is not a verification receipt

Current creation validation checks producer DID form and positive energy amount but does not bind `verified` to a verifier identity, method, proof, calibration chain or receipt.

Therefore:

```text
verified == true
!= qualified verification
```

Classification: `DerivedClaimBoundary`.

The productive-node adapter must either ignore that field for trust decisions or translate it only as a source-domain assertion with explicit provenance.

### Finding E2 — meter provenance is currently too weak for qualification-grade telemetry

`meter_reading: Option<f64>` carries a value but not an explicit meter/sensor profile, unit semantics beyond contextual kWh, calibration identity, interval identity or observation-currentness metadata.

Classification: `Gap` for qualification-grade metering; `ReuseAsSourceCandidate` for explicitly synthetic fixtures or bounded source assertions.

### Finding E3 — economic trade records remain downstream

`TradeOffer` and `Trade` are valuable energy-market semantics but are not required to establish physical production/consumption observations.

```text
energy produced
!= energy offered
!= energy traded
!= energy settled
```

Classification: `ReuseWithAdapter` / optional profile.

## 7. Compost — strong domain owner with recommendation/effect separation worth preserving

Current owner:

`mycelix-workspace/mycelix-commons/zomes/compost-control`

Observed current model already distinguishes:

- `CompostBatch`;
- `CompostReading`;
- `CompostAction`;
- `ActionRecommender::{Sensor, AI, Manual}`;
- optional `executed_by` / `executed_at`.

This is a useful local pattern because it can express:

```text
sensor/AI recommendation
!= executed compost action
```

Classification: `ReuseWithAdapter`.

### Finding C1 — compost observation semantics should remain compost-specific

Do not promote compost temperature/moisture/O2/pH directly into generic greenhouse root-zone semantics.

```text
CompostReading.ph
!= hydroponic reservoir pH observation
```

Shared quantity/unit infrastructure may be reused later, but domain subject identity remains explicit.

## 8. Fabrication — design metadata is not installation/outcome evidence

Current owner:

`mycelix-workspace/happs/fabrication`

Observed `Design` includes:

- HDC intent vector;
- parametric schema / constraints;
- material compatibility;
- repair manifest;
- safety classification;
- epistemic dimensions;
- `circularity_score`;
- `embodied_energy_kwh`.

### Finding M1 — design assessments remain design claims

```text
Design.circularity_score
!= measured circular outcome

Design.embodied_energy_kwh
!= metered manufacturing energy observation
```

unless backed by an explicit qualified calculation/measurement profile.

Classification: `DerivedClaimBoundary`.

### Finding M2 — design, manufacture, installation and repair outcome require separate identities

The productive-node repair chain must preserve:

```text
Design
!= DesignVerification
!= ManufacturedArtifact
!= InstalledArtifact
!= RepairOutcomeObservation
```

Classification: `SemanticSplitRequired` across existing owners/adapters.

## 9. Cross-domain adapter theorem

A productive-node adapter may project a source record only if it carries enough information to answer:

1. what exact source schema/profile produced it?
2. what exact source object/revision is referenced?
3. which fields are source observations versus derived claims?
4. what units and subject identity apply?
5. when was the observation made and when does it become stale/unknown?
6. what translation, if any, occurred?
7. what information was lost or added?
8. does the translated object carry zero, advisory, decision or effect authority?

If any required answer is missing, the adapter must return a typed partial/unknown/rejected disposition rather than silently strengthening semantics.

## 10. Candidate shared envelope

Do **not** introduce a universal domain event.

The smallest reusable interoperability envelope may look conceptually like:

```text
DomainObservationRef {
  semantic_ref,
  source_domain,
  subject_ref,
  observed_at,
  source_profile,
  currentness,
  provenance_ref,
  measurement_profile?,
}
```

with the actual domain payload remaining owned by Food, Water, Energy, Compost, Fabrication, etc.

This is an adapter concept only until reconciled against existing EPI/provenance/currentness primitives.

## 11. Negative controls

At minimum reject or downgrade:

1. hydroponic system encoded as a fake soil plot;
2. `WaterShare` treated as physical water availability;
3. `UsageRecord` treated as flow-rate telemetry;
4. `meets_who_standards=true` treated as independently verified potability;
5. potable assessment treated as hydroponic process admission;
6. `EnergyProduction.verified=true` treated as a verification receipt;
7. `meter_reading` without meter/profile identity treated as qualification-grade telemetry;
8. `Design.circularity_score` treated as measured lifecycle outcome;
9. compost pH treated as hydroponic root-zone pH;
10. fabricated design treated as installed repair;
11. cross-domain projection overwrites the source object;
12. missing source/currentness becomes nominal/zero.

## 12. Recommended follow-ons

### 018E — production-environment reference profile

Define the minimum typed bridge that lets `Crop` / output lineage attach to soil, hydroponic and later production environments without fabricating `SoilType`.

### 018F — physical measurement observation profile

Reconcile water/energy/greenhouse sensor observations with existing EPI/provenance/currentness primitives. Avoid a parallel generic telemetry bus.

### 018G — derived-assessment receipt profile

Create a typed distinction for profile-bound assessments such as water suitability, energy verification, circularity calculations and standards checks when no existing owner already provides it.

### 018H — productive-node conformance corpus

Extend H0 with cross-domain negative cases from this census before any physical integration.

## 13. Nonclaims

This census does not prove that the existing domain models are incorrect. They may be entirely suitable for their original application boundaries.

It proves only that several convenient fields cannot be reused as stronger cross-domain semantics without explicit translation and qualification.
