# MYC-INT-018K — Machine-Readable H1 Wet-Bench Manifest

Status: planning/fixture artifact only. Tracks #3241. Child of MYC-INT-018J / PR #3240.

## Purpose

Freeze the first machine-readable H1 bench profile before hardware selection or device-adapter implementation.

The manifest is deliberately **capability-class based**, not SKU based.

```text
manifest requirement satisfied
!= hardware qualified
!= sensor calibrated
!= process safe
!= crop/agronomic qualified
```

## Files

Canonical planning manifest:

`docs/interoperability/fixtures/MYC_INT_018K_H1_WET_BENCH_MANIFEST.json`

The manifest binds:

- H1a/H1b/H1c stages;
- physical wet/dry/ambient/containment zones;
- acquisition cadence classes;
- channel classes and semantic separations;
- calibration/check metadata expectations;
- vendor-neutral BOM capability classes;
- 18 H1 failure-injection cases;
- optional-runtime/non-dependency claims;
- explicit design-only claim ceiling.

## Channel design rules

The registry preserves distinctions that later implementation must not erase:

```text
pump command
!= pump energized state
!= measured flow

reservoir level
!= reservoir volume

pH observation
!= suitability assessment

EC observation
!= nutrient adequacy assessment

power
!= energy

missing
!= zero

stale
!= current

synthetic/replayed
!= physical
```

The channel registry is not yet a qualified Mycelix/EPI observation schema.

## Provider boundary

Native acquisition belongs to the Luminous Edge field-sensor provider line:

- `Luminous-Dynamics/luminous-dynamics#3354`;
- draft architecture PR `Luminous-Dynamics/luminous-dynamics#3355`.

Intended direction:

```text
native instrument / ecosystem
-> Edge read-only provider evidence
-> explicit adapter
-> future qualified 018F observation binding
```

Until the exact semantic/evidence dependencies in 018I qualify, the JSON identifiers in this manifest are fixture/profile identifiers only.

They are not substitutes for `SemanticRef`, EPI `ObservationId`, `SourceId`, or `AssessmentId`.

## Acquisition classes

The first profile records engineering defaults, not universal agronomic requirements:

- Class A: fast hydraulic/electrical state, candidate ~1 Hz where supported;
- Class B: chemistry/environment, candidate 5–15 s;
- Class C: ordinary persistent projection, candidate 1–5 min plus separate event/fault transitions.

A material cadence or aggregation change requires a new profile generation rather than silent mutation.

## Requirement states

Channels are classified as:

- `Required` — required by the named H1 stage/profile;
- `Conditional` — required only when the selected physical process/profile depends on it;
- `Optional` — useful but not required for the first H1 exit gate;
- `Deferred` — explicitly later.

An absent optional/conditional channel remains absent/unknown; it does not receive a nominal value.

## Calibration/check semantics

Manifest values such as `RequiredOrExplicitlyMissing` mean only that a run/profile must record whether calibration/check evidence exists.

```text
calibration evidence missing
!= instrument unusable in every context

instrument present
!= calibration current
```

Acceptance/currentness belongs to the future exact measurement/evidence profile.

## BOM semantics

The BOM section names capability classes rather than purchasable products.

Hardware selection should later produce a mapping:

```text
exact hardware subject
-> satisfies declared capability class under profile/evidence
```

rather than changing the manifest to match whatever was purchased.

No brand/model is protocol identity.

## Safety boundary

Every channel is fixed to:

`actuation_authority = false`

The H1 physical bench may use local/manual pump operation for controlled testing, but the machine-readable observation/channel profile creates no software authority to actuate the pump, dose nutrients, adjust pH, operate valves, or control climate.

Wet-side electrical safety remains a separate physical/facility engineering obligation.

## Failure-injection contract

The manifest freezes FI-01 through FI-18 so later hardware/provider qualification has a stable test vocabulary.

The expected strings are semantic expectations only. They are not an executable evaluator or physical PASS.

Examples:

```text
FI-08 pump command without flow
-> command and measured flow may disagree

FI-15 missing energy meter
-> energy unknown, not zero

FI-17 unauthorized actuation attempt
-> analysis/assessment path has no effect authority
```

## Runtime optionality

The manifest explicitly records that local acquisition does not require:

- Mycelix;
- Holochain;
- Symthaea;
- Integral ITC;
- Fleet.

This protects the field station from accidentally making federation, accounting or cognition load-bearing for basic sensing.

## Next use

After review, 018K should become an input to:

1. H1 hardware capability selection;
2. Edge SENSOR-001 adapter fixture planning;
3. H1 run-manifest generation;
4. physical fault-injection campaign design;
5. later 018F/018G projection after their dependencies qualify.

## Nonclaims

018K does not establish:

- hardware compatibility;
- procurement suitability;
- sensor accuracy/calibration;
- trusted time/currentness;
- electrical/water safety;
- hydroponic process suitability;
- crop performance;
- food safety;
- commercial viability;
- autonomous control authority;
- Mycelix/Symthaea qualification.
