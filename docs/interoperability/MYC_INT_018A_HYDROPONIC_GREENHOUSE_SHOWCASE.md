# MYC-INT-018A — Hydroponic greenhouse Integral showcase

Status: architecture / showcase profile only. No production, food-safety, governance-authority, or autonomous-control claim.

Tracks Mycelix #3222 and complements Symthaea #6213.

## 1. Purpose

Use a physically measurable controlled-environment agriculture system as the first **integrated public showcase** for the Integral compatibility program while preserving the deliberately smaller I0 water-system fixture as the minimal interoperability theorem.

```text
I0 water fixture
= small semantic / authority / delivery proof

I1 hydroponic greenhouse
= integrated operational showcase across all five Integral systems
```

I1 must not become a hidden prerequisite for proving I0.

## 2. Preferred reference system

The first reference build should be a **sunlight-first, recirculating hydroponic greenhouse** with optional supplemental lighting rather than a sealed, LED-dominant vertical farm.

Reasoning:

- hydroponics exposes water, nutrient, energy, labor and crop-health flows clearly;
- recirculation creates measurable closed-loop behavior;
- greenhouse sunlight keeps energy intensity visible and avoids making artificial lighting the default load;
- pumps, reservoirs, filters, dosing, sensors and crops create real operational failure modes;
- the system is small enough to prototype but rich enough to exercise all five Integral seams.

This is a showcase profile, not a claim that hydroponics is categorically better than soil agriculture.

## 3. Physical reference boundary

The reference system should be decomposable into replaceable modules.

```text
water source
   ↓
make-up / treatment
   ↓
reservoir
   ↓
pump + filtration
   ↓
grow loop(s)
   ↓
return / recovery
   ↓
reservoir
```

Supporting systems:

```text
sensors
climate envelope
power metering
nutrient dosing
safety interlocks
manual operator controls
crop handling
harvest / waste handling
```

The first grow method may be NFT or another recirculating leafy-green profile, but **grow method must remain an explicit profile**, not protocol identity.

## 4. Why leafy greens / herbs first

A short-cycle crop profile gives the showcase repeated observable cycles without changing the interoperability architecture.

Useful properties:

- repeated planting / transplant / harvest events;
- measurable fresh-mass output;
- visible stress response;
- rapid feedback on environmental changes;
- relatively simple handling compared with long-cycle fruiting crops.

Crop identity is a parameter of the showcase, not a Mycelix core concept.

## 5. Integral five-system mapping

### 5.1 OAD — design / knowledge / certification boundary

OAD-compatible records may reference:

- greenhouse geometry;
- grow-loop topology;
- hydraulic design;
- reservoir / pump / filtration design;
- sensor topology;
- actuator topology;
- BOM and revision lineage;
- operating envelopes;
- crop recipes / environment targets;
- maintenance procedures;
- lifecycle assumptions;
- ecological assumptions;
- design evidence;
- certification or admission evidence.

Preserve:

```text
DesignCandidate
!= CertifiedDesign
!= locally admitted operating configuration
```

A Symthaea-generated design alternative remains a candidate until separately admitted.

### 5.2 COS — operational source facts

COS-compatible source records may include:

- crop-batch creation;
- planting;
- transplanting;
- inspection;
- maintenance;
- cleaning / sanitation;
- water make-up;
- nutrient addition;
- component replacement;
- performed work;
- harvest;
- rejected biomass;
- packaging / distribution;
- inventory change;
- equipment downtime;
- restart / recovery.

Preserve:

```text
planned task
!= assigned task
!= performed task
!= verified work
```

and:

```text
BOM quantity
!= planned requirement
!= reserved material
!= physically consumed material
```

### 5.3 ITC — optional accounting projection

ITC compatibility may consume source-owned COS labor/material records.

It must not become the owner of the source observation.

```text
PerformedWorkObservation
        ↓
optional Integral adapter
        ↓
SkillWeightingResult
        ↓
CreditIssuance
```

These identities must remain distinct.

The showcase must remain operational with ITC disabled so the neutrality of the Mycelix substrate is demonstrable.

### 5.4 FRS — telemetry / analysis / review inputs

FRS is the natural analytical seam for the greenhouse.

Candidate **source observations**:

- reservoir volume;
- make-up water volume;
- flow rate;
- pump state;
- nutrient-solution pH;
- electrical conductivity;
- solution temperature;
- dissolved oxygen where applicable;
- ambient temperature;
- relative humidity;
- light / DLI where measured;
- canopy temperature;
- electrical energy;
- nutrient dosing;
- drainage / discharge;
- crop inspection observations;
- harvested fresh mass;
- rejected / waste biomass.

Candidate **derived values**:

- VPD;
- estimated nutrient uptake;
- water-use efficiency;
- energy intensity;
- yield forecast;
- disease-risk estimate;
- maintenance-risk estimate;
- anomaly score;
- forecast confidence;
- recommendation.

Preserve:

```text
source observation
!= derived metric
!= prediction
!= recommendation
```

### 5.5 CDS — deliberation / decision boundary

CDS-like deliberation may consume source evidence and derived analysis for bounded questions such as:

- operating-envelope change;
- crop-profile change;
- maintenance policy;
- nutrient / water conservation change;
- response to recurring failures;
- expansion / reconfiguration;
- acceptance or rejection of an analytical recommendation.

Preserve:

```text
Recommendation
!= Decision
!= Authorization
!= Effect
```

## 6. Mycelix semantic waist

The showcase should primarily compose already-owned semantics:

- `SchemaRef` / `SemanticRef`;
- EPI evidence/provenance;
- source-owned versus derived state;
- decision lineage;
- authorization lineage;
- effect attempt / effect receipt;
- retry / idempotency / duplicate delivery;
- source currentness;
- federation / foreign authority;
- translation receipts;
- external source drift.

Do not create a permanent `mycelix-hydroponics-core` package unless an explicit semantic census establishes independently reusable types not already owned elsewhere.

## 7. Symthaea integration

Symthaea should enter through the typed read-only analysis contract from `SYM-INT-001B`.

```text
source-bound Mycelix state
        ↓
AnalysisRequest
        ↓
Symthaea bounded profile
        ↓
AnalysisArtifact
  authority = none
        ↓
FRS / OAD / CDS / COS consumer
```

Initial analysis classes:

- pH / EC / flow anomaly finding;
- crop-stress diagnostic candidate;
- short-horizon resource forecast;
- yield forecast;
- counterfactual operating scenario;
- sensitivity analysis;
- nutrient / water / energy optimization candidate;
- maintenance-risk prediction;
- design alternative;
- abstention under insufficient evidence.

No direct actuator authority belongs in the first showcase.

## 8. Agribot reuse without semantic collapse

Existing `symthaea-agribot` already owns several greenhouse-relevant channels:

- canopy temperature;
- light;
- crop health;
- disease risk;
- water reserve;
- yield forecast;
- human proximity;
- forecast confidence;
- reserve margin;
- treatment confidence.

Reuse these only where semantics match exactly.

Do not reinterpret soil channels as hydroponic channels:

```text
soil_moisture
!= nutrient_solution_volume

soil_nutrients
!= EC

waterlogging_risk
!= low-flow / root-zone oxygen risk

soil_exhaustion
!= nutrient-solution depletion
```

Hydroponic root-zone state needs its own explicit profile.

## 9. Source / derived / command split

The showcase must classify each field before integration.

### Source-owned physical observation

Examples:

- pH probe reading;
- flow meter reading;
- power meter reading;
- weighed harvest mass;
- operator-confirmed nutrient addition.

### Derived analysis

Examples:

- VPD;
- estimated water-use efficiency;
- anomaly score;
- projected yield;
- maintenance-risk score.

### Command / intended effect

Examples:

- set dosing pump duty;
- open valve;
- start circulation pump;
- change ventilation setpoint.

These must never share a single generic event type.

## 10. Control authority model

Phase I1 should be analysis-first.

```text
Symthaea recommendation
        ↓
local review / policy
        ↓
Decision if needed
        ↓
separate bounded Authorization
        ↓
local safety controller
        ↓
actuator attempt
        ↓
EffectReceipt
        ↓
physical OutcomeObservation
```

Hard safety interlocks should remain local and able to reject higher-level requests.

Examples:

- dry-run protection;
- reservoir-low cutoff;
- dosing maximum;
- pump current fault;
- thermal cutoff;
- manual emergency stop.

A future Xenia-secured control path may protect remote commands, but transport security must not imply actuation authority.

## 11. Showcase perturbation suite

The demo should include deterministic and real-observation cases where practical.

### Hydraulic

- circulation pump stops;
- flow restriction;
- low reservoir;
- leak / unexplained water loss;
- restart after outage.

### Chemistry

- pH drift;
- EC drift;
- incorrect dosing observation;
- stale chemistry sensor;
- contradictory probes.

### Climate / biology

- high air temperature;
- high solution temperature;
- low dissolved oxygen where applicable;
- excessive humidity;
- crop-health decline;
- disease-risk signal;
- forecast contradicted by later crop observation.

### Digital / distributed

- delayed observation;
- duplicate observation;
- source missing after restart;
- partition and reconnect;
- stale derived view;
- unknown external schema version;
- foreign-node decision presented as local authority.

### Authority

- analytical recommendation attempts direct actuation;
- authorization expired before effect;
- authorization scope does not cover target actuator;
- effect retry after unknown delivery state;
- outcome contradiction attempts to rewrite decision history.

## 12. Metrics

The first showcase should expose dimensions rather than a synthetic headline score.

### Resource metrics

- make-up water / kg accepted output;
- nutrient input / kg output;
- nutrient discharge / loss;
- electrical energy / kg output;
- water recovery / recirculation fraction;
- component / consumable use.

### Production metrics

- crop-cycle time;
- accepted fresh mass;
- rejected / waste mass;
- batch loss;
- downtime;
- recovery time.

### Human / operational metrics

- labor time as observed source events;
- operator interventions;
- maintenance events;
- alarm burden;
- manual overrides.

### Information metrics

- sensor completeness;
- currentness;
- source disagreement;
- replay completeness;
- provenance completeness;
- export completeness;
- duplicate-delivery handling.

### Analytical metrics

- forecast error;
- anomaly true/false positives;
- abstention under missing evidence;
- calibration where applicable;
- recommendation resource delta when later evaluated.

No one metric is allowed to silently become `sustainability`.

## 13. Why energy must be first-class

The showcase must meter energy explicitly.

A hydroponic/CEA system can reduce direct water use while increasing pumping, cooling, heating or lighting energy. The system therefore should never infer:

```text
lower water use
=> lower total ecological impact
```

Instead preserve separate measured axes and let downstream policy define how they are evaluated.

## 14. Qualification ladder

### H0 — synthetic hydraulic / crop-state fixture

- deterministic source observations;
- no physical hardware;
- oracle-blind analysis fixtures;
- duplicate / stale / partition cases.

### H1 — wet bench

- reservoir;
- pump;
- plumbing loop;
- sensors;
- meters;
- no crop-dependent claim required.

Goal: qualify telemetry, replay, failure detection and authority separation.

### H2 — small crop rack / greenhouse bay

- short-cycle crop;
- real source observations;
- manual decisions / authorization;
- Symthaea read-only analysis;
- measured harvest.

### H3 — integrated greenhouse module

- multiple batches / cycles;
- controlled perturbations where safe;
- operations / maintenance history;
- OAD design iteration;
- FRS review loop;
- optional ITC projection.

### H4 — federation demonstration

- second node or simulated peer;
- foreign design / observation / recommendation exchange;
- local admission remains local;
- source provenance survives export/import;
- no foreign authority inheritance.

## 15. Showcase narrative

A public demo should tell one traceable story instead of showing unrelated dashboards.

Example:

```text
1. OAD revision defines the hydroponic module and operating envelope.
2. COS creates a crop batch and work plan.
3. Sensors produce source-owned measurements.
4. FRS observes a persistent flow decline and rising crop stress.
5. Symthaea emits a diagnostic / maintenance recommendation with uncertainty.
6. CDS-like review accepts, modifies or rejects the recommendation.
7. A separate bounded authorization permits maintenance / control action.
8. COS records work and replacement material.
9. Effect and outcome receipts record what actually occurred.
10. FRS compares recovery against the pre-action review basis.
11. OAD receives a design-improvement candidate if the failure exposes a recurring design weakness.
12. ITC may independently consume verified labor/material observations if enabled.
```

Every step has a distinct identity and owner.

## 16. Architecture ratchets

The showcase fails if any implementation collapses these distinctions:

```text
planned != observed
prediction != observation
recommendation != decision
Decision != Authorization
Authorization != EffectReceipt
source fact != derived summary
water efficiency != total sustainability
high yield != low energy
model confidence != authority
foreign authority != local authority
proof of computation != truth of physical assumptions
```

## 17. Current gates

Docs / fixtures / synthetic scenario design may proceed while the following execution qualifications are unresolved.

Do not call the relevant paths qualified until their exact workflows pass.

Physical automation should be later than semantic/analysis qualification and should require its own bounded control qualification.

## 18. Nonclaims

This showcase architecture does not claim:

- that Integral's governance or economic rules are preferable;
- that hydroponics is generally superior to soil farming;
- that a small showcase generalizes to commercial agriculture;
- that Symthaea currently has agronomically validated predictive accuracy;
- that automation is safer than skilled human operation;
- that measured water savings imply lower full-system environmental impact.

The purpose is to create a rigorous, measurable interoperability and systems-engineering demonstration whose results can be evaluated rather than assumed.
