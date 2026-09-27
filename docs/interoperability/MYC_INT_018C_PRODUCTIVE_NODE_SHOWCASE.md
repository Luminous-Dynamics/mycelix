# MYC-INT-018C — Modular Productive-Node Showcase

Status: architecture only. Tracks #3225. Child of MYC-INT-018A / PR #3223.

Observed design date: 2026-09-27.

## 1. Purpose

Hydroponics remains the anchor for the Integral showcase, but it should not be the entire demonstration.

The larger target is a small **productive node** that can show food, water, energy, organic residues, repair/fabrication, preservation/distribution, evidence, deliberation and optional accounting interacting through one neutral Mycelix substrate.

This document freezes a modular architecture rather than a requirement that all modules deploy together.

```text
showcase module
!= Mycelix core dependency
!= Integral protocol requirement
!= deployment prerequisite
```

A node that runs only the hydroponic greenhouse remains valid. A node may add or remove later modules without changing the meaning of source facts from the modules that remain.

## 2. Design theorem

The showcase is valuable only if cross-domain composition does not collapse ownership.

```text
source-domain fact
+ cross-domain projection
+ analytical artifact
+ governance decision
+ authorization/effect history

must remain separately identifiable
```

Therefore:

```text
energy meter reading
!= FRS energy summary

compost reading
!= greenhouse root-zone reading

food-waste record
!= compost input until admitted

fabricated part
!= installed part

harvest record
!= distributed food

Symthaea optimization candidate
!= node operating policy
```

## 3. Productive-node reference shape

```text
                         Integral / Mycelix node
                                  |
          +-----------------------+-----------------------+
          |                       |                       |
          v                       v                       v
     OAD designs              FRS evidence             CDS review
          |                       ^                       |
          v                       |                       v
  +-------+--------+--------------+-------------+   Authorization
  |       |        |              |             |         |
  v       v        v              v             v         v
 FOOD    WATER    ENERGY       ORGANICS      WORKSHOP   bounded
  |       |        |              |             |      effects
  +-------+--------+--------------+-------------+
          |
          v
    COS source facts
          |
          +----> optional ITC projection
          |
          +----> provenance / export / federation
```

## 4. Module P1 — Hydroponic greenhouse anchor

Owner: MYC-INT-018A / #3222 and H0 corpus #3224.

Preferred initial physical profile:

- sunlight-first greenhouse;
- recirculating hydroponics;
- explicit water, nutrient and electrical metering;
- bounded/manual actuation authority;
- read-only Symthaea analysis in the first generation.

This remains the most information-dense first physical module because it exposes source telemetry, design revisions, work observations, resource consumption, failures, analysis and governance in one loop.

The greenhouse is not treated as proof that hydroponics is universally preferable.

## 5. Module P2 — Water capture, quality and reuse

Reuse existing Commons owners:

- `water-capture`;
- `water-flow`;
- `water-purity`;
- `water-steward`;
- `water-wisdom`.

Potential physical components:

- rainwater capture when locally appropriate;
- raw-water / make-up-water meter;
- storage tank;
- treatment stage;
- quality sampling;
- hydroponic make-up connection;
- discharge/recovery measurement.

Required distinctions:

```text
CapturedWater
!= PotableWater
!= TreatedProcessWater
!= NutrientSolution
!= Discharge
```

A treatment or purity observation must carry its own source identity, method/profile and currentness.

Water captured by the node cannot silently become suitable for crops, humans or another process simply because it exists in the same resource graph.

## 6. Module P3 — Energy and microgrid measurement

Reuse `mycelix-energy` rather than creating an Integral-specific energy ledger.

First generation should begin with measurement:

- grid import/export;
- total node load;
- greenhouse pump load;
- climate-control load;
- supplemental-light load where used;
- later PV production;
- later battery charge/discharge/state where available.

The Energy cluster already has project and grid concepts for production and consumption records. The productive-node adapter should consume those semantics rather than duplicate them.

Required distinctions:

```text
renewable generation
!= node consumption
!= self-consumption
!= avoided grid import
!= resilience
!= environmental superiority
```

A battery/PV addition is a later profile, not a requirement for the first greenhouse.

## 7. Module P4 — Organic residue and compost

Reuse Commons:

- `waste-registry`;
- `waste-collection`;
- `compost-control`.

The existing compost domain already owns:

- compost batches;
- process method;
- input-material references;
- process lifecycle;
- physical sensor readings;
- recommended/executed process actions.

It therefore remains the source owner for compost state.

Candidate loop:

```text
crop residue / rejected biomass
        |
        v
waste/source record
        |
        v
explicit compost admission
        |
        v
CompostBatch
+ temperature / moisture / O2 / pH observations
        |
        v
finished compost material
```

Do not write:

```text
plant waste -> nutrient solution
```

as an implicit shortcut.

Finished compost may feed a soil/reference plot or another qualified destination. Nutrient recovery back into hydroponics would require a separately qualified process and chemistry profile.

## 8. Module P5 — Soil/regenerative reference bed

Add a deliberately small soil-grown comparison profile beside the hydroponic system.

This is an experimental comparison surface, not a competitive ranking.

The comparison should use a shared measurement vocabulary where semantics genuinely match:

- water input;
- electrical energy attributable to production;
- nutrient/fertilizer/material input;
- labor time;
- crop-cycle duration;
- accepted harvest mass;
- rejected/waste biomass;
- downtime/failure/recovery;
- relevant environmental conditions.

Domain-specific measurements stay distinct:

```text
soil moisture
!= hydroponic reservoir volume

soil nutrient measurement
!= hydroponic EC

soil temperature
!= nutrient-solution temperature
```

The evaluator may state separate measured advantages/disadvantages. It must not emit a universal winner or scalar sustainability grade.

## 9. Module P6 — Food preservation and distribution

Reuse Commons:

- `food-production`;
- `food-preservation`;
- `food-distribution`;
- `food-knowledge`.

The reason to add this module is not feature breadth. It closes the provenance chain after harvest:

```text
crop batch
-> harvest
-> quality disposition
-> preservation/handling if used
-> distribution/admission
-> loss/waste observation
```

This lets the showcase measure useful output rather than only production-stage yield.

Required distinctions:

```text
harvested
!= accepted
!= preserved
!= distributed
!= consumed
```

## 10. Module P7 — Fabrication, tools and repair

Reuse:

- Fabrication hApp;
- Mycelix Manufacturing;
- Commons `tool-library`;
- existing evidence/provenance owners.

Good first repair demonstrations are deliberately mundane:

- pump/sensor bracket;
- hose/fitting adapter;
- probe mount;
- service jig;
- replacement enclosure;
- greenhouse fixture.

The value is the full evidence chain:

```text
failure observation
-> repair need
-> design candidate
-> design admission/certification where required
-> manufacture
-> installation authorization
-> performed-work observation
-> outcome observation
```

Required distinctions:

```text
DesignCandidate
!= CertifiedDesign
!= ManufacturedPart
!= InstalledPart
!= SuccessfulRepair
```

## 11. Module P8 — Mushroom circular-bioeconomy extension

This is a later optional food pathway, not part of first qualification.

Potential value:

- consumes explicitly qualified agricultural/food by-products as substrate;
- creates a second production mode with different water/energy/material semantics;
- creates spent mushroom substrate requiring its own downstream disposition;
- exercises source admission between waste/resource and food-production domains.

Required boundary:

```text
available residue
!= safe mushroom substrate
```

Feedstock suitability, contamination, food safety and species-specific cultivation remain empirical/domain-qualified concerns.

## 12. Later-only modules

### Aquaponics

Useful eventually, but defer beyond first productive-node generation because it adds:

- animal welfare;
- coupled fish/plant microbiology;
- tighter oxygen/ammonia/nitrite/nitrate control;
- additional failure propagation;
- food-safety and stocking concerns.

### Anaerobic digestion / biogas

Useful circular-energy extension but defer because it adds:

- gas handling;
- pressure/fire/explosion hazards;
- digestate quality;
- process biology;
- additional permitting/safety requirements.

### Autonomous robotics

Use Symthaea Agribot/robotics work later only through a separately qualified control plane.

```text
AnalysisArtifact
!= actuator command
```

## 13. Integral five-system mapping

### OAD

OAD-facing profiles can carry designs, revisions, operating envelopes and evidence for:

- greenhouse;
- water capture/treatment;
- PV/storage;
- compost facility;
- soil/reference bed;
- preservation infrastructure;
- fabricated replacement parts.

No design artifact mints production/installation authority by itself.

### COS

COS is the natural source owner for physical operational facts:

- planting/harvest;
- material use;
- maintenance;
- treatment operations;
- compost operations;
- fabrication/installation work;
- preservation/distribution handling.

### ITC

ITC remains optional and downstream from source observations.

```text
performed work
!= contribution weighting
!= credit issuance
```

The complete productive node must remain usable when Integral-specific accounting is disabled.

### FRS

FRS becomes particularly useful at this scale because it can compose evidence across domains without becoming their source owner.

Examples:

- food-water-energy view;
- water-loss finding;
- energy-intensity trend;
- recurring maintenance issue;
- resource trade-off analysis;
- outcome-vs-plan review.

### CDS

CDS can deliberate bounded cross-domain questions such as:

- invest in PV or water-treatment improvement;
- alter crop/operating policy;
- add a compost process;
- repair versus replace;
- expand one production method;
- respond to repeated resource/failure evidence.

The analytical system may present alternatives. Acceptance remains external.

## 14. Symthaea analytical role

Use the generic typed read-only analysis protocol after its exact qualifier passes.

Candidate analytical tasks:

- anomaly detection;
- maintenance-risk prediction;
- resource-demand forecasting;
- water-energy-food counterfactuals;
- design alternatives;
- scheduling/operations candidates;
- multi-objective trade-off surfaces;
- outcome review.

Never hide priorities inside a scalar objective without exposing them.

```text
weighted objective function
!= legitimate institutional preference
```

If a caller supplies weights/constraints, those are request inputs with provenance.

## 15. Common measurement envelope

Maintain raw dimensions rather than a single score.

At minimum:

### Food/output
- input/crop batch identity;
- accepted output mass;
- rejected/waste mass;
- cycle time;
- quality disposition where supported.

### Water
- source/capture volume;
- make-up volume;
- recirculated/process volume where measurable;
- treatment events;
- discharge/loss.

### Energy
- imported energy;
- exported energy;
- local generation;
- storage charge/discharge where present;
- major subsystem consumption.

### Materials/nutrients
- nutrient/fertilizer input;
- consumables;
- replacement parts;
- recovered materials;
- waste/residue destination.

### Work
- performed-work observation;
- actor/source identity;
- duration/quantity where applicable;
- evidence;
- no ITC weighting in the source record.

### Reliability
- downtime;
- fault/recovery events;
- maintenance interventions;
- operator interventions;
- unresolved/unknown delivery or telemetry states.

## 16. Cross-module negative controls

The showcase must reject or preserve uncertainty for at least:

1. one domain overwrites another domain's source fact;
2. FRS summary is submitted as source observation;
3. energy production record is treated as proof of greenhouse consumption;
4. captured water is treated as potable without admission evidence;
5. compost output is treated as hydroponic nutrient input without a qualified conversion process;
6. fabricated part is treated as installed without work/effect evidence;
7. harvested food is treated as distributed without distribution evidence;
8. hydroponic observation is compared to a semantically different soil measurement as though identical;
9. one metric improves while another worsens and evaluator emits a scalar winner;
10. Symthaea optimization candidate attempts direct effect;
11. foreign-node resource policy attempts local authorization;
12. module removal makes unrelated source history unparsable.

## 17. Development sequence

Use staged breadth:

```text
P1  hydroponic greenhouse
 |
P2  water + energy metering
 |
P3  organic residue / compost
 |
P4  soil/reference comparison
 |
P5  preservation / distribution
 |
P6  fabrication / repair
 |
P7  optional circular extensions
```

Each stage has an independent exit gate. Later modules must not delay the first useful physical showcase.

## 18. Physical qualification progression

Retain the 018A ladder and broaden only after each stage is stable:

```text
H0 synthetic
-> H1 wet bench
-> H2 crop rack / reference bed
-> H3 integrated productive module
-> H4 federated second node
```

A module may have its own H0/H1 qualification before joining the integrated H3 node.

## 19. Replaceability / exit

The productive node is a protocol/composition demonstration, not a lock-in architecture.

Exports must preserve:

- source-domain identity;
- schema/version identity;
- observations/history;
- provenance;
- decisions/authorizations where used;
- unresolved conflicts/unknowns;
- module profile/generation.

Replacing Mycelix Energy, Commons Water, a farm controller, PostgreSQL, Holochain or Symthaea must not silently reinterpret historical facts.

## 20. Evidence posture

The showcase should report observations and trade-offs rather than advertising a predetermined solution.

Current controlled-environment and agrivoltaic research motivates integrated food-energy-water measurement because energy, shading, crop performance and water use can move in different directions depending on climate, crop and design.

Current circular-food research motivates testing resource-recovery pathways such as composting as distinct processes with their own inputs, outputs and evidence rather than assuming all residues are automatically reusable.

These findings justify the comparison architecture. They do not establish universal superiority of any module.

## 21. Nonclaims

MYC-INT-018C does not establish:

- hydroponics > soil agriculture;
- agrivoltaics > conventional greenhouse energy supply;
- composting > every alternative waste treatment;
- mushrooms as a safe outlet for arbitrary waste;
- Integral governance/economic superiority;
- Symthaea agronomic correctness;
- autonomous agricultural safety;
- food-safety certification;
- commercial viability.

The intended result is a modular, measurable systems-integration environment whose evidence remains useful even when conclusions are mixed.