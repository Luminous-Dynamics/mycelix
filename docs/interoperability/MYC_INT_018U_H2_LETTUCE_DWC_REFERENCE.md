# MYC-INT-018U — H2 Lettuce / DWC Reference Crop Research Profile

Status: research/design fixture only. Tracks #3309. Child of MYC-INT-018S/T.

## Purpose

Freeze the first evidence-based crop-class/system reference for H2 without binding a cultivar or turning literature targets into physical qualification criteria.

Reference crop class:

- `Lactuca sativa` leaf/butterhead lettuce class;
- small DWC/floating hydroponic reference system;
- cultivar `Unbound`.

```text
reference crop selected for evidence clarity
!= universally best crop
!= cultivar selected
!= crop qualified
```

## Why lettuce first

University/extension literature provides a comparatively explicit controlled-environment lettuce evidence model that aligns with the H1/H2 instrumentation chain:

- seed-to-harvest timing can be observed over a relatively short bounded cycle;
- useful output is directly countable/weighable;
- root-zone chemistry, temperature, oxygenation, light and ambient conditions are well-described measurement classes;
- crop disorders/failures remain observable rather than hidden behind a proprietary appliance score.

The first H2 profile uses lettuce as a reference conformer because it is legible and measurable, not because basil, pak choi, spinach or other crops are invalid.

## Source registry

The machine-readable profile retains source URLs, retrieval date and source-specific semantics. Important references include Cornell Controlled Environment Agriculture, Virginia Cooperative Extension, UF/IFAS and Oklahoma State Extension.

No literature value is promoted into a universal crop law.

## Reference values

The Cornell CEA lettuce reference describes an approximately 35-day seed-to-harvest cycle under its controlled environment. It also gives a reference DLI around 17 mol/m²/day, 24 °C day / 19 °C night air temperatures, approximately 50–70% RH, crop-profile pH around 5.6–6.0 and an EC expressed **above source-water EC**.

These values stay tagged to that reference profile.

Other extension guidance uses broader ranges; disagreement is retained rather than averaged away.

```text
source A target
!= source B target
!= physical acceptance threshold
```

## Light evidence upgrade

H1's generic optional `light` channel is not enough for H2 crop-performance evidence.

The crop profile requires a photosynthetically meaningful light measurement/profile:

```text
PPFD / PAR observation
+ exact integration window
-> DLI evidence
```

Lux, foot-candles, fixture electrical power or command state cannot substitute for crop-relevant PAR/DLI evidence.

A quantum/PAR measurement capability is therefore an explicit H2 hardware gap.

## Dissolved oxygen upgrade

H1 marks dissolved oxygen conditional. For the DWC lettuce reference profile it becomes `RequiredUnderCropProfile`.

The literature registry retains source-specific DO guidance; the physical acceptance threshold remains unbound until the exact H2 system/cultivar profile is frozen.

```text
DO sensor missing
!= DO adequate
```

## CO2 boundary

The first showcase profile does not require CO₂ enrichment.

Some controlled-environment reference systems use enriched CO₂. An ambient-CO₂ showcase must not claim equivalent growth timing/output merely because other environmental variables resemble those studies.

## Output evidence

A future physical H2 run should preserve:

- sowing/lot event;
- germination/emergence state;
- transplant event when used;
- plant/raft/bed count and losses;
- production window;
- harvest event;
- gross harvested mass/count;
- rejected/lost mass/count;
- usable-output disposition under an explicit profile;
- post-cycle inspection/outcome feedback.

No minimum harvest mass is frozen before cultivar/system geometry are selected.

## Risk vocabulary

The planned crop profile explicitly preserves observations for:

- tipburn/calcium-transport disorder;
- low/root-zone oxygenation event;
- root disease/rot symptom;
- pH/EC/nutrient drift;
- uneven delivery where applicable;
- DLI too low/high under selected profile;
- air/root-zone temperature excursion;
- germination failure;
- plant loss/mortality;
- contamination/unknown-quality event;
- sensor/calibration failure;
- pump/aeration/power outage;
- output below expected profile with cause unresolved unless independently evidenced.

A symptom is not automatically a causal diagnosis.

## Cultivar boundary

Cultivar remains unbound. A later admission profile should compare exact seed/cultivar candidates for provenance, availability, architecture, heat/tipburn tolerance, disease profile, expected cycle, output uniformity and compatibility with exact DWC geometry.

## Food-safety boundary

```text
harvest observed
!= edible qualified
!= food-safe qualified
!= marketable qualified
```

Food handling/safety/storage requires separate ownership and evidence.

## Nonclaims

018U does not establish physical crop duration, crop performance, food safety, cultivar superiority, commercial viability, N2 maturity, food independence, ecological sustainability, governance standing or federation authority.
