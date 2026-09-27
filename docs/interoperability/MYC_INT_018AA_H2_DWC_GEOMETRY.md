# MYC-INT-018AA — H2 DWC Geometry and Spatial Sampling Profile

Status: design/reference fixture only. Tracks #3325. Parent: MYC-INT-018U / PR #3310.

## Purpose

Bind the physical geometry and spatial-measurement semantics a future H2 DWC productive-loop run must use, without converting common university/extension layouts into universal optima.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_018AA_H2_DWC_GEOMETRY.json`

## Reference anchors

Virginia Tech DWC guidance describes production ponds/reservoirs, plant rafts, circulation and oxygenation; it notes that lettuce raft plant spaces are usually about 8 inches apart and that commercial rafts are often about 2 ft × 4 ft while varying by need.

UF/IFAS small floating-raft guidance describes a lined reservoir, floating foam surface, aerator and air stone.

Those facts are retained as source-specific reference anchors only.

```text
reference spacing
!= H2 acceptance criterion

reference raft dimensions
!= required showcase dimensions
```

## Exact physical generation

Before a physical crop cycle, `H2DwcGeometryV1` must bind:

- reservoir dimensions and usable depth;
- nominal/working/max volume profiles;
- freeboard/overflow/containment assumptions;
- raft dimensions/material/profile;
- exact planting-hole coordinates;
- hole/net-pot profile;
- spacing and edge setbacks;
- plant count and spacing stage;
- root-zone clearance;
- circulation inlet/outlet geometry;
- aeration positions;
- chemistry/DO sensor locations;
- level/distance/flow sensor geometry;
- leak containment geometry;
- service and harvest access;
- wet/dry electrical boundary;
- asset labels.

The design fixture intentionally leaves these physical values unbound rather than inventing dimensions around a literature example.

## PAR / DLI spatial evidence

A crop-light run must bind an exact sampling plane and coordinate set.

```text
one PPFD point
!= canopy PPFD map
!= canopy DLI
```

For a rotating single sensor, temporal aliasing and non-simultaneous spatial coverage remain explicit. Missing intervals must not be silently interpolated into a complete DLI claim.

## D.O. spatial evidence

A DWC run must record aeration geometry and exact D.O. sample location/depth.

```text
D.O. at one location
!= reservoir homogeneity
```

A primary sensor can still be useful, but its completeness ceiling must state the observed spatial scope.

## Cultivar admission relationship

018V candidates cannot progress beyond research admission until the geometry generation can evaluate canopy/head spread, planting density, root clearance, airflow/service access, harvest method and lighting implications.

## Run immutability

A productive run binds the exact geometry generation before evidence begins. Material geometry changes after evidence begins are retained as deviations or require a new run/profile generation; the original geometry subject is not rewritten after seeing output.

## Nonclaims

018AA does not establish optimal raft size, planting density, reservoir depth, aeration rate, PPFD uniformity, D.O. adequacy, cultivar compatibility, crop performance, food safety, economic viability or N2 maturity.
