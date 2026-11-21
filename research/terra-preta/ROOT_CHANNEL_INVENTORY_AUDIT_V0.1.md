# Root-channel pathway inventory auditor v0.1

**Status:** synthetic-only structural inventory and evidence audit. It does not segment CT images, infer root origin from appearance, reconstruct hidden 3D geometry, solve flow, estimate transport or make agronomic claims.

## Why this increment exists

A root-channel review published 25 September 2026 identifies a gap between classical dual-permeability representations and living root-channel systems. The latter are evolving preferential pathways with interactions among living/dead roots, root–soil interfaces, mucilage/exudates, microbes and transported nutrients. Classical mobile/immobile or matrix/macropore domains may match some observations but do not, by themselves, identify pathway origin or encode the temporal evolution of the root network.

The first software increment should not pretend to solve those physical processes. It should make the *input evidence* explicit and fail closed when image support cannot justify a stronger statement.

## Scope

The Rust auditor consumes a versioned inventory of:
- spatial observation support (2D projection or 3D volume);
- node coordinates with source/method/resolution provenance;
- pathway segments with explicit origin class, origin evidence, connectivity evidence and morphology;
- ordered temporal snapshots with explicit root-occupancy states.

It reports structural consistency, origin-evidence coverage and which connectivity assertions are supportable. It never estimates hydraulic conductivity, flow fraction, tracer transport or microbial activity.

## Typed origin classes

- `live_root_channel`: a pathway formed by / following a living root, with direct or synthetic-known-truth provenance.
- `legacy_root_channel`: a pathway associated with a previous root after turnover/decay; this is distinct from an abiotic crack.
- `abiotic_crack`: shrink–swell or other non-root fissure.
- `textural_finger`: a preferential flow path arising from textural / wetting-front instability.
- `faunal_burrow`: a pathway associated with soil fauna.
- `other`: another documented origin.
- `unknown`: the origin is unresolved; do not force a classification.

The reported class is not made true by a label. Every pathway separately records `origin_evidence_status`: directly observed, historical trace, morphology-only, unresolved or synthetic truth.

## Observation-support and connectivity rule

A 2D section can report features and *declared endpoint adjacency in that section*. It cannot certify a connected 3D pathway. In v0.1, `observed_3d` connectivity is rejected unless the observation support is volumetric 3D. Even for a 3D volume, the auditor only records that the inventory says connectivity was observed; it does not independently reconstruct or verify the raw image.

The key distinction is:
- `endpoint_graph_component_count` = topology of supplied node/edge identifiers;
- `direct_3d_connectivity_count` = pathways marked as directly observed in an explicitly volumetric source;
- neither metric proves that a pathway carries water or solute.

## Temporal root state

Each snapshot has a strictly increasing time and one root-occupancy state per pathway:
- `occupied`
- `partially_occupied`
- `unoccupied_legacy`
- `not_applicable`
- `not_observed`

Non-root origin classes must use `not_applicable`. Root-origin segments may transition through occupied, partially occupied and legacy/unobserved states. Missing states are not imputed.

## Fail-closed invariants

- All spatial/source digests are syntactically valid SHA-256 values.
- IDs are non-empty and unique; every edge endpoint resolves to a declared node.
- Coordinates have exactly 2 or 3 finite dimensions matching the declared support.
- Length, diameter, depth and tortuosity are finite and physically admissible (`tortuosity >= 1`).
- Snapshot times are strictly increasing; every snapshot identifies every pathway exactly once.
- Non-root paths cannot be labelled as root-occupied.
- A 2D projection cannot claim direct 3D connectivity.
- Unknown/morphology-only origin and connectivity evidence remain explicitly unresolved.
- Synthetic fixture status can never be interpreted as empirical evidence.

## What v0.1 does not do

No CT segmentation, image registration, flow solver, Richards equation, DualPerm, tracer likelihood, rhizosphere reactive transport, nutrient fate, parameter inference, root biological model, field validation or treatment effect. Those are separate qualification work packages.

## Follow-on scientific benchmark

After the inventory contract is stable, compare:
1. measured retention/flux interpolation and single-domain Richards baseline;
2. fixed dual-permeability references (HYDRUS DualPerm / MACRO where supported);
3. static morphology-only preferential pathway representation;
4. dynamic morphology + root-history covariates;
5. explicit evolving root-channel network coupled to a qualified specialist flow / solute solver.

Use identical forcing, boundary conditions, sample support and calibration budgets. Hold out complete plant/root functional groups, soil profiles, storm regimes, depth bands and sites where reliable identifiers exist.

Measure event arrival-time intervals, peak timing/concentration, rising limb, tailing, cumulative tracer recovery, spatial variance where supported, water/solute balance, calibration, abstention, transfer loss and full sensing/analysis cost. Never score only the peak or one classification F1.

## Scientific source

Agarwal, I., Ojha, R. & Guha, S. (2026). *A review of preferential flow and transport through root channels*. Vadose Zone Journal, first published 25 September 2026. DOI [10.1002/vzj2.70148](https://doi.org/10.1002/vzj2.70148).

The review identifies root-channel evolution and pathway-origin ambiguity as important gaps in current dual-permeability modelling. It also discusses limits of dye and image-based methods, including dye sorption and incomplete representation of natural root–soil interactions.

## Boundary

The fixture is manufactured. A structurally valid inventory is not evidence of a real root channel, 3D connectivity, preferential water movement, nutrient transport, microbial function, crop benefit or causal impact. Any physical study needs qualified review, permissions, ecological safeguards, consent and preregistration.
