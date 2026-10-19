# LKI-01 Conservative Serviceability Envelope Protocol v0.1 (draft)

**State:** software-screening protocol; not approved design data  
**Applies to:** concept-level modular product interfaces and service envelopes  
**Units:** millimetres; one named Cartesian reference frame per record

## Paradigm shift

A fit check is not a serviceability check. A replacement module can fit its bay and still be impossible to install through a doorway, pull out without striking structure, reach with the required tools, or remove without damaging the worktop. The engineering contract therefore models distinct volumes for:

- the module and its receiving bay;
- the installation approach / sweep;
- the service-access volume;
- the removal sweep;
- structural obstacles and explicit keep-out regions.

The v0.1 checker is deliberately conservative: it uses axis-aligned bounding boxes (AABBs), inflates moving volumes by a declared per-face clearance, and treats touching an obstacle as a conflict. AABB screening may produce false positives; it cannot validate rotated geometry, articulated motion, flexible components, human reach, thermal ventilation, electrical or plumbing interfaces, or manufacturer installation requirements.

## Data and evidence rules

1. Every envelope uses a named reference frame, explicit millimetres, lower-corner origin and positive dimensions.
2. A fit check references one module and one bay and specifies the minimum clearance at each opposing face.
3. Installation, service-access and removal envelopes are checked against all obstacle and keep-out envelopes after clearance inflation.
4. Every envelope cites a source record. Assumptions and synthetic fixtures can support only a screening result. Measured, controlled-drawing or approved sources require an exact revision and SHA-256 digest.
5. Unknown or unresolved references, missing/invalid geometry, failed face clearance, collision, or a requested physical qualification produce a blocked/failing result. Do not change unknowns to zero or omit inconvenient obstacles.
6. The checker always emits `physical_fit_qualified=false`. A passing result means only that the declared synthetic/recorded AABBs passed these narrow arithmetic checks; it never certifies a physical product.

## Software benchmark

The frozen v0.1 corpus contains ten known-answer cases:

| Cases | Required behavior |
|---|---|
| ENV-001, ENV-002 | Pass when module fit clears each face, including exact minimum clearance |
| ENV-003, ENV-004 | Fail when any face clearance is deficient |
| ENV-005, ENV-006 | Fail when installation/removal sweep collides, including after clearance expansion |
| ENV-007 | Treat a touching boundary as a conflict |
| ENV-008 | Block measured geometry with a missing digest |
| ENV-009 | Block the unsupported claim that a software screen establishes physical qualification |
| ENV-010 | Pass when the inflated moving envelope remains strictly separated from the obstacle |

The corpus is frozen once accepted; future scenarios should be added in a versioned extension, not edited to make a new implementation pass. This is a small synthetic regression suite, not an industry-standard benchmark or statistical evidence about product performance.

## Proposed gates and targets

These are programme targets to evaluate, not results already achieved:

- 100% of critical geometry claims bind to an exact revision and SHA-256 source digest before design release.
- 100% of designated modules have separately represented fit, install, service and removal checks before physical prototype release.
- Zero unresolved hard-gate failures on the approved controlled geometry.
- Zero permanent damage to the reusable structure in each designated non-destructive removal trial.
- 100% of claimed successful repairs include an observed post-repair check bound to the tested product/module revisions.
- Report P50, P90 and worst observed access/removal time by task and role; do not substitute a single average for a tail.
- Track false-positive and false-negative rates against independently reviewed CAD / physical fixtures before expanding beyond coarse AABB screening. A false negative (claimed-clear collision) is the critical safety failure; until validation exists, this checker must not be used as a safety or release authority.

## Practical next stage

1. Replace illustrative dimensions with a controlled drawing and exact source revision for a selected, locally available candidate module.
2. Independently review datum placement, envelope construction, fit clearances, and every obstruction/utility route.
3. Validate the checker against hand-computed cases and a CAD-derived fixture with known collision/non-collision answers.
4. Build an adjustable, unpowered, candidate-independent physical mockup; record actual access and removal trials only after the fixture has been inspected.
5. Revisit geometry with an appropriate CAD/kinematic collision engine before making any fit claim about rotated parts or real installation paths.
6. Before using powered or heated modules, complete the applicable manufacturer's installation conditions and qualified structural, electrical, thermal, fire, spill and appliance-safety assessments. This protocol grants no permission to energize a prototype.

## Research anchors

- ISO/TS 10303-4439:2025, Product life cycle support: https://www.iso.org/standard/91589.html. Relevant for product configuration, interfaces, maintenance tasks and lifecycle history; this draft does not claim conformance.
- EU Ecodesign for Sustainable Products Regulation (EU) 2024/1781: https://eur-lex.europa.eu/eli/reg/2024/1781/oj/eng. Digital product-passport requirements are product-specific and phased; do not imply this draft is already a compliant passport.
- 2025 observational study on user-perceived repairability: https://doi.org/10.1016/j.spc.2025.05.024. Small study (n=26) on small appliances; it motivates observation of opening and fault-identification barriers, not a universal effect size.
- IEC 60335-1 general appliance safety, including applicable Part 2 standards: https://webstore.iec.ch/. Applicability and current editions must be checked for the actual selected appliance and jurisdiction.

## Explicit non-claims

No product geometry is approved. No local South African module has been selected or qualified. No CAD/FEA/thermal/safety qualification, user study, physical fixture, physical fit test, time measurement, life test, repairability superiority, product life extension or whole-life environmental advantage has been established.
