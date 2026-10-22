# Forest Robot Observation Contract v0.1

**Purpose:** define an evidence envelope that can connect Symtropy's offline forest-scanning simulations and future field robots to Mycelix's wood-measurement provenance.

## Contract boundaries

- `ForestRobotObservation` links one observation to an immutable mission receipt, map snapshot, raw sensor artifact, coordinate-frame reference, calibration record and source references.
- Source measurement IDs resolve separately to `WoodMeasurementEvidence`. The observation envelope does not duplicate or silently rewrite those measurements.
- Coordinates are integer millimetres in a named frame; they are not presumed to be WGS84 latitude/longitude.
- The SHA-256 fields are format-validated as 64 lowercase hexadecimal characters. The caller that resolves the bytes must recalculate the digests; this crate does not resolve storage URIs or verify digital signatures.
- TerrainPatch, TrailSegment and Obstacle observations require sorted, unique `spatial_terrain_evidence_refs` that point to Sol Atlas RoboticsTerrainEvidenceEnvelope records. These references are not clearance decisions: consumers must resolve them, compare the exact map snapshot and coordinate frame, and preserve unknown/stale/conflicting hazard states.
- `position_uncertainty_mm` must be explicit and nonzero. It is a stated uncertainty, not a claim that the value is correctly calibrated.
- Review status is separate from wood evidence/grade status. `ReviewedForInventory` requires a reviewer reference and is allowed only for `FieldCaptured` observations. No field here means a timber grade, species certification, harvest permission, structural-use approval or machine-safety approval.
- The contract validates structure and references only. It does not change Holochain zome behavior, custody authority, site permission, actuator authority or any deployed workflow. It does not resolve Sol Atlas references or infer that unreported hazards are absent.

## Evidence class

| Class | Meaning | Inventory eligibility |
|---|---|---|
| `SyntheticSimulation` | Output generated entirely by a simulator | Never represents field capture |
| `HardwareInTheLoop` | Simulator/real hardware or sensors used in a controlled test | Test evidence only; not field inventory |
| `FieldCaptured` | Recorded by a physical platform in a real site | May progress through human review, subject to external validation |

The type system and validator intentionally prevent a synthetic record from reaching `ReviewedForInventory`. Model-based scenarios and hardware-in-the-loop tests remain useful evidence, but must not be counted as measured forest inventory.

## Acceptance tests

Fourteen tests cover typed round-trip serialization, digest syntax, sensor/calibration identity, expired/unknown calibration disclosure, nonzero pose uncertainty, duplicate measurement IDs, reviewer attribution, rejection reasons, review-field consistency, the synthetic-versus-field boundary, and the Sol Atlas terrain-evidence handoff for terrain/trail/obstacle records. The tests are committed in the stacked PR; they have not yet been reported as passing until exact-head CI completes.

## Next integration steps

1. Have the Symtropy receipt producer serialize a versioned observation record with its exact corpus/map/config digests.
2. Resolve each `spatial_terrain_evidence_refs` entry against the matching Sol Atlas evidence store and verify the envelope's map digest, frame, freshness, coverage and hazard disposition before use. Never convert an unknown hazard into an absent hazard during ingestion.
3. At ingestion, independently fetch artifacts and recompute all content hashes before resolving measurement IDs.
4. Reconcile repeated detections across robot passes by stable candidate identity + spatial/time windows; emit conflicts rather than silently merging trees/log lots.
5. Record field review separately from measurement acquisition and preserve rejected/ambiguous records for evaluation.
6. Benchmark tree detection, position/DBH error and ecological disturbance against independent plot ground truth, stratified by species, diameter, terrain, occlusion, season and sensor type.

Reference research: autonomous legged forest-inventory trials report typical DBH accuracy around 2 cm in a specific system, while a 2025 under-canopy drone study reports 3.33–3.97 cm DBH RMSE across its full sample, demonstrating that thresholds depend on platform and test population. Sources: https://doi.org/10.1109/TFR.2025.3583972 and https://www.tandfonline.com/doi/full/10.1080/01431161.2025.2579803.
