# Coupled hydrology–nutrient event record schema

This is an **event-level evidence contract**, not an executable transport model or agronomic recommendation.

## Validate

```sh
cargo test --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml -- \
  research/terra-preta/tools/coupled-event-audit/event-record.schema.json \
  research/terra-preta/tools/coupled-event-audit/fixtures/synthetic-event.json
```

## Contract

- One event, study/site, independently assigned unit, treatment/comparator pair, control volume and time interval per record.
- Water inputs and outputs use equivalent depth (mm) over the declared area and interval; storage change is signed.
- Soil-moisture observations preserve sensor, position, timestamp, depth, volumetric basis, calibration and uncertainty. Two observations are a structural minimum; the schema does not prove distinct depths or adequate sampling frequency.
- Dissolved exports preserve analyte, concentration, synchronized water depth, time interval, sampling design and collection coverage. `measured_qualified` requires measured water depth, complete coverage, uncertainties and flow-weighted or event-integrated concentration. A grab/spot sample alone is concentration-only.
- Particulate exports use dry sediment mass and its elemental concentration, not water volume.
- Preferential flow may be corroborated, sensor-signature-only, indeterminate or not detected. A corroborated classification requires an independent tracer/geophysical/water-table reference.
- Coverage reviews preserve unmeasured pathways. Complete coverage cannot list unmeasured pathways; incomplete coverage must state them.

Schema validation checks field shape and declared qualification. It does not prove timestamps align, event boundaries are complete, sensors are distinct in depth, concentration and discharge were truly synchronized, collectors recovered the full event or tracer evidence is scientifically valid. Those require a future executable auditor and independent review.

The fixture is synthetic software-test data only. Never merge it with field data or use it to estimate treatment effects.
