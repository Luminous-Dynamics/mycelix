# Coupled hydrology–nutrient event record schema

This is an **event-level evidence contract**, not an executable transport model or agronomic recommendation.

## Validate

```sh
cargo test --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/schema-audit/Cargo.toml -- \
  research/terra-preta/tools/coupled-event-audit/event-record.schema.json \
  research/terra-preta/tools/coupled-event-audit/fixtures/synthetic-event.json
```

## Run the executable auditor

From the repository root:

```sh
cargo test --manifest-path research/terra-preta/tools/coupled-event-audit/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/coupled-event-audit/Cargo.toml -- \
  research/terra-preta/tools/coupled-event-audit/fixtures/synthetic-event.json \
  /tmp/terra-preta-event-report.json
```

The report binds to the SHA-256 of exact event JSON bytes. Water residual
qualification requires complete reviewed coverage, measured boundary terms,
and declared uncertainty for every input/output/storage term. The conservative
linear sum of standard uncertainties is **not** a confidence interval.

Dissolved mass is calculated as
`kg element/ha = concentration (mg/L) × water depth (mm) × 0.01`.
Particulate mass is
`kg element/ha = dry sediment export (kg/ha) × sediment concentration (mg/kg) ÷ 1,000,000`.
First-order uncertainty bounds use linear product terms and do not model
covariance or systematic error. Plot mass is the per-hectare result
multiplied by the control-volume area.

Concentration-only samples are never converted to mass. Estimated records
remain estimates; partial collection is not scaled up to an event total.
Preferential-flow corroboration requires referenced sensors at two or more
distinct observed depths plus an independent reference method/evidence.
Solute-review incompleteness is reported separately from individually
qualified samples; no event-total nutrient loss is inferred from it.

## Contract

- One event, study/site, independently assigned unit, treatment/comparator pair, control volume and time interval per record.
- Water inputs and outputs use equivalent depth (mm) over the declared area and interval; storage change is signed.
- Soil-moisture observations preserve sensor, position, timestamp, depth, volumetric basis, calibration and uncertainty. The auditor verifies that referenced preferential-flow sensors occur at two or more distinct observed depths; neither schema nor auditor establishes adequate temporal sampling frequency.
- Dissolved exports preserve analyte, concentration, synchronized water depth, time interval, sampling design and collection coverage. `measured_qualified` requires measured water depth, complete coverage, uncertainties and flow-weighted or event-integrated concentration. A grab/spot sample alone is concentration-only.
- Particulate exports use dry sediment mass and its elemental concentration, not water volume.
- Preferential flow may be corroborated, sensor-signature-only, indeterminate or not detected. A corroborated classification requires an independent tracer/geophysical/water-table reference.
- Coverage reviews preserve unmeasured pathways. Complete coverage cannot list unmeasured pathways; incomplete coverage must state them.

Schema validation checks field shape and declared qualification. The Rust auditor additionally checks RFC3339 timestamp parsing and event containment, analyte/element consistency, source digest syntax, distinct observed sensor depths, status/coverage conditions and arithmetic. Neither layer proves that concentration and discharge were physically synchronized, event boundaries were complete, collectors recovered the full event, sensor calibration remained valid or tracer evidence was scientifically valid; those require primary evidence and independent review.

The input and report fixtures are synthetic software-test data only. The input
fixture has 18 mm water input − 5.5 mm water output − 12.5 mm storage gain = 0 mm
residual; 10 mg N/L × 4 mm = 0.4 kg N/ha; 80 kg dry sediment/ha × 500 mg P/kg =
0.04 kg P/ha. These values test arithmetic only. Never merge fixtures with
field data or estimate treatment effects from them.
