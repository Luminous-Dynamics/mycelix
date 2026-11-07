# Observation support and measurement-operator contract v0.1

**Status:** research metadata contract and synthetic fixture only. No field sampling or instrument deployment is authorized.

## Why this matters

A modality label is not a sufficient description of a measurement. A CT image represents a finite voxel volume and resolution; an event tracer represents a depth/time collection window and a sampling schedule; a water-retention point represents a core, pressure head and equilibration protocol. These measurements may be correlated because they share a core, batch, instrument, location, time window or processing transform, but the correlation cannot be inferred from modality names alone.

The support contract makes measurement comparability and overlap auditable before estimating a covariance matrix or a joint likelihood.

## Required measurement identity

Each measurement record must identify:

- stable measurement and support IDs;
- source sample, independent treatment-assignment unit and material/source batch as distinct identifiers;
- modality, measurand, value unit and versioned observation-operator ID;
- spatial geometry, depth extent, horizontal area, sample/imaged volume and resolution where applicable;
- temporal aggregation, start/end support and sampling cadence where applicable;
- instrument identity, calibration state, detection limit and calibration-record digest when available;
- standard measurement uncertainty, uncertainty unit and whether the value is repeatability-derived, calibration-derived, publisher-reported or only an explicit synthetic assumption.

Unknown or inapplicable values must be represented as null with a separate calibration/uncertainty state; they must not be replaced by zero.

## Support is not independence

- A set of CT slices is not a set of independent treatment units.
- Multiple tracer collection intervals from one core or plot are repeated observations, not independent replicates.
- Two modalities on the same assignment unit can be complementary in mechanism information and still have correlated errors through shared sample preparation or batch effects.
- Different measurement supports are not interchangeable just because the same scalar measurand is reported.
- Overlapping support is a reason to investigate covariance, not a license to set correlation to 1.0.
- Non-overlapping supports can still be correlated through spatial fields, temporal autocorrelation, shared sensors or common calibration drift.

## Covariance qualification

Before using an observation covariance model:

1. define the observation operator and support for each observation;
2. collect paired repeatability / inter-operator / cross-instrument evidence;
3. distinguish shared assignment-unit, source-batch, instrument, temporal and spatial effects;
4. estimate covariance only from appropriate replicated evidence or a declared simulator;
5. compare the correct joint likelihood with a diagonal-independence ablation;
6. report positive-definiteness, conditioning, shrinkage/regularization and parameter uncertainty;
7. evaluate predictive calibration on held-out support groups.

Do not derive a correlation coefficient solely from support overlap percentages; that mapping is process- and instrument-dependent.

## Synthetic example

The accompanying fixture illustrates CT image volume, tracer depth/time window, hydraulic retention core/pressure support and gas-permeability core support. IDs and numbers are invented to exercise the schema. They are not instrument specifications, a physical protocol, a soil dataset or a recommendation.

## Integration

This contract is the metadata foundation for #5632's covariance-aware likelihood work. It complements the assignment-unit and connected-component lineage rules in the Terra Preta evidence package. The next implementation should attach these support records to selector-visible action metadata and use them to parameterize/validate a joint observation model, without exposing evaluator truth.
