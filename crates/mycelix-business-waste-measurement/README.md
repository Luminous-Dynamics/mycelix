# mycelix-business-waste-measurement

A dependency-light, non-authorizing contract for evaluating the quality and coverage of food-waste observations in hospitality pilots.

The crate deliberately separates directly weighed observations, count-derived values, visual estimates, model imputations, and explicitly missing slots. It keeps the preregistered sampling denominator intact and emits a deterministic digest of the evidence set.

It does not prove that a scale was physically used, that a calibration document is authentic, or that all waste was captured. External evidence validation, field observation, and causal impact evaluation remain separate responsibilities.

See `docs/business/HOSPITALITY_OUTCOME_MEASUREMENT_PROTOCOL_V0_1.md` for the protocol and research limitations.
