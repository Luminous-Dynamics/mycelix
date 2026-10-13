# mycelix-business-waste-measurement

A dependency-light, non-authorizing contract for evaluating the quality and coverage of food-waste observations in hospitality pilots.

The crate deliberately separates directly weighed observations, count-derived values, visual estimates, model imputations, and explicitly missing slots. Each preregistered slot binds an exact target time, allowed observation window, and waste category; physical observations must fall within that slot window, same-category windows cannot overlap, and missingness cannot be declared before the slot closes. It keeps the full denominator intact and emits a deterministic digest of the evidence set.

It does not prove that a scale was physically used, that a calibration document is authentic, or that all waste was captured. External evidence validation, field observation, and causal impact evaluation remain separate responsibilities.

See `docs/business/HOSPITALITY_OUTCOME_MEASUREMENT_PROTOCOL_V0_1.md` for the protocol and research limitations.
