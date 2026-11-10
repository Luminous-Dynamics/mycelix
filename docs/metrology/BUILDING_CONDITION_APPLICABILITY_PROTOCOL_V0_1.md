# Building condition applicability v0.1

Owner: Mycelix #6057. Parent: Mycelix #6034 instrument passport. This package evaluates whether a declared capability/method envelope applies at a particular sample context; it is not sensor calibration, uncertainty propagation, a time-series engine or building approval.

## Domain profiles in the synthetic oracle

- Thermal environment: ISO 7726:2025-oriented ambient temperature and relative humidity dimensions.
- Water volume: ISO 4064-1:2024 / OIML R 49:2024 scope-sensitive fully charged conduit, flow profile and fluid-temperature conditions.
- AC power quality: IEC 61000-4-30:2025 with July 2026 corrigendum; declared 50 Hz context and wiring topology.
- PM2.5: ISO 16000-37:2019 reference-method co-location/aerosol profile and humidity context.

The numeric limits in this synthetic corpus are illustrative test values, not normative method tolerances, comfort limits, safety thresholds, or product specifications.

## Applicability semantics

For numeric envelope [L,U] and a declared expanded uncertainty interval [y-u, y+u]:
- `WITHIN_DECLARED_ENVELOPE` only when the full interval is contained in [L,U].
- `OUTSIDE_DECLARED_ENVELOPE` when the full interval is disjoint outside [L,U].
- `INDETERMINATE_BOUNDARY_OVERLAP` when the interval overlaps a boundary without full containment.

Categorical conditions require exact membership in the declared allowed vocabulary and a verified-configuration source. Numeric conditions require an independently admitted reference instrument, current calibration, valid QC, rights, matching method/profile, same timebase/clock epoch, spatial scope and validity interval covering the sample tick.

Missing evidence never defaults to within. A condition instrument cannot validate its own operating conditions. A controller setpoint, vendor datasheet, BMS state or device command is not a field observation.

## Claim ceiling

All records are synthetic. The 40-case oracle qualifies only profile/observation applicability logic. It does not qualify actual measurements, instrument accuracy, method conformity, calibration scope, electrical safety, water quality, IAQ, comfort, or building performance.
