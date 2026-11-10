# Building metrology profile v0.1

This package defines a synthetic requirements-only profile for admitting physical measurement instruments used in buildings and habitats. It contains no selected manufacturer, real sensor serial, calibration certificate, physical measurement or tolerance.

## Included channel families

Thermal environment, indoor air, ventilation, water quantity/quality, electrical energy/power and useful thermal service. The profile intentionally leaves numerical acceptance tolerances and physical thresholds unset until the exact use case, measurement method and error budget are justified.

## Admission result

Admission is purpose-specific. A channel admitted for one declared measurand/unit/method/installation/profile is not thereby admitted for a different channel or building. The classifier distinguishes comparability, local acquisition, firmware freshness, calibration, installation, reference method, range/uncertainty, rights and reading quality.

## Owners

- Mycelix #6025 owns this building-metrology profile.
- Mycelix #3243 remains H1/hydroponics-specific; its hardware-admission principles are inspiration, not an interchangeable schema.
- Symtropy #4507 owns thermal-outage measurement and metric semantics.
- Symtropy #1849 owns deterministic service-availability/recovery timeline semantics.
- Sol-Atlas #633 owns post-occupancy outcome comparability; #700 binds service results to owner-native outputs.

## Claim ceiling

The Rust oracle only validates the synthetic profile and 36 synthetic known-answer admissions. It does not qualify a real instrument, lab, sensor placement, building, comfort condition, air or water quality, energy outcome, device, or safety/code claim.
