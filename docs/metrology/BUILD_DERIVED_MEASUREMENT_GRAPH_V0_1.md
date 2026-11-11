# Derived building measurement graph v0.1

This first slice provides a 40-case adversarial corpus and implements exact source-graph identity, input ordering, dependency resolution, stale-parent blocking and deterministic execution of a deliberately synthetic arithmetic-mean test method.

## Physical method profiles are defined but blocked

- Mean radiant temperature: ISO 7726:2025 Edition 3 distinguishes basic directly measured quantities from derived quantities. It permits derived mean radiant temperature from six directional plane-radiant temperatures using projected-area factors, and from surface temperatures plus room geometry. This contract records the six-direction profile but does not execute it. The frozen v0.1 channel set does not admit `plane_radiant_temperature`.
- Useful thermal energy: the candidate method requires a metered flow series, matched supply/return water-temperature series, fluid identity and a versioned fluid-property/enthalpy model. `water_volume_flow` exists in v0.1, but `water_temperature` remains a proposed v0.2 channel under #6029. A constant heat-capacity shortcut is not silently promoted.

Method reference: [ISO 7726:2025](https://www.iso.org/standard/78238.html), Edition 3, published October 2025. Public abstract and technical summary distinguish basic measured quantities from derived mean-radiant/operative temperature; the exact normative text remains authoritative.

## What the executable test proves

The synthetic method takes two scalar parent observations (`2` and `4`) and computes their arithmetic mean (`3`). The oracle requires exact sample IDs/digests, channel/unit identity, calibration/installation/firmware, stream clock domain/epoch and tick-unit equality, sequence, rights, method/runtime/configuration digests, ordered input list, time window, uncertainty-model reference and invalidation receipt. This is a graph-resolution and deterministic test-method conformance result only.

## What it does not prove

It does not calculate real mean radiant temperature, operative temperature or useful thermal energy. It does not admit any physical derived observation, verify a normative method implementation, compute uncertainty or establish sensor accuracy. Those require channel admission, exact method artifacts, #6048 uncertainty, #6054 timebase, #6065 conditions and independent reference testing.

## Claim ceiling

The only successful case disposition is `SYNTHETIC_GRAPH_VALIDATION_ONLY`. No physical output can be promoted from this branch.
