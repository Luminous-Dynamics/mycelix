# Building metrology channel gap matrix v0.1

This proposal-only matrix responds to the seven missing channel groups identified by Symtropy's unoccupied habitat test-cell M0 design basis and Mycelix #6029.

It proposes 14 candidate channel identities:
- water pressure is split into gauge and absolute reference conventions;
- water temperature is separate from air/surface/globe temperature;
- tank level remains level, not volume;
- differential pressure is a two-port difference with declared upstream/downstream order;
- acoustics separates a calibrated raw pressure waveform from an interval- and weighting-defined derived sound level;
- electrical measurements separate RMS voltage, RMS current, frequency and distortion from existing power/energy;
- surface and globe temperatures remain distinct from air temperature and derived mean-radiant/operative measures.

**No channel is admitted by this matrix.** Every entry is PROPOSED_NOT_ADMITTED; methods and calibration routes are TO_BE_VERIFIED; no numerical range tolerance or physical threshold is set. Exact normative method editions, accreditation scopes, installation constraints and uncertainty budgets must be reviewed before any profile-v0.2 proposal.

The existing v0.1 channels are immutable. A future v0.2 profile must be additive and must not change existing quantity, unit, method or evidence semantics.

## Important semantics

- Gauge pressure is relative to a declared atmospheric reference; absolute pressure is relative to vacuum. They are not interchangeable.
- Tank level is not tank volume without an exact tank geometry and calibration relation.
- Differential pressure requires ordered ports and sign convention.
- A-weighted equivalent continuous sound level is a derived quantity over a declared integration interval and method; a bare dB value is not sufficient.
- Voltage/current and harmonic distortion are distinct from active power and accumulated energy.
- Surface temperature and globe temperature are not air temperature; mean radiant temperature requires a method that declares geometry, emissivity and air-speed conditions.

Claim ceiling: taxonomy/method requirements only; no real sensor, lab, building, air/water quality, electrical safety, comfort or physical performance is qualified.
