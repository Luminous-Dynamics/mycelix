# Building instrument passport: methods and accreditation crosswalk

## Scope

This note identifies method families and the evidence they can support. It deliberately does not assign universal tolerances, approve a laboratory, or encode clause-level requirements from copyrighted standards. The exact edition, scope, method, range and installation conditions must be verified for each measurand and decision purpose.

## Thermal environment

- **ISO 7726:2025, Edition 3** is the current published instrument/method reference identified for measuring and monitoring physical quantities characterizing thermal environments. It replaces the 1998 edition. Map air temperature, mean-radiant/globe temperature, humidity and air-speed channels to applicable provisions only after checking the full text, sensor type, response, radiation shielding, placement and averaging assumptions. It does not automatically calibrate a device or make a room placement representative.
- **ASHRAE Standard 55:2023** addresses combinations of environmental and personal factors that produce satisfactory thermal conditions for most occupants, using standard and adaptive methods. It is a comfort assessment reference, not a calibration certificate or a universal survival/health threshold.

## Indoor air and ventilation

- **ASHRAE 62.2-2025** applies to non-transient residential dwelling units and addresses ventilation, local exhaust and source control. Its scope explicitly says thermal comfort is outside its scope and that acceptable IAQ may not necessarily be achieved even when requirements are met. CO2 alone is not an all-pollutant measurement.
- PM2.5, CO2, outdoor-air flow, humidity and pollutant-specific gases require separate measurement methods and reference instruments. A low-cost optical PM or gas sensor can be useful for trends only within a validated response/cross-sensitivity envelope; it is not an independent assay by assertion.

## Calibration laboratory and traceability

- **ISO/IEC 17025:2017** is the current published edition identified by ISO; it was reviewed and confirmed in 2023. It addresses laboratory competence, impartiality and consistent operation. Accreditation is not blanket approval for every possible measurand/range.
- In South Africa, **SANAS** provides a public directory exposing each accredited facility's certificate and scope. Review the actual current scope schedule: exact measurand/discipline, range, method and uncertainty capability must cover the calibration in question. SANAS is the national accreditation body under Act 19 of 2006.
- **ILAC P10:07/2020** covers metrological traceability of measurement results; **ILAC P14:09/2020** covers measurement uncertainty in calibration. Keep traceability and the calibration statement linked to the exact certificate and scope.
- **JCGM 100:2008 (GUM)** establishes general rules for evaluating and expressing measurement uncertainty. BIPM currently lists Amendment 1:2026 on nonlinearity in measurement models and the GUM suite. The passport preserves components, source records and correlation groups; it does not calculate an uncertainty budget or assume independent components.

## Cross-laboratory and product-level performance

- **IEC TR 63250:2021** addresses repeatability, reproducibility, uncertainty and round-robin tests for household-appliance performance methods. Its scope explicitly excludes production-unit variability and representativeness of normal household use. For a Luminous appliance claim, join this laboratory repeatability stratum to the four separate evidence strata in Symthaea #10341: within-unit repeatability, between-laboratory reproducibility, production variation and household ecological validity.

## Method-to-channel rules

| Channel family | Method questions that must be explicit | Prohibited shortcut |
|---|---|---|
| Temperature / humidity / air speed | sensor principle, range, response, radiation effects, placement, height, shielding, averaging, environmental conditions | equating sensor calibration with comfort assessment |
| Mean-radiant / globe temperature | globe geometry/emissivity, view factors/air speed where method requires, exact calculation/method revision and input sensors | renaming globe temperature as mean radiant temperature without a method |
| CO2 / PM2.5 | reference instrument, calibration aerosol/gas, cross-sensitivity, humidity effects, inlet location, time response and co-location protocol | CO2 or AQI alone as universal IAQ / health proof |
| Air / water volume flow | flow profile, straight lengths/installation, fluid properties, pulsation, meter factor, zero drift, integration and boundary | treating a nameplate or pulse count as verified delivered volume |
| Water pH / conductivity | reference buffers/solutions, temperature compensation, sampling/handling, independent chemistry/microbiology as purpose requires | conductivity/pH as potability or pathogen-removal proof |
| Electrical power / energy | meter wiring/topology, phase, power factor, waveform/bandwidth, boundary, integration intervals, clock and calibration | treating instantaneous power as energy or mixing meter boundaries |
| Useful thermal energy | flow, supply/return temperature, heat capacity/fluid definition, integration interval and covariance of sensors | derived kWh_th without raw input channels and uncertainty propagation |

## Claim ceiling

These are standards/method pointers. This matrix is not an accredited method list, conformity assessment, building-code determination or SANAS scope decision. No instrument is qualified until its exact certificate and current accreditation scope are independently checked by a competent metrology reviewer.
