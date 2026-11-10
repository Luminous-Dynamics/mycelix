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

## Current method-edition refresh — 10 November 2026

This table records current published editions/status located for this research pass. A version pointer is not a substitute for the full standard, exact applicable clause, method procedure, local legal requirement or accredited laboratory scope.

| Measurand / role | Current reference found | Exact boundary and implication |
|---|---|---|
| Thermal-environment instruments | [ISO 7726:2025, Ed. 3](https://www.iso.org/standard/78238.html), published October 2025 | Replaces withdrawn ISO 7726:1998. It specifies minimum instrument characteristics and methods for physical quantities characterizing the environment. Bind the actual sensor type, response, radiation effects, placement and method. |
| Occupant thermal-comfort assessment | [ANSI/ASHRAE 55:2023](https://www.ashrae.org/technical-resources/bookstore/standard-55-thermal-environmental-conditions-for-human-occupancy) | Uses environmental and personal factors and standard/adaptive assessment methods. It is an assessment method, not calibration, thermal-survival, medical or South African statutory approval by itself. |
| Calibration laboratory competence | [ISO/IEC 17025:2017](https://committee.iso.org/standard/66912.html), confirmed current in 2023 | Applies to laboratory competence, impartiality and consistent operation. The lab's actual current accreditation schedule must cover the exact measurand, range and method. |
| Metrological traceability | [ILAC P10:07/2020](https://ilac.org/publications-and-resources/ilac-policy-series/) | Policy on traceability of measurement results. Traceability is a chain of calibrations, each contributing uncertainty; a logo/certificate number alone is not a traceability chain. |
| Calibration uncertainty statement | [ILAC P14:09/2020](https://ilac.org/publications-and-resources/ilac-policy-series/) | Policy for measurement uncertainty in calibration. Keep the certificate's reported uncertainty and coverage convention tied to the exact certificate, result, method and scope. |
| Measurement uncertainty and nonlinear models | [JCGM 100:2008/Amd.1:2026](https://www.bipm.org/documents/20126/2071204/JCGM_100_Amd1_2026.pdf/) and the [BIPM GUM catalogue](https://www.bipm.org/en/web/guest/publications/guides) | The 2026 amendment adds guidance on significant nonlinearity: use Monte Carlo propagation under JCGM 101:2008 or include higher-order Taylor terms as appropriate. Do not assume first-order linearization is adequate, and do not silently omit covariance. Detailed computation is delegated to Mycelix #6048. |
| Closed-conduit water volume metering | [ISO 4064-1:2024, Ed. 5](https://www.iso.org/standard/80868.html) and [OIML R 49:2024](https://www.oiml.org/en/publications/recommendations/49/2024/) | Both references concern measurement of actual water volume in a fully charged, closed conduit and meters indicating integrated volume. They are not blanket methods for open channels, partially filled pipes, tanks/level-to-volume inference, rainwater estimation, water quality or potable safety. Tank-level measurement requires its own geometry/datum calibration. |
| Indoor PM2.5 | [ISO 16000-37:2019](https://www.iso.org/standard/66283.html), reviewed/confirmed current in 2024 | Provides methods/strategies for indoor PM2.5 mass concentration. Its reference principle is filter collection after particle separation and gravimetric weighing; supplementary high-time-resolution instruments can be used for peaks/investigation/QC. A low-cost optical sensor is not the reference method merely because it reports µg/m³. |
| AC power-quality measurements | [IEC 61000-4-30:2025, Ed. 4, corrected version 2026-07](https://webstore.iec.ch/en/publication/71611) | Defines repeatable in-situ measurement/interpretation methods for 50/60 Hz AC systems, including Class A and Class S. Covered parameters include frequency, supply voltage magnitude, flicker, dips/swells/interruptions, unbalance, voltage harmonics/interharmonics, current magnitude/harmonics, rapid voltage changes and related phenomena. This is distinct from accumulated energy metering and useful thermal-energy measurement. Transducer effects are not fully specified by this standard; see the referenced IEC TR 61869-103 guidance. |

### Applicability consequences for Highveld / M0

1. **Do not use ISO 4064/OIML R 49 outside their closed, fully charged conduit scope.** WaterLoop may have storage tanks, rain capture, greywater treatment, partial-flow drains and non-potable loops. These need separately defined measurands and methods; a water meter is not automatically within R 49 / ISO 4064 scope.
2. **Separate electrical metering from power-quality characterization.** kWh and instantaneous W do not reveal voltage events, frequency excursions, unbalance or harmonic distortion. If the M0 cell is intended to characterize grid resilience, define a separate 50 Hz AC power-quality profile against IEC 61000-4-30:2025 and the correct instrument class. Do not infer Class A or Class S from a product marketing sheet.
3. **Separate reference and trend instruments.** For indoor PM2.5, document the reference method and co-location/quality-control procedure before interpreting high-resolution optical readings. Similar principle applies to gas sensors and flow measurement.
4. **Preserve measurement conditions as data, not as opaque IDs.** A capability claim valid under one conditions profile cannot be applied to a sample with missing or different ambient temperature, humidity, pressure, vibration, fluid, orientation or power-system conditions. This missing condition-binding layer is tracked separately under Mycelix #6059.
5. **Do not compute uncertainty from a flat list of component values.** The 2026 GUM amendment makes nonlinear measurement models a first-class issue. A future receipt must identify the model, input distributions, correlation structure, propagation method and coverage convention; Mycelix #6048 owns that computation.

### South African accreditation lookup rule

For any real M1 instrument, select a calibration/testing laboratory only after locating its current SANAS accredited-facility entry and exact scope schedule. The schedule must cover the requested measurand, method, range and relevant calibration capability. The accreditation body's public directory is a discovery source, not a substitute for the current scope certificate. No specific laboratory is selected or represented as approved in this synthetic programme.
## Claim ceiling

These are standards/method pointers. This matrix is not an accredited method list, conformity assessment, building-code determination or SANAS scope decision. No instrument is qualified until its exact certificate and current accreditation scope are independently checked by a competent metrology reviewer.
