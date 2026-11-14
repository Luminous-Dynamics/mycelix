# GCPN — Civilian autonomous-system hazard awareness benchmark v0.1

**Parent:** [Global Cooperative Protection Network #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child:** [GCPN-CIV-AUTONOMOUS-SYSTEM-SAFETY-001 #5934](https://github.com/Luminous-Dynamics/mycelix/issues/5934)  
**Status:** Synthetic event-awareness contract; no operational deployment or field performance claim  
**Review date:** 8 November 2026  
**Claim ceiling:** Frozen synthetic hazard-awareness semantics only

## 1. Purpose

This benchmark defines a civilian, non-weapon awareness and resilience layer for unidentified, unauthorised, malfunctioning or unsafe drones and ground robots around homes, ports, public spaces, industrial facilities and essential services.

It focuses on what is observed, the quality and independence of evidence, uncertainty, privacy, notification, human decision rights, the safety of machines we own, and independently verified recovery.

It does **not** build an effector, take control of third-party aircraft/robots, or determine hostile intent. Presence is an observation, not a threat conclusion.

## 2. Scope and hard boundaries

Permitted work:
- passive or lawfully operated sensing and source-health assessment;
- event-level detection/awareness and human verification;
- authenticated incident evidence, privacy-aware notification and escalation to the legally responsible authority;
- passive physical protection, shelter, service continuity and incident recovery;
- safety-rated protections on machines owned/operated by the site, provided the OEM and competent safety authority remain in control;
- synthetic simulation of uncertainty, false reports, degraded sensors and safe authority boundaries.

Excluded work:
- high-energy lasers, optical dazzle, retinal/thermal damage or destructive directed energy;
- acoustic/ultrasonic/infrasound systems intended to injure, disorient, incapacitate or harass;
- “psychotronic” or involuntary remote neuromodulation devices;
- RF jamming, GNSS spoofing, cyber takeover, forced landing, protocol exploitation or command injection against third-party systems;
- kinetic interception, countermeasure effectors, autonomous interception, target selection, engagement control, operator geolocation or tactical tracking;
- sensor placement, coverage/engagement geometry, defeat sequence or countermeasure optimization.

No effector prototype or physical countermeasure is authorised by this issue. Where a credible incident occurs, the system supports the site operator, SACAA, emergency services or other competent public authority.

## 3. Architectural paradigm shifts

### 3.1 Intent is not a sensor output

Keep distinct:
- physical observation;
- identity claim;
- authentication of the identity source;
- authorization claim and validity;
- sensor/measurement health;
- operator or flight intent;
- hazard determination by a competent authority.

An anomaly score, unknown transponder, absent RF transmission, unusual movement, or an unverified report cannot be promoted to hostile intent.

### 3.2 Detection is not neutralization

Use a typed, auditable sequence:

`Observation → SourceQualification → Corroboration → HumanReview → AuthorizedNotification → SiteProcedure → OutcomeVerification`

The network has no command path to third-party vehicles, aircraft, robots, weapons or effectors. The software's “stop” state cannot be represented as a machine safety-rated emergency stop.

### 3.3 Independence is an evidence property

Source counts are not independence. Multiple sensors may share a mast, power supply, network, clock, firmware, vendor pipeline, calibration process or data source. Record known shared dependencies and preserve “unknown” when the dependency inventory is incomplete.

### 3.4 Protect people and services, not a sensor score

Measure warning delivery, human review, essential-service continuity, safe shelter and independently verified recovery. Do not collapse public safety, privacy, availability and cost into one score that can hide a hard failure.

### 3.5 Degrade safely

Loss of cloud service, GNSS, time quality, source freshness, authorization or sensor health must not grant new authority. The system should preserve uncertainty, notify the accountable operator and defer to approved local procedures.

## 4. Permitted sensing roles

| Modality | Civilian role | Safety / privacy boundary |
|---|---|---|
| Visible / thermal optical | Passive event observation and human verification | No facial recognition, person tracking, targeting or public surveillance |
| Radar | Lawful site awareness feed | No fire-control or target-quality engagement tracking |
| RF | Passive awareness of available lawful emissions | No jamming, spoofing, private-content interception or takeover |
| Acoustic | Local event features and environmental-noise awareness | No harmful sound emitter; do not retain intelligible speech |
| LiDAR / laser metrology | OEM-certified, eye-safe measurement product used within its classification | No directed-energy or dazzling/damaging use |
| Robot onboard safety | OEM/site-authorized safety state for equipment the site owns | No control of third-party equipment; no substitute for safety-rated controller |

The 2026 modality survey identifies no universally reliable single sensor across clutter and degraded conditions: EO/IR suffers from occlusion/illumination; radar from low-return/multipath cases; acoustic sensing from ambient noise and limited range; RF from non-transmitting aircraft; and LiDAR from environmental sensitivity and resource cost. Radar/EO-IR fusion is described as a relatively mature balance, but remains vulnerable under combined occlusion and multipath. This is a research comparison, not a performance promise. [Johnson et al., September 2026](https://doi.org/10.1016/j.ijcip.2026.100870).

For the first demonstrator, reuse qualified site/vendor sensor outputs rather than design new radar, RF, optical, acoustic or laser hardware.

## 5. Industry comparison — awareness only

Compare the awareness/detection parts of established systems, not their mitigation/effector modules:
- **Dedrone RF-360:** passive network-attached RF sensing for drone detection/classification.
- **DroneShield DroneSentry-C2:** multi-sensor platform integrating RF, radar and optical feeds.
- **Sentrycs DTI:** detection/tracking/identification module is distinct from optional mitigation functions.

The public vendor product pages are capability references, not independent performance evidence:
- https://www.axon.com/resources/dedrone-rf-360-sensor
- https://www.droneshield.com/products-software
- https://sentrycs.com/our-technology/

A fair comparison uses identical lawful sensor inputs, site/observation window, source labels, operator review budget, privacy policy and release/configuration. Include the current manual workflow and a single-modality baseline. Record licensing, staffing, installation, calibration, connectivity, maintenance and downtime. Do not compare an integrated incumbent against an artificially weakened script.

## 6. Frozen synthetic corpus

The companion JSON freezes 12 non-geometric cases:
1. authorized service drone with an unverified authorization claim;
2. public-safety / medical drone with incomplete source metadata;
3. unidentified aircraft-like observation where intent remains unknown;
4. bird/aircraft/drone ambiguity and a required human review;
5. RF-silent or out-of-band aircraft: RF-only sources cannot report “no aircraft”;
6. stale sensor observation;
7. authenticated but uncalibrated sensor;
8. shared failure-domain sources incorrectly counted as independent;
9. revoked or expired identity credential;
10. ground robot whose task authorization expired;
11. third-party machine for which no command authority exists;
12. protected audio/visual data use or disclosure request.

The corpus contains no coordinates, trajectories, sensor placement, operator location, target/effector geometry or response tactics.

## 7. Metrics

Report separately:
- event-level precision/recall by declared benign/unknown/unsafe-observation class;
- false alerts per monitored hour/site-day at preregistered operating points;
- detection-to-human-review latency (p50/p95);
- calibration and selective risk versus abstention coverage;
- unknown/ambiguous fraction and correct abstention;
- source freshness, sensor-health detection and graceful degradation;
- common-mode sensitivity and provenance completeness;
- false escalation and unauthorized-command count;
- notification delivery, accessibility and acknowledgement;
- human review burden, privacy leakage and retention violations;
- site/service downtime and independently verified recovery time;
- full-lifecycle cost per comparable monitored service-hour.

No universal detection-rate target is claimed before the site, sensor release, weather/clutter profile and ground-truth protocol are selected. First establish baseline, then preregister a superiority threshold and held-out test set.

## 8. Hard conformance gates

- Zero third-party aircraft/robot command or actuation path.
- Zero autonomous neutralization or engagement.
- Zero promotion of presence/anomaly to hostile intent.
- Zero false absence conclusion from missing or RF-silent evidence.
- Zero unconsented person identification or operator geolocation.
- Zero disclosure beyond source rights, purpose or recipient permissions.
- Sensor degradation and source conflicts must remain explicit.
- Site-owned machine safety remains with OEM-validated safety controls and competent site authority.
- All compared runs use the same case frame, source inputs, observation window and human-review budget.

## 9. Standards and South African regulatory applicability

This is an applicability register, not a conformity claim:
- ISO 10218-1:2025 and ISO 10218-2:2025 for industrial robots and industrial robot applications/cells, subject to scope and exclusions.
- ISO 12100:2010 for machinery risk assessment and risk reduction; the published edition is under revision.
- ISO 13850:2015 for emergency-stop principles, not a complete safeguarding or safety-control system.
- ISO 3691-4:2023 for driverless industrial trucks/systems where applicable.
- IEC 60825-1:2014 Edition 3 for laser product classification and safety; IEC lists it as valid with a 2027 stability date and Edition 4 in development. Recheck before procurement or use.
- WHO environmental-noise guidance for health-based noise exposure assessment; not an acoustic-device design standard.
- ISA/IEC 62443 for connected industrial automation/control systems.

South Africa:
- Confirm the current operative SACAA Part 101/CAR/CATS requirements before any UAS research or site work. SACAA's public UAS page is a useful entry point but not a complete consolidated 2026 legal opinion.
- Government Gazette 55226 (21 August 2026) public indexes list two aviation amendment notices with inconsistent amendment numbering; obtain the exact operative notice, commencement and consolidated text from SACAA / aviation counsel before reliance.
- Obtain legal/NCACC classification advice before any potentially controlled dual-use technology research, prototype, manufacture or transfer. DTIC notes that registration/permits can apply to armaments development/manufacture and related controlled systems, components, technologies and services.

## 10. Professional review gate

Before a physical pilot, require:
- SACAA/aviation-law advisor and airspace/airport operator;
- registered electrical/telecommunications/EMC professionals for applicable installations;
- laser safety / optical-radiation specialist for optical measurement products;
- acoustic / occupational-hygiene specialist for microphones and audible alerts;
- robotics safety engineer and site owner for applicable machinery standards, guarding, stop functions and safe states;
- OT cybersecurity / functional-safety reviewer for connected industrial sites;
- privacy / information-governance and community review;
- legal/NCACC review for any potentially controlled dual-use technology.

Every review binds exact product/configuration, site/jurisdiction, source versions, reviewer competence/registration, scope, exclusions, risks, findings, disposition and date. CI or AI desk review is not professional approval.

## 11. Psychotronics claim handling

“Psychotronics” is not a validated engineering capability. Literature on parapsychology contains contested claims: a 2018 review argues for cumulative support for psi, while a controlled neuroimaging study reported no compelling evidence for psi and indistinguishable neural responses to psi/non-psi stimuli. There is no reproducible engineering basis here for a remote mind-control or psychotronic defence device.

Do not build, market or test involuntary human-influence devices. If an extraordinary claim arises, use a separate ethics-approved, preregistered falsification protocol with blinding, sham controls, preregistered outcomes, independent analysis and replication; no covert exposure or non-consenting participants. Clinical rTMS and low-intensity focused ultrasound are distinct medical research domains under clinical/device oversight.

References:
- https://pubmed.ncbi.nlm.nih.gov/29792448/
- https://dash.harvard.edu/entities/publication/73120378-86da-6bd4-e053-0100007fdf3b
- https://www.fda.gov/medical-devices/guidance-documents-medical-devices-and-radiation-emitting-products/repetitive-transcranial-magnetic-stimulation-rtms-systems-class-ii-special-controls-guidance

## 12. Claim ceiling

Passing a synthetic verifier establishes only integrity/semantic conformance of the frozen event-awareness cases. It does not establish real-world detection performance, hostile intent, airworthiness, regulatory permission, robot safety certification or protection from attack.
