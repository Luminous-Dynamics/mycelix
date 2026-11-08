# GCPN Civilian Autonomous-System Awareness — preliminary desk review v0.1

**Review date:** 8 November 2026  
**Review type:** AI-assisted scope, evidence and software-contract desk review  
**Parent issue:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child issue:** [#5934](https://github.com/Luminous-Dynamics/mycelix/issues/5934)  
**Implementation PR:** [#5935](https://github.com/Luminous-Dynamics/mycelix/pull/5935)  
**Disposition:** continue synthetic contract work; physical pilot remains blocked pending legal, site, privacy and specialist professional review.

This is not an aviation-law opinion, professional engineering sign-off, product conformity assessment, radio licence, SACAA approval, privacy determination or robot safety certification. No independent external professional has signed this document.

## 1. Exact material reviewed

| Artifact | Exact branch blob SHA |
|---|---|
| Benchmark specification | `e724707719b58829bd4fa1777aa2469e3fc36f8e` |
| Frozen 12-case JSON corpus | `9a3a96991ffa3d6437a4a5958f6a711108710230` |
| Rust verifier implementation at this review snapshot | `cfd3f6a5a45b4d878d5a1bb6a3a3447289e775e2` |
| Workflow | `762d4dc78da90a2e52f7d5c46e728aadd05de11d` |

The PR has a subsequent commit pinning the exact fixture digest and adding further negative controls; review this document together with the PR's latest head and CI run.

## 2. Disposition and principal conclusion

**Suitable for continued synthetic event-awareness and evidence-contract research. Not yet suitable for purchasing/installation, field sensing, incident classification in live operations, physical intervention or safety claims.**

The proposal's strongest architectural decision is to keep observation, identity authentication, authorization, sensor health, intent, authority and outcome separate. It also explicitly excludes active effectors and third-party control. Those boundaries are directionally sound and should be treated as non-negotiable.

The programme currently defines only a machine-readable event taxonomy and its verifier. It does not implement a sensor, detector, classifier, airspace service, radio receiver, acoustic classifier, machine safety controller or live incident workflow. A passing software test therefore establishes only that this finite synthetic contract is internally enforced.

## 3. Strengths observed

1. **Authority separation:** the manifest denies third-party aircraft/robot commands, autonomous neutralization, weapon/effector control and hostile-intent inference from mere presence.
2. **Explicit excluded capabilities:** ten prohibited capability families are represented as a required set; the verifier rejects a changed set.
3. **Source-quality distinctions:** identity authentication, calibration, physical validity, source independence, currentness, authorization and permission to rely are not collapsed into a signature or source count.
4. **RF missingness:** the corpus explicitly rejects interpreting RF silence or out-of-band operation as proof that no aircraft is present.
5. **Privacy:** acoustic speech retention and person-identification/disclosure gates are explicit; optical and audio use remains subject to purpose, access and retention controls.
6. **Strict schema:** typed Serde structures use `deny_unknown_fields`; tests cover unknown root/nested fields and duplicate known JSON members.
7. **Frozen evidence:** the exact event corpus SHA-256 is now pinned in the verifier; semantic-preserving byte changes are rejected until reviewed.
8. **Professional boundary:** site-owned robot safety remains with OEM and competent site authority; software stop or human-review record is not presented as a safety-rated stop function.

## 4. Findings before any physical pilot

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| CAA-PR-01 | Blocker | No site, operator, airspace jurisdiction, intended sensor or operating purpose is selected. | Identify the accountable site owner, operating area, purpose, users, privacy role and exact system boundary. |
| CAA-PR-02 | Blocker | South African UAS rules and the August 2026 amendment notices need a consolidated, authoritative legal determination. | Obtain current SACAA Part 101/CAR/CATS text, operative Gazette notices, commencement and an aviation-law applicability opinion before UAS work. Public indexes show amendment-number inconsistencies; do not infer the operative rule from a search snippet. |
| CAA-PR-03 | Blocker | Radio/radar/RF equipment and passive-monitoring arrangements have not been classified for licensing, spectrum, type approval or site permissions. | Obtain ICASA and competent radio/EMC/legal review for exact equipment and use; this review does not assume that “receive-only” automatically resolves all legal or privacy issues. |
| CAA-PR-04 | Blocker | No privacy impact assessment, lawful-purpose analysis, retention schedule or community consultation exists. | Complete POPIA/privacy review and a data-protection impact assessment as appropriate; prohibit casual audio/video collection and person/operator identification. |
| CAA-PR-05 | Blocker | No robot product, task, workcell, OEM safety file or operating envelope is identified. | For a Luminous/site-owned robot, define exact machine/task and apply competent risk assessment, OEM instructions, guards/interlocks and validated safety-rated controls. Do not add a software actuator path. |
| CAA-PR-06 | Major | Vendor product capabilities are desk-mapped, not independently measured. | Conduct a matched, lawful, detection-only comparison on the same inputs, observation windows, labels, site conditions and operator budget. Exclude vendor mitigation/effectors. |
| CAA-PR-07 | Major | The 12 synthetic cases do not provide empirical ground truth or detector performance. | Design a blinded, consented, non-operational event-label study with independent adjudication, class ambiguity, false-positive/false-negative definitions and held-out conditions. |
| CAA-PR-08 | Major | Sensor common-mode dependencies are only represented as a synthetic state; no real dependency inventory exists. | Obtain vendor/site architecture and calibration records; distinguish known shared dependencies from genuinely complete inventories and unknown dependence. |
| CAA-PR-09 | Major | Lidar safety is represented as an OEM eye-safe measurement role, but no product, wavelength, class, installation or use is selected. | A laser safety/optical-radiation specialist reviews exact OEM product classification, installation, environment, access and maintenance against the applicable IEC 60825 edition. No device modification or directed-energy use. |
| CAA-PR-10 | Major | Acoustic features are allowed but no microphone, signal processing, exposure or data-minimization design has been reviewed. | Acoustics/occupational-hygiene and privacy reviewers approve event-only processing, no intelligible speech retention, alert-sound levels, retention and access controls. |
| CAA-PR-11 | Major | A sensor alert or identity claim could be misunderstood operationally as proof of intent. | Human-factors review and tabletop exercise must test alert wording, uncertainty states, benign explanations, escalation delay and false-alarm burden. |
| CAA-PR-12 | Major | No communications outage, local fallback, incident correction, or restoration exercise has been physically tested. | Test offline/local notification and source reconciliation in a non-operational exercise; document failures and service recovery. |
| CAA-PR-13 | Major | The programme has not established a realistic lifecycle cost baseline. | Use #5930's matched civilian portfolio cost protocol; account for sensors, installation, licences, calibration, operators, privacy controls, networking, maintenance, inspection and downtime. |
| CAA-PR-14 | Observation | There is no verified capability here called “psychotronic defence.” | Exclude psychotronic devices. Treat any extraordinary human-influence claim as a separate ethics-approved falsification question; do not expose people without informed consent. |

## 5. Modality review

### Optical and thermal observation
Potentially useful for passive human verification, but affected by occlusion, illumination, weather, field of view and operator interpretation. Prohibit facial recognition, person tracking, operator geolocation and public live tracks. A detection score is not intent.

### Radar
A possible source for lawful site-level awareness feeds, but performance and lawful operation depend on exact product, spectrum, site and environment. The programme should consume a qualified vendor feed rather than design a new radar. No weapon fire-control or target-quality tracking.

### RF
A passive view of available emissions can support awareness, but does not cover all autonomous/out-of-band/silent aircraft. No jamming, spoofing, private-content interception or takeover. Missing signal remains unknown.

### Acoustic
Use only privacy-preserving environmental/event features where there is a lawful, justified purpose. Do not retain intelligible speech, create voice identity records or use high-intensity audible/ultrasonic/infrasound deterrents. Measure and control both ambient and alert noise exposure.

### Laser / LiDAR
Keep LiDAR as an OEM-certified, eye-safe measurement product used strictly within its classification and instructions. IEC 60825-1 is the relevant laser product safety/classification reference; confirm the exact current edition and national adoption at the time of use. No blinding, dazzle, heating, damaging or disabling use.

### Ground robots
For equipment the site owns, rely on the OEM's validated safety controller, guards, interlocks, emergency stop and lockout procedures. ISO 12100 and the applicable robot/industrial-truck standards are starting points, but the exact machine class and exclusions must be determined by a competent robotics safety professional. A model recommendation must not become an actuator command.

## 6. Industry comparison: fair and bounded

The public product pages for Dedrone RF-360, DroneShield DroneSentry-C2 and Sentrycs DTI are useful capability references, not independent performance data. Their detection, tracking, identification and mitigation functions must not be conflated.

For a fair comparison, include:
- the site's current manual procedure;
- a passive single-modality baseline;
- incumbent multi-sensor awareness feeds;
- the Luminous evidence/triage layer consuming the same feed(s).

Pin exact products/releases/configurations, lawful data rights, installation context, weather/clutter, observation windows, operator training/time, label adjudication and denominators. Report vendor feature claims separately from independent measured results. No comparison should include active mitigation or effectors.

## 7. Proposed qualification ladder

- **G0 — Synthetic contract:** current state; verify schema and authority semantics only.
- **G1 — Legal/privacy feasibility:** SACAA/ICASA applicability, site owner authority, POPIA/privacy, data rights and community consultation.
- **G2 — Vendor/OEM desk review:** exact passive sensor feeds, product safety files, calibration, maintenance and system boundary.
- **G3 — Non-operational tabletop:** synthetic events, human review, alert comprehension, false reports, outage and correction workflow.
- **G4 — Passive detection-only pilot:** only after legal/site/professional approvals; no effector, takeover or third-party control. Keep people and property protected by existing approved procedures.
- **G5 — Independent assessment:** held-out event data, blind adjudication, matched incumbent baseline, privacy audit, safety review and lifecycle cost.
- **G6 — Operational consideration:** separate authority, regulatory, insurance, procurement, cybersecurity and professional approvals. No automatic promotion from software evidence.

At every gate, Unknown remains Unknown; a missing sensor or uncertain identity does not justify escalation to hostile status.

## 8. Required independent reviewers

| Review seat | Scope |
|---|---|
| SACAA / aviation-law advisor | Applicable UAS category, airspace, operator responsibilities, incident reporting and any required permission |
| Site/airport/port operator | Site authority, local procedures, emergency escalation and data-controller role |
| ICASA/radio regulatory specialist and EMC professional | Exact equipment, spectrum/radio permission, site installation, EMC and power |
| Laser safety / optical-radiation specialist | Exact LiDAR product classification and installation; no effector system |
| Acoustics / occupational-hygiene specialist | Microphone purpose, environmental noise, exposure and warning sound level |
| Robotics safety engineer / OEM | Machine-specific risk assessment, standards applicability, guarding, interlocks and safety-rated stop/lockout |
| OT cybersecurity / functional-safety reviewer | Connected site systems, network boundary, change control and no actuator authority |
| Privacy/data-protection practitioner | POPIA lawful basis, proportionality, impact assessment, minimization, retention and rights |
| Community / resident representatives | Acceptability, accessibility, notice, complaint and appeal process |
| NCACC/legal counsel where applicable | Classification of any controlled or dual-use technology before development, manufacture, transfer or procurement |

Each review receipt must bind exact product/configuration, site/jurisdiction, source versions, reviewer competence/registration, scope, exclusions, findings, disposition and date. No professional may sign beyond their competency and appointment. No external sign-off has yet been received.

## 9. Standards applicability register

This list is a scoping aid, not certification:
- IEC 60825-1 for laser product safety/classification of the exact OEM LiDAR product.
- ISO 12100 for machinery risk assessment and risk reduction.
- ISO 10218-1/-2 for industrial robots and industrial robot applications/cells, where in scope.
- ISO 3691-4 for driverless industrial trucks/systems, where applicable.
- ISO 13850 for emergency-stop principles, not complete machine safeguarding.
- ISA/IEC 62443 for connected industrial automation/control systems.
- Current SACAA requirements and applicable South African radio, privacy and workplace safety law, confirmed by competent professionals.

## 10. Claim ceiling and final review disposition

This review supports only the synthetic research boundary and next-step plan. It does not establish drone detection performance, threat intent, airworthiness, authorization to operate, radio licensing, legal compliance, privacy compliance, robot safety, laser safety or protection from attacks.

**Final disposition:** proceed with synthetic research and vendor/standards desk analysis. Block physical pilot until G1–G3 and the relevant professional reviews are complete. Keep all active countermeasures, third-party control and weapon effectors out of scope.
