# GCPN Matched Civilian Awareness Comparison — preliminary desk review v0.1

**Review date:** 8 November 2026  
**Review type:** AI-assisted research-protocol / evidence-boundary desk review  
**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child:** [GCPN-AWARENESS-INDUSTRY-COMPARISON-001 #5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Draft implementation:** [PR #5941](https://github.com/Luminous-Dynamics/mycelix/pull/5941)  
**Disposition:** continue synthetic protocol and vendor/standards desk research; physical sensor track blocked pending permissions and independent professional review.

This is not a CWA conformity assessment, aviation-law opinion, ICASA determination, privacy/legal opinion, vendor acceptance test, professional engineering approval or physical sensor performance assessment. No external professional has signed this review.

## 1. Exact material and scope

Reviewed:
- GCPN_AWARENESS_INDUSTRY_COMPARISON_V0_1.md — matched comparison protocol.
- GCPN_AWARENESS_INDUSTRY_COMPARISON_CASES_V0_1.json — 10-case synthetic comparison manifest.
- Rust strict-manifest verifier and exact-head workflow.

The protocol contains four public-capability baseline classes (manual site workflow, Dedrone RF-360, DroneShield DroneSentry-C2, Sentrycs DTI) and two explicitly separate evaluation tracks:
- Track A: common-event-envelope replay for downstream evidence/provenance/human-review workflow.
- Track B: future physical sensor performance against independently established ground truth; explicitly blocked.

The implementation verifier's exact-head workflow passed 11 Rust tests, strict schema/boundary checks and CLI verification on 8 November 2026. The general Mycelix CI workflow was skipped on this draft PR. Those checks establish manifest semantics only.

## 2. Disposition

**The protocol is suitable to continue as synthetic research and desk comparison. Track B is not authorized and must remain blocked until legal, site, vendor, privacy, ground-truth, safety and professional gates are met.**

The strongest design choice is separating downstream event-envelope handling from physical sensor detection. A software evidence layer does not make a radar/RF/optical sensor more capable. Conversely, a sensor with high recall does not establish good evidence provenance, privacy, authorization or human decision quality.

The CWA 18150:2024 reference is appropriate as a methodological pointer, but this Luminous package does not reproduce its copyrighted test method and does not claim conformity. Obtain authorized access to the current standard and a qualified standards review before making any conformity statement.

## 3. Strengths observed

1. **Correct comparison decomposition.** Track A and Track B answer different questions and cannot be collapsed into one product leaderboard.
2. **Matched-budget discipline.** The protocol asks for pinned product/configuration, exact event set, denominator, observation window, human-review budget, privacy and lifecycle-cost basis.
3. **Fair vendor characterization.** Product pages are presented as public vendor claims, not independent results; hardware/firmware/release details remain unpinned until an actual evaluation.
4. **Privacy and authority boundaries.** Operator identity/location, precise tracks, raw audio/video, active mitigation and third-party control are excluded.
5. **Hostile-label resistance.** Vendor Friendly/Neutral/Hostile/Unknown labels cannot become ground truth, intent or action authority.
6. **Standards caution.** CWA 18150 is referenced, not copied; no conformity claim is enabled.
7. **Synthetic verification.** Typed structures reject unknown fields; the exact corpus is SHA-256 pinned; adversarial controls test track separation, hostile-label authority, operator location, hard gates and vendor claim status.
8. **Economics included.** Staff, licensing, installation, training, calibration, maintenance, energy, downtime and review burden must be costed, not silently treated as free.

## 4. Findings and required closure

| ID | Severity | Finding | Required closure |
|---|---|---|---|
| AIC-PR-01 | Blocker for Track B | No real site, operator, airspace jurisdiction, system configuration or trial purpose is selected. | Select a site only with documented owner authority, accountable data/controller roles, operating profile, insurance and legal basis. |
| AIC-PR-02 | Blocker for Track B | CWA 18150 full text has not been acquired or professionally interpreted. | Obtain authorized access; appoint an independent standards specialist to map the permitted test scope and applicability. Do not claim conformity from the public summary page. |
| AIC-PR-03 | Blocker for Track B | South African ICASA radio/spectrum/type-approval applicability is unresolved. | ICASA states that type approval may apply to radio equipment and that section 31(1) concerns using radio apparatus to transmit or receive radio signals subject to a spectrum licence except as legally provided. “Passive” / “receive-only” is not a blanket exemption. Obtain written equipment/frequency-specific applicability advice before import or test. |
| AIC-PR-04 | Blocker for Track B | UAS legal applicability and current Part 101 amendments are not yet reviewed by counsel for a site/use case. | Confirm exact operative consolidated CAR/CATS, amendments, commencement, exemptions and SACAA/site permissions. Gazette 55226 indexes separate GoN 7844 (33rd Amendment) and GoN 7845 (36th Amendment) on 21 August 2026; do not mistake separate notice numbers for a legal interpretation. |
| AIC-PR-05 | Major | Vendor claim metadata currently pins public pages, not exact customer hardware/software/configuration or export interface. | At procurement/POC stage capture dated model/serial/revision/firmware/software/subscription/configuration and vendor-supported export schema. Record missing/withheld fields; do not reverse engineer protected or restricted interfaces. |
| AIC-PR-06 | Major | Baseline performance remains unmeasured. | Do not publish detection/false-alarm/range/latency results until a lawfully authorized, blinded and independently adjudicated Track B study is completed. |
| AIC-PR-07 | Major | The 20% Track A improvement target is a hypothesis and may not be achievable or relevant to the chosen incumbent baseline. | Preregister outcome definitions, matched denominator, baseline, held-out split, reviewer workload measure and non-degradation margin. Report negative results. |
| AIC-PR-08 | Major | Event-envelope commonality may be incomplete because products export different event semantics and source fields. | Publish per-baseline mapping/coverage and missingness. Mark a comparison INCOMPARABLE when semantic parity cannot be established; do not fill unsupported fields. |
| AIC-PR-09 | Major | Independent ground-truth method and adjudicator have not been appointed. | Use an independent authority/source for event truth and event timing; separate reference instrumentation from evaluated systems; preserve uncertainty and disagreement. |
| AIC-PR-10 | Major | Privacy impact, POPIA lawful purpose, data-controller/processor roles, retention and community notice are not established. | Obtain privacy counsel/DPO review, data minimization plan, lawful basis, role allocation, access controls, breach process, deletion schedule and community engagement before any real data. |
| AIC-PR-11 | Major | Vendor "Hostile" labels, Remote ID/serial identity and authorization signals may be confused with actual intent or lawful authority. | Keep object observation, claimed identity, authentication, current authorization, intent and competent-authority hazard determination separate; never infer intent from presence or a vendor label. |
| AIC-PR-12 | Major | Operator geolocation/flight paths and exact facility data are present in some public vendor feature descriptions but are expressly excluded here. | Verify export redaction on a lab sample before accepting any vendor feed. No operator identity/location, exact trajectory, sensitive coordinates, private content or bystander data in Track A. |
| AIC-PR-13 | Major | Life-cycle cost is not yet based on quotes, measured staff time or a cost plan. | Apply Mycelix #5930's matched full-lifecycle cost policy and independently reviewed local cost basis; no assumed-free labour/data/maintenance. |
| AIC-PR-14 | Observation | Public vendor product claims and marketing language (including statements about legal authorization) may be misapplied across jurisdictions. | Treat each claim as source-specific marketing until independently verified; South African permissions are determined by local law/authority, not vendor wording. |

## 5. Industry baseline interpretation

The selected products are not identical types of system:
- Dedrone RF-360 is a passive RF sensor; do not compare it as though it were a multi-sensor C2 product.
- DroneShield DroneSentry-C2 is a C2/fusion platform with an ecosystem that includes optional effectors; score only approved awareness feed and evidence-presentation functions.
- Sentrycs DTI is a passive RF-based detection/identification module offered alongside a separate mitigation module; operator location and mitigation are not in this benchmark.
- The site's manual workflow is necessary as a real baseline and must include staff and administrative cost.

For a fair comparison, baseline feature counts, vendor-reported detection ranges, marketing statements and product screenshots are not outcome metrics. Actual scoring needs same eligible event set, ground truth, observation window, operator budget, privacy policy and complete resource cost.

## 6. South African regulatory notes

ICASA's official pages:
- Type approval: https://www.icasa.org.za/pages/type-approval
- Spectrum licensing: https://www.icasa.org.za/pages/spectrum-licensing

SACAA's UAS information:
- https://www.caa.co.za/industry-information/uas/

Government Gazette index:
- https://www.gov.za/documents/notices?0=&1=&page=3

These sources are a research starting point, not a legal opinion. No deployment or trial may proceed on the assumption that a receive-only RF sensor requires no licence or that a public vendor page establishes South African permission.

## 7. Required professional review panel

| Reviewer | Required scope |
|---|---|
| Independent CWA 18150 standards specialist | Authorized method access, applicability, test-case design and reporting limits |
| SACAA/aviation-law specialist | Correct UAS category, legal site context, permissions, reporting and current rules |
| ICASA/radio spectrum and type-approval specialist | Exact receiver/radio product, bands, location, licensing, type approval and import/use |
| Site/airport/port operator | Site authority, data controller, emergency procedure, operating envelope and local safety |
| Privacy/POPIA counsel or DPO | Lawful purpose, minimization, redaction, access, retention, data subject rights and community notice |
| Independent ground-truth adjudicator | Truth-source independence, event matching, bias, missingness and disputes |
| Qualified site-safety / insurance reviewer | Trial risk, insurance coverage, existing emergency procedures and no-effector scope |
| Quantity surveyor / economist | Full lifecycle cost, staffing, integration, maintenance and opportunity cost |
| Vendor/OEM representatives | Supported product release, lawful API/export, documented limitations and safe integration conditions |

Each reviewer must record exact artifact/product revision, jurisdiction, appointment, competence/registration where applicable, review scope, exclusions, findings, conditions, disposition and date. No external professional signature has yet been received.

## 8. Qualification stages

- **G0 — Synthetic comparison contract:** current state; schema and authority semantics only.
- **G1 — Standards/legal feasibility:** authorized CWA reference, SACAA and ICASA written applicability, privacy/insurance/site permissions.
- **G2 — Vendor desk/adapter qualification:** exact product/configuration, lawful export, provenance and redaction sample.
- **G3 — Track A replay:** same frozen, privacy-minimized event envelopes; independent event oracle; matched operator-review budget.
- **G4 — Track B physical sensor study:** only after G1–G3 review, lawful benign environment and independent ground truth; no mitigation or effectors.
- **G5 — Independent assessment:** held-out results, source/licence audit, privacy review, lifecycle cost and professional sign-off within each scope.

## 9. Final disposition and non-claims

**Continue Track A synthetic development and standards/vendor desk research. Track B remains blocked.**

This review does not establish sensor performance, vendor superiority, detection range, false-alarm rate, intent, UAS airworthiness, ICASA permission, SACAA permission, POPIA compliance, CWA conformity or operational protection. A CI pass is not professional review or legal authorization.
