# GCPN Evidence-Reconstructible After-Event Review Benchmark v0.1

**Status:** research specification and machine-readable scenario profile; executable review engine outstanding.  
**Parent issue:** #5656.  
**Baseline:** integrated evaluation v0.6 and evidence graph lifecycle specification #5647.  
**Scope:** offline synthetic early-warning system review, bitemporal evidence reconstruction and learning traceability.

## 1. Research basis

UNDRR, IFRC, ITU and WMO published *After-Event Review (AER): Methodological Guidance on Conducting After-Event Reviews of Early Warning Systems* on 15 July 2026. The guidance describes a structured and learning-oriented approach to review how monitoring, forecasting, warning dissemination, communication, coordination and response performed during a hazardous event. It emphasizes constructive, non-punitive learning rather than blame assignment.

The GCPN contribution should be a reproducible evidence/timeline substrate that helps a facilitator reconstruct what was expected, what was known, what occurred and why gaps arose. It must not replace local institutions, community participants, an AER facilitator or the authority responsible for review.

## 2. Paradigm shift: evidence-reconstructible learning

A retrospective narrative is necessary for context, but it can be vulnerable to hindsight bias, incomplete records, selection bias and unsupported causal stories.

A reproducible AER record should preserve:
- the pre-event plan and assumptions as they existed before the event;
- the evidence actually available at each decision time;
- event, source, forecast, authority, message, delivery and response timelines;
- what was observed, what was not observed and what arrived later;
- alternative explanations and contradictory evidence;
- system-level findings and recommendations traceable to the evidence.

Use the bitemporal graph from #5647. Late-arriving evidence may change the current review, but must never be silently inserted into a historical snapshot of what decision-makers could have known at the time.

## 3. Review objects

| Artefact | Minimum information |
|---|---|
| Pre-event plan | Version, approval, validity period, assumptions, named owner role |
| Expected milestones | Planned observation/forecast/authorization/dissemination/preparedness/response milestones |
| Observed timeline | Event time, recorded time, source, uncertainty and verification state |
| Forecast evidence | Model/source, issue time, valid time, lead, forecast vintage and outcome vintage |
| Warning evidence | Authority assertion, exact CAP message/version, update/cancel links and distribution records |
| Human-facing evidence | Accessible representation, language, comprehension-study status and limits |
| Response evidence | Authorized action records, observed completion and missingness |
| Outcome evidence | Outcome measure, observation window, provenance and confounding limitations |
| Findings | Supporting and contradictory evidence, causal status and alternative hypotheses |
| Recommendations | Evidence references, system-level issue, owner role, dependency, measurable follow-up and review date |

Do not generate a blame assignment or individual performance ranking. Roles and institutions may be named as process owners where source evidence and review governance permit, but the benchmark does not score personal suitability or fault.

## 4. Known-DGP scenario families

| Scenario | Known truth / challenge | Review failure to detect |
|---|---|---|
| Correct forecast, late warning | Forecast was valid; authorization/dissemination delay caused process gap | Incorrectly blaming forecast |
| Forecast uncertain, precautionary warning | Authorized warning was precautionary; hazard did not materialize | Treating non-occurrence as proof warning was wrong |
| Forecast miss, independent source acted | Forecast missed; another valid source prompted timely action | Single-component attribution |
| Correct message, inaccessible format | CAP message valid; format/channel mismatch excluded some groups | Delivery conflated with comprehension |
| Stale cached CAP version | Update issued; old version persisted in one channel | Missing lifecycle reconstruction |
| Local fallback during partition | Local relay worked while regional network was unavailable | Central-only performance view |
| Cross-jurisdiction delay | Authority handoff lagged; issue rights remained local | Treating relay as issuing authority |
| Late evidence | Critical record arrived after decision cutoff | Hindsight insertion |
| Dependent corroboration | Multiple sources shared an upstream provider | Inflated source independence |
| Compound hazard | Multiple hazard timelines overlapped | Incorrect event-window joins |
| Movement-constrained cohort | Some people could not move during warning window | Assuming all exposed people could evacuate |
| Outcome despite good warning | Warning chain performed well but adverse outcome still occurred | Attributing outcome to warning failure |
| No outcome despite poor warning | Hazard did not cause observed loss despite weak warning | Claiming warning quality from outcome alone |

Each synthetic scenario supplies a known causal graph and a pre-event plan snapshot. The review receives only evidence available by each declared cutoff. Ground truth is used to assess causal calibration, not to grant the reviewer retrospective clairvoyance.

## 5. Causal claim discipline

Keep these levels separate:
1. **Chronology:** A happened before B.
2. **Association:** A and B co-occurred.
3. **Process contribution:** a documented warning-chain step deviated from plan or constrained a later step.
4. **Causal effect:** a defined intervention changed a defined outcome relative to a defensible counterfactual.

A causal claim requires an explicit target question, causal graph/assumptions, confounders, outcome measurement, counterfactual definition and data limitations. A forecast miss, message receipt or action record is not by itself proof of the cause of an outcome.

The benchmark must include:
- confounding and common-cause scenarios;
- selection into observed recipient feedback;
- missing-not-at-random reports and survivorship/reporting bias;
- different outcomes under identical warning process quality;
- different warning process quality with similar outcomes;
- multiple interacting contributors;
- ambiguous cases where the correct review conclusion is "undetermined".

The benchmark must not reward overconfident causal explanations. No lives-saved or loss-avoided causal claim may be made from a synthetic process trace alone.

## 6. Hindsight and information-cutoff invariants

For every review question, declare an as-of cutoff (T). The review may only use evidence whose transaction/knowledge time is at or before T when judging what was available to a decision-maker by T.

Late source-created records, later forecast revisions, subsequent revocations, corrected message records and after-event participant feedback must remain visible in the current view but cannot rewrite earlier as-of views.

The benchmark must separately evaluate:
- event/valid time reconstruction;
- transaction/knowledge time reconstruction;
- source-created time where supplied;
- pre-event plan version in force;
- policy/authority version in force.

## 7. Metrics

Report independent dimensions:

- expected-versus-observed milestone traceability;
- chronology reconstruction accuracy;
- historical as-of reconstruction accuracy;
- late-evidence visibility without hindsight insertion;
- provenance and authority completeness;
- contradictory evidence retention and alternative explanations;
- process-deviation detection by chain stage;
- causal claim calibration against known synthetic DAGs;
- false causal attribution and unsupported certainty rates;
- missed-contributor rate;
- system-level finding evidence linkage;
- recommendation-to-evidence traceability;
- recommendation completeness and follow-up verifiability;
- unresolved-evidence proportion;
- review runtime and facilitator-facing evidence burden.

Do not combine these into a single system score or organization leaderboard. Structural completeness is not causal validity. A review can be complete and still correctly conclude that the causal effect is unidentified.

## 8. Proposed research targets

These are internal research hypotheses, not UNDRR/WMO/IFRC/ITU standards or measured results.

- 100% historical as-of invariants in frozen temporal mutation tests.
- Zero insertion of later evidence into earlier information snapshots.
- Zero causal claim accepted without the required target/assumption/provenance fields.
- Zero automated individual blame or ranking.
- 100% traceability of every recommendation to one or more cited evidence records and a declared system-level issue.
- Explicit unresolved/alternative hypotheses where the known DGP is not identifiable from the review evidence.
- Zero false certainty in scenarios where the causal effect is not identifiable.
- Deterministic replay from the exact DGP, plan version, evidence log, cutoff and evaluator version.

## 9. Integration boundary

This is a standalone benchmark first. It reuses the evidence graph from #5647 rather than creating a new provenance/event format. It does not add a ninth gate to the eight-module v0.6 conjunction.

A future integration requires:
1. complete synthetic timeline/plan fixture;
2. exact bitemporal reconstruction oracle;
3. causal calibration and hindsight controls;
4. evidence-backed recommendation linkage;
5. non-punitive and privacy review;
6. independent methodology review.

An automated report is a support artefact. It is not a replacement for facilitated AER work, community perspectives, institutional accountability or national review arrangements.

## 10. Evidence progression

1. synthetic timeline and plan replay;
2. independent review of causal DGP and information cutoffs;
3. community and practitioner co-design of review artefacts;
4. authorized, data-minimized historical replay under applicable governance;
5. facilitated tabletop/continuity exercises;
6. independent shadow AER support with no operational decisions;
7. adoption only by relevant institutions under their established review process.

## 11. Sources

- UNDRR/IFRC/ITU/WMO, *After-Event Review (AER): Methodological guidance on conducting after-event reviews of Early Warning Systems* (15 July 2026): https://www.undrr.org/publication/documents-and-publications/after-event-review-aer-methodological-guidance-conducting
- UNDRR, Arab States training for AER facilitators (2 March 2026): https://www.undrr.org/news/arab-states-strengthen-early-warning-systems-through-training-after-event-review-facilitators
- UNDRR/WMO/IFRC/ITU, *Early Warning Systems for mobile populations: Operational guidance* (15 July 2026): https://www.undrr.org/publication/documents-and-publications/early-warning-systems-mobile-populations-operational

## 12. Non-claims

Synthetic review methodology only. No live incident investigation, legal finding, individual blame assignment, personal ranking, operational warning decision, responder direction, causal lives-saved claim or public-sector performance leaderboard. National institutions, community participants and qualified facilitators retain their roles in actual after-event reviews.
