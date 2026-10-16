# Controller Assistance and Airspace Automation Benchmark V1

**Status:** Research protocol; not an operational ATC/UTM specification or safety case  
**Research checked:** 2026-10-15  
**Scope:** Mycelix AeroCommons research; synthetic scenarios only  
**Authority ceiling:** Offline evaluation and advisory research only  
**Related work:** [Airspace Coordination Architecture V1](AIRSPACE_COORDINATION_ARCHITECTURE_V1.md), [Geometry Interoperability Profile V1](AIRSPACE_GEOMETRY_INTEROPERABILITY_PROFILE_V1.md), [tracking issue #5023](https://github.com/Luminous-Dynamics/mycelix/issues/5023)

## 1. Decision

The first target is **controller assistance, not wholesale controller replacement**. Begin with strategic planning, early conflict detection, data-quality monitoring, workload reduction, and explainable candidate resolutions. Explore autonomous coordination only in a bounded operating domain whose rules, traffic participants, aircraft capabilities, containment, oversight, and degraded modes are explicitly defined.

This is not a conclusion that humans must always own every function. It is a sequencing rule: automate a specific task, define its operating assumptions, and qualify the automation for that task before granting more authority. The FAA's September 2025 *Safety Framework for Aircraft Automation* explicitly recommends describing which tasks are automated instead of relying on broad labels such as “autonomous aircraft.” [FAA Safety Framework](https://www.faa.gov/aircraft/air_cert/step/safety_framework_aircraft_automation)

## 2. Claims we are and are not testing

### Testable research claims

- The system identifies synthetic conflicts at least as reliably as the declared deterministic baselines within the model's bounds.
- Proposed schedule or route changes reduce one or more operational costs without violating hard constraints.
- Missing, stale, contradictory, or incompatible inputs degrade the result explicitly; they do not silently become a reassuring “clear” result.
- An independent checker can reject candidate plans that violate the benchmark's deterministic invariants.
- Explanations, provenance, and uncertainty markers help a qualified reviewer detect unsuitable recommendations without causing excessive display or workload burden.
- Results are reproducible from a pinned corpus, code revision, configuration, and seed.

### Claims excluded

This benchmark will not establish real-world separation assurance, prove a physical reachable set, validate any jurisdiction's procedures, certify a system, approve flights, or establish that an automated agent can replace licensed controllers. The existing crate's constant-velocity/local-coordinate models and synthetic thresholds must not be treated as operational aircraft dynamics or separation standards.

## 3. Evaluation tracks

Keep the tracks separate. Passing one track never implies passing another.

| Track | Question | Permitted output in this phase |
|---|---|---|
| A. Data and situation awareness | Are tracks, intents, constraints, timestamps, frames, and freshness represented correctly? | Qualified display/record; explicit unresolved state |
| B. Advisory conflict management | Does the system detect conflicts and propose viable alternatives? | Ranked candidates with reasons, costs, uncertainties, and constraints |
| C. Strategic flow management | Can it improve sequencing, demand/capacity balance, and disruption recovery? | Offline schedule/trajectory proposals |
| D. Bounded UTM-style coordination | Can a narrowly defined synthetic operation coordinate cooperative participants and degrade safely? | Research simulation only; no live authorization |
| E. Human-machine interaction | Does assistance improve or impair controller performance and situation awareness? | Logged, controlled simulator study |

**Tactical separation service, aircraft control, and unrestricted autonomous ATC are out of scope.** They require separate system definitions, operational evidence, authorities, and assurance paths.

## 4. Baselines and experimental fairness

Compare the same scenario, starting state, information cutoff, constraints, compute budget, and scoring rules across:

1. **Declared deterministic baseline:** the current simple rules and conflict-screening behavior, with its exact limitations written down.
2. **Rule-based candidate planner:** a transparent, deterministic method with no learned component.
3. **Optimizer candidate planner:** a bounded search or optimization method; every proposal still passes through the independent checker.
4. **Symthaea advisory agent (later):** prediction, anomaly detection, explanation, and candidate generation only. It may not certify its own output.
5. **Human-controller baseline (later):** representative, appropriately qualified participants in an approved simulator study, with scenario order counterbalanced and performance assessed independently.

Do not assume existing controller actions are perfect ground truth. Real-world operational logs, if ever used, need permission, de-identification, provenance, context and selection-bias analysis, and an approved research procedure. Start with synthetic, generated scenarios and independently specified expected invariants. Avoid unlicensed use of operational datasets.

Every planner must run against an identical immutable scenario snapshot. A planner must not receive future state, cleaner inputs, or more compute than other methods unless that difference is the stated experimental variable.

## 5. Scenario matrix

Build the corpus by explicit scenario ID and controlled parameters. Every scenario records the tested invariant, expected classification or property, and the limitations of its oracle. Include nominal cases as well as negative controls and deliberately adversarial inputs.

| Family | Required initial cases |
|---|---|
| Ordinary flow | Parallel tracks; crossing; merge; departure-arrival interaction; multiple independent conflicts |
| Resolution quality | One-conflict/two-option trade-off; interacting three-aircraft conflict; no feasible candidate; candidate oscillation / route churn |
| Airport and flow disruption | Runway closure; reduced arrival rate; missed approach; departure queue spillback; weather-driven capacity reduction |
| Dynamic constraints | Restriction introduced, moved, expired, withdrawn, or received after the nominal issue time |
| Emergency / precedence | Emergency claim; diversion request; conflicting emergency and routine intents; explicit authority review rather than improvised precedence |
| Data integrity | Duplicate identity; stale track; missing update; sequence gap/fork; contradictory providers; unknown datum/frame/time scale |
| Degraded operation | Communications loss; provider partition; partial surveillance loss; delayed rejoin; failed clock qualification |
| Unexpected participants | Non-cooperative/unknown track; abrupt manoeuvre outside the simplified model; insufficient information to classify |
| Security and abuse | Replay, forged-looking-but-unverified source labels, conflicting updates, excessive request volume, denial-of-service style work bounds |
| Human factors | Alert flood; misleading confidence; explanation disagreement; automation unavailable at peak load; handover during an unresolved event |

Initial corpus target: at least 30 hand-authored scenario families/variants before optimizing a planner, with metamorphic and property-based expansion afterwards. This is a research coverage target, not evidence that “30 scenarios” proves safety. Each fixture must say whether its expected result is exact, bounded to the toy model, or only a qualitative review prompt.

### Essential negative controls

- A stale or partitioned snapshot must not produce a clean “no conflict” conclusion where missing information could change the answer.
- A syntactically valid reservation or high reputation value must not authorize a manoeuvre.
- A recommendation that is geometrically plausible but violates a hard constraint must be rejected by the independent checker.
- If no candidate satisfies every hard constraint, report **no feasible candidate / unresolved**; do not relax the constraint quietly.
- Emergency status must not invent jurisdiction-specific priority rules.
- Inputs that exceed declared time, geometry, entity, or computation bounds must fail closed with a typed outcome.

## 6. Two-layer evaluation architecture

Separate *proposal generation* from *acceptance checking*.

1. **Qualified state builder:** checks schema version, identifiers, provenance, time basis, coordinate reference, vertical datum, units, freshness, and snapshot completeness.
2. **Candidate generator:** produces zero or more route/sequence proposals; may be deterministic, optimized, or learned.
3. **Independent invariant checker:** evaluates every proposed candidate against the declared synthetic model and rejects disallowed candidates. It must not import the candidate generator's “safe” verdict as evidence.
4. **Scorecard:** computes operational costs only for candidates that pass the hard constraints. Never combine hard safety requirements into a weighted score that lets higher throughput compensate for a violation.
5. **Audit record:** binds the output to the source snapshot, code revision, configuration, corpus revision, random seed, checker revision, and result.

If the checker cannot decide within its declared model or work budget, the answer is unresolved—not safe. This independent checker is only as strong as its model; it cannot make a wrong or incomplete model correct, and it cannot be described as a certified safety monitor.

## 7. Scorecard: safety gates first, utility second

### Gate 1 — invariant conformance

Report counts and all counterexamples for:

- known conflicts missed by the model-defined oracle;
- candidates accepted despite violating any declared hard constraint;
- stale / incomplete / incompatible inputs incorrectly treated as fresh and clear;
- identity or event-chain ambiguity not rejected;
- state-machine, validity-window, or authority-boundary violations;
- non-deterministic replay given identical pinned inputs;
- work-limit violations, panics, invalid arithmetic, or untyped failures.

Acceptance rule for a declared deterministic invariant is zero counterexamples in the finite test corpus, plus property-based and adversarial testing. This is *fixture conformance*, not a numerical probability of aviation safety.

### Gate 2 — robustness and coverage

Report performance by scenario family and degraded mode, not just one average. Record corpus coverage, unique counterexamples, boundary cases, metamorphic properties, fuzzing configuration, seeds, and minimized failures. Add a new regression fixture for every confirmed defect before fixing it.

### Gate 3 — utility among valid candidates

After hard gates, compare:

- delay and delay distribution;
- throughput / capacity under the same constraints;
- route length and estimated energy/fuel proxy;
- time exposed to weather or other synthetic hazards;
- ground-impact/noise proxies for low-altitude studies, with explicit assumptions;
- route churn, number of trajectory changes, and operational complexity;
- computation time (median, p95, p99, maximum), memory/work budget, and response deadline misses.

Do not collapse safety and utility into one weighted score. Use a Pareto view for competing utility measures, and disclose who bears delays or detours. Averages must not hide tail cases or disproportionately burden a region, operator, or class of aircraft.

### Gate 4 — human factors (only when a proper study is possible)

In a simulator with qualified research oversight, assess time to detect and correct seeded hazards, workload, situation awareness, alert acknowledgement, trust calibration, inappropriate acceptance, override behavior, handover/recovery quality, and performance after the aid disappears. Use counterbalanced scenarios and predeclared measures. Do not infer human benefit from a model-only simulation.

EUROCONTROL's STRESS work specifically examined workload, vigilance, attention, stress and the transition between high and low automation; its SHAPE material highlights trust, situation awareness, and teamwork. ICAO similarly notes that automation can reduce routine workload but create new demands during dynamic, time-critical failures. [EUROCONTROL STRESS](https://www.eurocontrol.int/project/human-performance-neurometrics-toolbox-highly-automated-systems-design) · [EUROCONTROL SHAPE](https://www.eurocontrol.int/product/solutions-human-automation-partnerships-european-atm) · [ICAO Human Performance](https://www.icao.int/operational-safety/HP/considerations)

## 8. Proposed stages and authority ceilings

| Stage | Evidence required | Authority ceiling |
|---|---|---|
| 0 — Offline corpus | Schema validation, deterministic replay, boundary/property/adversarial tests | Synthetic research only |
| 1 — Shadow evaluation | Time-aligned, independently reviewed comparisons; documented data authority and privacy | No operational recommendations relied upon |
| 2 — Advisory simulator | Human-factors study, display/alert evaluation, failover rehearsal | Qualified human decides; aid can be ignored or disabled |
| 3 — Narrow approved operational use | Domain-specific hazard analysis, independent verification/validation, cyber/security case, training, regulator and service-provider approval | Only the approved task and operating domain |
| 4 — Broader autonomy research | Evidence across independent systems, severe degraded modes, safety case, formal approvals and ongoing monitoring | No presumption of authority; separately approved increment only |

A human “override button” is not a complete fallback. The fallback must be reachable in time, based on usable information, rehearsed, and available under the failure modes that trigger it. If the system relies on a human noticing an alert, evaluate alert load, response time, automation surprise, and skill decay.

## 9. Reproducible result record

Each future benchmark run should persist, at minimum:

- schema and corpus versions plus fixture IDs;
- repository commit SHA and built artifact identity;
- model/planner and checker versions;
- exact scenario/input snapshot digest (real cryptographic digest, not an opaque label);
- configuration digest, fixed random seed, and declared compute budget;
- per-scenario expected property, observed outcome, and counterexample;
- candidate count, rejected-candidate reasons, unresolved reasons, and resource-limit outcomes;
- metric definitions, units, summaries, tail distributions, and excluded runs with reasons;
- toolchain/dependencies and exact test commands;
- whether a result is locally reproduced, CI-verified at the same SHA, or still pending.

**Queued is not passed.** A result from an earlier SHA does not qualify a later SHA. Tool-side independent checks are useful corroboration but do not replace compiling and testing the committed Rust implementation.

## 10. First implementable milestone

The next implementation should be a deterministic *offline candidate-assessment harness*, not a live traffic optimizer:

1. Define a versioned schema for synthetic multi-aircraft situations, candidate proposals, and checker verdicts.
2. Add a small independently hand-reviewed corpus spanning conflicts, benign traffic, constraints, stale inputs, no-feasible-plan cases, and candidate violations.
3. Add a minimal transparent candidate generator (for example, a few declared synthetic speed/heading/altitude alternatives—not realistic manoeuvre guidance).
4. Run every candidate through the independent checker; separately report candidate rejection, unresolved state, and permitted toy-model outcomes.
5. Make verification fail when expectations drift or any hard invariant is violated.
6. Pin fixture count, unique IDs, schema version, code SHA and reproducible output; add counterexamples as regression tests.

Do not begin with an RL policy that directly chooses manoeuvres, a single composite “safety score,” or a live data feed. First make the scenario model, constraints, evaluation record, and negative controls explicit and reviewable.

## 11. Research references and interpretation

- [FAA, Safety Framework for Aircraft Automation (September 2025)](https://www.faa.gov/aircraft/air_cert/step/safety_framework_aircraft_automation) — task-level automation framing and safety continuum. Aircraft automation guidance is relevant to the assurance approach but is not, by itself, ATC system certification.
- [FAA, UTM Concept of Operations v2.0](https://www.faa.gov/researchdevelopment/trafficmanagement/utm-concept-operations-version-20-utm-conops-v20) — cooperative information exchange for low-altitude UAS, complementary to ATM. It is not a template for unrestricted crewed-aircraft autonomy.
- [FAA, UTM overview](https://www.faa.gov/uas/advanced_operations/traffic_management) — current description of UTM as distinct from but complementary to air traffic services, with ongoing work on interoperability and service recognition.
- [ICAO, Human Performance considerations](https://www.icao.int/operational-safety/HP/considerations) — automation changes people’s roles and may move workload toward time-critical recovery.
- [EUROCONTROL, Concept of Operations for Future Skies](https://www.eurocontrol.int/article/concept-operations-future-skies) — strategic oversight and increased automation as a direction for future ATM, not proof of present-day readiness.
- [Reuters, 13 October 2026 report on the FAA SMART tool](https://www.reuters.com/legal/government/lawmakers-ask-faa-pause-use-new-ai-tool-dc-area-flight-management-2026-10-13/) — reported debate about transparency, safety-evaluation evidence and personnel training for a decision-support deployment. Treat this as reporting on a live policy debate, not a technical assessment of the system’s performance.

Research references last checked 2026-10-15. Applicable rules, accepted standards and competent aviation authorities must be checked again for any specific jurisdiction or real operational proposal.
