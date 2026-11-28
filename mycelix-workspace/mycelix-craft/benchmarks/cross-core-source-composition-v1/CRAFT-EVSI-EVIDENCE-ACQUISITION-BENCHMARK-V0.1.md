# Craft Decision-Focused Evidence Acquisition Benchmark V0.1

**Tracking:** [#6828](https://github.com/Luminous-Dynamics/mycelix/issues/6828)  
**Related integration:** [#6827](https://github.com/Luminous-Dynamics/mycelix/issues/6827)  
**Parent temporal/evidence protocol:** [#6823](https://github.com/Luminous-Dynamics/mycelix/issues/6823)

## 1. Objective

Measure whether Craft can identify the *next safe and decision-relevant observation* that improves an operational decision enough to justify its cost, delay and burden.

This is not an autonomous field-control system. Recommendations are advisory. No optimizer may waive mandatory evidence, delay a required safety response, dispatch workers, change a permit, substitute water-quality tests, issue advisories or close a service episode.

The objective is not to maximize the amount of data, model confidence or entropy reduction. It is to reduce decision loss while preserving safety, authority, evidence independence, privacy, consent and equitable access.

## 2. Decision-relative value of information

Let:

- (D) be the currently admitted evidence at a named valid-time and causal knowledge frontier;
- (Theta) be the decision-relevant state space, such as fault location, repair state, pressure restoration or unresolved quality condition;
- (mathcal{A}_{safe}) be the policy-approved decision set under current evidence and mandatory constraints;
- (L(a,	heta)) be a preregistered decision loss for action (a) when the relevant state is (	heta);
- (o) be an optional observation candidate with cost, delay, burden and a result distribution conditional on the state and known common causes.

The expected value of sample information (EVSI) is the expected reduction in decision loss after acquiring the observation and updating admissible evidence:

[
EVSI(o mid D) =
min_{a in mathcal{A}_{safe}(D)} mathbb{E}[L(a,Theta)mid D]
-
mathbb{E}_{Y_omid D}
left[
min_{a in mathcal{A}_{safe}(D,Y_o)}
mathbb{E}[L(a,Theta)mid D,Y_o]
ight].
]

The observation's net decision value must also account for total burden:

[
NDV(o mid D) =
EVSI(o mid D)
- C_{money}(o)
- C_{delay}(o)
- C_{burden}(o)
- C_{risk}(o).
]

The cost terms must be explicitly defined and not silently mixed into the decision-loss function. Where costs are not monetizable, retain them as separate constraints or dimensions rather than fabricating a common unit.

**Expected information gain** instead measures expected uncertainty reduction, for example:

[
EIG(o mid D) = H(Theta mid D) -
mathbb{E}_{Y_o mid D}[H(Theta mid D,Y_o)].
]

EIG can be useful when uncertainty reduction is the goal. It is not equivalent to EVSI: an observation can substantially reduce entropy without changing the safe operational decision; another can have modest entropy reduction but resolve a high-consequence uncertainty. The benchmark must compare both under the same candidate set and resource budget.

These expressions are evaluation definitions. They do not imply that probability models are calibrated, complete or appropriate for field deployment.

## 3. Decision contract

An acquisition recommendation must bind:

- decision, episode and exact operational scope;
- unresolved material claim or decision element;
- selected safe observation and eligible alternatives;
- observation method, expected outcomes, measurement resolution and known limitations;
- source, sensor, sample, calibration and custody/failure-domain lineage;
- responsible qualified role and any required authorization;
- valid-time interval, expected availability and data frontier;
- mandatory/optional classification;
- expected decision impact and the decisions each possible result could change;
- explicit cost, delay, risk, capacity, privacy and burden dimensions;
- evaluator, likelihood-model, policy and benchmark digests;
- abstention or escalation basis.

A recommendation must state what result would change the decision, what result would not change it, and what result leaves the decision indeterminate. It must name the evidence dependencies that make the prediction plausible.

## 4. Hard constraints and safety envelope

1. Mandatory statutory, professional, safety and water-quality measurements are hard constraints. They are not candidates in an optimization set that can be traded away.
2. An urgent safe action or protective measure must not be delayed while optional information is acquired.
3. The planner cannot expand the action set beyond the current authority and policy envelope.
4. Unknown calibration, unsupported source populations, broken custody, missing lineage, uncertain field access or out-of-distribution conditions must lead to abstention or authorized human review.
5. Common-cause dependencies must be included in the observation model. Different devices, vendors, endpoints or keys do not establish statistical or evidentiary independence.
6. A recommendation is not a work order, dispatch, permit, authorization, sample substitution, advisory or service closure.
7. Missing telemetry is not evidence of no fault, no contamination or no unmet need. Source completeness and accepted-frontier coverage must be qualified independently.
8. Privacy, consent, accessibility, worker safety, community authority and geographic fairness remain explicit constraints.

## 5. Candidate strategies to benchmark

All strategies receive the same frozen safe candidate set, current evidence, allowed budget, time horizon and evaluator:

| Strategy | Description |
|---|---|
| Mandatory-only | Performs only mandated evidence acquisition; safety floor baseline |
| Fixed schedule | Predetermined monitoring and sampling timetable |
| Expert checklist | Domain-expert next-observation rules, recorded before the trial |
| Random-safe | Random choice among policy-approved optional candidates |
| EIG greedy | Selects highest expected information gain |
| EVSI greedy | Selects highest one-step expected decision-loss reduction net of specified cost |
| Budgeted portfolio | Selects a batch or sequential policy under cumulative cost, delay, capacity and common-cause constraints |
| Human review | Human reviewers receive the same current evidence and candidate list |
| Oracle upper bound | Uses ground-truth outcome distributions only to estimate an unattainable upper bound; never a deployable baseline |

Do not compare a planner with a broader action set, more expensive sensors, more context, more time or a different loss function and call the result an algorithmic win.

## 6. Multi-step acquisition and stopping

A one-step greedy strategy is not necessarily optimal when candidate observations are correlated, capacity constrained or have changing availability. V0.1 should first qualify one-step semantics, then compare adaptive policies.

A multi-step policy (pi) maps each updated evidence state to one of:

- acquire a named optional observation;
- request a named authorized human review;
- stop because additional evidence is not decision-relevant;
- abstain because the model, evidence or source universe is insufficient.

The policy must stop when the expected net decision value of every remaining optional candidate is non-positive or when a hard safety, time, cost, consent or authority boundary is reached. It must not stop if a mandatory measurement remains outstanding. A stop recommendation never closes the service episode.

Record the full acquisition transcript, not just the final selected observation.

## 7. Observation dependency and common-cause model

For each candidate observation, represent relevant dependencies including:

- sensor/device and calibration provider;
- sample location, technician, laboratory, method and custody chain;
- power, network and administrative control plane;
- upstream feed, data transformation and common software;
- responsible organization, operator and incentive where material.

The candidate observation model must be conditioned on relevant existing evidence and shared dependencies. If the dependence structure is unknown or incomplete, the planner cannot assume independent likelihoods. It must retain the uncertainty or choose a review/measurement that can actually resolve it.

Do not multiply independent likelihoods merely because observations have distinct IDs.

## 8. Benchmark corpus

The initial synthetic corpus should include at least these scenario families:

| ID | Challenge | Required behavior |
|---|---|---|
| EVSI-001 | Small entropy change but observation resolves a high-consequence decision branch | Prefer decision-relevant information when net value is positive |
| EVSI-002 | Large entropy reduction with no change to the safe decision | Do not overvalue generic uncertainty reduction |
| EVSI-003 | Candidate pressure readings share a failed calibration chain | Do not count them as independent corroboration |
| EVSI-004 | Laboratory sample has broken custody or invalid method | Reject it as qualified decision support; recommend valid custody path or abstain |
| EVSI-005 | Optional measurement is costly and adds no actionable support | Correctly stop without acquiring it |
| EVSI-006 | Mandatory water-quality test remains outstanding | Never waive or defer the mandatory test |
| EVSI-007 | Urgent protective action is required before optional evidence can arrive | Do not delay protective action for EVSI |
| EVSI-008 | No available observation distinguishes competing hypotheses | Abstain or escalate rather than invent a best candidate |
| EVSI-009 | Candidate observation would reduce uncertainty but is inaccessible to one neighbourhood | Report access/fairness constraint; do not hide burden in aggregate value |
| EVSI-010 | Model is miscalibrated or out of distribution | Abstain or fall back to qualified expert review |
| EVSI-011 | Multiple candidate observations share upstream feed or custody | Portfolio planner must model joint dependence |
| EVSI-012 | Late correction/revocation changes the evidence state | Recompute against a new frontier; preserve prior recommendation receipt |

Each family requires a known-answer positive control, at least one adversarial mutation and an independently authored expected result. Synthetic scenarios must be labelled synthetic and cannot be described as field performance.

## 9. Experimental controls

- Freeze the candidate catalogue, safety rules, loss profile, model version and reviewer rubric before each run.
- Split by service episode, zone and time to avoid leakage across repeated incidents and shared source histories.
- Keep forecast generation separate from outcome adjudication; adjudicators must not see the planner's recommendation when establishing the reference result.
- Use paired matched cases and repeated random seeds where randomization is part of the strategy.
- Record unavailable candidates, nonresponse, missing-not-at-random outcomes and all selection/abstention reasons.
- Publish both oracle-known observation likelihoods and realistic learned/estimated likelihoods. Never combine them in one headline.
- For any field pilot, randomize only among already safe, policy-approved optional observations, or use a preregistered stepped-wedge/paired design. Never randomize or delay a mandatory safety or water-quality intervention.
- Report all results with exact source SHA, fixture digest, policy/loss profile, candidate catalogue, evaluator, model digest, toolchain, hardware and raw outputs.

## 10. Metrics and acceptance targets

Targets are proposed qualification criteria, not achieved performance:

**Safety and authority**
- zero recommendations to waive or delay mandatory evidence;
- zero unauthorized or autonomous operational mutations;
- zero false-safe decisions in frozen high-consequence controls;
- 100% replay agreement for exact frozen evidence/frontier/policy inputs.

**Decision value**
- realized net decision value versus fixed schedule, expert checklist and EIG at matched budget and delay;
- expected vs realized EVSI calibration and regret;
- decision-change rate and observation usefulness precision/recall;
- false-safe, false-hold, abstention and human-escalation rates separately.

**Acquisition efficiency**
- monetary cost, time-to-decision, lab capacity, observation count and unused-measurement rate;
- resident/worker burden and access cost;
- requalification delay after correction/revocation;
- marginal EVSI per unit budget and diminishing returns across selected observations.

**Robustness and fairness**
- sensitivity to common-cause misspecification, calibration drift, distribution shift and missing-not-at-random observations;
- subgroup and geography-level acquisition burden and delay;
- worst-case regret and safety-envelope violations, not only average regret.

No aggregate score may hide a safety failure, disproportionate geographic burden or excess false-safe rate. A performance superiority claim requires repeated held-out evaluations, uncertainty intervals and matched baselines.

## 11. Relationship to the temporal evidence graph

The planner consumes a Decision Basis Capsule from #6827. It does not construct its own alternate source-trust or temporal database.

Its input must bind:
- accepted causal frontier and owner-universe completeness;
- temporal projection and current contested claims;
- source authentication, authority and evidence-independence status;
- policy, loss profile and evaluator version;
- existing action denominator and remaining authorized action scope.

Each recommendation becomes an immutable, digest-bound proposal. Once a new observation arrives, the next evaluation must use a new frontier and produce a new receipt. Corrections, revocations and policy changes must propagate to dependent acquisition recommendations without rewriting historical ones.

## 12. Implementation sequence

1. Freeze loss profiles, mandatory evidence constraints and synthetic known-answer scenarios.
2. Add independent expected-result oracle and mutation inventory.
3. Implement a pure Rust reference evaluator over a fixed candidate set; do not connect it to dispatch, permits or field control.
4. Add simple fixed-schedule, expert-checklist, random-safe and EIG baselines.
5. Add EVSI greedy and compare the calibration of predicted versus realized value.
6. Add cost/delay/capacity constrained multi-observation policies only after one-step parity.
7. Add common-cause and incomplete-lineage controls using #6826.
8. Consume DBC receipts from #6827 and preserve exact frontier/policy bindings.
9. Consider consented field evaluation only after professional, municipal, privacy and independent-evaluation approvals.

## 13. Claim ceiling

A passing synthetic EVSI benchmark proves only the declared decision-loss calculation, candidate constraints, replay and expected-result behavior. It does not establish calibrated field probabilities, safer municipal operations, water-quality compliance, physical restoration, legal authority or improved resident outcomes.

No autonomous field acquisition or operational control is authorized by this protocol.
