# Institutional Performance Benchmark (IPB) v0.1

**Status:** Proposed benchmark specification; no scores or performance claims have been established.  
**Purpose:** Compare institutional arrangements on matched tasks, including conventional organizations, cooperatives, federations, public agencies and DAO-like systems.

## 1. Benchmark principle

Benchmark institutional capabilities and outcomes, not institutional labels. A result is meaningful only when the task, context, participant resources, legal constraints, threat model, baseline and measurement procedure are disclosed.

Do not produce a single composite "institution quality" score in v0.1. Publish a multidimensional profile with uncertainty intervals and disaggregated impacts.

## 2. Benchmark units

Each test case defines:
- Institutional context and legal jurisdiction.
- Mission and service objective.
- Stakeholder groups and affected non-participants.
- Rights, obligations and non-negotiable constraints.
- Decision and execution authorities.
- Resource and time budgets.
- Information available to each actor.
- Threat model and adversarial capabilities.
- Expected outcomes and acceptable failure conditions.

## 3. Metric families

| Family | Candidate metric | Measurement |
|---|---|---|
| Legitimacy and participation | Affected-party inclusion | Eligible affected parties represented / identified eligible affected parties; report exclusions |
| Participation quality | Effective participation rate | Participants who can understand, influence and contest a decision; survey plus process evidence |
| Authority integrity | Unauthorized-action rate | Actions outside documented authority / all consequential actions |
| Procedural integrity | Rule-conformance rate | Decisions meeting applicable procedure / decisions sampled |
| Institutional observability | Formal-to-operational rule gap (FORG) | Human-reviewed sample of divergences between declared rules, implemented policy and observed rules-in-use; classify each divergence and report inter-rater agreement |
| Evidence quality | Evidence completeness | Required provenance, uncertainty, conflict and rationale fields present |
| Accountability | Time to trace decision | Median time for independent reviewer to reconstruct authority, evidence and execution |
| Contestability | Appeal accessibility | Time, cost, language and procedural burden to file and resolve an appeal |
| Remedy | Corrective-action closure | Valid findings with independently verified remedy |
| Capture resistance | Attack success rate | Pre-registered adversarial scenarios resulting in unauthorized control or benefit |
| Collusion resistance | Detection and containment | Detection latency, harm before containment, false positives |
| Resilience | Service degradation and recovery | Availability of essential function, recovery time and residual harm |
| Interoperability | Cross-organization task completion | Successful completion across independent governance domains |
| Adaptability | Legitimate change lead time | Time from approved amendment to verified adoption |
| Succession | Continuity after turnover | Essential functions retained after founder/leader/provider removal |
| Economic inclusion | Benefit and burden distribution | Disaggregated outcome distribution, including non-members and externalities |
| Efficiency | Full cost per successful outcome | Staff time, participant burden, technology, compliance, dispute and externalized costs |
| Ecological impact | Resource and externality profile | Task-specific physical indicators, not generic sustainability claims |

All metrics require an explicit denominator, sampling method, missing-data treatment and disaggregation policy. FORG must preserve the difference between lawful adaptation and unauthorized divergence; the raw gap rate must never be used as a standalone institutional-quality score.

## 4. Required benchmark scenarios

### S1 — Routine collective decision
A group must decide on a shared resource allocation under a fixed budget and deadline.

Measure decision quality, participation, time, cost, procedural compliance, minority impact and post-decision satisfaction.

### S2 — Conflict of interest
An authorized decision-maker has a material personal interest in a proposed expenditure.

Test disclosure, recusal, quorum, substitute authority, audit trail and appeal.

### S3 — Malicious capture
An attacker attempts to gain disproportionate influence through identity farming, wealth concentration, collusion, credential compromise or agenda manipulation.

Test prevention, detection, containment, recovery and false-positive harms.

### S4 — Cross-organization coordination
Three or more independent organizations coordinate a shared service while retaining different internal rules.

Test mutual recognition, data minimization, liability allocation, completion rate and dispute handling.

### S5 — Leadership and provider failure
Remove a founder, key administrator, infrastructure provider or signing authority during an active process.

Test continuity, succession, key recovery, authority revalidation and constitutional compliance.

### S6 — Emergency response
A time-bounded emergency requires resource coordination under incomplete information.

Test response latency, authorization boundaries, safety, information quality, post-hoc review and sunset of emergency powers.

### S7 — Minority-rights conflict
A majority proposes a decision that materially harms a protected minority or violates a constitutional constraint.

Test detection, protected rights, appeal, remedy and prevention of indefinite paralysis.

### S8 — False or uncertain evidence
Introduce conflicting sources, manipulated reports, missing data and calibrated uncertainty.

Test whether the institution distinguishes unknown, disputed, verified and falsified claims.

### S9 — Institutional exit and dissolution
A member organization leaves a federation or an institution dissolves.

Test portability, settlement, data return/deletion, continuity of essential services, obligations and non-retaliation.

### S10 — Constitutional amendment
A proposal changes governance rules, including the amendment procedure itself.

Test notice, participation, quorum, versioning, rights constraints, independent review and effective date.

### S11 — Participation integrity and feedback

Participants submit proposals, evidence and objections before a consequential
decision. The decision authority must disclose in advance how inputs can
influence the decision and later publish a reasoned disposition.

Test whether input changes the decision, changes its options, is rejected with
reasons, or is ignored without explanation. Include under-represented
participants, accessibility barriers, agenda control and coordinated comment
flooding. Measure effective influence, input disposition, participant burden,
representation and decision-maker follow-through.

### S12 — Formal rules versus rules-in-use

Compare the authoritative constitutional or policy text with implemented
workflow rules and sampled observed practice. Include lawful local adaptations,
informal workarounds, selective enforcement and explicit bypasses.

Classify material gaps as aligned, authorized adaptation, locally supplemented,
contradictory, bypassed or unknown. Use independent human coding and report
inter-rater agreement. A gap is a diagnostic signal, not automatic proof of
misconduct.

### S13 — AI-mediated deliberation and steering

Compare distinct interventions rather than one generic AI-assisted condition:
- no AI / platform-native workflow;
- trained human facilitation;
- non-directive private pre-deliberation reflection;
- retrospective summarization or argument mapping;
- in-conversation AI facilitation;
- explicit recommendation, counterargument or agenda-setting assistance.

Use group/session-level randomization where spillovers require it. Measure argument justification, counterargument engagement, participant comprehension, private-to-public opinion changes, allocation effects, perceived inclusion, observed influence, participation equity, minority-position retention, trust calibration and decision-maker follow-through. Include controls for framing, omitted arguments, fabricated consensus signals, unequal access, AI overconfidence and strategic agenda manipulation.

A stance change is not automatically harm; agreement is not automatically quality; participant satisfaction is not proof of neutrality. Use human-adjudicated measures and appropriate ethics review for human-subject studies.

### S14 — Formal authority versus effective institutional power

Use institutions with matched nominal voting weights but vary agenda control, delegation, evidence issuance/verification, approval/disbursement, veto, emergency powers and appeal adjudication.

Measure nominal voting share separately from formal power indices (e.g. Banzhaf and Shapley–Shubik where the declared voting game permits exact calculation), minimal winning coalitions, blocking power, role concentration, separation-of-duties conflicts, appeal independence and removal/recall resilience. Add operational observations of wealth, lobbying, coercion, information access or social influence only when those are independently evidenced.

Test proposer/self-approval, issuer-as-only-verifier, agenda-controller-as-record-editor, disburser-as-sole-approver, non-independent appeal and correlated absenteeism. Report formal-game results separately from observed informal influence. A power index is not a democracy, legitimacy or justice score.

### S15 — Institutional feedback, lock-in and intergenerational burden

Run identical starting worlds with alternative institutional responses to scarcity, service outage, concentrated dependency, emergency centralization, care burden and ecological depletion.

Test whether an intervention that improves short-term service shifts costs into later dependency, reduced exit viability, maintenance debt, unpaid care, ecological loss or future-cohort obligations. Include a countervailing rights-floor/capacity-building intervention and cases where centralization or market-based responses legitimately perform better.

Use the existing Civilization Twin, power/dependency observables and physical accounting owners. Report short-, medium- and long-horizon trajectories, impulse response, recovery time, effective alternatives, switching cost, rights-floor breaches, cohort-incidence and unresolved model disagreement. Do not build a second simulator or universal flourishing score.

## 5. Baselines

For each task, compare at least:
1. A documented conventional organization workflow.
2. A cooperative or member-governed workflow where applicable.
3. A federated/polycentric workflow.
4. A software-mediated governance workflow where applicable.
5. The proposed LISE/Mycelix workflow.

Do not assume the proposed system is the winner. Record baseline maturity and implementation quality. Where a baseline is unavailable, mark the comparison incomplete.

## 6. Experimental controls

- Pre-register hypotheses, metrics, exclusions and analysis.
- Use identical task difficulty, information, resources and deadlines where possible.
- Randomize scenario variants and participant assignment when ethical and feasible.
- Separate training from evaluation cases.
- Include hidden adversarial cases and negative controls.
- Repeat across multiple runs and organizations.
- Report confidence intervals and effect sizes, not just point estimates.
- Report participant burden, accessibility and harms.
- Include independent evaluators and blinded scoring where feasible.
- Publish raw or privacy-preserving derived data, scripts and environment details.
- Treat simulations and field studies as different evidence classes.

## 7. Proposed initial target gates

These are engineering targets for pilot design, not current results or universal thresholds. Validate and revise them with domain experts and pilot participants.

- **Authority integrity:** zero unauthorized consequential actions in the defined test suite.
- **Auditability:** 100% of benchmark consequential actions reconstructable from recorded authority, evidence, execution and review records.
- **Negative controls:** 100% of seeded prohibited actions blocked or escalated in the tested scenarios.
- **Recovery:** every critical role has a tested succession or recovery path; no single individual is the sole holder of irreplaceable institutional knowledge or authority.
- **Interoperability:** complete at least 3 cross-organization workflows with different internal governance rules and no forced merger of member organizations.
- **Appeal:** every benchmark decision has a documented, accessible contestation path and measured resolution time.
- **Data minimization:** no unnecessary personal data is required for the benchmark's defined purpose.
- **Resilience:** complete at least 5 fault/capture scenarios with measured service degradation, recovery time and residual harm.
- **Efficiency:** demonstrate a statistically and practically meaningful improvement over at least one matched baseline without worsening protected-rights, participation or harm metrics.

Targets are gates for specified scenarios, not guarantees about all real-world institutions.

## 8. Scoring and reporting

Report a profile, not a leaderboard number:

- Capability score by metric family.
- Rights and safety violations (reported separately; not averaged away).
- Cost and participant burden.
- Distribution of benefits and harms.
- Confidence intervals and missing data.
- Scenario-specific failure analysis.
- Maturity and evidence class.
- Independent replication status.

A system that is fast but violates rights cannot compensate by scoring highly on efficiency.

## 9. Reproducibility package

Every benchmark release should include:
- Versioned scenario definitions.
- Machine-readable case schema.
- Baseline workflow descriptions.
- Test fixtures and adversarial scripts.
- Measurement and analysis code.
- Environment and dependency lockfiles.
- Run manifest and immutable input digests.
- Raw outputs or privacy-preserving substitutes.
- Independent evaluator notes.
- Known limitations and change log.

## 10. First implementation slice

Start with S2 (conflict of interest) and S3 (malicious capture). They are narrow enough for deterministic fixtures and expose whether authority separation and contestability are real or merely descriptive.

A minimal fixture should model:
- Members and scoped credentials.
- A proposal and its supporting evidence.
- Conflict-of-interest disclosure.
- Authorization policy and quorum.
- Execution receipt.
- Independent review.
- Appeal and remedy.
- Adversarial attempts to bypass each step.

The benchmark must prove both that valid actions succeed and invalid actions fail. A test suite containing only happy paths is insufficient.

## 11. Evidence labels

Use explicit labels:
- Proposed
- Implemented
- Structurally checked
- Tested (include exact revision and command)
- Field-evaluated
- Independently replicated

A CI job queued, skipped or unavailable is not a pass. Never infer a pass from the existence of a test or a green result on a different revision.


## 12. First executable S2 fixture (research branch)

A Rust crate now exists at
mycelix-governance/crates/institutional-assurance and is included in the
existing mycelix-governance Cargo workspace. It implements twenty deterministic
S2 cases: six positive and fourteen negative controls.

- positive controls: independent approval, exact amount-limit acceptance, inclusive grant-start acceptance, just-before-expiry acceptance, a valid substitute after proposer recusal, and resource-specific scoped approval;
- negative controls for beneficiary self-approval and proposer self-approval;
- a seeded conflict-of-interest denial;
- fail-closed behavior when conflict screening is incomplete;
- missing, expired, future-dated and malformed authority windows;
- wrong authority holder, scope and resource;
- proposal substitution, zero-value requests and an amount above the grant limit.

Each case emits a machine-readable receipt containing the policy version,
scenario and actor identifiers, decision and reason, plus SHA-256 digests of
the policy, input and receipt material. Fixture time is fixed; the runner uses
no RNG, wall clock, network or external service.

**Evidence status:** implementation and test source are present on the research
branch. The first exact-head governance workspace compile reached this crate
and found E0505 borrow/move errors in receipt construction; the source was
reordered to compute the receipt digest before moving the digest strings. The
twenty-case revision still requires a successful exact-head format, workspace
test and runner result. Do not label this slice Tested or production-integrated
until those commands pass and the fixture is mapped to the real Holochain zome
boundary.

Run from mycelix-governance:

    cargo test -p institutional-assurance
    cargo run -p institutional-assurance --bin ipb-run

This fixture tests a declared policy, not the real-world discovery of hidden
conflicts. It is not an authorization service and does not establish legal
compliance, institutional legitimacy or field effectiveness.

## 13. Institutional Coordination Cost (ICC)

For each successful outcome, record the full coordination cost:

    ICC_per_outcome =
        (C_administration
         + C_negotiation
         + C_verification
         + C_dispute_resolution
         + C_participant_burden
         + C_technology_and_compliance
         + C_externalized_costs)
        / successful_outcomes

Report every component separately as well as the normalized total. Include
staff time, participant time, out-of-pocket cost, delay and costs transferred
to non-members or other organizations. Do not count a task as successful only
because a transaction completed: define minimum outcome quality and rights
conditions first.

Compare ICC only for matched tasks, jurisdictions, stakeholder groups,
resource budgets and outcome quality. A lower ICC obtained by excluding
affected parties or making appeals inaccessible is a failed result.

## 14. Statistical and causal discipline

- Pre-register the primary outcome, minimum practically important effect,
  non-inferiority margins, exclusions and analysis before evaluation.
- Use a development set for tuning and a held-out scenario set for evaluation.
- Where assignment can be randomized ethically, randomize scenario variants
  or workflow assignment. Otherwise document confounders and use a matched,
  stepped-wedge or other defensible quasi-experimental design.
- Report effect sizes, uncertainty intervals, missingness, attrition and
  disaggregated outcomes. Do not use a p-value alone as evidence of impact.
- Use repeated seeds for stochastic simulations and separate model calibration
  from held-out validation.
- For a zero-failure result, state the trial count and sampling assumptions.
  The approximate one-sided 95% upper bound under independent representative
  trials is 3/n (the rule of three). Zero failures in a small or correlated
  test suite do not establish a low real-world failure rate.
- Publish both false negatives and false positives. A policy that blocks all
  actions can achieve a perfect negative-control score while making the
  institution unusable.

## 15. Expanded capability targets

Targets below are proposed engineering gates for bounded experiments, not
universal guarantees.

| Capability | Proposed target | Required counter-metric |
|---|---|---|
| Constitutional/procedural integrity | Zero invariant violations in the locked test suite | Legitimate-action acceptance and false-positive rate |
| Evidence reconstruction | 100% of sampled benchmark decisions replayable from recorded inputs and policy version | Privacy leakage and reviewer effort |
| Capture resistance | Zero successful seeded attacks in the defined red-team suite | False alarms, attacker cost and residual harm |
| Succession | 100% of defined critical-role removal tests retain essential functions | Recovery time, privilege escalation and knowledge loss |
| Participation | Every input receives a traceable disposition and reason | Representativeness, participant burden and actual influence |
| Interoperability | At least three independent organizations complete one shared workflow | Data disclosed, negotiation time, liability disputes and exit cost |
| Economic coordination | Candidate pilot target: at least 10% lower ICC per quality-qualified outcome versus a matched baseline | Rights, access, quality, distribution and non-member externalities |
| Institutional digital twins | Held-out predictive calibration reported against a simple baseline | Model drift and subgroup prediction error |

For the 10% ICC target, the pilot must pre-register a minimum practically
important effect and confidence interval. The 10% figure is an initial
hypothesis, not an established sector benchmark. If baseline variability or
stakeholder priorities make it inappropriate, revise it before the trial, not
after seeing the result.

## 16. Digital-twin validity gates

An institutional digital twin must not be judged by visual sophistication or
agent count. It should pass staged validity checks:

1. **Conceptual validity:** variables and rules correspond to observed
   institutional mechanisms, including formal rules and rules-in-use.
2. **Implementation verification:** code does what the model specification
   says; deterministic replay reproduces each run.
3. **Calibration:** model parameters are estimated only from declared
   calibration observations.
4. **Held-out validation:** predictions are tested on organizations, time
   periods or scenario variants not used for calibration.
5. **Sensitivity and uncertainty:** report which assumptions drive outcomes
   and where predictions are unstable.
6. **Field comparison:** simulated trajectories are compared with observed
   organizational processes and outcomes; discrepancies update the model.
7. **Decision boundary:** simulations advise scenario exploration only; they
   cannot confer legal authority or establish consent.

A twin is useful when it improves prediction or counterfactual decision support
against a simpler baseline, with calibrated uncertainty—not because it
produces a large number of simulated agents.
