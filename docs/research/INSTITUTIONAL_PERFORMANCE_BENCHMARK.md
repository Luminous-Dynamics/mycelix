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

All metrics require an explicit denominator, sampling method, missing-data treatment and disaggregation policy.

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
