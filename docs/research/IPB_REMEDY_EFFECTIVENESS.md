# IPB-REM-001 — Effective remedy beyond appeal visibility

**Status:** Draft v0.1 companion evaluation profile  
**Tracking issue:** Mycelix #6145  
**Parent evaluator:** Mycelix #6069  
**Manifest:** `IPB_REMEDY_EFFECTIVENESS.yaml`  
**Claim ceiling:** Synthetic conformance only; no legal compliance, real-world remedy adequacy or field-effectiveness claim.

## 1. Why this companion exists

Mycelix #6069 already owns the bitemporal institutional reconstruction and contestability benchmark: what was known at decision time, what changed later, how challenges attach to specific claims/evidence/authority, and whether historical and current views remain distinct.

IPB-REM does not reimplement that system. It evaluates what happens **after a person tries to contest a decision**:
- can they actually reach and use the mechanism;
- can they inspect and correct the evidence;
- can a reviewer independently change the outcome;
- can the institution order and complete a remedy;
- does the institution address recurrent causes;
- are external and collective routes preserved?

The critical non-equivalences are:

`appeal_available != appeal_accessible`

`human_review_promised != independent_review`

`independent_review != reviewer_has_authority`

`remedy_ordered != remedy_completed`

`case_closed != harm_repaired`

## 2. Paradigm shift: from complaint handling to a closed remedial control loop

Treat remedy as a lifecycle with observable transitions, not a complaint-management checkbox:

`harm/decision → notice → evidence access → challenge → independent review → authorized outcome → remedy order → execution → independent observation → recurrence prevention`

The lifecycle reuses the native records and semantics from #6069, #2953, #3117, #3126 and #4835. IPB-REM only creates a scenario, a read-only evaluation projection and metrics.

The loop has two distinct closure conditions:
1. **Individual repair:** the affected person receives the case-specific remedy that was ordered, with completion evidence.
2. **Systemic learning:** a recurrent cause is corrected and the correction is independently evaluated for effectiveness.

A policy update, apology, review meeting or closed ticket is not automatically either form of closure.

## 3. Seven contestability guarantees, plus remedy execution

The 2026 Chen study formalizes seven guarantees for an individual consequential decision: notice, reasons, access to the data used, correction, appeal, human review and remedy authority. IPB-REM maps these into testable case conditions and adds execution/observation of the resulting remedy.

The seven guarantees are not treated as a single scalar. Each is separately reported as supported, contradicted, contested, indeterminate, unavailable or out of scope. An eighth check asks whether an ordered remedy was actually completed and independently observed.

### Human review is not enough

A reviewer who can read the file and leave comments but cannot suspend, vary, reverse or remand the decision is not an effective decision-changing review for this benchmark.

Similarly, the reviewer's authority is not proven by their title. The evaluator needs the exact current mandate, scope, conflict status and applicable policy version. When any of these are unavailable, the result is indeterminate or unsupported—not PASS.

## 4. Remedy completion and appropriateness

The OECD Responsible Business Outlook 2026 lists several possible remedy forms: apology, restitution, rehabilitation, financial or non-financial compensation, sanctions and guarantees of non-repetition. The benchmark must not assume that any one form is universally correct.

For each synthetic case, freeze the remedy objective and required evidence before evaluation. Report:
- what was requested;
- what the reviewer ordered and under which authority;
- what was actually executed;
- what an independent observer verified;
- what remains contested or incomplete;
- who had authority to accept, reject, vary or close the remedy.

**Substantive adequacy is not a model-scored scalar.** It requires an independent domain assessment and, in later human pilots, input from affected people/rightsholders. The software fixture can check declared procedural and evidence obligations, not decide whether a person has been made whole in every context.

## 5. Twenty-two deterministic scenarios

The YAML manifest is authoritative. Its 22 cases cover:
- notice, reasons and exact evidence access;
- data correction without historical rewrite;
- counter-evidence and attributable disposition;
- accessibility and assistance;
- published stages and deadline transparency;
- human review without decision-changing authority;
- reviewer conflict and reassignment;
- independent review with variation/reversal;
- interim safeguards;
- reprisal/retaliation protection;
- reasoned rejection with external route preserved;
- remedy ordered but not executed;
- independently observed restitution/restoration;
- non-financial remediation;
- systemic non-repetition;
- collective challenge and privacy;
- coercive waiver of external routes;
- indeterminate reviewer mandate;
- self-attested remedy completion;
- privacy over-disclosure;
- delay without notice.

Positive, negative and indeterminate conditions are intentionally mixed. The correct result is not always to uphold the challenge: a procedurally independent and reasoned review may uphold a decision, while preserving the complainant's further routes.

## 6. Metrics: keep access, authority and remedy separate

Report each metric with its denominator, case mix, exclusions and uncertainty.

### Access and process
- awareness/access under the declared participant profile;
- completion of each contestability guarantee;
- evidence-access and correction latency;
- challenge abandonment and burden;
- published-stage deadline misses;
- status-update coverage;
- accessible language/modality/assistance coverage.

### Review integrity
- reviewer conflict rate;
- reviewer mandate/scope completeness;
- share of reviews with actual decision-changing authority;
- uphold/vary/reverse/remand outcomes;
- independent evaluator agreement;
- unresolved evidence and dissent retention.

### Remedy execution
- remedies ordered;
- remedies started;
- remedies completed;
- completed remedies independently observed;
- time-to-remedy and partial completion;
- interim safeguard activation and duration;
- repeat harm after claimed correction;
- corrective-action effectiveness for systemic causes.

### Rights and privacy
- retaliation allegations and protected-route disposition;
- external/judicial/collective route preservation;
- unauthorized disclosure count;
- group-level remedy coverage;
- subgroup accessibility/burden differences in consented research.

No averaging across these dimensions. A good completion rate cannot compensate for a conflicted reviewer, a coerced waiver, a rights violation or an unexecuted remedy.

## 7. Industry and standards comparison

### UN Guiding Principles on Business and Human Rights — Principle 31

The UN Guiding Principles set effectiveness criteria for non-judicial grievance mechanisms: legitimacy, accessibility, predictability, equity, transparency, rights-compatibility and continuous learning. Operational-level mechanisms additionally require engagement and dialogue. The commentary explicitly identifies language, literacy, cost, physical access and fear of reprisal as barriers. It cautions that poorly designed grievance mechanisms can deepen disempowerment and disrespect.

IPB-REM translates these principles into adversarial, observable test cases. It does not certify compliance.

### ISO 10002:2018

ISO 10002:2018 remains the current published edition, confirmed in 2023. It provides a complaints-handling process baseline from planning and design through operation, maintenance, analysis, audit and effectiveness/efficiency review. Its stated scope covers complaints about products/services within an organization and excludes external dispute resolution and employment-related disputes. IPB-REM complements that process standard with authority to change the decision, rights to external remedies and evidence of actual remedy completion.

### OECD Responsible Business Outlook 2026

The OECD distinguishes a grievance mechanism from a commitment to remediate. Its report states that 17% of companies in the cited analysis reported a formal human-rights grievance mechanism and 10% committed to providing remediation. It lists multiple remedy forms and emphasizes low-barrier access. This supports a key benchmark distinction: channel availability and actual reparation are separate outcomes.

### AI contestability research (2026)

Chen (2026-11-06), *Can people challenge an AI decision? Measuring the contestability chain in law and corporate disclosure*, AI and Ethics, DOI 10.1007/s43681-026-01471-y, proposes the seven-guarantee individual-decision chain. The study reports that human-review promises often fail to specify whether the reviewer can change the outcome and that access to/correction of input data and remedy authority are particularly weak in the disclosure sample. Those reported results are study-specific; IPB-REM should reproduce neither the paper's estimates nor its instrument as ground truth. It should use the conceptual chain to define a matched benchmark task.

## 8. Matched baselines

Evaluate competent implementations of:
1. Manual ombuds/grievance process with a documented independent reviewer and actual remedy authority.
2. Conventional complaint-management workflow configured against ISO 10002-style process controls.
3. A non-judicial mechanism aligned to UNGP Principle 31.
4. AI-assisted decisions with each of the seven contestability guarantees separately enabled/disabled.
5. Mycelix owner-native decision/evidence/appeal/observation lifecycle with IPB-REM read-only evaluation.

Hold constant the synthetic decision, evidence, participant profile, accessibility needs, reviewer budget, time constraints, remedy authority and evaluator. Mark unsupported/out-of-scope features explicitly; do not penalize a product for a feature outside its intended purpose. Do not create a universal vendor or institutional ranking.

## 9. Owner-native composition

- #6069: bitemporal current/as-known-at reconstruction, challenge, appeal, correction and supersession.
- #2953/#3117: decision, current authority, attempt, execution and outcome.
- #3126: contested/contradictory evidence and indeterminate states.
- #4835: independent evaluator/oracle separation.
- #5999: deliberation, participant voice and objection preservation.
- #5627/#5625: accessible reach and lifecycle cost.
- IPB-REM: scenario manifest, matched baseline, outcome vector and claim discipline.

No grievance service, appeal engine, authority registry, evidence store, complaint schema or remedy engine is added.

## 10. Qualification sequence

1. Review the 22-case overlay against #6069's existing corpus and de-duplicate scenarios.
2. Freeze case IDs, expected dispositions, remedy objectives, deadlines and observation requirements.
3. Build a read-only evaluator using native owner references.
4. Add deterministic synthetic fixtures and an independently authored oracle replay.
5. Keep software conformance and human accessibility/experience studies separate.
6. Any human pilot requires ethics review, informed consent, privacy minimization, independent adjudication and a non-retaliation/non-disruption plan.

The initial benchmark proves only the declared synthetic remedy-effectiveness contract. It does not prove legal compliance, actual accessibility, human trust, substantive adequacy of an individual remedy or field effectiveness.
