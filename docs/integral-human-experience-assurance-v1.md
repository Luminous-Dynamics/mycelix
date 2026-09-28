# Integral Human Experience Assurance v1

**Related:** #3353  
**Status:** Design specification; empirical validation remains open.

## Purpose

Integral should be understandable and usable by ordinary participants without requiring them to internalize the five-system architecture. Human comprehension, agency, predictability, reversibility, accessibility, and trust calibration are system requirements—not documentation polish.

This specification deliberately does **not** assert that Integral will make people happy or flourish. Those are empirical outcomes to be measured with real participants over time.

## Core principle

> Complexity belongs in the machinery; understandable choices belong at the human boundary.

Participants should be able to act, understand consequences, inspect reasons, disagree, appeal, recover from mistakes, and leave without depending on an AI system as an authority.

## Human Experience Assurance obligations

| ID | Requirement | Evidence target |
|---|---|---|
| HXA-001 | A participant can complete common tasks without learning all five system names. | Task-based comprehension studies |
| HXA-002 | Before a consequential action, the participant can identify expected consequences, commitments, and reversibility. | Prediction tests + action previews |
| HXA-003 | Explanations distinguish source evidence, system interpretation, recommendation, and final human/community decision. | Provenance-linked explanation tests |
| HXA-004 | Uncertainty, disagreement, missing evidence, and contested claims remain visible. | Uncertainty comprehension tests |
| HXA-005 | Consequential automated recommendations cannot silently become authoritative decisions. | Deterministic authority-boundary tests |
| HXA-006 | Consequential actions have an explicit confirmation/authorization boundary. | Interaction and audit tests |
| HXA-007 | Reversible actions expose recovery paths and preserve decision lineage. | Recovery drills + lineage checks |
| HXA-008 | Appeals can reach the evidence and decision context that produced an outcome. | Appeal-path tests |
| HXA-009 | Interfaces resist dark patterns, coercive defaults, automation bias, and authority laundering. | Adversarial UX testing |
| HXA-010 | Cognitive load is measured for novice, experienced, and role-specific participants. | Time/error/load studies |
| HXA-011 | Accessibility and localization do not remove substantive rights, uncertainty, or provenance. | Accessibility/localization conformance |
| HXA-012 | Participants retain meaningful choice among legitimate alternatives where governance permits pluralism. | Choice-set and agency studies |
| HXA-013 | Trust is calibrated to evidence quality rather than interface confidence or AI fluency. | Calibration experiments |
| HXA-014 | Privacy and data minimization apply to comprehension assistance as well as governance records. | Data-flow and privacy review |
| HXA-015 | Longitudinal studies measure lived outcomes without turning one aggregate score into a legitimacy oracle. | Longitudinal participant research |

## Progressive disclosure

The default participant experience should expose only the information needed for the current decision, with deeper layers available on demand:

1. **What happened?**
2. **What does it mean for me?**
3. **Why?**
4. **What evidence supports that explanation?**
5. **Who/what had authority to decide?**
6. **What can I change, challenge, reverse, or appeal?**
7. **Technical/provenance details.**

No layer may fabricate certainty that is absent from the underlying evidence.

## Symthaea boundary

Symthaea may provide cognitive assistance such as:

- plain-language explanations;
- role-appropriate summaries;
- accessibility transformations;
- interactive “what happens if…” simulations;
- clarification questions;
- comparison of documented options;
- uncertainty and provenance navigation;
- personalized learning paths.

Symthaea must **not** be the source of legitimacy. In particular:

- model output is not evidence merely because a model produced it;
- recommendation is not decision;
- generated explanation cannot mutate authoritative records;
- consequential actions require the appropriate explicit human/community authorization;
- explanations should link back to authoritative provenance where available;
- uncertainty should be preserved rather than hidden by fluent language.

## Mycelix boundary

Mycelix should provide the machine-verifiable substrate for:

- identity and consent;
- provenance and evidence lineage;
- authority and authorization boundaries;
- decision/outcome lineage;
- appeal and reversal references;
- privacy and data-access constraints;
- auditable distinctions between observation, assessment, recommendation, authorization, and execution.

This composes naturally with the existing COS/OAD/ITC/FRS/CDS boundary work.

## Evidence model

Human Experience Assurance uses an evidence ladder rather than a single “human flourishing score”:

- **L0 — hypothesis:** design claim not yet tested.
- **L1 — usability evidence:** controlled task/comprehension result.
- **L2 — behavioral evidence:** repeated participant interaction evidence.
- **L3 — longitudinal evidence:** sustained real-world observations.
- **L4 — cross-context evidence:** replication across communities, cultures, roles, and accessibility needs.

Evidence must retain population, context, instrument, uncertainty, and limitations.

Aggregate indicators may summarize evidence, but no scalar score can by itself establish that a community is flourishing, that participants are happy, or that an institutional decision is legitimate.

## Minimum participant test suite

Before consequential deployment, test at least:

1. Explain a decision in plain language.
2. Predict what will happen before committing an action.
3. Identify whether a statement is observation, interpretation, recommendation, or decision.
4. Find the evidence supporting a consequential claim.
5. Find who/what had authority.
6. Identify uncertainty and disagreement.
7. Reverse or appeal a reversible consequential action.
8. Complete the task without accepting an AI recommendation.
9. Complete the task after the AI recommendation is intentionally wrong.
10. Complete the task under accessibility/localization variants.
11. Explain why an outcome can be challenged.
12. Demonstrate that leaving the interface does not forfeit rights or legitimate participation.

## Integration rule

Human Experience Assurance is a cross-system qualification layer, not a sixth authority system.

- CDS remains the deliberative/community-choice boundary.
- OAD remains the design/admission boundary.
- ITC remains the accounting/coordination boundary.
- COS remains the production/observation/conformance boundary.
- FRS remains the recommendation/evidence boundary.
- Symthaea remains assistive cognition.
- Mycelix remains the verifiable coordination/provenance substrate.

The human remains the locus of legitimate agency wherever the governing process requires human/community choice.

## Non-claims

This specification does not claim:

- that Integral will produce happiness;
- that Integral will improve well-being;
- that Symthaea will increase human flourishing;
- that usability tests prove societal legitimacy;
- that formal verification proves lived outcomes.

Those claims require empirical research with actual participants and communities.


## Executable assurance boundary

The machine-checkable subset of HXA is implemented in
`mycelix-manufacturing/crates/cos_conformance/src/integral_hxa_formal.rs`
as bounded reference-model witnesses for HXA-I01 through HXA-I12.

These witnesses deliberately establish **boundary semantics**, not human outcomes:

| HXA | Machine-checkable boundary | Evidence still required outside the reference model |
|---|---|---|
| I01 | recommendation cannot itself become authority | participants can recognize that distinction |
| I02 | generated explanation does not become evidence | participants can identify evidence vs explanation |
| I03 | explanation is non-authoritative | participants can challenge an explanation |
| I04 | uncertainty survives transformation | users correctly interpret uncertainty |
| I05 | consequential action requires explicit authorization | users understand consequences before authorizing |
| I06 | human override path remains available | users actually override appropriately when warranted |
| I07 | appeal does not require the recommending AI | participants can successfully contest outcomes |
| I08 | transformations preserve provenance | accessibility/localization remains understandable |
| I09 | disagreement is preserved | participants can see and use disagreement |
| I10 | AI identity is disclosed | participants notice and understand system identity |
| I11 | system can degrade without Symthaea authorization | humans can complete essential tasks without AI assistance |
| I12 | generated artifacts cannot launder legitimacy | participants do not mistake AI output for governance authority |

The formal witnesses use no aggregate happiness, satisfaction, trust, or
"flourishing" score as a closure criterion. Human evidence therefore remains
a separate assurance layer.

### Trust-calibration implication

The participant protocol should measure behavior rather than asking only
whether participants "trust" the assistant. Useful measures include:
recommendation acceptance/rejection, override accuracy, error detection,
evidence-location accuracy, uncertainty/authority identification,
time-to-correct, AI-independent task completion, and the ability to explain
why an AI recommendation can legitimately be rejected.

This is consistent with recent human-AI research finding that explanation
length can increase user confidence without increasing answer accuracy, while
uncertainty information and explicit transparency can improve calibration in
some settings. citeturn0search1turn0search5 Recent work also proposes
override rate and override accuracy as practical indicators of calibrated
reliance. citeturn0search2

### Current claim ceiling

Passing the executable HXA witnesses means only that the bounded reference
model satisfies its encoded boundary conditions. It does **not** establish:

- that Integral is easy to learn;
- that participants will understand every decision;
- that Symthaea explanations are correct;
- that users will trust AI appropriately;
- that users will be happy;
- that Integral causes flourishing;
- that the production implementation refines the reference model; or
- that any population has achieved a desired quality of life.

Those claims require staged participant evidence and, for lived outcomes,
longitudinal and cross-context evaluation.
