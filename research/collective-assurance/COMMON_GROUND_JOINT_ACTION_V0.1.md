# Common Ground Joint-Action and Repair Qualification v0.1

**Status:** observable trace contract and reference validator. No Common Ground benchmark reproduction, human-study result, general model comparison or SOTA win is claimed.

## Research motivation

Poelitz, Doshi-Velez and Lindley's 2026 *A Benchmark to Assess Common Ground in Human-AI Collaboration* ([arXiv:2602.21337](https://arxiv.org/abs/2602.21337)) studies collaborative puzzle solving with a Helper and Worker. The Helper has the target arrangement; the Worker has a set of candidate pieces. The task requires iterative dialogue to select and arrange pieces. The study varies role and whether the Helper can see the Worker's task state, with repeated trials to examine reference conventions and repair.

The confirmatory human study reports 40 participants and a fixed GPT-4.1 condition. This is a useful benchmark adapter and mechanism study, not broad evidence across model families or populations.

The key lesson is that fluent acknowledgements, repeated phrases or "I understand" statements do not establish common ground. The more defensible evidence is successful joint action, correct reference resolution, observed task-state visibility, and effective repair when a misunderstanding occurs.

## Paradigm shift: from conversational similarity to coordinated action

Move from:

`the participants said the same thing → shared understanding`

to:

`state visibility → delivered communication → reference clarification → cross-role confirmation → joint action → independent environment outcome`

The benchmark should not infer private mental states. It records observable, role-bound information exposure and action consequences.

## Reference implementation

`src/common_ground.rs` implements `qualify_common_ground_trial`.

### Input contract

The trial freezes:
- benchmark version and task digest;
- independent task-oracle specification, expected final-state digest and oracle attestation reference;
- Helper and Worker identities;
- Worker task-state digest/version;
- view condition: `SharedWorkerState` or `PrivateWorkerState`;
- exact required object-to-slot placements;
- time and delivered-turn budgets.

The event log records:
1. **TaskStateVisible:** who actually saw the Worker's state, which version, and an exposure receipt.
2. **MessageDelivered:** sender, recipient, payload digest and delivery receipt. Message creation is not delivery.
3. **ClarificationRequested:** requester and the actually delivered message containing an ambiguous referent.
4. **ClarificationResolved:** the message recipient resolves the referent to a concrete object and provides a rationale digest.
5. **ReferentConfirmed:** the other participant confirms the resolution.
6. **JointActionExecuted:** both roles participate in the action, which binds an object, slot, optional clarification request, environment acceptance and effect receipt.
7. **RepairStarted / RepairCompleted:** repair trigger and measured duration.
8. **VerbalAcknowledgement:** counted as a communication event, never treated as proof of grounding.
9. **TrialCompleted:** final environment-state digest and completion receipt.

### Qualification vs task outcome

The validator deliberately separates **trace qualification** from **joint-task success**.

A well-formed trace with incorrect placements or a wrong final state is a valid unsuccessful observation: `qualification=PASS`, `outcome=Incorrect`. Missing task-state exposure, missing completion evidence or unfinished repair remains `INDETERMINATE`; malformed event sequences, unauthorized role mappings, undelivered clarifications or missing cross-role confirmation fail qualification.

Joint success requires both:
- exact completion of the oracle's required object-to-slot placements, with no incorrect/rejected, duplicate-slot or duplicate-object actions; and
- the observed final environment-state digest to match the frozen oracle expectation.

A verbal acknowledgement cannot substitute for either condition.

### View-condition checks

For `SharedWorkerState`, both Worker and Helper must have an exposure receipt for the same frozen Worker state digest/version. For `PrivateWorkerState`, only the Worker may have that state exposure. The condition is checked from recorded view events, not a self-report that a participant "could see" the state.

### Repair checks

Repair starts must refer to an earlier event and have one completion receipt. The reported duration must equal the difference between observed event timestamps. Incomplete repairs remain indeterminate.

## Outcomes

The report keeps separate:
- joint success and outcome status;
- exact/required placements;
- incorrect/rejected actions;
- duplicate slot/object actions;
- clarification requests, resolutions and cross-role confirmations;
- verbal acknowledgements;
- repair count and repair durations;
- message count and task-state exposure count;
- shared-state exposure to the Helper;
- elapsed time.

This avoids a single "common-ground score" that confounds task success, communication burden, repair, view condition and acknowledgement frequency.

## Official reproduction and extension

First reproduce the published role × view conditions with:
- the official task specification and scorer;
- pinned model and prompt;
- same time/turn budgets;
- exact task-oracle and environment-state receipts;
- four repeated trials per task where the official protocol requires them.

Then extend to multiple task families, models and modalities under matched task/scorer/resource conditions. Any new role × view × information-order factorial is a separate powered and preregistered experiment; do not modify #6616's existing four arms.

### Proposed targets (not results)

- 100% of joint-success claims bind to an independently attested final environment state and exact required placements.
- Zero cases where a verbal acknowledgement alone produces joint success.
- 100% of ambiguity-dependent actions require delivered-message, resolution and cross-role confirmation evidence.
- Report repair latency and communication burden separately; do not reward unnecessary clarification.
- At least 90% held-out prediction accuracy for whether a pair completes a task under a preregistered task/model/view condition, after a sufficiently powered and independent validation set is available.
- Report subgroup and role-asymmetry results; no claim of universal common ground from the original 40-person, fixed-model study.

## Integration

- Mycelix #6676 owns dynamic human–AI complementarity and the six-stage capability trajectory. This module is its observable joint-action/common-ground adapter.
- Mycelix #6616 owns information-order and anchoring randomization.
- Mycelix #6089 owns distributed information acquisition and evidence rights.
- Mycelix #6688 owns relational evidence refinement and RER→ACB handling.
- Symthaea SAT #13574 owns wider system-level sovereignty, adaptive control and pluralistic-agency evaluation.

This module does not implement a new communication runtime, deliberation engine, authority kernel, memory system or human-subject recruitment pathway.

## Claim ceiling

A `Success` result qualifies one observed task trial under its declared oracle, roles, view condition and budgets. It does not prove private mental-state alignment, general shared understanding outside the tested task, broad model superiority, or a positive long-term human capability effect. The validator checks receipt references and trace structure; it does not independently authenticate the source systems that created those receipts.
