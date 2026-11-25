# Common Ground Joint-Action and Repair Qualification v0.1

**Status:** observable trace contract and reference validator. No benchmark reproduction, human-study result or SOTA win is claimed.

## Research motivation

Poelitz, Doshi-Velez and Lindley's 2026 *A Benchmark to Assess Common Ground in Human-AI Collaboration* ([arXiv:2602.21337](https://arxiv.org/abs/2602.21337)) uses collaborative puzzle tasks with Helper and Worker roles. The Helper has the target arrangement; the Worker has candidate pieces. The pair must coordinate through iterative dialogue. The study varies role and whether the Helper can see the Worker's task state, and examines repair and reference conventions across repeated trials.

The confirmatory human study reports 40 participants and a fixed GPT-4.1 condition. It is a useful protocol to reproduce, not broad evidence across model families or populations.

**Key principle:** fluent acknowledgements, repeated words and "I understand" statements do not establish common ground. Stronger evidence is correct joint action, observed task-state exposure, reference resolution, cross-role confirmation and repair when something goes wrong.

## Paradigm shift

Move from:

`the participants said the same thing → shared understanding`

to:

`state visibility → delivered communication → reference clarification → cross-role confirmation → joint action → independently attested environment outcome`

The benchmark must not infer private mental states.

## Reference implementation

`src/common_ground.rs` exposes `qualify_common_ground_trial`. It validates a frozen benchmark/task/oracle identity, Helper and Worker roles, Worker state version/digest, shared/private view condition, required object-to-slot placements, time and turn budgets, and an event log.

The event log distinguishes:
1. actual task-state visibility, with an exposure receipt;
2. message delivery, separate from message creation;
3. clarification requests bound to a delivered message;
4. resolution by the message recipient, with a concrete object and rationale digest;
5. confirmation by the other participant;
6. joint actions with both roles, exact objects/slots, environment acceptance and effect receipts;
7. repair start and completion with measured duration;
8. verbal acknowledgements, counted but never accepted as proof;
9. trial completion with a final environment-state digest and receipt.

### Qualification vs outcome

A structurally valid trace can be an unsuccessful trial. The report distinguishes `qualification=PASS` from `outcome=Incorrect` or `Incomplete`. Missing task-state exposure, incomplete repair or missing completion evidence is `INDETERMINATE`; malformed sequences, invalid delivery/resolution bindings or missing cross-role confirmation fail qualification.

Joint success requires both the exact oracle placements (without incorrect/rejected, duplicate-slot or duplicate-object actions) and a final environment digest matching the frozen expected state. A verbal acknowledgement cannot substitute for either.

### View and repair

For `SharedWorkerState`, both Helper and Worker must have an exposure receipt for the same frozen Worker state digest/version. For `PrivateWorkerState`, only the Worker may have that view. Repair starts must reference an earlier event and have one completion receipt whose duration matches the observed timestamps.

## Metrics

The report keeps separate joint success, exact/required placements, incorrect actions, duplicate slots/objects, clarification requests/resolutions/confirmations, acknowledgements, repair count/duration, message count, task-state exposure count, shared-state exposure to the Helper and elapsed time.

No single opaque "common-ground score" is produced.

## Reproduction and extensions

First reproduce the published role × view conditions under pinned task, scorer, model, prompt and budgets. Then extend to additional tasks/models with matched conditions. Any role × view × information-order factorial is a separate powered and preregistered design; do not change #6616's existing four arms.

Prospective targets, not achieved:
- 100% of joint-success claims bind to exact required placements and independent final-state evidence;
- zero cases where acknowledgement alone yields success;
- 100% of ambiguity-dependent actions require delivered-message, resolution and cross-role-confirmation evidence;
- report repair latency and communication burden separately;
- validate held-out outcome prediction only after an adequately powered dataset exists.

## Integration

- Mycelix #6676 owns dynamic human–AI complementarity and the six-stage capability trajectory. This module supplies its observable joint-action/common-ground adapter.
- #6616 owns information-order/anchoring experiments.
- #6089 owns distributed information acquisition and evidence rights.
- #6688 owns relational evidence refinement and RER→ACB.
- Symthaea SAT #13574 owns system-level sovereignty and adaptive-control evaluation.

No new messaging runtime, deliberation engine, authority kernel, memory store or recruitment process is created.

## Claim ceiling

A `Success` result qualifies one observed task trial under its declared oracle, roles, view condition and budgets. It does not prove private mental-state alignment or general shared understanding. The validator checks receipt references and trace structure; it does not independently authenticate the systems that produced those receipts.
