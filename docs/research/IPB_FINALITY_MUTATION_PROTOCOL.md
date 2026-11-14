# IPB-FINALITY mutation and metamorphic test protocol

**Status:** research plan for the next benchmark increment. This document does not claim these generated tests have been implemented or run.

## 1. From fixed fixtures to generated assurance properties

A finite known-answer panel is useful for exact regression but weak against overfitting. The next step is to treat each assurance property as a generator of controlled state mutations and metamorphic relations.

The 34-case v0 panel remains the public development seed. It must not be silently re-labelled as a hidden holdout. A separate, preregistered generator should produce the confirmatory panel from a frozen seed and configuration, with the oracle constructed independently.

## 2. Mutation dimensions

### Exact operation identity

Mutate one dimension at a time, then selected pairs and triples:
- presenting principal / credential subject;
- action type and semantic profile version;
- target resource and destination;
- recipient / beneficiary / disclosure audience;
- amount, currency, quantity and operation count;
- purpose and required preconditions;
- resource revision / etag / ownership;
- canonical operation digest and action digest;
- idempotency identity and replay/consumption reference;
- effectuation route / provider / endpoint.

For each mutation, place the change at one of the stage boundaries: proposal, approval, Ready, dispatch preflight, provider dispatch, or retry. Reuse friendly request IDs intentionally to test that they do not substitute for exact operation equality.

### Authority and constitution

Generate:
- constitution epoch increments with changed and unchanged digests;
- digest changes while numeric epoch stays the same;
- designated-current constitution switches;
- approval revocation or supersession;
- capability expiry, future validity, revocation, suspension and unknown status;
- scope changes between approval and dispatch;
- local policy/evaluator revision changes;
- parent/child/grandchild delegation scope changes.

For every state transition, vary effective time and knowledge/observation time independently. A claim can become effective before it is known. The evaluator must answer both the current question and the historical as-known question without rewriting history.

### Sessions, caches and failover

Generate races involving:
- long-lived session with prior authority epoch;
- cache refresh before/after revocation observation;
- offline/disconnected executor;
- delayed or dropped revocation event;
- session epoch lower or higher than the current authority epoch;
- primary coordinator resumes after failover;
- stale fencing token reaches the protected resource;
- two concurrent sessions spend the same bounded capability.

### Delegation and trajectory-wide privilege

Generate:
- every individual scope dimension amplified or attenuated;
- a bounded parent with an unbounded child;
- expiry and not-before boundary races;
- amount/operation ceilings across multiple active credentials;
- ambient service tokens not used by the final successful call;
- forwarded credentials presented by a different actor;
- transitive child grants that reintroduce privileges excluded by the immediate parent;
- capability grants that are active in different portions of the same trajectory.

Report both the grant union and the per-operation scope relation. Where amount/operation ceilings have resource-specific semantics, the adapter must preserve those dimensions rather than collapsing them into one universal scalar.

### Complete mediation and alternate paths

Generate reachable and unreachable variants for:
- canonical API;
- bulk import and batch endpoints;
- secondary connector/tool;
- direct database or file access;
- shared service account;
- legacy endpoint;
- plugin or dynamic tool;
- emergency/operator route;
- cross-tool side effects.

For every reachable path, vary the enforcement-point identity. A route that passes through *some* PEP but not the designated PEP is still a failure for a claim about the designated boundary. A missing or incomplete route inventory is indeterminate, not complete mediation.

### Provider and outcome semantics

Generate provider declarations with:
- same-key idempotency guaranteed, not guaranteed or unknown;
- status query available, stale or unavailable;
- cancellation supported, best-effort or unsupported;
- atomic batch contract, per-action contract or no atomicity guarantee;
- fencing support and resource-side rejection;
- acknowledgement before/after effect;
- timeout before dispatch, after dispatch, after partial commit or after durable completion;
- compensation as a separate forward action.

Generate outcome states independently: not attempted, confirmed no effect, confirmed applied, partially applied, unknown, and partial-with-unknown. Never infer one from transport status alone.

### Evidence and observation

Generate:
- provider acknowledgement only;
- provider status query only;
- independent owner observation;
- observer with known shared dependency;
- observer with unknown dependency;
- duplicated/mirrored observations;
- missing expected action suffix;
- unexpected or duplicate action IDs;
- incomplete source lineage;
- decision log dropped by filter or rate limit.

Observation identity and dependency lineage must be preserved. Multiple source names do not imply independent evidence.

## 3. Metamorphic properties

The following relations should hold across generated mutations. Each property should have a positive, negative and boundary test.

1. **Evidence monotonicity:** removing required evidence cannot turn HOLD/INDETERMINATE into ALLOW.
2. **Authority monotonicity:** revoking or narrowing a grant cannot increase the set of permitted operations.
3. **Constitution currentness:** changing the current digest or epoch invalidates an old approval unless a new approval binds the new state.
4. **Exact-operation binding:** changing any approval-bound operation dimension without a new approval cannot preserve ALLOW.
5. **Delegation attenuation:** every child scope must remain a subset of its immediate parent; a grandchild cannot reintroduce an excluded privilege.
6. **Trajectory privilege monotonicity:** adding an active grant cannot reduce the union of available privileges.
7. **Complete mediation monotonicity:** adding a reachable route without the exact designated PEP cannot improve route coverage.
8. **Session freshness:** a session whose authority epoch differs from the authoritative epoch is not permitted to execute until reconciled, whether behind or ahead.
9. **Unknown-outcome safety:** losing the acknowledgement or status observation after dispatch cannot change UNKNOWN into confirmed-no-effect or authorize a blind retry.
10. **Partial-result preservation:** adding one more confirmed action observation may refine a partial outcome, but missing/unknown actions cannot be treated as applied or not-applied.
11. **Independent-evidence non-inflation:** duplicating or mirroring the same source cannot increase independence or corroboration.
12. **Provider-guarantee monotonicity:** removing a provider guarantee cannot make a retry more permissive.
13. **Bitemporal non-rewrite:** later knowledge may change the current projection but cannot alter the as-known reconstruction at an earlier cutoff.
14. **Compensation non-erasure:** a compensating action does not erase the original effect or its audit lineage.
15. **Clarification necessity:** removing a field that identifies the exact protected operation must cause clarification or hold, never infer an authority-expanding default.
16. **Log-loss non-inference:** dropping a decision-log event cannot prove that no decision, dispatch or effect occurred.

## 4. Composition matrix

The next panel should include isolated mutations and pairwise compositions, with selected three-way races. Minimum combinations include:
- authority revocation × stale session × lost acknowledgement;
- constitution epoch drift × exact recipient substitution × retry;
- delegation amplification × ambient credential × alternate tool;
- route bypass × policy log drop × provider status unavailable;
- partial multi-action effect × failover × duplicate retry;
- effective-time correction × late knowledge × succession/actor change;
- shared observer dependency × missing action suffix × provider self-report.

Do not exhaustively form the full Cartesian product. Use a published pairwise/covering-array strategy, but force every high-severity cross-boundary interaction into a named scenario and retain its exact denominator.

## 5. Confirmatory target design

The prospective target is at least 200 held-out cases across:
- four or more protected-resource/asset classes;
- three or more source/provider families;
- two or more jurisdictional or institutional-authority profiles;
- all six assurance dimensions: exact act, authority/currentness, delegation, mediation, effect finality, evidence/reconstruction.

Use at least 30 repeated runs per case for stochastic systems. Treat the scenario as the primary cluster when calculating confidence intervals. Report case-level worst outcomes and all hard-negative false permits. A zero observed false permit must be accompanied by a confidence bound and the exact numerator/denominator; it is not proof of a zero true-world rate.

## 6. Generator/oracle separation

- Generator inputs, random seed, version and configuration are frozen before evaluation.
- Expected outcomes are produced in a registry separate from system inputs.
- Systems under test receive no oracle or hidden-test metadata.
- An independent evaluator derives outcome classes from owner/provider semantics and checks assertions against raw traces.
- Retain minimized counterexamples, seed, mutation path and exact source references.
- Re-run every minimized counterexample against all comparator arms.
- Do not tune the generator or targets after holdout exposure.

## 7. Research targets — not achieved results

- Zero high-severity false permits on the frozen development panel.
- 100% detection of each one-field operation substitution in the finite mutation matrix.
- Zero permits with stale or unknown authority at the execution boundary.
- Zero acceptance of reachable routes that bypass the designated PEP.
- Zero blind retries after unknown external outcomes.
- 100% preservation of partial/unknown distinctions in the finite panel.
- At least two independent adjudicators and three version-pinned comparator arms before any superiority claim.

The point of generation is not to inflate scenario count. It is to demonstrate that the assurance properties remain true under controlled perturbations and compound state races.
