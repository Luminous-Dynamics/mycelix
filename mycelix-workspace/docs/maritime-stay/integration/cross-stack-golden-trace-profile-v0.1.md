# Maritime Commons Cross-Stack Golden-Trace Profile v0.1

**Status:** declarative synthetic integration corpus; no trace has been executed  
**Tracking:** Mycelix #5974 under cross-stack programme #5968  
**Composes:** Mycelix #5970/#5972, #5963/#5964, #5904/#5923/#2631; Sol-Atlas #115/#654; Symtropy #1612/#1644/#1955/#4276; Symthaea #1259/#11173/#10826

## Purpose

This corpus tests whether the four Maritime Commons owner systems preserve claim boundaries when their artifacts are composed into one coastal-stay workflow.

It is not a replacement for the 50-case adapter semantic-fidelity corpus in #5964. That corpus tests source-to-target mapping behavior. This one tests cross-owner hand-off behavior, including spatial evidence, physical/operational feasibility, bounded cognitive decision support, provider authority, common-mode provenance and historical reconstruction.

## Ownership and oracle separation

- Sol-Atlas owns its source/asset/projection provenance and spatial correctness.
- Symtropy owns deterministic operational/physical feasibility, model applicability and run replay.
- Symthaea owns bounded read-only synthesis, uncertainty, alternatives, abstention and human-review output.
- Mycelix owns participant recognition, provider/community authority, commitments, effect evidence, reconciliation and historical stay disposition.
- Mycelix #5904/#5923 own common-mode and time-qualified dependency closure. The golden-trace oracle must consume their result and exact snapshot reference; it must not reimplement a second dependency graph.
- Mycelix #5972 validates manifest structure and local hand-off references. This corpus does not duplicate that verifier.

The independent golden-trace oracle checks the *composition* of owner-native outputs. It must not reimplement Sol-Atlas GIS transforms, Symtropy physics/optimization, Symthaea cognition or Mycelix provider state reducers.

## Outcome semantics

The manifest distinguishes:
- source context: qualified, stale, unknown, unsupported, conflict, not applicable;
- feasibility: feasible, infeasible, unknown, unsupported, conflict, not evaluated;
- decision support: recommend, abstain, request information, human review, not evaluated;
- provider commitment: present, absent, pending, needs reconciliation, conflict, not evaluated;
- dependency assessment: shared dependency, unknown dependency, no declared common domain, not evaluated;
- stay disposition: confirmed, rejected, pending, unknown, conflict, needs reconciliation, not evaluated, incomparable.

A test case's expected disposition is the correct protocol outcome for that scenario, not a claim that the product has passed the benchmark. A runner must report case correctness separately from corpus execution status.

## Evidence lineage rule

A source observation, its Sol-Atlas projection, a Symtropy run derived from that projection and a Symthaea summary derived from the run are one evidence lineage with successive derivations. Four artifact IDs are not four independent origins.

Use Mycelix #5904/#5923/#2631 for failure-domain, temporal and evidence-frontier relations. Preserve:
- SHARED_DEPENDENCY;
- UNKNOWN_DEPENDENCY;
- NO_DECLARED_COMMON_DOMAIN.

NO_DECLARED_COMMON_DOMAIN means only that a complete/current declared inventory contains no common domain for the assessed scope. It is not proof of physical or statistical independence.

## Initial corpus

The companion JSON manifest contains 35 synthetic scenario definitions:
- positive controls for a valid source projection, supported feasibility, provider-authorized commitment, correct currency interpretation and reconstruction;
- negative controls for authority promotion, false spatial specificity, stale evidence, unsupported physics, privacy expansion and key/payload conflict;
- ambiguity controls for missing datums, DST fold/gap, unreviewed translation, unknown dependency and missing provider evidence;
- recovery controls for lost provider response, out-of-order events, coordinator replacement and historical correction;
- metamorphic controls for semantically insignificant source representation changes and derivative-lineage preservation.

No real provider payloads, personal itineraries, facility security details or live external effects are included.

## Required evaluator behavior

The future evaluator must:
1. Bind each case to exact fixture bytes/digests, owner commit/profile references and evaluator build.
2. Require one explicit result for each applicable owner output; missing output is UNKNOWN or INCOMPARABLE, never silently negative.
3. Verify that no owner output promotes another owner's authority.
4. Compare the integrated disposition against the frozen independent reference oracle.
5. Preserve per-owner dispositions and hard-gate failures; do not reduce them to a composite score.
6. Reconstruct historical decisions from the exact evidence frontier; later evidence must not leak into prior views.
7. Report coverage, correctness, abstention, unknown, unsupported, conflict, hard-gate failures and runtime separately.

## Benchmark measures

- false provider/legal/navigation authority promotion;
- false-feasible and stale-to-current promotion;
- false berth/resource/subject joins;
- source-to-derived lineage completeness and source/derivative overcount;
- shared-dependency detection, unknown-dependency abstention and false-independent corroboration;
- owner hand-off coverage and correctness;
- exact input/run/decision digest replay;
- independent coordinator reconstruction agreement;
- privacy/rights/purpose violations;
- partial-success and NeedsReconciliation correctness;
- human-review and clarification burden;
- marginal provider/product-profile integration effort.

Hard gates are non-compensatory: zero false authority promotion, wrong subject/resource binding, stale-to-current promotion, false-feasible output from missing mandatory inputs, double allocation, duplicate qualified provider effects, unauthorized disclosure and false-independent claims.

## Industry comparison

Compare matched workflows across:
- provider-native software and workflows;
- manual operator coordination;
- centralized aggregator/orchestrator;
- Maritime Commons federation.

Classify capability evidence as vendor-described, API-documented, authorized sandbox-observed, operationally observed or independently reproduced. Unknown public documentation is not proof that a vendor lacks a capability.

DCSA Port Call 2.0 conformance is a distinct external standards suite for appropriate port-call roles and event semantics; it is not a substitute for this stay/federation corpus. IHO conformance is product-profile-specific.

## Claim ceiling

A future PASS will establish only cross-stack contract composition and historical result agreement for the exact pinned synthetic owner artifacts, corpus and evaluator. It will not establish source authenticity, physical accuracy, hydrographic correctness, current berth availability, provider partnership, legal compliance, navigation safety, financial settlement, field performance or industry superiority.
