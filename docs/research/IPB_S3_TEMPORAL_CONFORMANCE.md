# IPB-S3 Temporal Capture and Authority-Path Conformance v0

**Status:** development specification  
**Issue:** [#6134 — IPB-S3-TEMPORAL-001](https://github.com/Luminous-Dynamics/mycelix/issues/6134)  
**Scenario seed:** [IPB_S3_TEMPORAL_ATTACK_PATHS.yaml](IPB_S3_TEMPORAL_ATTACK_PATHS.yaml)  
**Parent:** IPB-S3-ICAP-v0 / [#6115](https://github.com/Luminous-Dynamics/mycelix/pull/6115)

## 1. Purpose and claim boundary

Static capture analysis answers whether a declared coalition can exercise a configured set of control domains at one cutoff. It does not answer how the coalition acquired those controls, when the controls became effective, whether a revocation happened before execution, or how later evidence changes the current view of an earlier event.

This specification extends the benchmark question from a static topology to a **temporal attack path**. It composes owner-native decision, authority, capability, execution, observation, bitemporal reconstruction, uncertainty and dependency records.

The fixture event vocabulary is a benchmark projection only. It is not a second production event schema, authority store, appointment service, succession registry, evidence store or institutional simulator.

The 14 scenarios are synthetic development cases. The confirmatory target is at least 30 frozen sequences and 10 paired temporal relations. Neither count has been met or independently evaluated by this specification.

## 2. Core model

For event reference e, retain at minimum:

- the exact owner-native source system, event reference and content digest;
- effective-time semantics and the event's effective interval;
- knowledge/recorded time;
- causal or predecessor references where supplied by the owner;
- actor/capability and subject references;
- exact authority, policy or constitution reference;
- action scope and evidence disposition.

Do not generate a new value for an owner field that is missing. Missing or ambiguous fields produce UNSUPPORTED or INDETERMINATE, depending on whether the source contract itself is unsupported or the state cannot be determined.

The evaluator reconstructs a projection at both a chosen effective-time cutoff and a chosen knowledge-time cutoff:

    InstitutionalState(effective_cutoff, knowledge_cutoff)

This is not an assertion that every owner currently exposes both axes. Adapter coverage must be explicit. If a subsystem only provides one temporal axis, scenarios requiring the other axis are not evaluated.

### Historical and current views

For a late correction, preserve two distinct products:

1. **Historical decision-time view:** what could be reconstructed from evidence known at the original decision cutoff.
2. **Current corrected view:** what is understood now after later-arriving evidence and corrections.

The current view may supersede or challenge the historical assessment. It must not erase the earlier view or imply that the institution knew evidence before it was recorded.

### Authority at execution

A capability valid at proposal or approval time does not automatically authorize a later execution. The evaluator must inspect the owner-native authority and revocation state at the consequential execution cutoff, and preserve the earlier decision/approval evidence separately.

### Unknown execution

After an ambiguous external attempt, the correct outcome is not inferred from a timeout. Preserve the operation/idempotency reference and require owner-native reconciliation before a new mutation. The benchmark must distinguish "not sent", "sent with unknown result", "confirmed applied", "confirmed not applied", and "partially applied" where the owner can establish those states. If the owner cannot, remain indeterminate.

## 3. Temporal attack-path semantics

A sequence is not capture merely because it contains several coalition appointments. It becomes a capture result only if the independent oracle establishes that:

- each event is valid or invalid under the exact policy/constitution in force at its effective time;
- the coalition can exercise all required critical functions in the resulting state;
- the declared independent oversight predicate is compromised for the configured threshold or interval;
- no valid independent challenge/remedy route remains within the scenario's declared bounds; and
- the observed event sequence is sufficiently complete to support the result.

A sequence that cannot prove one of these conditions is not silently classified as safe. It receives INDETERMINATE or UNSUPPORTED.

The attack-path report should include the earliest cutoff at which the predicate holds, the exact event references that establish the path, the first available independent challenge point, and any interval during which high-impact operations were possible while oversight was unavailable.

## 4. Event ordering and concurrency

Do not assume a total order from timestamps alone.

- Use owner-native causal/predecessor relationships when available.
- Treat events as concurrent only when the owner-native contract permits that interpretation.
- A metamorphic reordering relation is valid only for event pairs declared independent and commutative.
- Conflicting revocation/use, appointment/replacement or policy/authorization events must follow the owner contract's serialization/reconciliation rule.
- A timestamp tie with no source-defined ordering is INDETERMINATE, not an invitation to choose whichever order makes the test pass.

The paired T09/T10 sequences are intentionally different: T09 tests a declared independent permutation; T10 tests a same-cutoff conflict where no order can be assumed.

## 5. Scenario panel

The YAML seed contains 14 bounded cases:

| Family | Scenarios | Required finding |
|---|---|---|
| Benign continuity | T01 | Valid substitute succession preserves bounded authority and oversight |
| Control accumulation | T02–T03 | Sequential appointments and agenda access may compose into capture |
| Authority timing | T04–T06 | Revocation, expiry and delegation are checked at execution and cannot amplify scope |
| Bitemporal evidence | T07 | Late corrections revise current understanding without rewriting historical knowledge |
| Oversight availability | T08 | High-impact actions pause during a declared independent-oversight gap |
| Ordering | T09–T10 | Declared-independent events commute; conflicting events require source-defined reconciliation |
| Unknown execution | T11 | No blind retry or duplicate mutation after an ambiguous outcome |
| Dependency uncertainty | T12 | Partial dependency closure cannot be promoted into independence |
| Federation exit | T13 | Exit/revocation accounts for outstanding capabilities, obligations and remedies |
| Emergency authority | T14 | Repeated bounded extensions cannot be assumed safe merely because each has an expiry |

The cases intentionally include expected outcomes that depend on preconditions. In particular, T02/T03 only test capture if the initial authorities and each appointment event are established under the pinned constitution. Those premises must be part of the frozen fixture oracle, not inferred from the names or sequence labels.

## 6. Independent reference and candidate adapter

The candidate adapter consumes owner-native references and emits a read-only normalized projection. It does not mutate source systems or decide policy.

The independent finite reference state machine must be implemented separately from the candidate adapter. It must not reuse the candidate's event-ordering, authority evaluation or attack-path search helper. For each frozen sequence, retain:

- exact scenario and oracle digests;
- source event references and digests;
- constitution/policy versions and effective intervals;
- adapter and evaluator identities/versions;
- expected state at every required cutoff;
- expected attack path or benign control;
- terminal status;
- negative and contradictory evidence;
- replay instructions and counterexample minimization trace.

The system under test must not control the oracle, holdout sequence selection or qualification policy. The evaluator's own result cannot grant authority or execute a remedy.

## 7. Matched comparison programme

Compare identical frozen sequences, source references, time cutoffs, evaluator privileges and compute budgets.

| Baseline | Role |
|---|---|
| Static S3 v0 | Measures the incremental value of sequence-aware reconstruction over a one-cutoff topology |
| Hand-authored temporal panel | Establishes the minimum manual expert baseline |
| Property-based bounded sequence generation | Measures discovery beyond fixed authored sequences |
| Explicit-state / model-based exploration | Measures finite state coverage and earliest counterexamples |
| Temporal-logic/model-checking reference | Checks invariants when a tractable formal model can be written |
| LISE owner-native projection plus independent oracle | Tests actual cross-owner compositional assurance |
| Manual independent reconstruction | Measures reviewer effort and reconstruction reliability |

Do not claim superiority because one approach explores more synthetic cases. Report exact reference-state agreement, missed high-severity mutants, false alarms, first-counterexample time, state/transition counts, memory, runtime and budget exhaustion. If an adapter cannot access the same owner records or authority semantics, report OUT_OF_SCOPE/UNSUPPORTED rather than a numeric head-to-head score.

## 8. Metrics and gates

Report each dimension independently:

- false-PASS rate for high-severity unauthorized execution, capture, oversight, sovereignty/rights and emergency-power sequences;
- missed capture paths and false-positive capture alarms;
- earliest effective-time detection point;
- duration of oversight loss and number of consequential actions possible during that interval;
- authority-at-proposal versus authority-at-execution mismatch;
- historical reconstruction accuracy at each effective-time/knowledge-time cutoff;
- replay, duplicate delivery and unknown-outcome behavior;
- dependency closure completeness and exact known-overlap retention;
- exact reference-state agreement and held-out family results;
- runtime, memory, state/transition count and budget exhaustion.

Non-compensatory safety gates:

1. No stale, expired or revoked authority accepted at execution time in the finite known-answer panel.
2. No blind retry after unknown execution without reconciliation.
3. No later knowledge-time correction silently rewrites the historical decision-time view.
4. No incomplete dependency closure produces a numeric independence claim.
5. No high-impact action is accepted during a declared oversight gap when the pinned policy requires that oversight.
6. No evaluator output directly authorizes policy change, execution or remedy.

Proposed research targets are zero false PASS on preregistered high-severity sequences, 100% independent reference reconstruction on the finite panel, at least 30 frozen sequences, and at least 10 paired temporal relations. These are targets, not results.

## 9. Standards and prior art

This benchmark should compose with, not replace:

- NIST SP 800-63C-4 for identity federation and assertions; credentials remain distinct from local institutional authority.
- W3C Verifiable Credentials 2.0 and the 13 October 2026 JOSE/COSE Recommendation for credential representation and securing mechanisms.
- Existing bitemporal database/event reconstruction and event-sourcing practices, with owner-native semantics and correction history.
- Established model-based testing, explicit-state exploration and temporal logic/model checking.

NIST's federation guidance describes assertions between separately administered relying parties; it is an identity/federation foundation, not an institutional power or remedy model. W3C's JOSE/COSE Recommendation standardizes mechanisms for securing verifiable credentials and presentations; it does not prove the truth of a claim or grant local authority.

## 10. Implementation sequence

1. Freeze the 14-case seed and define required initial state/oracle premises for each sequence.
2. Map T01/T04/T05/T11 to the owner-native decision/authority/execution/reconciliation lifecycle.
3. Map T07 to the bitemporal reconstruction and correction owner.
4. Map T02/T03/T08/T14 to constitutional appointment, oversight and power-analysis owners.
5. Keep T12 indeterminate until dependency closure is qualified; keep T13 unsupported until federation exit/revocation owner semantics are available.
6. Implement a bounded read-only adapter and independent state-machine oracle for the first supported subset.
7. Add seeded path generation and minimization only after fixed cases reproduce exactly.
8. Freeze held-out temporal compositions before comparing candidate approaches.

## 11. Current validation

This file and the YAML manifest are authored research artifacts. They have not yet been parsed by the repository's schema/validation tooling, compiled, run against owner-native records, independently reviewed, or evaluated against external platforms. The synthetic case count is 14; the confirmatory target of 30 remains unmet.

No CI, Rust test, standards-conformance, legal-compliance, production-authority or field-effectiveness claim is made.
