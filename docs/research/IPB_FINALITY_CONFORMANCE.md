# IPB-FINALITY-001 — Constitutional-epoch and exact-operation effectuation benchmark

**Status:** development contract  
**Issue:** [#6136](https://github.com/Luminous-Dynamics/mycelix/issues/6136)  
**Scenario corpus:** [IPB_FINALITY_SCENARIOS.json](IPB_FINALITY_SCENARIOS.json)

## 1. Research question

Can an independent evaluator establish that the exact operation becoming externally effective is the operation that was authorized, that the authorization remains current under the local constitution and protected state at the effectuation boundary, and that the resulting effect is independently accounted for?

This is a cross-owner benchmark, not a production subsystem. It does not replace an authorization policy engine, capability service, constitution registry, execution coordinator, provider contract or event store.

## 2. Prior art and industry position

This benchmark must be honest about the boundary between established authorization and effectuation assurance.

### Policy evaluation

Open Policy Agent evaluates a policy query and can emit decision logs containing the policy query, input, bundle metadata, result and timestamp. Those logs help audit and debug an authorization decision. The documentation also describes configurable decision filtering and rate limiting; events can be dropped when a configured maximum decisions-per-second limit is exceeded. Therefore a missing OPA log entry is not proof that no policy decision or downstream operation occurred. The benchmark includes R20 to test this distinction. See the [OPA decision-log documentation](https://www.openpolicyagent.org/docs/management-decision-logs).

Cedar evaluates a principal/action/resource/context request against policies and returns Allow or Deny with diagnostics. Cedar deliberately supports independently understandable permit/forbid policy logic. Its documented algorithm skips a policy whose evaluation returns an error; diagnostics can be inspected by the application to choose a different handling path. This is a design choice, not automatically a vulnerability. The LISE benchmark includes A19 to test whether a mandatory high-impact guardrail that fails to evaluate is silently treated as satisfied by an application integration. Cedar also requires callers to supply the entity data relevant to the request. See the [Cedar authorization reference](https://docs.cedarpolicy.com/auth/authorization.html) and [entity-data validation guidance](https://docs.cedarpolicy.com/policies/level-validation.html).

Google Zanzibar is an important comparator for large-scale, consistent authorization data and causal ordering of access-control changes. Its published USENIX ATC 2019 paper describes external consistency across authorization changes at global production scale. A Zanzibar-style check does not, by itself, prove provider-side effect completion or physical outcome. See the [USENIX paper](https://www.usenix.org/conference/atc19/presentation/pang).

### Execution-finality research

The 2026 individual IETF Internet-Drafts on execution finality explicitly address exact-act binding, protected current state, replay, revocation, crash-consistent effectuation and unknown outcomes. These drafts are **not endorsed by the IETF and have no formal standing in the IETF standards process**. They are important research comparators, not adopted standards:

- [Execution-Finality Architecture for AI and Autonomous Critical Systems](https://datatracker.ietf.org/doc/html/draft-das-execution-finality-deployment-01)
- [Execution-Finality Authority and Independent Validation for Autonomous Operations](https://datatracker.ietf.org/doc/html/draft-das-execution-finality-foundations-00)
- [Execution-Finality Enforcement, Protected State, and Delegated Operations](https://datatracker.ietf.org/doc/draft-das-execution-finality-enforcement/)

LISE must not claim to invent exact-operation binding, execution-time checks, replay prevention, fencing, idempotency, crash recovery, or independent effectuation controls.

### LISE's distinct benchmark question

LISE evaluates whether those controls compose with institutional authority and rights:

- the exact local constitution epoch is current at effectuation;
- the operation is exactly bound to the approved action;
- capability/delegation is current and non-amplifying;
- provider guarantees are evidenced rather than assumed;
- execution and observed effect are kept distinct;
- partial/unknown outcomes are preserved;
- an independent evaluator can reconstruct and challenge the event;
- evaluation never becomes authorization.

## 3. Stage model

The benchmark freezes distinct stage references:

    Candidate
      -> Approval
      -> Ready
      -> Execution preflight
      -> Effectuation attempt
      -> Independent observation
      -> Outcome reconciliation

A stage result cannot silently satisfy a later stage. In particular:

- Ready is not an execution lease.
- A historical approval under constitution epoch N is not current authority under epoch N+1.
- A policy Allow is not proof of current effectuation authority.
- A transport acknowledgement is not independent evidence of an external effect.
- A successful local dispatch is not proof that the provider applied the effect.
- A partial action prefix is not failure-no-effect.
- An unknown outcome is not permission to issue another mutation.

## 4. Exact operation correspondence

The normalized operation projection is deliberately domain-neutral but not universal. Each adapter declares which dimensions it supports and which are unavailable.

Where relevant, bind the exact institution/project, constitution digest/epoch, policy revision, principal/capability, action, target/resource, amount/quantity, recipient/destination/audience, purpose, preconditions/resource revision, canonical payload digest, logical operation identity, provider route, idempotency reference and source evidence.

Do not accept friendly-name equality as exact operation equality. A stable request ID may remain the same while amount, beneficiary, destination, asset, scope or payload changes. Those changes must be detected before provider dispatch.

The adapter must not invent a universal canonical action schema. It must bind the owner's actual semantic projection and expose unsupported dimensions. Hashing a lossy projection does not establish semantic completeness.

## 5. Constitutional epoch and currentness

The benchmark consumes constitutional epoch/currentness owners #60, #1161 and #1174.

At minimum, test constitution transitions at three boundaries:

1. after proposal/approval but before Ready;
2. after Ready but before effectuation;
3. during failover/recovery, when an older coordinator resumes.

Exact protected constitution digest matters. Matching only institution ID or human-readable epoch number is insufficient if the authoritative profile digest differs.

A historical decision may remain valid historical evidence while being ineligible to authorize a new action under the currently designated constitution. Any carry-forward or reauthorization must be explicit and bound to the successor epoch.

## 6. Authority, execution and outcome composition

Consume the owner-native semantics:

- #2953 — decision → policy evidence → current authority → capability/lease → execution attempt → independent observation → reconciliation.
- #3117 — governance decision → authorization → implementation → outcome → review/supersession.
- #1607 — ordered actions, partial completion, unknown result and reconciliation-before-retry.
- #6069 — effective-time/knowledge-time reconstruction and challenge history.
- #3126 — open-world uncertainty.
- #4835 — independent evaluation/oracle separation.
- #5762 — known dependency overlap and incomplete dependency closure.

Provider-specific guarantees require exact versioned provider evidence. If idempotency, status lookup, cancellation, compensation, atomicity, fencing or reconciliation is not supported, the benchmark must report that fact rather than assume a guarantee.

No universal exactly-once claim is permitted across arbitrary external providers. Use logical operation identity, idempotency semantics and explicit reconciliation. Compensation is a new governed forward action; it does not erase an already-observed external effect.

## 7. Scenario panel

The JSON corpus now contains 20 synthetic cases: six positive controls and fourteen attack/recovery/unsupported cases.

| Cases | Family | Core oracle |
|---|---|---|
| F01–F06 | Exact valid act, new-epoch reauthorization, unchanged precondition, bounded delegation, safe retry, fully observed multi-action | Permit only when all required facts remain current and supported |
| A07–A08 | Constitution changes after Ready; digest mismatch despite matching labels | Deny before provider dispatch |
| A09–A10 | Recipient or amount substitution | Deny before provider dispatch |
| A11–A13 | Revocation race, resource precondition drift, delegation amplification | Deny before provider dispatch |
| A14 | Replay of consumed authority/operation | Prevent second effect |
| R15 | Crash after dispatch but before acknowledgement | Hold for reconciliation; no blind retry |
| R16 | Successful prefix plus unknown suffix | Preserve partial state; no false atomicity or failure-no-effect |
| R17 | Stale primary resumes after failover | Resource-side fencing rejects obsolete epoch |
| R18 | Provider has no qualified idempotency or reconciliation | Indeterminate; no blind retry |\n| A19 | Mandatory policy guardrail evaluation errors while a general permit rule matches | Preserve diagnostics; do not dispatch on incomplete mandatory evaluation |\n| R20 | OPA-style decision log event is dropped or filtered | Log absence is not proof of no decision/effect; hold and reconcile |

The 20 cases are a development seed, not a frozen confirmatory benchmark. The independent oracle, initial states, exact provider contracts and adapter scope need review before qualification.

## 8. Matched comparators

Run the same frozen scenario inputs and state transitions against:

1. OPA policy evaluation with decision logging.
2. Cedar policy evaluation with version-pinned policy/entity data.
3. Zanzibar-style consistent authorization or a documented equivalent adapter, where available.
4. Conventional signed approval/workflow plus execution checks.
5. An execution-finality reference implementation/profile, with its exact revision and provider model pinned.
6. LISE composition across local constitution, current authority, exact operation, effect attempt, independent observation, bitemporal reconstruction and remedy.
7. Manual expert reconstruction.

Record product/version/configuration, data visibility, supported operation dimensions, current-state semantics, provider assumptions, failure modes and unsupported features. A policy engine must not be judged as if it promised external transaction atomicity; a finality sink must not be judged as if it established constitutional legitimacy.

## 9. Metrics and non-compensatory gates

Report independently:

- false-PASS / false-accept rate on high-severity attacks;
- exact-operation substitution detection by field;
- constitution epoch/currentness mismatch detection at each boundary;
- revoked/expired authority acceptance;
- replay and duplicate external effect count;
- false denial on positive controls;
- confirmed-applied / confirmed-not-applied / partially-applied / unknown outcome classification accuracy;
- blind retry after unknown outcome;
- resource-state precondition mismatch detection;
- evidence-source independence and observation coverage;
- reconciliation latency, runtime, throughput, memory, provider calls and operator burden.

Rights, authority, operation substitution, duplicate effects and loss of outcome evidence are non-compensatory failures. Faster execution cannot offset them.

Prospective research targets (not results):

- zero false-PASS on frozen high-severity epoch, authority, operation, replay and unknown-outcome cases;
- 100% rejection of one-field operation substitutions in the finite panel;
- 100% rejection of stale constitutional digests between Ready and effectuation;
- zero blind retries after unknown outcome unless the exact owner/provider contract establishes replay safety;
- 100% preservation of partial/unknown distinctions in the finite known-answer panel;
- at least three independent/version-pinned comparator arms before any performance or superiority claim.

## 10. Validation and implementation sequence

1. Validate the JSON corpus against a strict schema and check IDs, references, stage order and oracle separation.
2. Review operation-projection completeness by domain; document unsupported fields rather than inventing them.
3. Map F01/F02/A07/A08 to #60/#1161/#1174.
4. Map F03/A12 to owner-native resource preconditions.
5. Map F04/A13 to current authority and non-amplifying delegation.
6. Map F05/A14/R15/R16/R17/R18 to #1607 and #2953 provider/execution semantics.
7. Bind independent observation and bitemporal history through #6069/#4835.
8. Implement a read-only adapter for the first owner-native workflow.
9. Freeze holdout mutations and run matched comparator arms.

This specification and JSON manifest are authored development artifacts only. JSON syntax and scenario-ID/oracle disposition consistency were checked during authoring; no external schema validator, Rust runtime, provider integration, independent review, CI pass, legal opinion or field-effectiveness result is claimed.


## 11. Authorization assurance profile extension

The scenario set has been extended from 24 to 32 synthetic cases. New cases exercise refreshed/stale active sessions, revocation propagation, transitive delegation, alternate route bypass, trajectory-wide ambient privilege, credential subject forwarding, incomplete route inventories and unavailable revocation status.

See [Institutional Authorization Assurance Profile](./INSTITUTIONAL_AUTHORIZATION_ASSURANCE_PROFILE.md) for claim–obligation–enforcement–evidence–effect semantics and the five properties being tested. The profile is research guidance, not a standard or certification.

The Rust reference adds:
- transitive delegation-scope attenuation across action, resource, purpose, field, validity, amount and operation-count dimensions;
- a union of all active task, ambient and session grants for trajectory-wide privilege review;
- complete-mediation assessment over reachable routes with an explicit route-inventory completeness state;
- session revocation checks binding authoritative and observed revocation epochs and bitemporal action/knowledge times.

These functions consume owner-resolved inputs. They do not discover credentials, routes or revocation events in a real deployment.

## 12. New research references

- Elmasry, *Authorization assurance for tool-using LLM agents*, published 10 November 2026: https://link.springer.com/article/10.1007/s10207-026-01343-w
- AuthorizationBench, 20 October 2026: https://arxiv.org/abs/2610.27548
- SABench, 25 October 2026: https://arxiv.org/abs/2610.34167
- OPA decision-log completeness and configurable drop/rate-limit semantics: https://www.openpolicyagent.org/docs/management-decision-logs

These references motivate test dimensions; their results are not evidence that Luminous has achieved the corresponding targets.
