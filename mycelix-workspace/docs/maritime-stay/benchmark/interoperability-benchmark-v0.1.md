# Maritime Commons Interoperability Benchmark v0.1

**Status:** Draft research specification; not executed  
**Owner:** Mycelix / Maritime Commons  
**Tracking:** [#5932](https://github.com/Luminous-Dynamics/mycelix/issues/5932)  
**Parents:** [#5916](https://github.com/Luminous-Dynamics/mycelix/issues/5916), [#5920](https://github.com/Luminous-Dynamics/mycelix/issues/5920), [#5921](https://github.com/Luminous-Dynamics/mycelix/issues/5921), [#5925](https://github.com/Luminous-Dynamics/mycelix/issues/5925)

## 1. What this benchmark must falsify

The benchmark tests whether a thin, provider-neutral coordination layer can compose independent maritime service providers while preserving each provider's authority over its own inventory, customer relationship, terms and commitments.

It must compare equal tasks, not feature lists or vendor marketing. It must be possible for the benchmark to show that federation is slower, more expensive to integrate, more difficult to operate, or less reliable than an incumbent workflow. A benchmark that cannot falsify the thesis is not useful.

The v0.1 benchmark is a protocol and evaluation contract. It is not a product qualification, market study, operator endorsement, certification, or claim of superiority.

## 2. Comparison arms

Use the same synthetic task, resources, policy constraints, fault schedule and outcome definitions for each applicable arm.

| Arm | Description | Evidence boundary |
| --- | --- | --- |
| MANUAL | Current siloed process: separate marina, accommodation and local-service requests, with human handoffs and reconciliation | Measure participant/operator actions and active labor separately from waiting time |
| NATIVE | Each provider uses its own existing system and remains the source of truth | Requires provider-authorized access or an operator-observed workflow |
| NETWORK | An incumbent booking/discovery network or marketplace is used where the required capability is actually offered | Compare only available, region-relevant, permissioned functionality |
| FEDERATED | Maritime Commons coordinates provider-owned offers, commitments and receipts through replaceable adapters | No central authoritative inventory mirror and no hidden write authority |

Not every case is comparable across every arm. Record "not applicable" with a reason rather than manufacturing parity. A public marketing page is not a runtime trial. Lack of public API documentation is "unknown", not proof that a vendor has no integration capability.

## 3. Evidence grades

Every asserted capability and result must carry one of these grades:

1. public_documented: published by the responsible organization; not independently reproduced.
2. vendor_reported: a claim or metric published by the vendor; retain the original attribution.
3. authorized_sandbox_observed: tested with permission in an explicitly identified sandbox.
4. operator_observed: directly observed with the operator's permission; sample and procedure recorded.
5. independently_reproduced: repeated by an independent evaluator from frozen inputs and documented procedure.
6. synthetic_only: demonstrated only by generated fixtures or a local simulation.
7. unknown: not established. This must not be converted into a negative capability claim.

Separate each result's evidence grade from its confidence interval and from the protocol's own conformance status.

## 4. Workload model

The v0.1 workload composes:

- one berth resource with time interval and vessel-compatibility constraints;
- one independently operated accommodation resource;
- one optional local service resource;
- provider-specific fee, cancellation and refund terms;
- optional community-recognized eligibility;
- an independent port/statutory clearance condition that is never implied by a marina booking.

Each provider is the authority for its own availability and commitment. The benchmark may compare a coordination layer's representation of evidence, but it cannot grant a provider authority or infer a physical or statutory fact from a message.

The machine-readable workload manifest is interoperability-workload-v0.1.json. It references the existing semantic and webhook corpora; it does not duplicate them.

## 5. Scenario matrix

The initial frozen matrix includes:

- complete mandatory stay across three independent providers;
- mandatory provider rejection;
- optional provider rejection;
- provider commits durably, then the response is lost;
- repeated and out-of-order event delivery;
- two competing claims for one capacity-one resource;
- stale offer or superseded provider revision;
- provider adapter replacement;
- partition during a multi-provider stay;
- key rotation or revocation before an attempted action;
- an entitlement not recognized by the destination community;
- fee mutation, partial cancellation and refund ambiguity;
- overbroad disclosure versus purpose-minimized disclosure;
- marina booking presented incorrectly as port-entry or border clearance.

The matrix is not exhaustive. New cases must be versioned rather than silently replacing a failed case.

## 6. Measures — no composite score

### 6.1 Correctness and safety

Report counts and rates for:
- valid completed stays;
- partial stays and unresolved stays;
- false-confirmed stays;
- double-committed scarce resources;
- duplicate financial effects;
- unauthorized actions or disclosures;
- stale/revoked observations promoted to current commitments;
- unresolved contradictions and manual reconciliation.

A zero in a synthetic corpus means only that the implementation passed those exact cases. It does not establish a zero real-world failure rate.

### 6.2 Coordination effort

Record separately:
- wall-clock end-to-end elapsed time;
- active participant minutes;
- active provider/operator minutes;
- machine processing latency;
- number of human handoffs;
- number of systems/channels touched;
- manual reconciliation interventions;
- successful completion and abandonment.

Do not merge machine latency, human waiting time and active labor into one headline number.

### 6.3 Integration and lifecycle cost

Record:
- engineering/person-hours to first successful adapter conformance;
- operator/admin time and permissions required;
- number of provider-specific mapping rules;
- number of custom code paths;
- number and sensitivity of fields disclosed;
- time and code changes to replace a provider adapter;
- maintenance effort after provider schema/profile revision;
- rollback time and behavior after adapter revocation.

Count the ongoing operator burden as well as the one-time developer effort.

### 6.4 Provider autonomy and local governance

Verify:
- who owns authoritative availability and allocation;
- who can create, alter, cancel and refund a provider effect;
- whether the provider can replace or revoke its adapter;
- whether provider operations continue if the federation is unavailable;
- whether a community can reject an otherwise valid credential or entitlement;
- whether local resident-access, accessibility and ecological rules remain enforceable.

### 6.5 Privacy and economic transparency

Measure purpose-specific data fields, retention, unnecessary copies, fee disclosure before acceptance, invoice arithmetic, refund evidence and reconciliation outcomes.

A valid signature does not establish that a payment settled. A payment provider's finality reference must be independently recognized under the relevant payment/accounting contract.

### 6.6 Interoperability and recovery

Inject duplicate, missing, delayed, out-of-order and contradictory events; stale cache entries; endpoint replacement; key rotation; credential revocation; partitions; process restarts; and failed reconciliation.

Report the exact time and evidence required to move from Unknown or NeedsReconciliation to a justified terminal disposition.

## 7. Non-compensatory safety gates

A run fails the safety comparison if any of these occurs:

- false-confirmed stay;
- double-committed capacity-one resource;
- duplicate financial effect;
- unauthorized or wrong-purpose action;
- acceptance of revoked or locally unrecognized authority;
- disclosure outside the declared purpose/data-minimization policy;
- stale/offline observation represented as a current provider commitment;
- marina booking represented as port-entry, customs, immigration, navigation, pilotage or safety authorization.

Do not average a safety failure away with faster completion or better usability.

## 8. Reproducibility and statistics

Every run must record:

- benchmark and case-set version;
- exact repository commit/tree identity for software under test;
- fixture and input digests;
- provider adapter and profile version;
- evaluator version and independent-oracle identity;
- environment and machine configuration;
- fault schedule and random seed, if any;
- all attempts, including timeouts, failed and excluded runs;
- raw per-case outcomes and reasons for exclusions.

Use paired task runs where possible. Report sample counts, median, p90/p95 and confidence intervals. Pre-register the human study procedure and participant roles. Keep synthetic runs, sandbox runs and field observations in separate result tables.

A practical initial design is at least 30 paired observations per benchmark arm for exploratory timing distributions, followed by a power analysis using observed variance before confirmatory human-performance claims. This is a planning floor, not a guarantee of statistical power.

No result is reproducible if the evaluator silently changes fixtures, drops failures, relaxes a gate or uses a different software head without a new benchmark version.

## 9. Proposed hypotheses — not measured targets

Test these hypotheses against the frozen baselines:

- at least 30% lower median end-to-end coordination time than the matched manual baseline;
- lower active human coordination time without increasing provider/operator effort;
- adapter replacement without transferring provider inventory or customer databases;
- lower unnecessary personal-data disclosure for the same task and purpose;
- adapter onboarding effort decreases as reviewed profile reuse increases;
- ambiguity after a lost provider response remains explicit until provider-authoritative reconciliation.

The 30% target is a hypothesis only. Report the measured baseline first. Do not tune the workload after seeing results.

## 10. Industry and standards comparison boundaries

Dockwa, BerthApp, Navily and other products have overlapping but different booking, discovery, operational and marketplace functions. A comparison must use the same region, user task, provider permissions and actual feature availability. Preserve vendor attribution for vendor-reported metrics.

DCSA Port Call 2.0 is a structured exchange profile for port-call service events. It is not a recreational marina booking or cross-provider stay protocol. Its published API specification leaves subscriber registration and authentication/authorization out of scope. Maritime Commons may adapt relevant event semantics, but must provide its own reviewed federation authorization and identity bindings.

IHO S-100 product specifications describe defined maritime information products. They are not live berth inventory, booking confirmations or port-entry permissions. Pin exact product edition, producer, coverage, datum, validity and transformation lineage; never use one generic S-100 compliant flag.

A standards adapter proves only the declared mapping/conformance subset. It does not prove operational acceptance by a port, marina, authority or ECDIS equipment.

## 11. Stop conditions and field-study permissions

No live booking, reservation hold, payment, cancellation, customer-data export, webhook activation or production endpoint connection is part of this benchmark without separate written authorization from the responsible provider/operator and appropriate legal/security review.

A provider sandbox or sample payload must be clearly identified as authorized. Synthetic payloads must never be represented as real provider traffic.

A South African candidate register is not a partnership or pilot selection. Keep ZYC/BerthApp and Durban Marina as screening candidates until operators explicitly agree to participate.

## 12. Acceptance

- Comparison arms use matched tasks and explicit applicability rules.
- Claims are assigned evidence grades and vendor attribution.
- The workload manifest is machine-readable and references the existing semantic/webhook corpora.
- Safety gates are non-compensatory and have negative controls.
- All failures, timeouts and missing observations remain visible.
- The evaluator does not import production orchestration or capacity-transition helpers.
- Synthetic, sandbox, operator-observed and independently reproduced results remain separate.
- No SOTA, performance, certification, partnership or production-readiness claim is made without the evidence required for that claim.
