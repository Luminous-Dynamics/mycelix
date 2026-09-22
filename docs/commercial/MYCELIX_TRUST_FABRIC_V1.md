# Mycelix Trust Fabric v1

Status: Draft commercial projection  
Tracking: #881 (`MYC-COM-001`)  
Technical dependency: AGENT roadmap #805 and its exact qualified descendants  
Authority: Packaging/product design only. This document does not create a parallel authorization stack.

## 1. Product thesis

Mycelix should present a narrow commercial surface while preserving the broader architecture underneath it.

The v1 product thesis is:

> Mycelix is independently verifiable authority and evidence infrastructure for humans, workloads, AI agents, organizations and, later, federated institutions.

The commercial projection is:

```text
qualified Mycelix semantics
        +
operations
adapters
assurance
support
managed infrastructure
        =
commercial product
```

Commercial packaging must not silently redefine qualified authority semantics.

## 2. Three product surfaces

### 2.1 Mycelix Authority

Customer question:

> What is this exact actor allowed to do right now, under whose authority and constraints?

Target dependencies, only as each becomes qualified:

```text
PrincipalId
+ stable agent/runtime identity
+ runtime assurance where required
+ intent envelope
+ grant/delegation lineage
+ conserved budget where required
+ currentness/freshness
+ exact-action qualification
+ credential-less broker
```

The result is not a universal trust score. It is an exact, bounded authority decision under named profiles.

### 2.2 Mycelix Evidence

Customer question:

> What was attempted, what durable effect is known, and can another party independently verify the evidence?

Target dependencies:

```text
exact action identity
+ durable effect classification
+ AgentActionReceipt / successor receipt semantics
+ independent reconstruction/verifier
```

Effect disposition must preserve the existing distinction:

```text
Committed
DefinitelyNotCommitted
IndeterminateCommit
```

`success: true` is not a sufficient consequential-effect model.

### 2.3 Mycelix Federation

Customer question:

> Can independently governed organizations exchange authority and evidence without sharing one administrator?

The federation layer should compose organization-owned trust roots, portable evidence, explicit federation policy and independently verifiable receipts.

Federation never means that Mycelix Systems becomes the sovereign administrator of every participant.

## 3. AGENT roadmap projection

The commercial model maps onto existing technical work rather than forking it.

| AGENT stage | Technical responsibility | Commercial projection |
|---|---|---|
| AGENT-002 | stable principal ↔ runtime instance | actor/runtime identity |
| AGENT-003 | runtime provenance/assurance | workload assurance |
| AGENT-004 | bounded mission/intent | approved task envelope |
| AGENT-005 | delegation + conserved budgets | spend/scope/resource limits |
| AGENT-006 | exact current authority | Authority decision |
| AGENT-007 | exact-action qualification | action guard |
| AGENT-008 | credential-less broker | protected execution broker |
| AGENT-009 | durable effects + receipt | Evidence receipt |
| AGENT-010 | independent verifier | portable verification |
| AGENT-011+ | protocol adapters/TCK/privacy/commerce | interoperability/federation |

A commercial release may not claim a later capability because an earlier ancestor passed.

## 4. Two-lane development model

Commercial learning should not be blocked unnecessarily, but experimental integration must not become production authority.

### 4.1 Authority qualification lane

This lane:

- follows exact theorem dependencies;
- is fail-closed;
- emits authority only through qualified production paths;
- preserves nonclaims;
- never upgrades `queued`, `draft`, `mergeable`, `planned` or ancestor state to `PASS`.

### 4.2 Non-authoritative conformance lane

This lane may start earlier and is structurally incapable of granting production authority.

Examples:

```text
SPIFFE identity      -> candidate identity evidence
OAuth/OIDC           -> candidate credential evidence
WIMSE artifact       -> workload/authorization evidence
VC 2.0 credential    -> portable claim evidence
MCP request          -> requested action proposal
A2A message          -> communication/task proposal
SCITT statement      -> transparency/evidence projection
```

Hard invariants:

```text
authenticated != authorized
authorized != exact action qualified
exact action qualified != effect committed
committed != correct reasoning
external protocol participation != Mycelix trust root
```

## 5. Authority Receipt

The commercial primitive for a consequential action should be derived from qualified semantics and should be able to bind, as applicable:

```text
accountable principal
actor/runtime identity
grant/delegation references
intent/action identity
policy/profile identities
budget/currentness evidence
target/audience/resource
effect classification
evidence package identity
independent verifier disposition
explicit nonclaims
```

A UI object, copied JSON, third-party receipt or `verified=true` flag cannot self-promote into a positive Mycelix authority token.

## 6. External standards strategy

External standards are adapters, evidence sources or export targets unless a separately qualified profile gives them a stronger role.

Every adapter should state:

1. what the external artifact actually proves;
2. what Mycelix evidence category it may populate;
3. what it does not establish;
4. exact protocol/profile/version bindings;
5. replay/currentness semantics;
6. downgrade/substitution negatives;
7. maturity: experimental, qualification-only or production-eligible.

The initial interoperability TCK should be non-authoritative and include positive and negative golden vectors.

## 7. First design-partner workflows

Initial paid pilots should prefer bounded workflows where authority ambiguity is expensive and measurable but catastrophic physical/regulatory effects can be disabled or human-confirmed.

### 7.1 Privileged SaaS / cloud administration

Example:

```text
human principal
-> delegated remediation authority
-> support/security agent
-> exact resource/action constraints
-> human confirmation for high-risk changes
-> durable action receipt
```

Metrics may include approval time, orphaned credentials, unauthorized attempts, audit reconstruction time and deployment effort.

### 7.2 Support/remediation agent

Example:

```text
ticket/case
-> bounded intent
-> customer/tenant/resource scope
-> exact permitted actions
-> brokered credentials
-> action/effect receipt
```

Metrics may include mean resolution time, escalation rate, credential exposure and evidence completeness.

### 7.3 Purchase / operational approval preparation

The agent may prepare or request a consequential transaction, but v1 pilots can keep final movement of regulated funds or irreversible execution human-confirmed.

Metrics may include approval latency, policy exceptions, reconciliation effort and evidence completeness.

## 8. Maturity model

Every bounded product/profile receives its own maturity state.

| Level | Meaning |
|---|---|
| R0 Research | design/theorem not yet qualified |
| R1 Qualified Core | narrow core theorem passes exact qualification |
| R2 Simulation Integration | adapters/demos with no production effects |
| R3 Controlled Pilot | effects disabled or explicit human confirmation |
| R4 Bounded Production | qualified path plus required external assurance |
| R5 Federated Production | multi-organization interoperability demonstrated |
| R6 Infrastructure / Sovereign | public/critical infrastructure profile with additional assurance |

"Mycelix is production-ready" is not a valid global claim.

## 9. Commercial units

Candidate billable units include:

- governed identities;
- governed runtime/agent instances;
- active delegations;
- exact qualified actions;
- verified evidence receipts;
- managed organizations/nodes;
- evidence retention;
- connector tiers;
- assurance/support packages;
- sovereign/private deployment contracts.

The pricing model should reward useful adoption rather than depend on token appreciation or artificial lock-in.

## 10. Forward-deployed learning rule

Early customer engineering is acceptable, but every engagement must make later deployments easier.

Repeated work should converge into reusable:

```text
adapter
policy/profile
schema
TCK fixture
operational runbook
product feature
```

Permanently bespoke work should be accounted for as services rather than disguised as recurring product revenue.

## 11. Commercial promotion gates

A profile should not move to a higher commercial maturity level without explicit gates.

Examples:

### R1 -> R2

- exact technical PASS;
- stable public API/profile;
- non-authoritative integration harness.

### R2 -> R3

- design-partner workflow and baseline metrics;
- effects disabled or human-confirmed;
- incident/recovery runbook;
- evidence instrumentation.

### R3 -> R4

- bounded production authority path qualified;
- required external security review completed;
- critical findings resolved or explicitly accepted;
- measurable customer value;
- rollback/reconciliation procedures.

### R4 -> R5

- independent organizational trust roots;
- cross-organization evidence exchange;
- interoperability/TCK evidence;
- repeatable operations and support model.

## 12. Nonclaims

This specification does not prove:

- product-market fit;
- customer demand;
- regulatory compliance;
- agent reasoning correctness;
- runtime safety;
- any theorem not backed by its exact qualification evidence;
- revenue, ARR or valuation.

The commercial rule is simple: **sell the qualified trust fabric; do not invent a second authority system merely because it is easier to market.**
