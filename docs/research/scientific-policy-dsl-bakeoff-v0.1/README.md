# Mycelix scientific authorization: policy DSL bake-off

**Status:** Research and conformance-corpus proposal  
**Baseline:** \`DefaultScientificAuthorizationPolicy\` v1.0.0  
**Baseline revision:** \`061e34f24d406f19f6de0e61867305b6989f455f\`  
**Corpus:** `cases.json` (30 pure-policy cases, 10 projection cases, 11 outer security/commit gates)

## The paradigm shift: benchmark semantics before syntax

Mycelix already has a real, typed authorization policy in
\`mycelix-desci/src/core/src/scientific_governance.rs\`. It also has receipt-time
identity/key checks, acting-organization membership checks, idempotent replay
handling, and signed authority receipts. A new language must preserve those
semantics; it should not replace them with a toy RBAC demonstration.

The experiment is therefore not "is our DSL nicer than Cedar or Rego?" It is:

> Can one language-neutral, versioned authorization contract be evaluated by
> the existing Rust policy, Cedar, and OPA/Rego with identical decisions,
> explanations, and fail-closed behavior—while keeping non-delegable integrity
> gates outside the editable policy?

The first conformance corpus was extracted from the current implementation.
It is intentionally marked as research data, not as a test suite already
executed against all engines. The corpus is stored in `cases.json` with a Draft 2020-12 contract in `cases.schema.json`; the schema constrains each case's action, actor roles, claim/attestation context, expected decision, and outer gate shape.

## Existing implementation: semantic baseline

The current pure policy has these rules:

| Action | Allow condition | Deny controls |
|---|---|---|
| \`claim_proposed\` | Empty stream and role is Contributor, Editor, or Institution | Existing stream or no eligible role |
| \`legacy_claim_imported\` | Empty stream and MigrationService role | Existing stream or any other role |
| \`evidence_attached\` | Existing claim and actor is creator, Editor, or Institution | Unrelated actor |
| \`attestation_recorded\` | Existing claim and role is Reviewer, Contributor, or Institution | Missing role; independent replication by creator; acting organization equal to creator organization |
| \`attestation_corrected\` / \`attestation_withdrawn\` | Existing attestation and actor is its author, Editor, or Institution | Missing attestation or unrelated non-editor |
| \`claim_superseded\` / \`claim_retracted\` | Existing claim and actor is creator, Editor, or Institution | Unrelated non-editor |

Two details are easy to accidentally change in a rewrite:

1. **Editor is not a general reviewer role.** The current policy lets Editor/Institution manage some lifecycle and evidence actions, but an Editor-only actor does not satisfy the \`attestation_recorded\` role check.
2. **Independent replication is stricter than an ordinary attestation.** The claim creator cannot attest their own independent replication, and an acting organization equal to the creator organization cannot count as independent.

The corpus represents these distinctions explicitly. Its 30 cases include 18 allows and 12 denies. They are one-fault decision vectors; they do not yet cover the full cross-product of every actor, role, organization, and event shape.

## Separate permission from evidence qualification

An accepted `AttestationRecorded` event is not automatically counted as
independent support. `ClaimProjection::recompute_evidence_profile` has its
own qualification semantics:

- An ordinary Review by the claim creator, or by an actor acting for the
  creator organization, does not increase `review_count`.
- Reviewer count is deduplicated by actor identity.
- Computational reproduction and independent replication sources are
  deduplicated using the acting organization when present, otherwise actor
  identity.
- An IndependentReplication contributes supportive independent evidence only
  when the actor is not the creator and the acting organization is not the
  creator organization. Non-supporting and inconclusive outcomes are tracked
  separately.
- A self-submitted ordinary review is currently authorizable for a contributor,
  but is not evidence of an independent reviewer. The corpus records this as
  two separate expectations instead of incorrectly treating one as the other.

The corpus now includes two explicit review cases:
`AUTH-029` records that a non-creator reviewer without a declared acting
organization can currently submit an independent replication (with actor
identity used as the source key); whether that is sufficient independence is
a normative policy question, not something the port may silently decide.
`AUTH-030` records that the claim creator can submit an ordinary review under
the authorization policy, while the evidence projection gives that review no
reviewer credit. Both cases carry explicit projection-effect expectations for
the resulting evidence profile.

The projection fixtures are now consumed by a Rust test in
`scientific_events.rs` that rebuilds signed event streams and compares the
resulting evidence-profile counters with the fixture expectations. The test
source is committed but has not yet completed compilation or execution.

This creates two separate equivalence targets: **authorization equivalence**
(what events may be appended) and **epistemic qualification equivalence**
(which accepted events count toward evidence maturity). Neither is a substitute
for the other.

## Keep the authority kernel non-delegable

The pure policy must not be able to waive the following gates already enforced by
the governed event log:

- Verify the event signature.
- Resolve the signed actor; reject unresolved or mismatched identity.
- Bind the signing key to that actor and check key validity/revocation using the service-observed \`received_at\`, never a caller-supplied backdated time.
- Require membership in any claimed acting organization.
- Reject duplicate event IDs that identify different signed bytes.
- Treat an exact retry of an already committed event with an attested receipt as idempotent, not as a new authorization.
- Require a valid signed authority receipt and a contiguous receipt chain.
- Commit the event and authority receipt atomically where the store supports it, or use the existing prepare/append/finalize recovery protocol.

Those eleven boundary cases are enumerated separately as \`prePolicyAndCommitGates\` in the JSON corpus. **They must not be rewritten as optional Cedar/OPA policy clauses.** An editable policy can add denials and narrower permissions, but it cannot disable the integrity kernel.

## Projection conformance contract

The JSON corpus now has ten projection cases, separate from the 30 allow/deny
policy vectors. They cover creator reviews, same-creator-organization reviews,
actor-identity fallback, reviewer de-duplication, organization-level source
de-duplication, supporting and non-supporting reproduction/replication,
inconclusive outcomes, and withdrawn attestations.

The Rust projection adapter in
`mycelix-desci/src/core/src/scientific_events.rs` builds actual signed event
streams, including the evidence artifact required by reproduction/replication
attestations, replays them through `ClaimProjection::rebuild`, then asserts
the exact six profile counters recorded by each fixture. This is a test of
the projection semantics, not a policy decision test and not a statement that
the evidence artifact was independently verified.

One explicit open governance decision remains: `PROJ-006` freezes the current
behavior in which two distinct individuals with no acting organization count
as two independent sources. That behavior may be suitable for person-level
independence, but it is not equivalent to institutional independence. A future
policy version should introduce an explicit source-independence standard and
migration/versioning behavior rather than changing this silently.

## First candidate implementations

The same pure-policy contract now has three reference artifacts:

- `cases.json` plus `cases.schema.json`: the frozen test inputs and their versioned structural contract.
- `mycelix-desci/src/core/src/scientific_governance.rs`: a Rust unit-test adapter builds signed synthetic events and typed claim projections, then calls the existing `DefaultScientificAuthorizationPolicy` directly for each of the 30 expected decisions. This preserves the real typed policy as the oracle; it does not test the governed storage/receipt kernel with synthetic bypasses.
- `cedar-schema.json`, `cedar-policy.cedar`, and `cedar-tests.json`: a Cedar 4.14.0 candidate plus **30 native Cedar CLI test requests** generated from the frozen Rust corpus. Genesis actors can lack creator metadata; the schema declares that attribute optional and the policy guards every access with `has`.
- `opa-policy.rego` and `opa-policy_test.rego`: an OPA/Rego v1 candidate with a default-deny decision object, **30 generated corpus tests**, and eight additional focused positive/negative controls.

A dedicated workflow, `.github/workflows/scientific-policy-dsl-conformance.yml`, now runs the Rust authorization and projection tests, validates the Cedar policy against its JSON schema, executes all 30 Cedar `run-tests` cases, checks the Rego modules in strict mode, and runs all 30 Rego corpus cases plus eight additional controls with OPA 1.22.0. The OPA Linux static binary is pinned by its release SHA-256. The workflow has been committed but has not yet completed on the current PR head.

The Cedar and Rego implementations are initial translations. Both now have test inputs for all 30 frozen policy cases: Cedar uses the project's native JSON test format and `cedar run-tests`; Rego contains one generated test per case plus eight focused controls. This allows the dedicated workflow to test decision equivalence without a custom wrapper. The tests and adapter files are committed but have not yet completed hosted execution; exact 30/30 parity remains unproven until the workflow succeeds.

## What to compare

| Candidate | Strengths to measure | Main limitations / fair boundary |
|---|---|---|
| Existing Rust policy | Lowest integration distance; exact current behavior; direct typed access to claim projection and signed event models | Harder for non-Rust governance authors to edit; currently emits prose reasons rather than stable machine-readable decision codes |
| Cedar | Purpose-built authorization language; PARC request model; JSON schema validation; policy analysis and refactoring support; Rust implementation | Must express Mycelix's claim lifecycle and actor/organization context without moving cryptographic/event-log checks into policy |
| OPA/Rego | General-purpose policy engine; mature profiling, benchmarking, partial evaluation, and deployment options | General-purpose semantics can make authorization schemas and deny/error conventions easier to misuse; separate server/sidecar modes add operational variables |

Cedar is the most directly relevant language baseline for authorization. Its
reference guide describes schema validation that catches unknown entity types,
actions, attributes, and unsafe optional-attribute access. It also documents
validation soundness and a differential-testing comparison between the Rust
implementation and Lean model. See
[ Cedar validation ](https://docs.cedarpolicy.com/policies/validation.html)
and the [verification-guided Cedar paper](https://arxiv.org/abs/2407.01688).

OPA remains a useful second baseline because its performance documentation
separates policy loading/parsing/compilation from query evaluation, and offers
\`opa bench\`, profiling, \`benchstat\`, and partial-evaluation guidance. We must
report those costs separately. See
[OPA policy performance](https://www.openpolicyagent.org/docs/policy-performance)
and [OPA benchmark results](https://openpolicyagent.github.io/opa/index.html).

Do not compare an in-process Rust evaluator to a remote policy service and call
the difference a language result. Run Cedar and Rust in-process; measure OPA
embedded/in-process separately from its sidecar/server round-trip.

## Language-neutral input and output contract

The test harness should project the existing Rust domain objects into a frozen
authorization input, independent of any engine-specific PARC or Rego shape.

The normalized input includes:

- actor ID, roles, authorized organizations, and authority-registry revision;
- action and exact action payload class;
- stream existence and claim creator identity/organization;
- target attestation existence and author identity;
- attestation kind (ordinary vs independent replication);
- optional acting organization;
- trusted receipt-time context supplied by the outer kernel.

The policy result is a structured triple:

- \`ALLOW\` or \`DENY\`;
- a stable rule/decision code;
- a human-readable explanation.

A missing or malformed required value is \`INDETERMINATE\`, which the write
boundary maps to deny. Do not treat evaluator errors, unknown actions, absent
attributes, or timeout as allow.

The current Rust \`AuthorizationDecision\` carries a boolean and free-form reason.
That is sufficient for today's direct implementation but insufficient for
robust cross-engine conformance. A follow-up should add stable rule identifiers
without breaking existing receipt serialization unexpectedly. Receipts should
continue to bind the policy identity/version and authority snapshot; a later
protocol revision can consider a cryptographic policy artifact digest.

## Benchmark design

### 1. Decision equivalence and safety (hard gates)

| Measurement | P0 target | Evidence |
|---|---:|---|
| Agreement with current Rust policy on the 30 frozen cases | 100% | Exact per-case decisions and diffs |
| False allows on a negative-control corpus | 0 | Per-case negative results |
| Fail-closed behavior for unknown/missing fields | 100% | Mutation vectors |
| Outer signature/identity/key/org/replay/receipt gates | 100% preserved | Guard tests remain in Rust and cannot be overridden by policy |
| Policy upgrade equivalence | 100% where semantics are declared unchanged | Old/new policy differential report |

Do not make the 30-case corpus the final security proof. Generate at least
1,000 deterministic mutations from valid and deny fixtures before calling the
first version robust. Keep one-fault mutations separate from compound attacks;
publish both denominators and any disagreements.

### 2. Metamorphic and adversarial tests

Add generated tests for:

- Reordering role and organization sets does not change the decision.
- Adding unrelated roles does not turn a deny into an allow.
- Removing a required role never creates an allow.
- Replacing the creator with a different actor flips owner-only operations from allow to deny, absent editor/institution authority.
- Changing a target attestation from existing to missing denies correction/withdrawal.
- Changing independent-replication context from creator organization to a different *authorized* organization may allow, but an unauthorized acting organization is still denied by the outer kernel.
- Revoking or expiring a key blocks new writes at receipt time; it must not rewrite a previously signed authority receipt or break exact retry semantics for an already committed event.
- Unknown action or malformed context produces \`INDETERMINATE\`/deny, never implicit allow.

### 3. Performance and operational costs

Run on the same frozen input set, same policy cardinality, same hardware, and
same number of worker threads. Separate:

1. cold policy parse/load/compile;
2. schema validation;
3. warm in-process decision evaluation;
4. explanation/diagnostic generation;
5. input projection + serialization;
6. end-to-end authorization call, including outer gates and storage-independent receipt construction.

Report p50, p95, p99, decisions/second, CPU, allocations/bytes, peak RSS,
compile/update latency, and error rate. Repeat at 1, 16, and 64 concurrent
workers, and at 10, 100, and 1,000 policies/rules. Add realistic mixes:
owner writes, reviewer attestations, independent replications, denials,
missing attributes, and policy-upgrade revalidation.

OPA's documented \`opa bench\` methodology explicitly treats query-evaluation
time separately from parsing/loading/compilation. Follow the same discipline
for every candidate; report sidecar/network overhead as a separate column.

### 4. Explainability and governance operations

Measure whether an authorized reviewer can:

- locate the exact rule causing an allow/deny;
- reproduce the decision using the exact policy and authority snapshot;
- inspect the difference between two policy versions;
- verify that a policy edit narrows or preserves permissions;
- restore a prior policy revision without changing old receipts;
- obtain a bounded, privacy-preserving explanation without leaking sensitive
  attributes from an actor or claim.

A useful paradigm shift is **permission-diff before deployment**: policy changes
should produce a machine-checkable set of requests whose decisions change,
including proof that constitutional invariants still hold. Cedar's policy
analysis/refactoring support is a strong benchmark to compare against, not
something we should claim to have surpassed before testing.

## Proposed P0/P1 decision

1. Keep the existing Rust policy as the reference oracle for v0.1.
2. Add stable rule codes and a normalized input/output contract.
3. Complete the Cedar request adapter and run all 30 vectors; do not write a new
   parser or custom language yet.
4. Run the generated 30-case Rego suite alongside the eight focused controls.
5. Compare decision equivalence first; compare latency only after all three
   systems return identical decisions.
6. If Cedar handles the rules cleanly, use Cedar for editable authorization
   policy while the Rust authority kernel remains non-overridable. If a concrete
   expressivity or integration gap appears, document it before designing a
   Mycelix-specific syntax.

## Explicitly not claimed

- The Rust authorization and projection adapters, Cedar native test corpus, and Rego test corpus are committed but have not yet completed a hosted compile/test run. Exact 30/30 cross-engine parity and 10/10 projection parity remain unproven until the workflow succeeds.
- The 30 expected decisions are an extraction from one Rust source revision,
  not an independent proof that the policy itself is normatively correct.
- No latency numbers are collected here.
- No new language is selected by this document.
- Passing an authorization policy does not prove an event is authentic, an
  actor's real-world identity is true, or a scientific claim is valid.

The first result is a grounded benchmark contract and comparison plan—not a
claim of beating Cedar, OPA, or the existing Rust implementation.
