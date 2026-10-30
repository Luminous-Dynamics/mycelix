# Scientific Policy Equivalence and CEL Evaluation v0.1

**Status:** Research plan; no CEL result or formal proof claimed  
**Issue:** [SCIENTIFIC-POLICY-EQUIV-001 #5530](https://github.com/Luminous-Dynamics/mycelix/issues/5530)  
**Parent:** [Scientific authorization DSL bake-off #5522](https://github.com/Luminous-Dynamics/mycelix/pull/5522)

## 1. Executive decision

The next quality step is not to add more hand-written authorization examples. It is to establish the set of requests for which the current Mycelix policy allows or denies, compare implementations over the same request space, and produce a counterexample whenever a policy change alters that set.

Three different evidence strengths must be reported separately:

1. **Example agreement:** the engines return the same decision on a frozen list of requests.
2. **Bounded exhaustive agreement:** the engines agree on every request in a declared finite abstract domain.
3. **Formal equivalence:** a symbolic verification method proves equivalence under an explicit schema, theory and request-space model.

A pass at one level must never be described as a stronger level.

## 2. Verified baseline and current blocker

The current scientific-policy PR #5522 contains:
- 30 frozen Rust-baseline-derived policy cases;
- Cedar validation and execution for those 30 cases;
- OPA strict module checking and execution for those 30 plus eight additional controls;
- denominator/integrity checks over the frozen corpus.

The exact workflow result at time of this design:
- Cedar 30/30: passed;
- OPA 38/38: passed;
- frozen corpus and adapter-integrity checks: passed;
- Rust policy oracle: failed during compilation because the source tree declares and imports scientific_credentials, but the provider module file is missing;
- epistemic projection tests: skipped as a consequence.

This is not three-engine parity. The Rust reference remains unexecuted, and the 30/38 denominators are not interchangeable. See Mycelix issue #5189 and credential-registry resolution PR #4968.

The current policy boundary must be retained: signature validity, durable actor/key binding, key revocation/expiry, organization membership, replay protection, authority receipts and atomic commit remain in Rust and cannot be disabled or overridden by a policy DSL.

## 3. Industry capability comparison

### Cedar

Cedar is the authorization-specific reference:
- typed policy schema validation;
- default-deny authorization semantics and forbid/permit handling;
- a Lean formal model with proofs for core authorization and validation properties;
- differential testing between Lean and the production Rust validator;
- symbolic compilation and verification including equivalence/implication properties.

Relevant references:
- https://docs.cedarpolicy.com/policies/validation.html
- https://github.com/cedar-policy/cedar-spec/blob/main/cedar-lean/README.md
- https://docs.cedarpolicy.com/

The Mycelix target is not to claim we have surpassed Cedar's assurance. The useful experiment is whether Cedar's analyzer can produce meaningful permission-change counterexamples for the actual Mycelix request schema and constitutional gates.

### OPA / Rego

OPA is the mature general-purpose policy engine:
- strict policy/module checking;
- query evaluation and policy profiling;
- query preparation and partial evaluation;
- explicit benchmark tooling with allocations, latency distributions and query metrics.

Its own benchmarking documentation distinguishes query evaluation from loading/parsing/compilation. Keep these costs separate in all comparisons:
- https://www.openpolicyagent.org/docs/policy-performance
- https://openpolicyagent.org/projects

### CEL

CEL is a serious third candidate because it offers a portable, non-Turing-complete expression language and now has a formal-verification path:
- Rust cel crate version 0.15.0 was released 2026-10-06 and supplies a Rust parser/interpreter.
- CEL-Java's Formal Verification framework translates expressions/policies into SMT using Z3 and supports safety-invariant checks, satisfiability, validity, equivalence and counterexample generation.
- CEL Policy provides a structured YAML composition model that compiles to CEL ASTs; implementation support differs by language.

References:
- https://docs.rs/crate/cel/0.15.0
- https://github.com/cel-rust/cel-rust
- https://cel.dev/overview/cel-overview
- https://cel.dev/reference/api-reference

Important limitations to test:
- A Rust CEL interpreter passing a small set of examples does not prove conformance to the CEL specification.
- CEL-Java formal verification does not prove that the Rust interpreter has identical semantics.
- The Java verifier may be used as an independent authoring-time proof tool, while the Rust runtime adapter must separately pass CEL conformance vectors.
- Custom functions, missing/null attributes, error propagation, type-checking, timestamps and optional values must be frozen before formal results are trusted.

## 4. Request-space model

Freeze a versioned request schema and source-derived abstract domain before generating vectors.

The current Rust policy has eight actions and seven role values. The initial finite domain includes:

| Dimension | Abstract values |
|---|---|
| Action | All 8 declared scientific actions |
| Roles | All 2^7 subsets of the 7 current roles |
| Claim stream | Absent / present |
| Actor identity | Claim creator / different actor |
| Creator organization | Absent / present |
| Acting organization | Absent / equal / different |
| Attestation kind | Ordinary / independent replication |
| Target attestation | Absent / actor-authored / other-authored |

This is a deliberately finite abstraction, not all possible strings, IDs, timestamps, key material or organization graphs.

Action-specific pruning is allowed only where the source match branches make a dimension irrelevant. Every pruned dimension and reason must be recorded. At least one adversarial mutation must challenge each important pruning assumption.

The generated case ID binds:
- request semantic fields;
- baseline source commit and source blob;
- request-schema revision;
- generator revision;
- abstract-domain revision.

The expected decision is derived from the exact Rust source branch semantics and remains marked unqualified until an executable Rust oracle evaluates the vectors. Cedar/Rego/CEL agreement with a hand-transcribed model is useful but not a substitute for the Rust baseline.

## 5. Common engine input and output contract

Every engine receives the same normalized JSON request, schema snapshot and exact corpus revision.

Each engine adapter returns a normalized record:

~~~json
{
  "case_id": "AUTH-GEN-000001",
  "engine_id": "cedar",
  "engine_version": "pinned-version",
  "policy_digest": "sha256:...",
  "schema_digest": "sha256:...",
  "decision": "allow",
  "stable_rule_code": "CLAIM_OWNER_ATTACH_EVIDENCE",
  "evaluation_status": "completed",
  "error_code": null
}
~~~

The adapter must not normalize an evaluation error into an allow. Error, missing data and unsupported input are distinct statuses. A decision mismatch is reported with the full minimized request, engine versions and both result digests.

The common suite must contain:
- the existing 30 cases;
- the eight controls currently present only in the OPA test set;
- generated exhaustive abstract-domain cases;
- malformed/missing/null/wrong-type input cases;
- policy and schema mutation controls.

Do not combine different corpus denominators in an aggregate parity percentage.

## 6. Permission-diff and equivalence report

A policy update generates:

~~~json
{
  "analysis_schema": "mycelix-policy-diff-v1",
  "prior_policy_digest": "sha256:...",
  "candidate_policy_digest": "sha256:...",
  "request_space_digest": "sha256:...",
  "coverage_kind": "bounded_exhaustive",
  "prior_allow_candidate_deny": [],
  "prior_deny_candidate_allow": [],
  "unchanged_classes": [],
  "counterexamples": [],
  "non_delegable_invariants": {
    "status": "pass"
  },
  "conclusion": "equivalent"
}
~~~

Allowed conclusions:
- EquivalentWithinDeclaredDomain
- PermissionExpansion
- PermissionContraction
- MixedPermissionChange
- CounterexampleFound
- Unknown
- InvalidAnalysis

The report must state the exact entity/attribute assumptions and request-space boundary. A sample-based diff cannot be labeled exhaustive. If the symbolic theory cannot represent an attribute or function, return Unknown and explain why.

Cedar's symbolic verifier and CEL's formal verifier should be used where their supported theories permit. OPA/Rego receives the same concrete differential corpus and any supported static analysis; do not invent a universal equivalence claim for a tool that did not perform the proof.

## 7. Mutation programme

Generate deterministic mutants for:
- removal or inversion of every role condition;
- owner/creator identity comparison changes;
- absent/present claim precondition changes;
- independent-replication self-review guard removal;
- same-organization independence guard removal;
- target-attestation existence and ownership changes;
- default-deny and no-match fallback changes;
- missing vs explicit null semantics;
- action/resource scope widening;
- rule shadowing and overlapping allow/forbid rules.

Separate semantically equivalent mutants from non-equivalent mutants using symbolic analysis or independent bounded enumeration. Report both:
- mutation detection rate over known non-equivalent mutants;
- equivalent-mutant rejection rate, so the mutation score is not inflated by counting equivalent mutants as failures.

Targets before v0.2:
- at least 1,000 deterministic mutants;
- zero surviving known non-equivalent mutants affecting constitutional hard guards;
- at least 95% detection of the broader non-equivalent mutation set;
- no unclassified survivor silently treated as an equivalent mutant.

## 8. Benchmark protocol

For each engine measure separately:
- source parsing;
- schema/type checking;
- policy compilation/preparation;
- warm single-request evaluation;
- end-to-end request evaluation;
- policy update/reload;
- memory and allocations;
- concurrent throughput.

Workloads:
- 10, 100 and 1,000 policy rules;
- 1, 16 and 64 concurrent workers;
- common corpus and generated corpus;
- 1 KiB, 10 KiB and 100 KiB request shapes where applicable.

Report p50, p95, p99, throughput, memory, allocations, compilation time and policy update cost. Pin CPU, runtime, compiler, engine version, policy artifact, schema, corpus and request digest.

Correctness and security equivalence are hard gates before performance is compared.

## 9. Staged implementation

1. Keep #5522 frozen and immutable; do not silently change its 30-case corpus.
2. Complete the credential-registry protocol decision (#5189) and restore a truthful Rust oracle or explicitly authorize a new protocol/genesis with migration semantics.
3. Freeze the common 38-case suite so the same controls are executed by every engine.
4. Specify the abstract decision space and generator; retain source-derived expected outcomes as provisional until the Rust oracle runs.
5. Add CEL policy expression adapter and run upstream CEL conformance tests for all used constructs.
6. Add CEL-Java formal analysis as an independent proof lane; bind exact AST/environment/version.
7. Add Cedar/CEL policy-diff counterexample reports and bounded exhaustive cross-engine enumeration.
8. Add mutation testing and matched performance reports.

## 10. Claim ceiling

- Cedar schema validation is not proof that Mycelix's normative governance rules are correct.
- CEL AST equivalence is only as strong as the verifier's modeled environment and supported functions.
- Rust/Cedar/OPA/CEL example agreement is not universal equivalence.
- Bounded exhaustive agreement proves only the frozen finite abstract domain.
- A signature or receipt authenticates the policy artifact, not the legitimacy of its rule.
- Authorization to submit an attestation is not qualification as independent evidence.
- Evidence qualification is not proof that a scientific claim is true.

**Exit criterion:** reviewers can see every request-space assumption, every changed permission class and its counterexample, the proof strength achieved, and the exact security gates that remain outside the DSL.