# Scientific Authorization Policy Benchmark Protocol v0.1

Status: benchmark protocol for the frozen Cedar/OPA corpus and counterexample-shrink work. It does not supply the missing authentic Rust authorization oracle.

## 1. Separate the questions

There are four distinct benchmark questions. Never merge their results into one “policy engine is faster/correct” number.

1. **Decision semantics:** do pinned engines return their expected decisions for the declared pure-policy inputs?
2. **Metamorphic stability:** do semantics-preserving input transformations preserve decisions and determining-policy identifiers?
3. **Diagnostic quality:** can an engine mismatch be reproduced, minimized and bound to exact inputs and binaries?
4. **Performance:** what are warm evaluator cost, cold startup cost, batch throughput, memory and allocation behavior?

The actual Mycelix Rust authorization provider, signature/key/credential gates, membership, replay protection, receipt verification and durable commit remain outside the present Cedar/OPA pure-policy benchmark. Rust parity remains pending until authentic source and canonical v1 input/output semantics are recovered.

## 2. Frozen functional corpus

The current source-pinned abstract input corpus contains 62,464 rows. Preserve:
- exact case ID, source revision and Git blob;
- corpus and engine-input SHA-256;
- actor, action, resource, context and entity projections;
- explicit null/pending expected Rust decisions;
- engine version, policy/schema bytes and exact binary hashes.

The current metamorphic panel includes four invariance transformations:
- JSON object-member serialization order;
- insignificant JSON whitespace;
- Cedar entity order under unique UID constraints;
- addition of a unique, unreferenced, schema-shaped Actor.

It also includes one targeted negative control: remove the only contributor role from the unique absent-claim `claim_proposed` Rego projection. Exactly one case should change from Allow to Deny in the candidate policy. This is a known seeded control, not an authentic Rust expected decision.

Targets for the frozen finite panel:
- zero decision or determining-policy divergence on all four declared invariance transformations;
- exactly one Allow-to-Deny transition on the targeted negative control;
- zero missing, duplicated or remapped case IDs;
- zero engine errors silently converted into Deny.

A policy-ID divergence, decision divergence and evaluation error are distinct outcome dimensions.

## 3. Counterexample quality

A mismatch report is incomplete without exact case and execution provenance. The reducer records raw source digest, normalized JSON digest, predicate, protected identity paths, candidate trace and engine/policy/schema/configuration digests.

Engine-integrated shrinking preserves:
- original Cedar and OPA decisions;
- Cedar's determining-policy ID set;
- OPA's determining Rego rule ID;
- request principal/action/resource and source case identity;
- the policy-relevant Actor identity/roles and claim existence fields.

The seeded control currently asserts Cedar Allow under its claim-proposal permit and OPA Deny under `DEFAULT_DENY`. Every accepted reduction must preserve the same reasons. A completed run claims only deterministic 1-minimality in the declared JSON reduction neighborhood, never global minimality.

## 4. Fair engine performance measurements

Report these separately:

### Cedar in-process

The existing 62,464-row runner reports request/entity construction, authorization-only latency, end-to-end per-case latency, throughput and peak RSS separately. The first 256 cases warm the in-process engine and are excluded from measured latency distributions.

### OPA native query benchmark

The workflow runs OPA 1.22.0 `opa bench` for one source-bound representative input from each of the eight authorization actions. It uses native query benchmarking with metrics, allocation reporting and five independent benchmark repetitions. These measurements target repeated query evaluation rather than process startup or 62,464-case batch I/O.

The action-level OPA bench input is taken from the same paired corpus row used to define the action. Policy SHA-256, OPA binary SHA-256, source corpus SHA-256, tool version, OS, CPU count and raw benchmark JSON are published.

### OPA one-shot batch

The full OPA CLI evaluation over 62,464 cases includes process startup, input conversion, result serialization and file I/O. This is an end-to-end batch measure; it is not directly comparable to Cedar's warm in-process per-case authorization time.

### Counterexample minimization

The shrink summary separately records:
- Cedar request construction + authorization time per valid candidate;
- OPA one-shot candidate process time (including CLI startup and policy loading);
- candidate count, process invocation count and accepted reductions;
- total minimization wall time.

These timings characterize the shrink process, not Cedar-vs-OPA evaluator speed.

## 5. Performance protocol

For native engine measurements:
- pin compiler/toolchain, engine versions and binary hashes;
- use fixed source-bound inputs and policies;
- report warm-up policy and repeated-run count;
- report p50/p95/p99 where per-operation samples exist; for native `opa bench`, retain raw JSON and all repetitions;
- report allocations/bytes and memory where the engine exposes them;
- record CPU model, core count, OS, memory and thread configuration;
- keep parse/construction, evaluator, output conversion and full process costs separate.

Do not compare across different action distributions, policy semantics, hardware, compiler modes, memory limits, warm/cold states or input projections without explicitly qualifying the difference.

Initial performance objective: preserve all functional/assurance gates while reducing warm evaluator p95, allocations and memory on the same pinned inputs. Establish a baseline first; do not invent an absolute nanoseconds/op target before the measurements exist.

## 6. Industry comparison boundary

- OPA native `opa bench` is the appropriate Rego query-evaluation reference; full `opa eval` batch is a separate operational measure.
- Cedar in-process `authorization_latency` is the closer Cedar comparison to repeated native Rego query evaluation; its `construction_latency` is additional and should be reported rather than hidden.
- Compare full authorization products only when outer identity, signature, credential, membership, revocation, replay, audit and commit semantics are implemented equivalently. The current pure-policy benchmark does not cover those gates.

References:
- OPA CLI: https://www.openpolicyagent.org/docs/cli
- OPA policy performance: https://www.openpolicyagent.org/docs/policy-performance
- Cedar authorization: https://docs.cedarpolicy.com/auth/authorization.html

## 7. Claim ceiling

A successful result establishes only the pinned engine's behavior on the declared abstract corpus and metamorphic relations. A minimized seeded discrepancy validates the shrink workflow and policy reason binding. Neither establishes Cedar/OPA equivalence, authentic Rust parity, credential-registry correctness, full-system authorization safety or production latency.
