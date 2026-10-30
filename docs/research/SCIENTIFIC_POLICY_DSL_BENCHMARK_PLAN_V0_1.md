# Mycelix policy-language benchmark plan v0.1

**Status:** Proposed measurement protocol; no latency or throughput result is claimed.  
**Semantic baseline:** \`DefaultScientificAuthorizationPolicy\` v1.0.0 from source revision \`061e34f24d406f19f6de0e61867305b6989f455f\`.  
**Corpus:** 30 authorization decisions, 10 evidence-projection cases, and 11 outer authority/commit gates.

## 1. Objective

Determine whether Cedar or OPA/Rego can improve policy authoring, validation,
analysis, explainability, deployment, or throughput without changing Mycelix's
security semantics.

This is not a contest to optimize an isolated boolean expression. The measured
system includes identity context projection, authorization, decision reasons,
receipt-policy identity, and the outer authority kernel. Report the pure policy
microbenchmark and the full governed append path separately.

Cedar is the directly relevant authorization-language baseline. Its official
documentation describes validation soundness: policies accepted by the schema
validator avoid most request-time evaluation errors for schema-conforming
requests, and the soundness property is formally proved in Lean and checked
against the Rust implementation through differential testing. Mycelix should
therefore treat Cedar validation as a strong quality baseline, not claim to have
surpassed it before formal analysis exists.

OPA is a mature general-purpose policy engine. Its documented \`opa bench\`
workflow measures query evaluation separately from policy loading and
compilation and reports percentiles, nanoseconds per operation, bytes per
operation, and allocations per operation. Use the same separation for every
candidate. Do not conflate an OPA CLI or sidecar round trip with pure Rego
evaluation.

## 2. What counts as correctness

| Layer | Frozen vectors | Required result |
|---|---:|---|
| Existing Rust authorization policy | 30 | Expected allow/deny, 30/30 |
| Cedar policy candidate | 30 | Exact parity with Rust, 30/30 |
| Rego policy candidate | 30 | Exact parity with Rust, 30/30 |
| Evidence projection | 10 | Exact six evidence-profile counters per case |
| Outer authority kernel | 11 | Fail-closed rejection/idempotency semantics preserved |
| Unknown actions and malformed context | Generated adversarial set | Deny/indeterminate; never implicit allow |

The pure policy corpus currently contains 18 allows and 12 denies. The projection
corpus is deliberately separate: permission to submit an event is not the same
as evidence qualification. The outer gates are deliberately separate as well:
no policy-engine candidate can make an invalid signature, unresolved actor,
revoked key, unauthorized organization, replay conflict, or broken authority
receipt acceptable.

### Required adversarial growth

The initial 30 decision vectors are a seed corpus, not proof of security.
Generate at least 1,000 deterministic single-fault mutations and a separately
reported compound-attack corpus. Cover:

- role removal/addition, unknown roles, duplicated roles, empty role sets;
- actor/creator substitution, missing creator metadata, stale authority revision;
- acting-organization removal, creator-organization substitution, unrelated org;
- missing action/context/resource attributes and unknown action identifiers;
- missing or withdrawn target attestations;
- exact retry, duplicate event ID with different signed bytes, expired/revoked key;
- receipt-time backdating, future-dated event, chain sequence gaps;
- policy-version swaps and changes that expand or contract permissions.

Publish the seed, generator revision, denominator, false allows, false denies,
and expected/observed diagnostic codes. Keep one-fault controls isolated so a
second violation cannot mask a broken invariant.

## 3. Fair performance methodology

Compare equivalent execution modes:

| Candidate | Primary in-process benchmark | Deployment benchmark |
|---|---|---|
| Native Rust | Call \`DefaultScientificAuthorizationPolicy::authorize\` directly with prebuilt typed input | Governed event append through the existing identity/receipt kernel |
| Cedar | Use the Cedar Rust API (\`PolicySet\`, \`Request\`, \`Entities\`, \`Authorizer\`) with the same normalized input | CLI and any service/sidecar path measured separately |
| OPA/Rego | Use \`opa bench\` against the frozen input/query workload; record the OPA version and policy bundle digest | Persistent local OPA server/SDK path measured separately from process startup and network overhead |

Do not compare a prebuilt Rust request against a newly spawned OPA process and
call the difference a language performance result. If only the CLI is available,
label that result as an end-to-end CLI measurement and do not use it to rank
engine throughput.

Measure these stages independently:

1. cold schema/policy parse, load, and compile;
2. schema or policy validation time;
3. identity/claim/context projection and serialization;
4. warm pure policy evaluation;
5. decision diagnostics/reason generation;
6. full governed append including identity lookup, signed authority receipt,
   receipt-chain checks, and store commit.

Run two workload families: the frozen 30-case semantic corpus and a generated
high-cardinality workload. The high-cardinality workload should scale actor
entities, claims, policies/rules, role counts, organization counts, and request
mix independently so that no engine gets a different workload by accident.

### Run matrix

| Axis | Required values |
|---|---|
| Policy/rule cardinality | 10, 100, 1,000 |
| Actor role count | 1, 3, 8, 16 |
| Organization set size | 0, 1, 4, 16 |
| Concurrency | 1, 16, 64 workers |
| Request mix | 50% allow / 50% deny; production-like mix reported separately |
| Warm repetitions | At least 30 samples per configuration after documented warm-up |
| Reported sizes | 1 KiB, 10 KiB and 100 KiB normalized input bundles where applicable |

Report p50, p95, p99, throughput, CPU time, peak RSS, allocations and bytes per
decision, policy load/compile time, and error rate. For cold-start measurements,
repeat fresh process launches separately from warm steady state. Retain raw
samples and the exact scripts, hardware, OS, Rust toolchain, Cedar CLI/API
version, OPA version, policy/schema/corpus digests, and commit SHA.

## 4. Proposed release gates

These are project targets—not an industry-established leaderboard or measured
results.

| Gate | Required threshold |
|---|---:|
| Rust oracle corpus | 30/30 expected decisions |
| Cedar parity | 30/30 exact decisions; zero evaluation errors |
| Rego parity | 30/30 exact decisions; zero evaluation errors |
| Evidence projection | 10/10 exact profile-counter vectors |
| Outer authority kernel | 11/11 preserved in independent security tests |
| Adversarial false allows | 0 / at least 1,000 deterministic one-fault mutations |
| Rego/Cedar unknown input behavior | 100% fail-closed |
| Reproducibility | Identical normalized inputs and policy digests produce identical decisions and stable reason codes |

For an engine to be eligible for deployment as an editable authorization
policy, all correctness gates are hard requirements. A faster engine with one
false allow is a failed candidate.

For performance adoption, first measure the Rust baseline. A provisional
engineering threshold is Cedar warm in-process p50 at or below 2x the Rust
oracle, and Rego warm evaluation at or below 3x, with no material p99 or memory
regression. These ratios are internal decision thresholds, not claims about
Cedar or OPA's published performance. Revise them only from measured workload
needs, not to make a preferred engine pass.

## 5. Beyond latency: capabilities that matter more than syntax

Measure the operational and governance features that current native Rust does
not provide as a complete authoring experience:

- **Policy diff:** exact request set whose decisions change between versions.
- **Permission expansion alarms:** any newly allowed request requires explicit
  review, rationale, and reviewer sign-off.
- **Constitutional invariant analysis:** an editable rule cannot disable
  identity binding, signature verification, key revocation, receipt continuity,
  or durable commit.
- **Replayability:** reproduce an old decision using the policy digest,
  authority snapshot, engine version, and normalized request.
- **Reason quality:** stable rule IDs, complete allow/deny reasons, and no
  sensitive attribute leakage.
- **Upgrade compatibility:** old receipts remain verifiable after policy edits;
  historical policy IDs/digests remain addressable.
- **Failure injection:** engine unavailable, malformed request, policy compile
  error, stale policy version, corrupted cache, timeout and partial rollout all
  deny safely.

A potentially differentiating product feature is **permission-diff before
deployment**: treat a policy update as a migration whose changed decisions
must be reviewed before it takes effect. Compare Cedar's native schema
validation and policy-analysis tools with an equivalent Mycelix workflow.
Do not claim superiority until a reviewer study and differential tests show
measurable improvement.

## 6. Reproducibility record

Each benchmark result must bind:

- repository and exact commit SHA;
- corpus JSON SHA-256 and schema SHA-256;
- policy source digest and normalized policy-engine configuration;
- Rust toolchain and dependency lock digest;
- Cedar CLI/API version, OPA Go module version, pinned Go toolchain, and Go module checksum database verification;
- host CPU, memory, operating system, and runtime settings;
- seed, warm-up, iteration count, concurrency and raw output digest.

Every report must distinguish \`implemented\`, \`validated\`, \`measured\`, and
\`proven\`. A committed benchmark harness is not a benchmark result. A passing
policy test suite is not a formal proof. A correct authorization decision is
not proof of identity, evidence quality, or scientific truth.
