# MYC-INT-006I — Implementation-neutral I0 candidate protocol

Status: protocol/evaluation contract. This document does not establish that any implementation passes I0 or that any runtime is preferred.

## Why this exists

The first executable conventional candidate in MYC-INT-006F correctly separated candidate execution from the oracle, but its sanitized input identity was named for that first implementation family:

```text
myc-int-006f-conventional-candidate-input
```

That is acceptable as historical generation-1 evidence, but it is the wrong long-term interoperability boundary.

```text
benchmark input identity
!= implementation identity
```

A conventional service, a Mycelix/Holochain node, a hybrid implementation, and an independent conformer should receive the same benchmark-owned candidate-input protocol and emit the same benchmark-owned candidate-result protocol.

Do not rewrite or relabel historical 006F/006H evidence. This is a new protocol generation.

## Files

- `myc-int-006i-i0-candidate-input.schema.json`
- `myc-int-006i-i0-candidate-results.schema.json`

Both schemas use JSON Schema Draft 2020-12 and reject unknown fields by default.

## Neutral candidate input

Canonical identity:

```text
candidate_input_id      = myc-int-i0-candidate-input
candidate_input_version = 1.0.0
profile                 = runtime-neutral-candidate-input-v1
```

The input contains only:

- corpus and stimulus identity/version;
- SHA-256 commitments to the source corpus and stimulus bytes;
- semantic subject ID, kind and semantic reference;
- executable stimulus operation and conditions.

It contains no implementation family.

It also contains no:

```text
expected_disposition
reason_code
oracle
assertion
predicate
```

because those belong only on the evaluator side.

A source commitment is not an authority or an oracle leak. It binds the sanitized projection to exact source bytes while leaving the candidate unable to inspect oracle semantics through this protocol.

## Neutral candidate results

Canonical identity:

```text
result_protocol_id      = myc-int-i0-candidate-results
result_protocol_version = 1.0.0
profile                 = runtime-neutral-candidate-results-v1
```

The implementation identifies itself separately:

```text
implementation:
  implementation_id
  family
  version
```

This keeps:

```text
result protocol
!= implementation identity
```

The result also binds itself to the exact candidate-input bytes through `candidate_input_sha256`.

## Per-case result

Every result reports:

```text
case_id
+ disposition
+ implementation_rule
+ effect_count
+ optional non-authoritative tags
+ generic semantic facts
```

`implementation_rule` is deliberately opaque to the evaluator. A SQLite implementation might name a local state-machine rule; a Holochain implementation might name a validation/coordinator path; another conformer can use its own stable identifier.

The evaluator judges the shared disposition/facts contract, not whether implementations use the same internals.

## Semantic fact registry

The v1 registry is intentionally small and directly derived from the existing I0 oracle requirements:

```text
distinct_semantic_subjects
effect_authority_granted
authorization_subject_bound
source_schema_preserved
provenance_preserved
unknown_state_preserved
conflict_preserved
logical_effect_count
historical_mutation
review_candidate_created
local_authority_granted
observation_promotion
receipt_promotion
outcome_promotion
translation_loss_declared
idempotent_replay
expired_authority_rejected
stale_schema_rejected
```

These are benchmark-observable semantic facts.

They are not storage instructions and do not prescribe SQL tables, Holochain entry/action layouts, event topics, caches, logs or transport implementation.

## Identity boundaries

```text
candidate input ID
!= implementation ID

semantic subject ID
!= SQL row ID
!= Holochain ActionHash
!= transport message ID

implementation rule ID
!= oracle predicate

tag
!= semantic fact
```

Tags are diagnostic only. They cannot satisfy an oracle assertion.

## Cross-record checks

JSON Schema cannot prove every cross-record invariant. A follow-on validator/migration tranche should additionally require:

- candidate case set exactly equals the bound stimulus case set;
- every candidate subject operand exists;
- result case IDs are unique and exactly equal the candidate-input case set;
- result `candidate_input_sha256` matches the exact bytes consumed;
- required semantic facts for the selected evaluator profile are present;
- no result was produced for an unexecuted case.

## Evolution

Historical generation-1 identities remain historical:

```text
006F conventional-specific candidate-input identity
006G generation-1 result profile
006H generation-1 qualification manifest
```

They are not silently renamed to 006I.

The next conventional migration should:

1. emit the 006I neutral candidate input;
2. consume that exact neutral input;
3. emit the 006I neutral result protocol with an explicit conventional implementation descriptor;
4. prove semantic equivalence to the existing conventional generation under I0;
5. create a new qualification generation rather than rewriting previous evidence.

A future Mycelix/Holochain candidate should consume the same 006I candidate-input bytes and emit the same 006I result protocol.

## Comparison boundary

Only after two or more independent implementations consume the same neutral input is it meaningful to compare implementation behavior under one workload.

```text
same prose scenario
!= same benchmark

same neutral candidate-input bytes
+ same evaluator generation
= comparable semantic workload
```

Even then, a comparative result does not establish universal architectural superiority.

## Relationship

- #3152 / MYC-INT-006C — frozen I0 oracle corpus
- #3154 / MYC-INT-006D — strict corpus validator
- #3161 / MYC-INT-006E — executable stimulus / oracle separation
- #3163 / MYC-INT-006F — first oracle-blind conventional candidate
- #3165 / MYC-INT-006G — separate semantic evaluator
- #3167 / MYC-INT-006H — process-separated qualification orchestration
- #3169 / MYC-INT-006HQ — never-merge exact qualifier
- #3170 — planning issue for this protocol generation

## Nonclaims

This protocol does not establish that conventional, Holochain, hybrid or any other implementation passes I0. It does not qualify production behavior, security, scalability or Integral compatibility beyond the synthetic benchmark semantics. It does not select or recommend a technology stack.
