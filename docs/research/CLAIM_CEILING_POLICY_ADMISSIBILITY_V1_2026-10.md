# Claim Ceiling Policy Admissibility V1

**Status:** implementation candidate; exact-head hosted qualification is required before calling this gate qualified.  
**Parent:** [Claim Ceiling Meet Reference V1 / PR #5249](https://github.com/Luminous-Dynamics/mycelix/pull/5249), [EPI-FABRIC-005 / issue #3475](https://github.com/Luminous-Dynamics/mycelix/issues/3475).  
**Claim ceiling:** policy-profile fit only. No evidence admission, factual truth, or action authority.

## Paradigm shift

A meet is mathematically conservative but policy-blind. If two inputs contain the same forbidden claim atom, their intersection can preserve it:

`meet(A, A) = A`

That is correct algebra and still potentially an unsafe policy result. Therefore **intersection and admissibility are separate predicates**:

1. the claim-ceiling algebra computes what the two inputs share;
2. the policy-profile gate checks whether the complete claim atom and all its scopes fit a versioned allowlist.

The gate validates the whole coupled atom—predicate, evidence class, derivation, intended use—not four independent fields. This prevents invalid cross-products such as taking an allowed predicate from one atom, an evidence class from another, and an operational use from a third.

## Policy profile semantics

A `ClaimPolicyProfileV1` contains a stable profile identity and a `maximum_ceiling`. Candidate ceilings are admissible only when:

- every full claim atom is in the profile's explicit allowed set;
- subject, context, time-window and action scopes are contained in the profile's maximum scopes;
- all policy-required qualifications, independence controls, and provenance requirements remain present.

The same `is_no_broader_than` relation used by the meet reference serves as a defence-in-depth invariant. A separate dimension audit emits stable violation codes so a rejection is explainable rather than reduced to one boolean.

A profile allowing zero action scopes can still admit a research or display ceiling, but a pass **never grants authority**. The result is `admissible_within_profile`, not “truth”, “admitted evidence” or “authorized”.

## Frozen controls

The positive fixture is fully inside the measurement/research profile. The negative fixture mutates all eight audited dimensions: it adds an ownership claim for operational execution, an unregistered subject and context, widens the temporal interval, adds an action scope, and drops mandatory qualification, independence and provenance requirements.

The Rust tests additionally demonstrate the key separation directly: a pure meet of two identical forbidden candidate ceilings preserves their shared atom, while the independent policy-profile check rejects it.

## Receipt and reproducibility

The policy receipt binds:
- raw request SHA-256;
- serialized policy-profile and candidate-ceiling SHA-256;
- request- and receipt-schema digests;
- exact source commit when available;
- a deterministic disposition and sorted, stable violation records.

A pinned receipt is emitted only when the CLI's source-commit argument matches the checked-out commit and the tracked source tree is clean. The dedicated workflow runs formatting, Rust tests, strict Clippy, byte-identical repeat runs, and jq-level positive/negative receipt checks.

## Proposed acceptance gates

| Gate | Target |
|---|---|
| Policy scope | 100% rejection of frozen mutations that widen any one permitted dimension or drop a mandatory control |
| False acceptance | Zero forbidden coupled claim atoms accepted in the frozen policy corpus |
| Explainability | 100% of rejected fixtures emit the expected dimension code and affected value |
| Reproducibility | Byte-identical receipts for identical input, schema, executable, and pinned source |
| Portability | Independent verifier agrees on disposition, violated dimensions, raw input digest, and no-authority fields |
| Scaling | Benchmark 10, 100, 1,000, and 10,000 policy atoms/scopes; report p50/p95/max and memory before setting an SLO |

These are proposed gates, not an industry certification or a claim that the policy vocabulary is universally correct. The allowlist remains policy-author responsibility. Real integrations must bind profile approval, generation and revocation semantics through Mycelix's canonical authority/admission boundary.

## Nonclaims

A passing policy check does not authenticate its publisher, establish source independence, prove the claim true, verify trusted time, admit the evidence into canonical state, or authorize an operation. It proves only that the serialized candidate ceiling fits the separately digest-pinned finite profile under this implementation. The expected digest is still only as trustworthy as the external policy-delivery/configuration channel.

## Policy pinning correction


6. **The approved policy is a separately pinned artifact.** The candidate request does not carry its own policy profile. The CLI reads policy bytes from a separate file, requires an expected SHA-256 supplied through an out-of-band trusted channel, and returns a structured rejection when the profile bytes do not match that pin. Hashing an arbitrary caller-provided policy and trusting that hash would only prove self-consistency, not policy approval.
