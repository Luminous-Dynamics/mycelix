# Agent Memory Leaderboard Add/Search Adapter v0.1

**Status:** pure-Rust, read-only evidence qualifier with a synthetic Smoke fixture. It does not host an Add/Search API, make a submission, run the official evaluator, or claim an AML result.

## Why an adapter now?

Agent Memory Leaderboard Cycle 2 published results on 19 November 2026. Its submission and formal-evaluation windows have closed. This work prepares a future-cycle integration and makes the public Add/Search protocol independently replayable; it does not enter Cycle 2 retroactively.

The benchmark separates Textual, Coding and Multimodal tasks. Their scores are not interchangeable. It uses participant-hosted Add/Search endpoints and a fixed benchmark-side Answer/evaluation/scoring pipeline. The participant must return retrievable memory candidates, not generated answers.

## Four independent qualification axes

1. **API compatibility:** Add/Search envelopes, scope, idempotency, durable-before-ack receipts, ordered candidates, Top K, and content-part rules.
2. **Model-policy eligibility:** component model identities satisfy the chosen release/division policy. A native HDC/CfC architecture can be evaluated in `native_unrestricted` mode, but this is not an official Open-source Methods division result.
3. **Release provenance:** exact release ID, evaluation pipeline, dataset bundle, question manifest, Answer model, scoring contract and API contract digests match the pinned release bundle.
4. **Evaluation coverage:** every expected question is accounted for, and a Full run is marked complete.

Only a `Full` run with all four axes eligible can be considered for an official score. `Smoke` proves integration only. `NativeUnrestricted` supports architectural research and can never be reported as an official AML score.

## Cycle 2 public policy snapshot

`benchmark-manifests/aml-cycle2-public-policy-v1.json` records the public release facts without inventing inaccessible release artifact hashes.

| Public contract | Cycle 2 value |
| --- | ---: |
| Search Top K | 100 |
| Textual conversation chunk | 20 messages / 2,000 words |
| Inline image size | 10 MiB per image |
| Multimodal Search response cap | 30 MiB |
| Open-source embedding component | `text-embedding-v4` |
| Open-source LLM-related component | `gpt-4o-mini` |
| Open-source reranker | Unrestricted |
| Commercial Products model restrictions | None |

The 20-message/2,000-word chunk rule is applied only to ordinary Textual conversation ingestion. Textual Streaming, Coding repository trajectories and Multimodal sequences have distinct typed ingestion modes and are not incorrectly rejected by that Textual-only ceiling. Exact per-release behaviour must be rechecked if a future AML cycle changes its protocol.

The local `max_add_attempts = 32` in the reference contract is a conservative adapter guardrail, **not** a claimed official AML protocol limit.

`AmlReleaseContract::cycle_2()` deliberately leaves the release ID and official artifact digests blank. The public policy webpage and leaderboard result pages do not themselves provide a complete pinned evaluation bundle. Until the actual official release manifest, pipeline, dataset, question set, Answer model, scoring and API contract digests are captured, release provenance is INDETERMINATE and an official-score claim is blocked.

## Add evidence

Each Add operation binds its stable request ID, sample/task/user, source-session digest, payload digest, typed ingestion mode, ordered content parts, and all captured attempts.

A retry must reuse the same logical request ID and operation scope. A successful HTTP 200 requires a durable-before-ack persistence receipt. The receipt binds:

- request ID and attempt number;
- sample, task and user scope;
- source-session and payload digests;
- monotonic durable sequence;
- commit and acknowledgement timestamps;
- independent storage-observer artifact digest.

The receipt digest is recomputed using the domain-separated `MYCELIX-AML-ADD-PERSISTENCE-RECEIPT-V0.1` encoding. This establishes structural field binding only. It does not authenticate the issuer, prove durable storage by itself, or replace independent storage instrumentation and trusted receipt verification.

A fully captured retry sequence exhausted without HTTP 200 is a known API failure. An absent attempt inventory is INDETERMINATE. An HTTP 200 without a matching receipt is FAIL.

## Search evidence

Every Search observation binds a stable question ID/digest and exact sample/task/user scope. Repeated transport attempts for one logical question are grouped rather than counted as separate questions, but query or scope changes across those attempts fail.

Each candidate must carry a stable memory ID, deterministic response order/rank, source Add request, source sample/task/user, content digest and non-empty ordered content parts. Candidates cannot be generated final answers. Every source Add must resolve to the same principal, sample and task. Missing source evidence is INDETERMINATE; an observed cross-scope candidate is FAIL.

If all candidates expose relevance scores, the returned order must be non-increasing by score. The adapter preserves raw list order; its `rank` is a zero-based normalized ordinal in the evidence envelope, not a claim about a vendor-specific response field.

## Multimodal evidence

Text and image parts are ordinal-preserving. Image parts must be inline Base64 data URIs, identify an `image/*` media type, carry a content digest, and fit the release's per-image and per-response limits. Remote image URLs are not silently treated as inline payload evidence. Multimodal limits are applied to each Search response, not summed across unrelated questions.

## Current competitive baselines and target ladder

Cycle 2's reported open-source and commercial leaders are track-specific:

| Track / division | Cycle 2 reference | Prospective Mycelix target |
| --- | ---: | ---: |
| Textual / open source | Resurface 61.22 | 63.22 initial; 65.0 stretch |
| Textual / commercial | MemoraX 69.26 | Compare only within this division |
| Coding / open source | InsightMem 63.33% Overall Resolved | 65.33% initial |
| Multimodal / open source | Khipu 43.26 | Reproduce exact task/evaluator first |

The Textual leaderboard also reports separate fact, relational, temporal, governance, personalization, rules/workflow, epistemic/privacy and streaming dimensions. A strong overall recall score must not conceal weak governance or privacy results. The Coding benchmark reports task resolution under Relevant/Noisy and new-feature/bug-fix strata. Multimodal includes relational, temporal, continual, personalization and evidence-governance dimensions. Do not combine these scores or infer one track's performance from another.

Targets are aspirations, not Mycelix results. A superiority claim requires the same release, dataset, task IDs, component policy, Top K, scoring pipeline and evaluator, plus an independently replayed run. The official Cycle 2 open-source policy constrains components; native HDC/CfC experiments that do not meet that policy remain a separate, valuable unrestricted research track.

## Run

```sh
cargo fmt --manifest-path research/collective-assurance/Cargo.toml --all -- --check
cargo test --locked --manifest-path research/collective-assurance/Cargo.toml
cargo clippy --locked --manifest-path research/collective-assurance/Cargo.toml --all-targets -- -D warnings
```

The JSON Schema and example fixture are under `schema/` and `fixtures/`. The fixture uses synthetic IDs/digests and `mode = smoke`; it is an executable-contract example, not leaderboard data and not an official result.

## Claim ceiling

This code is an ingestion/qualification layer, not a service, official submission, production privacy guarantee, or independent evaluator. Structural persistence receipts are not authenticated signatures. No official release hashes or Cycle 2 score are claimed. A complete future-cycle run must be separately pinned and replayed. Cost, p50/p95/p99 latency, token use, storage footprint and privacy governance should be reported as separate metrics alongside each benchmark track.

## Integration owners

- #6755: future-cycle AML API interoperability and release adapter.
- #6735 / #6742: audience, applicability and disclosure boundaries.
- #6730 / #5462 / #6753: longitudinal observer-trace privacy.
- #6719 / #6734: transitive lineage, revocation and descendant closure.
- #6686 / #6746 / #6749: coordination value and TeamBench reproduction.
- #2952 / #6700: actual evidence exposure and causal influence.
