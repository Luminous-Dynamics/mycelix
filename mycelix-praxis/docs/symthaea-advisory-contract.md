# Praxis / Symthaea Advisory Integration Contract v1

**Status:** Proposal and contract-validator prototype  
**Date:** 2026-10-11  
**Scope:** `mycelix-praxis` in `Luminous-Dynamics/mycelix`

## Decision

Use Symthaea as an optional, replaceable **pedagogical advisor**, not as an unrestricted tutor agent or source of educational truth. Praxis owns learner consent, curriculum scope, assessment policy, learner state, and every consequential write. A Symthaea response may select an allowed next-step suggestion; it cannot change a grade, mastery estimate, credential, curriculum enrollment, privacy setting, progress record, or DHT entry.

The Rust contract and bounded wire parser are implemented in
`crates/praxis-core/src/symthaea_advisory.rs`. The parser rejects raw response
bodies above 16 KiB before JSON deserialization, then applies the strict
unknown-field-rejecting wire type. The semantic validator separately checks
request scope and trusted allowlists. This is **not** a live Symthaea transport,
is not wired into the Leptos interface, and is not evidence that Symthaea
improves learning outcomes.

## Packaging and transport boundary

A source review of the standalone Symthaea repository at commit
[`b9af191f380d635f0459b24284c1d1fd6a5516aa`](https://github.com/Luminous-Dynamics/symthaea/commit/b9af191f380d635f0459b24284c1d1fd6a5516aa)
shows that its root package is part of a multi-crate workspace, while
`symthaea-core` has its own non-trivial dependency set and default
`simd` / `parallel` features. Praxis `praxis-core` is shared with Holochain
zomes, so pulling the full Symthaea facade into `praxis-core` would couple a
small protocol/wire crate to a much larger runtime and could contaminate WASM
dependency resolution.

**Do not add a Symthaea dependency to `praxis-core` or the Holochain guest
zomes.** Keep this schema and validator lightweight. When a real adapter is
built, put it behind a separate host-only boundary and pin the exact Symthaea
source commit, features, toolchain, and dependency lock. Prefer a local sidecar
for a desktop/offline deployment or an explicitly opted-in authenticated
service for browser deployments. Do not use an unauthenticated loopback HTTP
endpoint as a shortcut. The adapter must use `parse_symthaea_advice_response` (which enforces a 16 KiB
pre-deserialization limit), authenticate its process/transport independently,
hash the exact request bytes it sends using `digest_advisory_request_bytes`, and
preserve the standard deterministic fallback when no advisor is available. The
parser guard does not provide transport timeouts or process authentication,
and it receives an already-buffered byte slice: the host transport must enforce
the same body ceiling while streaming so a huge response is not fully allocated
before this parser rejects it.

This is a deployment recommendation derived from source inspection, not an
implemented transport or proof that the current Symthaea facade can yet satisfy
the Praxis tutoring contract.

### Compatibility check against the inspected Symthaea API

The inspected [Symthaea API router at the pinned source revision](https://github.com/Luminous-Dynamics/symthaea/blob/b9af191f380d635f0459b24284c1d1fd6a5516aa/src/api/mod.rs)
is a benchmark API: its route set covers benchmark submission, result retrieval,
leaderboards, datasets, comparison, dimensional sweeps, metrics, audit events,
and health. It does **not** expose the finite, request-digest-bound learning
advice contract defined here. In particular, Praxis must not call
`POST /v1/submit` and treat a benchmark submission as a tutoring-advice request.

The inspected [API binary entrypoint at the same revision](https://github.com/Luminous-Dynamics/symthaea/blob/b9af191f380d635f0459b24284c1d1fd6a5516aa/src/bin/symthaea-api.rs)
defaults to loopback, but an absent bearer token permits requests in its
development-mode configuration. That is another reason not to attach learner
data to this general-purpose API by convenience or assume loopback implies
authentication. This review did not establish a contract-compatible advice
endpoint. A future integration needs a purpose-built endpoint or local sidecar
that explicitly accepts this request schema and returns the response schema,
with authenticated process/transport, streaming body caps, bounded timeouts,
explicit consent/privacy handling, and deterministic failure fallback. Pin and
review that service revision and its exact enabled features before connecting
it to Praxis.

## Host-only bounded reader

The workspace now includes `crates/praxis-symthaea-adapter`, a native-only
boundary with no Symthaea-runtime or HTTP-client dependency. It depends on
`praxis-core` only for the versioned wire type and strict JSON parser. A
`compile_error!` prevents this crate from being built for WASM targets, and no
guest zome depends on it.

`read_bounded_symthaea_advice(reader)` wraps the supplied `Read` stream with
Rust's `Read::take(MAX_ADVISORY_RESPONSE_BYTES + 1)`, reads at most 16,385
bytes, and rejects the result if it exceeds the 16 KiB contract limit. Only a
body within the limit is passed to JSON deserialization. This follows Rust
standard-library reader limiting semantics and addresses oversized payload
allocation at the host-reader boundary; see the
[Rust `Read::take` documentation](https://doc.rust-lang.org/stable/std/io/trait.Read.html#method.take)
and OWASP's [API4:2023 unrestricted resource consumption guidance](https://api-security.owasp.org/editions/2023/en/0xa4-unrestricted-resource-consumption/).

The host adapter remains a **bounded decoder**, not a live transport. The
supplied reader must still have appropriate authentication and read-timeout
behavior. Prefer
`PreparedAdvisoryRequest::read_and_validate_response_once(reader, advisor_enabled, sandbox_active)`.
It reads within the limit, parses the strict response, validates it against the
same frozen request scope, and allows only one received response for the
prepared request. A reader/transport error leaves the prepared request
retryable, so callers can resend the exact same body and digest; after a
bounded response is received (even malformed or oversized), a second response
is rejected. Rejections fail closed. The lower-level
`read_bounded_symthaea_advice` function returns an untrusted wire object, and
`read_bounded_and_validate_symthaea_advice(reader, context)` remains available
for explicit custom contexts but does not supply the prepared-request one-shot
guard. Neither should replace the preferred one-shot path.
Regression tests assert that an oversized stream is read only through the
maximum-plus-one byte, exact-limit input reaches the JSON parser, malformed
bodies and I/O failures fail closed, and trusted validation accepts only
allowlisted suggestions.

## Why this boundary

The OECD's *Digital Education Outlook 2026* reports that generative AI can improve task output while excessive dependence can reduce metacognitive engagement and durable learning. Systems designed around explicit pedagogical strategies show more promise than unconstrained answer generation. A 2025 systematic review of 28 K–12 intelligent-tutoring studies covering 4,597 learners found generally positive effects, but the benefits were often smaller when compared with other intelligent tutoring systems, and the evidence base needs longer, more diverse evaluations.

Symthaea's own project description calls the architecture experimental and explicitly does not claim that its consciousness-related indicators prove consciousness. Neither Phi-like metrics nor derived neuromodulator proxies should be treated as validated measures of a learner's stress, focus, motivation, ability, or readiness.

References:
- OECD (2026), https://www.oecd.org/en/publications/oecd-digital-education-outlook-2026_062a7394-en.html
- Létourneau et al. (2025), https://doi.org/10.1038/s41539-025-00320-7
- Symthaea project status and evidence table, https://github.com/Luminous-Dynamics/symthaea

## Architectural alignment with the existing Praxis evidence work

Do not introduce a parallel learning-evidence, mastery, or credential framework. The ongoing PRAX-EVIDENCE / PRAX-ESTIMATE / PRAX-POL sequence explicitly separates observations, admitted evidence, estimates, analysis, credentials, and authorization. Once those contracts are available on the target base, the Symthaea adapter should consume their evidence-bound, scope-specific projections—not invent provenance from legacy scalar fields or model-generated explanations.

Until that migration is available on the current base:
- treat the advisor contract as an isolated prototype;
- do not connect it to the legacy mutable `SkillMastery` path as authority;
- do not materialize new Holochain entry types, links, public learning records, or DNA changes;
- do not make credential or employer decisions from advisor output.

## Contract invariants

1. **Opt-in:** reject advice unless the learner or appropriately authorized configuration enables it. No consent means no advisor call.
2. **Sandbox wins:** sandbox mode rejects all model-driven adaptation.
3. **Exact scope:** require exact configured model ID/version, skill ID, curriculum revision, and request digest. The request digest must be canonical `blake3:` plus 64 lowercase hexadecimal characters.
4. **Finite actions:** the response uses a bounded enum of suggestions. It cannot mutate authoritative state.
5. **Trusted action allowlist:** every response action must appear in an allowlist assembled by Praxis for this exact turn. The model cannot enable an action by choosing it in its own output. A break option is omitted from the allowlist unless a trusted learner interaction explicitly requests it.
6. **Trusted reason allowlist:** every reason code must be independently supported by the trusted request/evidence context. The model cannot claim that the learner requested help or a break simply by returning that enum. Action/reason compatibility is checked as an additional rule, not as a replacement for the trusted allowlists.
7. **Grounded references:** content IDs must be from the trusted allowlist for this request. Recommendations requiring content grounding are rejected if they provide no references. An allowed ID proves only that an existing item was named; it does not prove a generated explanation is true.
8. **Learner-authorized rest:** a break may be suggested only when the trusted context explicitly permits that action following the learner's request. Inferred affect, Phi-like indicators, or proxy neuromodulators cannot compel breaks or label the learner's mental state.
9. **Stale means discard:** mismatch of skill, curriculum revision, model version, or request digest makes the response unusable.
10. **Confidence is not authority:** model-reported confidence is uncalibrated metadata; it does not establish correctness, mastery, or eligibility.
11. **Local-first and data-minimized:** send only the minimum observation needed. Do not transmit raw answer histories, child identity, inferred affect, demographics, or an entire learner profile by default. Prefer local inference; a remote provider requires explicit disclosure and consent.
12. **Graceful absence:** service absence, invalid output, timeout, or unsupported version falls back to the normal Praxis learning path without blocking a lesson. Bound raw response bytes before deserialization.
13. **Learner choice:** show an accepted suggestion as an invitation with accept/dismiss controls. Never execute an action automatically merely because the model proposed it.
14. **Independent learning evidence:** separately record the learner's response and update evidence-derived projections using the authorized Praxis pathway. The model does not record an attempt or update mastery on behalf of the learner.

## Request and response binding

The host-only adapter now builds a deterministic, minimized v1 request body from a fresh opaque request nonce, trusted model/version, skill and curriculum scope, finite action/reason allowlists, and at most eight allowlisted content IDs. The nonce is a canonical lowercase-hex 128-bit value generated automatically by the public host-only request builder using the operating system's cryptographic random source. If secure randomness fails, preparation fails closed. Each new exchange gets a new nonce; retain and resend the same `PreparedAdvisoryRequest` bytes/digest for a retry after a transport/reader failure instead of preparing a replacement request. This makes otherwise-identical request bodies—and therefore their BLAKE3 digests—different across turns. A nonce is not authentication or a standalone replay defense, and the caller must re-check that the response is still for the current request before display. The preferred prepared-request response method rejects a second response for a request, but custom-context lower-level APIs do not provide that one-shot guard. The nonce is not learner identity or model-generated content.

The builder sorts content IDs and finite enum lists into protocol order, rejects duplicate or invalid policy values, caps the serialized body at 16 KiB, and computes the BLAKE3 digest over the **final exact body bytes**. `PreparedAdvisoryRequest` keeps the immutable bytes, digest, and trusted validation scope together. Callers obtain the exact body via `body_bytes()`, the digest via `input_digest()`, and use `read_and_validate_response_once(reader, advisor_enabled, sandbox_active)` to validate one bounded response against that same frozen scope without exposing a reusable context.

Send the body bytes unchanged. Convey the digest as authenticated transport metadata (for example, a request header); do not append it to the body being hashed, which would create a self-referential digest. On response, the preferred host API combines streaming-size enforcement, strict JSON decoding, and validation against the same frozen request scope. The digest is freshness binding only, not model authentication or proof of correctness. Authenticate the adapter/transport independently and recheck context immediately before displaying the suggestion.

The request intentionally does not include learner identity, raw answer history, demographics, or inferred affect. The eligible reason codes are the trusted evidence projection available for the current turn; the builder is a serialization and scoping boundary, not itself the authority that determines which evidence is true.

### Native entropy and the workspace's `getrandom/custom` feature

The Praxis workspace enables `getrandom` 0.2's `custom` feature for unsupported-target/WASM compatibility. The host adapter now directly pins `getrandom = "=0.2.17"`; Cargo should unify the workspace's compatible `0.2` dependency on that reviewed patch while preserving additive features. In the reviewed `getrandom` 0.2.17 implementation, custom registration affects targets that otherwise lack a supported backend; supported OS targets continue to use their normal secure platform implementation. See the [pinned custom-implementation notes in the 0.2.17 source](https://github.com/rust-random/getrandom/blob/v0.2.17/src/lib.rs#L124-L134).

This repository currently has no committed `Cargo.lock` at the Praxis workspace or repository root. The direct nonce dependency is therefore pinned, but the full transitive graph is not yet reproducibly locked; exact Cargo resolution and feature unification still require CI/build qualification. The adapter fails closed if the entropy call returns an error. This source review is not a substitute for running the exact native build and test suite.

The response contract uses a finite action and reason vocabulary instead of allowing model-generated free text to control system behavior. When an explanation is needed, retrieve from reviewed curriculum sources, show provenance, and continue to treat generated prose as unverified until independently checked.

## Suggested interaction loop

1. Praxis deterministically selects actions eligible for the current learning context.
2. Symthaea may rank or choose among only those actions and return typed reason codes plus allowlisted content references.
3. The adapter validates the output with `validate_symthaea_advice`.
4. Praxis discards stale responses and renders a localized optional suggestion.
5. Only the learner's explicit choice triggers the supported Praxis action.
6. Praxis separately records the actual attempt and updates authorized evidence projections.
7. If Symthaea is unavailable or its output fails validation, Praxis continues with its ordinary deterministic learning pathway.

## Existing legacy adaptive path: pre-integration blocker

The current Leptos source still contains a separate legacy path that this
contract-only PR does not change:

- `apps/leptos/src/learning_engine.rs` derives a field named `cortisol` from
  Spore neuromodulator proxies, classifies values above a threshold as
  `NeedsBreak`, maps that state to `TakeBreak`, and can override the proposed
  action when a session interval has elapsed.
- `apps/leptos/src/cognitive_adaptivity.rs` also derives a stress estimate and
  has a configurable teacher/parent threshold that creates a break suggestion.

These are heuristics in the application, not validated measurements of
cortisol, distress, attention, cognition, or a learner's need for rest. Their
current source comments acknowledge the signal mapping is approximate, but the
student-facing wording and action semantics still risk overstating what the
system knows.

**Do not route Symthaea output into this legacy path yet.** Before live
integration, make rest behavior an explicit learner-visible option except for
clearly documented, human-configured safeguarding policies; use plain language
that does not diagnose mental states; distinguish elapsed-session reminders
from cognitive inference; and add tests that show declining a suggestion does
not block learning or reduce autonomy. Any child safeguarding exception must
be configured by an authorized human, explained to the learner, narrowly scoped,
and reviewed independently. This follow-up is separate from the advisory
contract prototype and is not claimed as fixed here.

## Evaluation before enabling by default

Do not promote the integration based on engagement, chat volume, subjective fluency, Phi, or task-completion speed alone. First run deterministic offline scenarios and compare the advisor with the existing baseline. Then use a consent-based, appropriately reviewed evaluation to measure:
- immediate performance and delayed retention;
- transfer to unseen problems and unaided retrieval;
- quality of learner explanations and use of hints;
- unsupported-source / hallucination rate;
- results across languages, accessibility needs, and device/connectivity conditions;
- fallback correctness, privacy boundaries, and stale-response rejection.

Freeze scenario manifests and exact inputs; report failures as well as aggregate results. Preserve educator oversight for age suitability, curriculum alignment, and high-stakes decisions.

## Implementation status and next gates

Implemented in this branch:
- versioned typed suggestion and reason enums;
- exact model/version, skill, curriculum revision, and input-digest checks;
- opt-in and sandbox rejection;
- trusted per-turn action and reason allowlists, preventing model output from self-authorizing a break or fabricating a learner-request reason;
- canonical BLAKE3 input-digest format, `digest_advisory_request_bytes` over the exact bytes sent, and exact request binding;
- bounds on confidence fields and content references;
- allowlist/duplicate/grounding validation;
- action/reason pairing rules (including learner-requested breaks only);
- strict request and response JSON Schemas that reject unknown fields and are checked against the actual Rust wire fields, enums, and request-list bounds;
- an independent standard-library Python checker for actual Rust request/response field parity, schema/Rust enum parity, request example shape/order, response/request bounds, trust gates, and the required adversarial test inventory;
- a focused GitHub Actions workflow that checks out the exact PR head and runs the schema checker, formatting/tests for both advisory crates, and clippy, including while the PR is a draft;
- exact-head CI dependency qualification that resolves a Cargo lockfile once per run, switches metadata/tests/Clippy to `--locked`, and uploads the generated lockfile, full resolved Cargo metadata, and source/toolchain/lock fingerprint for independent inspection (per-run consistency only; not cross-run pinning);
- a 16 KiB bounded JSON parser in `praxis-core` plus a native-only host reader that caps streaming reads at 16,385 bytes before rejecting oversize bodies;
- a preferred prepared-request method that bounds the response, validates against the frozen request scope, and rejects a second response for the same prepared request; reader failures remain retryable with the exact same request bytes/digest;
- deterministic minimized request preparation with exact-body BLAKE3 digest, canonical allowlist ordering, duplicate rejection, pre-copy action/reason list bounds tied to the finite enum counts, and response-validation context frozen to the same prepared scope;
- twenty-seven `praxis-core` regression tests and twenty-six host-adapter tests covering valid decoding, oversized and exact-limit streams, malformed JSON, typed entropy failure, fresh nonce generation, retry after reader failure, cross-turn stale-response rejection, rejection of second responses after accepted or oversized bodies, trusted-policy acceptance/rejection, deterministic request bytes/digest, invalid nonce rejection, duplicate inputs, invalid identifiers, and content/action/reason allowlist bounds.

Not yet implemented or qualified:
- live Symthaea service/process transport or production call site (the host crate currently provides request preparation and bounded read/validate primitives, not a network or local-process client);
- transport/process authentication, configured timeouts, and production call-site wiring to the bounded reader;
- linkage to the evidence-bound PRAX projection pipeline;
- UI accept/dismiss wiring and accessibility checks;
- local inference/remote-provider privacy enforcement;
- a committed `mycelix-praxis/Cargo.lock` that freezes dependency resolution across distinct CI runs (the generated per-run lock artifact is evidence, not a repository-level pin);\n- Rust build/tests, formatter, clippy, workflow execution, runtime, or learning-outcome qualification.

The claim ceiling for this branch is **contract plus host-side request/response boundary prototype only**. No live integration, educational-effectiveness result, validated cognitive-state inference, credential authority, or production readiness is claimed.
