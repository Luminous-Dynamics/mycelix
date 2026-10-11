# Praxis / Symthaea Advisory Integration Contract v1

**Status:** Proposal and contract-validator prototype  
**Date:** 2026-10-11  
**Scope:** `mycelix-praxis` in `Luminous-Dynamics/mycelix`

## Decision

Use Symthaea as an optional, replaceable **pedagogical advisor**, not as an unrestricted tutor agent or source of educational truth. Praxis owns learner consent, curriculum scope, assessment policy, learner state, and every consequential write. A Symthaea response may select an allowed next-step suggestion; it cannot change a grade, mastery estimate, credential, curriculum enrollment, privacy setting, progress record, or DHT entry.

The initial Rust contract is implemented in
`crates/praxis-core/src/symthaea_advisory.rs`. It validates response shape and bindings. It is **not** a live Symthaea transport, is not wired into the Leptos interface, and is not evidence that Symthaea improves learning outcomes.

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
3. **Exact scope:** require exact configured model ID/version, skill ID, curriculum revision, and request digest.
4. **Finite actions:** the response uses a bounded enum of suggestions. It cannot mutate authoritative state.
5. **Grounded references:** content IDs must be from the trusted allowlist for this request. Recommendations requiring content grounding are rejected if they provide no references. An allowed ID proves only that an existing item was named; it does not prove a generated explanation is true.
6. **Learner-authorized rest:** a break may be suggested only when the learner explicitly requested one. Inferred affect, Phi-like indicators, or proxy neuromodulators cannot compel breaks or label the learner's mental state.
7. **Stale means discard:** mismatch of skill, curriculum revision, model version, or request digest makes the response unusable.
8. **Confidence is not authority:** model-reported confidence is uncalibrated metadata; it does not establish correctness, mastery, or eligibility.
9. **Local-first and data-minimized:** send only the minimum observation needed. Do not transmit raw answer histories, child identity, inferred affect, demographics, or an entire learner profile by default. Prefer local inference; a remote provider requires explicit disclosure and consent.
10. **Graceful absence:** service absence, invalid output, timeout, or unsupported version falls back to the normal Praxis learning path without blocking a lesson. Bound raw response bytes before deserialization.
11. **Learner choice:** show an accepted suggestion as an invitation with accept/dismiss controls. Never execute an action automatically merely because the model proposed it.
12. **Independent learning evidence:** separately record the learner's response and update evidence-derived projections using the authorized Praxis pathway. The model does not record an attempt or update mastery on behalf of the learner.

## Request and response binding

The trusted Praxis adapter should assemble a canonical request containing:
- schema version;
- current curriculum revision and capability/skill identifier;
- minimum necessary, scope-specific evidence from the authorized learning-evidence pathway;
- allowlisted content references and the finite actions available for this turn;
- an input digest computed over the exact canonical request.

The digest binds a response to its request and helps reject stale output. It is not model authentication or proof of correctness. Authenticate the adapter/transport independently and recheck context immediately before displaying the suggestion.

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
- bounds on confidence fields and content references;
- allowlist/duplicate/grounding validation;
- action/reason pairing rules (including learner-requested breaks only);
- contract regression tests.

Not yet implemented or qualified:
- live Symthaea service/process adapter;
- trusted request assembly and canonical digest computation;
- adapter/process authentication and pre-deserialization size limits;
- linkage to the evidence-bound PRAX projection pipeline;
- UI accept/dismiss wiring and accessibility checks;
- local inference/remote-provider privacy enforcement;
- build, test, CI, runtime, or learning-outcome qualification.

The claim ceiling for this branch is **contract prototype only**. No live integration, educational-effectiveness result, validated cognitive-state inference, credential authority, or production readiness is claimed.
