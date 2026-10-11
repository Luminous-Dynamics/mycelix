# ADR-0002: Symthaea as a bounded learning advisor

**Status**: Proposed
**Date**: 2026-10-11
**Scope**: `mycelix-praxis` only

## Context

Praxis already contains adaptive-learning logic, spaced review, learner-controlled
interaction modes, and a cognitive-adaptivity surface. Symthaea is an experimental
cognitive architecture that could help select useful next learning actions, organize
context, and make the interaction with a tutor more responsive. It is not a
validated educational intervention, and its consciousness-related indicators are
not evidence that a learner is stressed, focused, motivated, or ready to learn.

External evidence supports *pedagogically designed* AI tutoring more strongly than
unconstrained answer generation. The OECD's *Digital Education Outlook 2026* warns
that task completion can improve while metacognitive engagement and durable learning
decline. A 2025 systematic review of 28 K–12 intelligent-tutoring studies (4,597
learners) reports generally positive effects, but smaller advantages versus other
intelligent tutoring systems and a need for longer, more diverse evaluations.

References:
- https://www.oecd.org/en/publications/oecd-digital-education-outlook-2026_062a7394-en.html
- https://doi.org/10.1038/s41539-025-00320-7
- https://github.com/Luminous-Dynamics/symthaea

## Decision

Use Symthaea as an **optional, replaceable, advisory component**. Praxis owns the
pedagogical policy and all authoritative state. A Symthaea result can suggest a
small next step; it cannot change mastery, grade a learner, issue or revoke a
credential, write learner history, publish to the DHT, change privacy settings,
or force a break.

The first integration boundary is implemented in
`crates/praxis-core/src/symthaea_advisory.rs`. It is a contract validator, not a
live Symthaea transport or a claim that Symthaea is already improving outcomes.

## Contract invariants

1. **Opt-in**: advice is rejected unless the learner or authorized configuration
   enables it. No consent means no advisor call.
2. **Sandbox wins**: sandbox mode rejects all model-driven adaptivity.
3. **Exact scope**: the response must match the configured model ID/version, the
   current skill, curriculum revision, and exact request digest.
4. **Finite actions**: responses may choose only typed suggestions. They have no
   capability to mutate mastery, credentials, enrolment, or privacy state.
5. **Grounded references**: content references must be from Praxis's allowlist for
   this request. Recommendations needing content grounding are rejected when they
   provide no references. References prove only that a known item was named; they
   do not prove a generated explanation is true.
6. **Learner-authorized rest**: a break can be offered only when the learner
   explicitly requested one. Inferred affect, Phi-like indicators, or proxy
   neuromodulators must not compel breaks or label a learner's mental state.
7. **Stale means discard**: mismatch of skill, curriculum revision, model version,
   or request digest makes the response unusable.
8. **No confidence authority**: model confidence is uncalibrated metadata. It does
   not unlock powers, establish correctness, or issue credentials.
9. **Local-first and data-minimized**: send only the minimum observation required
   for the recommendation. Do not transmit raw answer histories, child identity,
   inferred affect, demographics, or a whole profile by default. Local inference
   is preferred; any remote provider needs explicit disclosure and consent.
10. **Graceful absence**: no service, invalid output, timeout, or unsupported
    version falls back to the deterministic Praxis learning path without blocking
    a lesson. Raw responses must be size-bounded before deserialization.

## Suggested request shape

The trusted Praxis adapter should create a canonical request containing:
- schema version;
- current curriculum revision and skill/objective ID;
- minimal, scoped learning evidence (e.g. aggregate attempt counts, review due
  status, and confidence in that estimate);
- allowlisted content IDs and the actions available for this turn;
- a BLAKE3 digest over the exact canonical request.

The digest is a stale-response/freshness binding; it is not authentication by
itself. Authenticate the adapter/transport independently, and recheck current
context before displaying advice.

## Suggested interaction loop

1. Praxis selects eligible actions using deterministic pedagogical rules.
2. Symthaea may rank or select among those actions, returning typed reason codes
   and allowlisted content references.
3. Praxis validates the response with `validate_symthaea_advice`.
4. Praxis renders a localized suggestion with a clear accept/dismiss choice.
5. Only the learner's explicit choice triggers the corresponding Praxis action.
6. Praxis separately records the learning attempt, updates mastery estimates,
   and reschedules future retrieval; the model does none of these.
7. On every failure, fall back to the ordinary learner-controlled flow.

For generated explanations, use curriculum-grounded retrieval and show provenance.
A model-written explanation is still a hypothesis until checked; allowed content
IDs are not a factuality certificate.

## Evaluation before enabling by default

Do not promote this integration based on engagement, number of chats, subjective
fluency, Phi, or task-completion speed alone. Shadow-test it first against the
current deterministic advisor with fixed scenarios and exact expected outcomes.
Then evaluate opt-in usage on:
- immediate performance and delayed retention;
- transfer to previously unseen problems and unaided retrieval;
- hint count and learner ability to explain reasoning;
- subgroup outcome gaps, including language and access conditions;
- hallucination / unsupported-source rate;
- fallback correctness and privacy boundary violations.

Use a holdout evaluation set, freeze scenario/input manifests, and report failures
as well as aggregate results. Teacher judgment remains important for age suitability,
curriculum alignment, and high-stakes educational decisions.

## Consequences

Positive:
- Symthaea can contribute cognition-aware context without becoming an authority.
- Praxis stays usable offline or when Symthaea is absent.
- Suggestion handling is deterministic, auditable, and independently testable.
- The design can later support other advisor implementations without changing
  learner-facing pedagogy or data authority.

Costs and limitations:
- This is not the live transport integration.
- The typed contract does not prove a suggestion is pedagogically effective.
- Authenticity, raw response size limits, input digest calculation, adapter lifecycle,
  and UI accept/dismiss wiring must be enforced by the eventual integration.
- Mastery estimates and learning efficacy remain subject to independent evaluation.
