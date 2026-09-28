# D5 — Machine-readable decision and explanation trace

D5 adds the reference-model seam that a future Integral cockpit can render without inventing meaning in the UI or asking an AI system to supply missing authority.

## Trace

`Proposal → Design → Decision → Authorization → ExecutionIntent → Observation → ITC Projection → FRS Assessment → Recommendation → Human Decision → Outcome → Appeal`

Each trace event carries:

- stable event identity and monotonic sequence;
- explicit provenance class;
- actor identity (human, Symthaea, or system);
- local/foreign origin;
- source/evidence reference;
- design generation;
- uncertainty state;
- authority reference where applicable;
- reversibility and challengeability;
- explicit recommendation-only status.

## Safety properties

The reference validator rejects:

- illegal provenance transitions;
- sequence or generation regression;
- recommendation authority laundering;
- consequential events without explicit authorization;
- consequential outcomes without recovery/contestability;
- disappearance of uncertainty;
- foreign-origin laundering;
- replayed events whose authoritative payload changed.

A later production implementation should additionally bind these fields to immutable event IDs, cryptographic evidence, real authorization records, and runtime-specific persistence. D5 does not claim those properties.

## Explanation boundary

An explanation is a **view over a trace**, identified by `trace_ref`. It is never authoritative and never becomes evidence.

Progressive disclosure is deterministic:

1. **Summary** — what happened.
2. **Rationale** — relevant evidence and lineage.
3. **Assurance** — authority, uncertainty, and recovery/appeal.
4. **Technical** — full trace-oriented detail.

If Symthaea is unavailable, the system can still render a non-authoritative explanation view from the trace. If Symthaea is available, it may assist with presentation, but the trace—not the model output—remains the source of authority and provenance.

## Human boundary

The cockpit can therefore answer, from structured data:

**What happened? Who/what produced it? What evidence supports it? Which generation is this? What is uncertain or disputed? What authority was required? Can it be reversed? How can it be challenged? Is this only a recommendation?**

This is deliberately stronger than an AI-generated narrative because the UI can distinguish missing information from a confident-sounding explanation.

## Claim ceiling

`ReferenceModelOnly`.

D5 does not establish Integral ratification, production correctness, economic validity, security/privacy compliance, comprehension, trust calibration in participants, satisfaction, flourishing, or human outcomes.
