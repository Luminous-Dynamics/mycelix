# D6A — Deterministic Integral cockpit projection

D6A adds a reference-model presentation contract between the authoritative D5 trace and a future user interface.

## Design principle

The cockpit does not invent a narrative. It projects structured facts already present in the trace.

`authoritative trace → HXA/validation → deterministic cockpit facts → optional Symthaea presentation`

Symthaea may help a participant understand the facts, but it cannot add authority, evidence, consent, or legitimacy that is absent from the trace.

## Human questions

The projection is designed around:
- What happened?
- Who produced it?
- What evidence is referenced?
- What authority exists?
- What remains uncertain?
- Is this a recommendation or a decision?
- What can be reversed?
- How can it be challenged?
- Which design generation is involved?
- Is the source local or foreign?

## Progressive disclosure

Four levels are shared with D5:
1. **Summary** — event meaning and uncertainty.
2. **Rationale** — actor, evidence, lineage, uncertainty.
3. **Assurance** — authority, recovery, challengeability, generation, origin.
4. **Technical** — all reference-model fields, including recommendation-only status.

The projection is deterministic: the selected fields come from the requested disclosure level and trace events.

## Important distinction

`CockpitFact.authoritative` means that the fact is copied from authoritative reference-trace data. It does **not** mean that the UI itself has authority.

A UI must never turn:
- an explanation into evidence;
- a recommendation into a decision;
- an assessment into an observation;
- a foreign artifact into a local-origin artifact;
- a missing recovery path into a claim of reversibility.

## Claim ceiling

`ReferenceModelOnly`.

D6A does not establish Integral ratification, production correctness, economic validity, security/privacy compliance, usability, comprehension, trust calibration, satisfaction, flourishing, or human outcomes.

The next UI increment can render these facts as an actual cockpit while keeping the machine-readable trace as the source of truth.