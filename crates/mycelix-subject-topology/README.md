# mycelix-subject-topology

Pure `#![no_std]` AMSAP-004A crate for explicit candidate-subject topology and individuation boundaries.

This crate answers one narrow question:

```text
what exact operational candidate is being assessed?
```

It does **not** answer whether that candidate is conscious, sentient, a moral patient, one metaphysical person, legally recognized, or entitled to political authority.

The crate extends existing `OperationalSubjectRef` semantics rather than replacing them. A `CandidateSubjectBoundary` can represent a single runtime, scaffolded/composite agent, multi-agent collective hypothesis, service pool, fork family, distributed execution, conditional-expert system, training process/population, or an explicitly unknown composite.

Core invariants:

```text
process count != subject count
instance count != moral-patient count
component evidence != whole-system evidence
whole-system evidence != component evidence
coordination != collective consciousness
shared weights != shared current subject
```

`SubjectBoundaryId` is a stable reference identifier. `TopologyManifestCommitment` is a separate opaque commitment to exact topology bytes. Reusing one boundary ID for materially different topology is equivocation. Canonical cryptographic encoding/domain separation remains AMSAP-005C scope.

Components and composition edges are explicit and canonically ordered. Operational subjects may be used as component references, while opaque components allow the boundary to represent non-runtime parts without pretending those parts are themselves subjects.

`TopologyEpoch` is nonzero. Material topology changes should produce a new boundary/epoch and later feed AMSAP-004B causal-lineage receipts plus AMSAP-005B applicability review.

The crate deliberately exposes operational component counts but no API equivalent to `moral_subject_count`, `person_count`, `voter_count`, or `collective_is_conscious`.

A validated boundary establishes only that the supplied topology description is structurally coherent. It grants no consciousness, valence, moral patienthood, metaphysical identity, welfare protection, legal standing, CivicId, currentness, deployment authority, governance authority, or external-effect authority.
