# ADR-0014: Evidence-Policy-Driven Instructional Adaptation

- Status: Proposed
- Subject: PRAX-SCIENCE-001
- Scope: DNA-neutral instructional adaptation semantics

## Context

The legacy adaptive surface uses learning-style vocabulary that can blur three different questions:

1. what a learner says they prefer;
2. what accessibility support a learner requires;
3. which instructional intervention is appropriate for a task.

A learner preference is not, by itself, evidence of a cognitive aptitude or evidence that matching presentation to that preference improves learning. The 2024 meta-analysis of the learning-styles matching hypothesis found an overall effect across eligible outcomes, but crossover evidence supporting the matching hypothesis was present in only a minority of outcomes/studies and the authors concluded that the evidence did not warrant widespread adoption.

By contrast, retrieval practice, spacing, and interleaving have substantial evidence bases, while effects remain dependent on domain, implementation, and population. A systematic review in radiology education found evidence across several interventions but also noted the limited number of domain-specific trials.

UNESCO guidance on AI and education emphasizes learner agency, privacy, transparency, auditability, and protection against inappropriate data use.

## Decision

Praxis separates:

```text
PresentationPreference
!= AccessibilityRequirement
!= TaskAffordance
!= InstructionalStrategy
```

### Presentation preference

Learner-authored preference may record a presentation modality. The semantic name is PresentationModality; it must not be interpreted as a stable learner aptitude.

### Accessibility requirement

Accessibility requirements are independently expressible and are not optional style preferences. They may constrain how content is presented regardless of any learner preference.

### Task affordance

A task may require or benefit from a representation because of its content or action space. For example, pronunciation can require audio, while spatial reasoning may require a diagram. The adaptation rationale belongs to the task context, not to a learner-style label.

### Instructional strategy

Strategies are selected through a named and versioned EvidencePolicy. The policy records scope and rationale and is explicitly contextual rather than a universal efficacy assertion.

Supported strategy vocabulary includes retrieval practice, distributed practice, interleaving, elaboration, concrete examples, complementary representations, worked examples/fading, practice with feedback, and transfer practice.

### Experimental assignment

If Praxis tests a preference-matching or other novel hypothesis, the assignment must be explicitly experimental and carry an experiment identifier, hypothesis, outcome measure, and assignment version. Experimental treatment must not be silently represented as ordinary personalization.

## Scientific boundary

The contract intentionally does not encode claims such as:

```text
"I prefer diagrams"
=> "I am a visual learner"
=> "visual matching improves my learning"
```

Instead:

```text
preference -> presentation option
accessibility -> required accommodation
task affordance -> representation constraint/opportunity
evidence policy -> instructional strategy
experiment -> measured hypothesis
```

Evidence-supported strategies are still context-sensitive. Policy versioning makes later empirical updates possible without rewriting historical assignments.

## Authority boundary

Instructional adaptation is advisory/pedagogical state. It grants:

- no credential authority;
- no trust authority;
- no runtime authorization.

A strategy assignment may influence content sequencing or presentation only through an explicit consuming policy.

## Privacy boundary

The system should retain only the preference, accessibility, task-context, and strategy information necessary for the adaptation being performed. Learner preference and inferred analytics should remain private by default, with any disclosure represented separately and minimized. This follows privacy-engineering and learner-data guidance emphasizing risk management, minimization, transparency, and learner agency.

## Migration

Historical VARK/learning-style records remain historical compatibility data. Migration must not convert old modality scores into authoritative aptitude claims.

Future migration should:

1. recover genuinely learner-authored preferences where authorship is defensible;
2. preserve accessibility requirements independently;
3. attach task-affordance reasoning to task/content context;
4. express instructional interventions through named policy/version receipts;
5. isolate experiments from ordinary personalization;
6. preserve explicit legacy/incomplete status when provenance is unavailable.

## Qualification requirements

Before DNA materialization, qualification should test:

- preference alone cannot create a strategy-efficacy claim;
- accessibility survives independently of preference;
- task affordance can select presentation without learner-style inference;
- an evidence policy ID/version is mandatory for strategy assignment;
- experimental assignments require explicit experiment metadata;
- stale policy versions cannot be rewritten in place;
- strategy assignments cannot grant trust, credential, or authorization authority.

This ADR is source-semantic only. It does not authorize Holochain DNA materialization.
