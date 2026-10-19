# ADR-0011: Learner Profile Privacy and Authority

- Status: Proposed
- Subject: `PRAX-STATE-001E`
- Scope: DNA-neutral learner-profile contracts

## Context

The legacy adaptive `LearnerProfile` is a public entry that combines several different semantic layers:

- learner preferences such as session length and preferred difficulty;
- inferred modality scores;
- inferred attention span;
- observed/derived accuracy and completion rates;
- learning velocity;
- retention estimates;
- activity timing and volume;
- a global profile confidence/data-point count.

This makes a single public object simultaneously act like authored preference, telemetry aggregate, model output, and learner-state summary.

## Decision

Praxis separates three concerns:

```text
LearnerPreferenceIntent
!= LearnerProfileProjection
!= LearnerProfileDisclosureProjection
```

### LearnerPreferenceIntent

This is learner-authored intent only. It may contain:

- preferred modalities;
- preferred session length;
- preferred time window;
- preferred difficulty;
- custom preference labels.

It deliberately contains no performance, retention, attention, completion, velocity, confidence, or evidence-derived state.

The full preference intent is private by default. The disclosure enum has no generic `Public` variant.

### LearnerProfileProjection

This is a recomputable private projection over exact evidence/session/derived-state dependencies.

Every signal binds to:

- a descriptive signal kind;
- a typed value;
- support-confidence metadata;
- the exact dependency subset used to produce it.

The projection records analyzer ID/version/parameter digest and an explicit expiry. There is no global profile score and no implication that normalized support confidence is statistically calibrated.

Declared inputs that support no signal are rejected.

### LearnerProfileDisclosureProjection

Sharing is a separate data-minimized operation.

A disclosure:

- references the exact source profile projection kind/ID/digest;
- names an explicit audience;
- contains only selected signals;
- carries its own expiry;
- does not repeat the source projection's full evidence/dependency graph.

This is intentionally different from publishing the full private profile.

## Privacy theorem

```text
private learner state
+ exact provenance
!= permission to publish
```

Learner preferences, activity patterns, inferred attention, performance, retention, and learning behavior can be sensitive. Storage and model availability do not imply disclosure authority.

Later Holochain materialization should default preference intent and full learner-profile projections private/local and avoid public DHT indexes by convenience.

## Authority boundary

Neither a learner-profile projection nor its disclosure grants:

- source learning-evidence authority;
- general trust authority;
- credential authority;
- runtime authorization.

A consuming system may use disclosed signals only under its own explicit policy and must retain the source projection reference/version semantics.

## Legacy migration

Do not copy the old public `LearnerProfile` into the new source model as though all fields were learner-authored.

In particular:

```text
legacy preference fields
!= legacy derived performance fields
```

Historical legacy profile records may remain visible as historical compatibility data, but migration must not invent exact event lineage or analyzer provenance for old inferred fields.

The transition should preferentially:

1. recover genuinely learner-authored preferences where provenance exists;
2. mark ambiguous historical preference fields as legacy/uncertain rather than silently asserting authorship;
3. recompute new profile projections from provenance-complete evidence when possible;
4. require explicit learner action for future disclosure.

## DNA boundary

This ADR changes no Holochain entry visibility or link type yet. Those changes belong to the isolated Praxis migration after repository-level Holochain migration authority is closed and the exact semantic stack is executable-qualified.
