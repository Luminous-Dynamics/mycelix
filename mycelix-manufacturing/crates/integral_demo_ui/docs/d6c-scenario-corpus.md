# D6C — Executable Integral scenario corpus

D6C binds the Leptos cockpit to executable reference-model scenarios.

## Principle

The UI does not contain the truth of a scenario. Each scenario is an owned ScenarioFixture containing the trace events, first-class graph relations, expected validation boundary, claim ceiling, and Symthaea availability. The reference validator evaluates that fixture, and the cockpit renders it.

    scenario
      ↓
    reference fixture
      ↓
    trace validation
      ↓
    expected / actual boundary
      ↓
    cockpit projection
      ↓
    Leptos

## Ten scenarios

| Scenario | Reference behavior |
| --- | --- |
| Normal flow | Valid bounded path |
| Rejected CDS decision | Fails closed when explicit authority is absent |
| Stale design | Fails on generation regression |
| Uncertain observation | Fails if uncertainty is silently lost |
| Conflicting observations | Graph-native Disputes edge preserves both observations |
| Recommendation accepted | Human decision explicitly records acceptance |
| Recommendation rejected | Human decision explicitly records rejection |
| Foreign evidence | Fails on foreign→local provenance laundering |
| Appealed outcome | Preserves recovery/challenge metadata |
| Appeal reopen without new path | Fails closed until a new decision path exists |
| Appeal reopen with new path | Valid only when the new decision responds to the appeal |
| No-Symthaea | Valid path remains available without Symthaea |

## Graph-native adversarial semantics

D6C now makes relations first-class and owned by the fixture rather than embedding borrowed/static relation slices inside individual events. This removes leaked allocations and lets the validator reason about graph edges independently of event adjacency.

In particular:

- `Disputes` makes conflicting observations explicit;
- `Supersedes` makes design-generation lineage explicit;
- `RespondsTo` keeps human disposition distinct from a recommendation;
- `Appeals` and `Reopens` make review/reopening explicit;
- relation endpoints must resolve to events in the same fixture.

The linear event sequence remains deterministic, but it is no longer the only representation of meaning.

## Decision boundary

Human decision disposition is now explicit in the trace:

- `Some(true)` means the human decision explicitly accepted the recommendation;
- `Some(false)` means the human decision explicitly rejected it;
- missing disposition is invalid for a HumanDecision event.

This prevents the UI from inferring acceptance/rejection from sequence position or from Symthaea's recommendation.

## Claim ceiling

ReferenceModelOnly.

D6C does not establish Integral ratification, production correctness, economic validity, security/privacy compliance, scalability, or improved human outcomes.

## D6D branch closure

D6D extends the graph validator from endpoint integrity to causal-path integrity.

Executable witnesses now cover:

- rejected decisions cannot acquire executable descendants;
- superseded designs cannot be consumed by later decision/authorization/execution events;
- reopening an appeal requires a new explicit decision path responding to the appeal;
- the valid reopened path is represented by new events rather than mutating the old decision in place.

The validator deliberately keeps event sequence and graph causality separate: sequence provides deterministic ordering, while relations provide causal meaning. Fixture replay is also graph-aware: identical event and relation payloads may replay, while relation mutations are rejected.

## Next hardening

The next increment should make the graph validator stricter about **branch closure**:

- a rejected CDS decision must have no executable descendant;
- a superseded design must not be used by a later authorization;
- an appeal reopening must lead to a new explicit decision/authorization path rather than merely changing status;
- conflicting observations should be able to feed separate assessments without collapsing provenance;
- alternative paths should be explicit rather than inferred from event order.

The claim ceiling remains `ReferenceModelOnly`.
