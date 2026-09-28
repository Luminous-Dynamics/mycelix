# D6C — Executable Integral scenario corpus

D6C binds the Leptos cockpit to executable reference-model scenarios.

## Principle

The UI does not contain the truth of a scenario. A scenario produces a machine-readable trace, the reference validator evaluates it, and the cockpit renders the resulting evidence state.

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
| Conflicting observations | Explicitly PresentationOnly until the trace model has conflict relations |
| Recommendation accepted | Human decision explicitly records acceptance |
| Recommendation rejected | Human decision explicitly records rejection |
| Foreign evidence | Fails on foreign→local provenance laundering |
| Appealed outcome | Preserves recovery/challenge metadata |
| No-Symthaea | Valid path remains available without Symthaea |

## Important non-overclaim

The conflicting-observation scenario is deliberately not reported as validated. The current D5 trace is linear and cannot establish semantic conflict merely because two observations occur adjacent to each other.

That limitation is surfaced in the scenario result rather than hidden behind UI language.

## Decision boundary

Human decision disposition is now explicit in the trace:

- `Some(true)` means the human decision explicitly accepted the recommendation;
- `Some(false)` means the human decision explicitly rejected it;
- missing disposition is invalid for a HumanDecision event.

This prevents the UI from inferring acceptance/rejection from sequence position or from Symthaea's recommendation.

## Claim ceiling

ReferenceModelOnly.

D6C does not establish Integral ratification, production correctness, economic validity, security/privacy compliance, scalability, or improved human outcomes.

## Next hardening

The next trace increment should replace linear adjacency assumptions with explicit relations for:

- supports;
- authorizes;
- responds-to;
- disputes;
- supersedes;
- alternative-to;
- appeals/reopens.

That will allow conflict, rejection, branching, and appeal reopening to become first-class executable semantics rather than presentation-only scenarios.
