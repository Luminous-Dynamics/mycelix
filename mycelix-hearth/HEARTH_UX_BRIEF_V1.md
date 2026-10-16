# Hearth Household Brief v1

## Product hypothesis

The Hearth home page should help residents orient without presenting every domain object at once. Three small, directly navigable summaries—active care plans, open household decisions, and unresolved alerts—are a simple first experiment. They do not claim to understand residents' priorities.

## Research implications

- A 2026 *Journal of Systems and Software* study of explanations in smart environments (N=159) reported improved task performance and system understanding when users received explanations versus no explanation. It also found that context-adapted explanations are not always better than static explanations. This argues for brief, relevant explanations—not a novel generated explanation for every card. [Paper and abstract](https://www.sciencedirect.com/science/article/pii/S0164121226000610).
- Apple's Human Interface Guidelines recommend that feedback tell people what is happening, what an action did, and what they can do next, matching interruption strength to the importance of the event. [Feedback](https://developer.apple.com/design/human-interface-guidelines/feedback).
- Home Assistant's dashboards separate domain views and reusable cards, reinforcing that a coherent overview need not collapse all controls into one crowded screen. [Dashboard sections](https://www.home-assistant.io/dashboards/sections/).

These are design inputs, not proof this specific implementation improves usability. Human evaluation remains required.

## Deliberate non-goals

- No AI or heuristic ranking of household importance.
- No inference of care due-times from recurring templates.
- No claim that an open household decision is the current resident's personal vote.
- No relationship-health score or score-derived "neglected bond" callout.
- No implicit dispatch or mutation authority.
- No claim of source completeness from an aggregated count.

The count helpers are scoped to the currently presented Hearth identity. Without a selected target, they return an unavailable value (rendered as an em dash), not a false zero and not a count aggregated from other/unbound data. An established target with an empty source list produces zero. This is a UI scoping guard, not proof that current-household selection is authoritatively established; the global data trust notice remains necessary.

## Qualification

Unit tests cover no selected Hearth, active-only care plan counting, open-only decisions, resolved-alert exclusion, and cross-Hearth isolation. CI/build execution and resident usability testing are separate required evidence.

## Benchmarks (targets, not results)

- 90% or greater first-try completion of five common navigation tasks.
- Under 10 seconds median time from home page to a familiar care/decision/emergency destination.
- Four of five first-time participants correctly identify the cards as navigation summaries, not an urgency ranking.
- Zero relationship-quality labels inferred from bond-strength scores.
- Zero horizontal overflow at 320, 360, 390, 768, and 1280 CSS pixels.
- 100% of summary counts scoped to the currently selected Hearth key in the client model; this does not substitute for backend authorization or catalog qualification.
