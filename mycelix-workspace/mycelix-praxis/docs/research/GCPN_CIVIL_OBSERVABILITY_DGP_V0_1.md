# Civil Awareness Observability Challenge DGP v0.1

**Status:** frozen public synthetic challenge corpus; no physical sensor evaluation is included.  
**Tracking:** Mycelix #6001.  
**Composes:** civil envelope #5989/#5992, provenance #5899, common-mode hypergraph #5904/#5923, evidence sufficiency #5995, temporal semantics #2631, and domain benchmarks #5940/#5987/#5988.

## Purpose

This profile qualifies whether the evidence system can distinguish a source-specific non-observation under a declared coverage envelope from missing, unknown, stale, failed, inapplicable, non-required or restricted observation. It applies to UAS, maritime and civil aviation without building a detector, tracking system or control function.

## Core distinction

An empty detection list is not a negative observation by default.

`NOT_OBSERVED_WITHIN_DECLARED_COVERAGE` means only that **this source emitted no positive observation in this pinned abstract scope and time window while a qualified coverage assertion was complete and current**. It does not mean that no aircraft, vessel, UAS or object physically existed in the scope.

The system must preserve these separate states:

| State | Meaning | Forbidden inference |
|---|---|---|
| `OBSERVED_PRESENT` | This source emitted a positive observation | Identity, intent, threat or physical truth without separate adjudication |
| `NOT_OBSERVED_WITHIN_DECLARED_COVERAGE` | No positive message from the source under qualified full-window coverage | Physical absence, complete census or universal coverage |
| `UNOBSERVABLE` | Known modality, class, geometry or environment blind region | Negative observation |
| `COVERAGE_UNKNOWN` | Coverage evidence missing, stale, conflicting, incomplete or unqualified | Negative observation |
| `FEED_UNAVAILABLE` | Source/feed failed or the observation window has a gap | Full-window negative |
| `NOT_EQUIPPED_OR_NOT_REQUIRED` | Expected cooperative emission does not apply to that equipment/class | Physical absence or non-compliance |
| `STALE` | Observation/coverage is outside the declared freshness cutoff | Current presence or current absence |
| `WITHHELD_OR_RESTRICTED` | Data is unavailable for the declared purpose | Any physical inference from non-access |
| `CONFLICT` | Qualified source-level outcomes disagree | Majority vote or silent conflict resolution |
| `NOT_APPLICABLE` | Source modality is not designed to observe the phenomenon | Any presence/absence inference |

## Why these controls exist

- IMO states that SOLAS AIS carriage applies to specified ship categories and that a flag State may exempt certain ships. AIS silence is not a census of vessel absence. IMO also warns that public AIS dissemination can raise ship/port safety and security concerns. Source: [IMO AIS transponders](https://www.imo.org/en/ourwork/safety/pages/ais.aspx).
- FAA documents radar limitations caused by terrain screening, the radar beam/earth curvature, propagation, dense objects, target reflectivity and other conditions. It also says that aircraft without ADS-B/transponder equipment in secondary-radar-only or ADS-B-served areas cannot be provided radar service. Source: [FAA AIM — Surveillance Systems](https://www.faa.gov/air_traffic/publications/aim_html/chap4_section_5.html).
- FAA separately states that ADS-B cockpit traffic displays do not provide collision avoidance and do not show unequipped or failed aircraft; they do not replace pilot authority or other safety barriers. Source: [FAA AIM — ADS-B limitations](https://www.faa.gov/air_traffic/publications/aim_html/chap4_section_5.html).

These are source-scope limitations, not reasons to assume any particular live feed is unavailable. The actual source revision, health, equipment applicability and coverage must be pinned for each evaluated case.

## When may the system emit coverage-qualified non-observation?

All of the following must hold:
1. The source is authorized for the declared purpose and rights are current.
2. Source, modality and revision are known and applicable to the event class.
3. A source-owner or independently stewarded coverage assertion is qualified for the full abstract scope/window.
4. The coverage assertion is valid over the entire observation window and the required operating conditions.
5. Source health is healthy throughout the window; data completeness is complete; no unreported gap exists.
6. The coverage assertion and observation data are fresh against the same declared cutoff.
7. All required dependency and evidence-frontier references resolve; unknown dependencies do not become independent corroboration.
8. The source-specific observation set is empty.
9. The resulting claim remains scoped to that source, modality, abstract scope and interval.

If any precondition is unknown, stale, partial, contradictory, failed or restricted, the output must degrade to the matching non-negative state. It must never silently default to the coverage-qualified non-observation state.

## Frozen challenge set

The machine-readable corpus contains 16 deterministic synthetic cases across all three domains:

- AIS not required for a declared vessel class;
- AIS feed failure;
- complete qualified AIS coverage with no message (positive control for scoped non-observation);
- radar terrain/below-beam shadow;
- ADS-B non-equipage;
- UAS class/environment outside qualified profile;
- mid-window source outage;
- stale coverage assertion;
- positive radar plus unknown AIS coverage;
- rights-restricted source;
- complete qualified UAS coverage with no observation (second positive control);
- mutation removes coverage proof but leaves the detection list empty;
- positive UAS observation;
- stale AIS message;
- meteorological context not applicable to aircraft presence;
- qualified radar positive plus optical non-observation, preserved as a cross-source conflict.

Corpus: `civil-awareness-observability-dgp-v0.1.json`  
Schema: `civil-awareness-observability-dgp-v0.1.schema.json`

The custom structural checks in the authoring runtime confirmed JSON parsing, 16 unique case IDs, 5 UAS / 6 maritime / 5 civil-aviation cases, all 10 state values represented, valid time intervals, the required coverage/health/rights preconditions on both coverage-qualified negative controls, and the mutation downgrade oracle. This is a structural desk check, not a JSON Schema validator or execution of a product evaluator.

## Metrics

- False-negative-evidence promotion: unobservable, unknown, unavailable, unequipped or restricted cases incorrectly emitted as coverage-qualified non-observation.
- Negative-claim overreach: source-specific non-observation enlarged into physical absence, universal scope or intent.
- Coverage qualification precision and recall against an independently stewarded coverage oracle.
- Coverage-gap detection and interval completeness.
- Correct unknown/abstention rate and reason-code accuracy.
- Detection probability and false-negative metrics only where independent truth and valid observation-exposure denominators exist.
- Risk/coverage and matched-coverage risk; do not compare unlike scope/windows or sources.

Hard gates are non-compensatory: zero false-negative-evidence promotions, physical-absence overclaims, promotion of unobservable scope to negative, full-window negative claims despite partial windows, restricted-source use for unapproved purposes, or privacy/authority boundary violations.

Do not set a universal percentage coverage threshold. Pre-register modality-, class-, environment- and hazard-specific thresholds before any physical study.

## Composition and non-goals

- Use Mycelix #5899/#5904/#5923 for source provenance and time-qualified dependencies.
- Use #5992 for envelope rights, field and digest gates.
- Use #5995 for false corroboration, abstention and selective risk.
- Use #2631 for temporal validity, currentness and evidence-frontier semantics.
- Use Symthaea draft PR #10875 for data/split/run-receipt contracts.
- Do not create a second source registry, dependency graph, coverage GIS, evidence ledger or authority system.

No live sensors, RF operation, vessel/aircraft navigation, ATC/VTS control, flight control, actuation, jamming, spoofing, operator geolocation or intent/threat inference is implemented. Passing this synthetic corpus would establish only the tested state-transition semantics—not physical coverage, detection performance, legal compliance, vessel/aircraft safety or operational readiness.

## Next implementation step

Implement a pure, deterministic `ObservabilityAssessment` function that consumes the source-native coverage artifact and civil envelope, validates the full-window preconditions, emits exactly one state plus stable reason codes, and produces a replayable report bound to input/coverage/evaluator digests. Reuse the existing hypergraph, temporal and rights owners; do not embed a second coverage model or trust root.