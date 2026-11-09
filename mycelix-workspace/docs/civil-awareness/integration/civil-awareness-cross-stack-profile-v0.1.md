# Civilian Awareness Cross-Stack Golden-Trace Profile v0.1

**Status:** frozen synthetic contract profile; no owner-native event artifacts or physical sensor records are included.  
**Tracking:** Mycelix #5998, under GCPN #5934/#5940 and civil-domain issues #5987/#5988.  
**Composes:** Mycelix #5923/#5904/#5451/#5989/#5992; #5972 handoff verifier; #5980 golden-trace pattern; Symthaea #10875 dataset/split/receipt contracts.

## Purpose

Test whether UAS, maritime and civil-aviation evidence can cross Luminous owner boundaries without changing source meaning, silently promoting evidence, losing unknowns or manufacturing authority. This profile covers **evidence handoff and review disposition only**. It does not test physical sensor performance.

This is a reference graph across native artifacts, not a new event bus, evidence store, trust root, GIS database, simulation format or universal model representation.

## Owner responsibilities

| Owner | Native output used in a trace | Must not claim |
|---|---|---|
| **Mycelix** | Source provenance, rights/purpose status, temporal dependency snapshot from #5923, integrity/receipt and correction references | That an authenticated signature proves physical truth or that source count proves independence |
| **Sol-Atlas** | Source-bound spatial/temporal context, edition/CRS/datum, validity, coverage and transformation lineage | That a map layer is an observation, a clearance or navigational instruction |
| **Symtropy** | Synthetic scenarios, replay, fault injection and model/run applicability references | That simulator output is physical evidence or that replay establishes live sensor detection |
| **Symthaea** | Read-only evidence triage, contradiction analysis, uncertainty, abstention and human-review request | Threat/hostile intent, operator/pilot/crew identity or location, ATC/VTS authority or external action authority |

The profile reuses the existing [cross-stack handoff manifest schema](https://github.com/Luminous-Dynamics/mycelix/blob/feat/maritime-handoff-verifier-v0-1/mycelix-workspace/docs/maritime-stay/integration/cross-stack-handoff-manifest-v0.1.schema.json) and the offline verifier in #5972. It reuses the separate-oracle pattern from #5980. No second handoff schema/verifier is introduced.

## Evidence-flow contract

`Mycelix source/rights/dependency reference → Sol-Atlas source-bound context → Symtropy synthetic replay reference → Symthaea read-only triage → Mycelix historical receipt/reference`

Edges identify the input/output artifacts, transformation and fidelity disposition. A digest identifies bytes; it does not authenticate the publisher, establish rights, prove the underlying observation or authorize reliance.

A dependency reference must point to the exact Mycelix #5923 snapshot, event-time scope and evidence frontier. Preserve:
- `SHARED_DEPENDENCY`;
- `UNKNOWN_DEPENDENCY`;
- `NO_DECLARED_COMMON_DOMAIN`.

The last means no common failure-domain ancestor is declared within a complete/current inventory for the queried dimension. It is not proof of physical or statistical independence. The cross-stack oracle does not implement a second graph.

## Disposition semantics

- `REVIEWABLE_WITHIN_DECLARED_EVIDENCE`: all declared structure/rights/source/context gates pass and no known shared dependency or contradiction prevents review. This is a reviewability disposition, not a safety verdict or field-use permission.
- `REVIEW_WITH_CORRELATED_SOURCES`: sources share a known dependency or are duplicates/relays; review may proceed, but the observations cannot be counted as independent corroborations.
- `UNRESOLVED`: a required source, time, calibration, freshness, dependency or semantic condition is unknown, stale, unsupported or unverified.
- `CONFLICT`: material owner/source assertions conflict and the supplied evidence does not adjudicate the disagreement.
- `QUARANTINED`: rights, authorization, permitted purpose or privacy policy is pending, denied or not established.
- `REJECTED`: malformed schema/digest, prohibited field, privacy violation, failed integrity or attempted action-authority promotion.
- `NOT_EVALUABLE`: required owner handoff, lawful reference, test oracle or required output is absent.

These dispositions are mutually exclusive at the case-result level. Hard gates are non-compensatory.

## Initial synthetic challenge set

The corpus and paired owner-result input define 14 cases:
- UAS clean positive within a complete declared dependency inventory;
- UAS shared clock/collector, duplicate/relay and unknown lineage;
- maritime AIS/radar disagreement, shared port processing, missing report/coverage and stale chart/context;
- aviation ADS-B/MLAT shared time source, independent-within-inventory positive control, stale feed and weather/NOTAM version conflict;
- cross-domain rights pending, prohibited-field injection, incomplete owner handoff and action-authority promotion.

The fixtures contain no actual coordinates, tracks, pilot/controller/crew details, private comms, raw audio/video, biometric data or live sensor payload.

## Required oracle invariants

1. Rights/purpose gate failure never becomes a reviewable evidence disposition.
2. Source signature/integrity, calibration, freshness and time quality remain separate.
3. Duplicate/relayed observations and shared upstream feeds do not count as additional independent sources.
4. Unknown dependency never becomes `NO_DECLARED_COMMON_DOMAIN`.
5. `NO_DECLARED_COMMON_DOMAIN` never becomes physical independence.
6. Conflicting AIS/radar/SAR or ADS-B/MLAT/weather/NOTAM assertions remain `CONFLICT` unless an independent adjudication reference is present.
7. Missing/stale/unsupported Sol-Atlas context cannot be made current by a later receipt.
8. Symtropy replay remains synthetic and cannot create a physical observation.
9. Symthaea can request review or abstain; it cannot create navigation or action authority.
10. Mycelix receipt records what was issued and by whom; it does not prove the underlying observation true.
11. Corrections append superseding evidence; historical dispositions retain original input digests.
12. Cross-owner reference completeness and source truth are different checks.

## Benchmark measures

This corpus scores handoff and disposition semantics, not physical surveillance:
- owner-handoff completeness and exact-reference resolution;
- rights/purpose and privacy disposition accuracy;
- source freshness/time/calibration preservation;
- dependency outcome agreement and false-corroboration prevention;
- conflict preservation and correct abstention;
- source-to-derived lineage and correction propagation;
- historical replay agreement and artifact digest reproducibility;
- operator-facing review routing completeness.

CWA 18150 DTI detection/tracking/identification metrics are kept in a separate physical-sensor benchmark. Do not mix event-envelope handoff correctness with radar detection probability, track completeness or identification F1.

## Execution status and claim ceiling

The initial manifest is deliberately `contract_template` with empty artifact and edge arrays. The owner-result corpus is synthetic. Passing the handoff verifier proves only local manifest structure; passing the golden-trace oracle proves only the declared rule table over the synthetic fixtures.

It does not establish owner-native runtime integration, source authenticity, physical independence, actual UAS/vessel/aircraft detection, navigation safety, airworthiness, VTS/ATC authority, regulatory compliance, vendor superiority or field performance.

## Next stage

1. Pin one actual, permitted owner-native artifact per subsystem and inspect its claim ceiling.
2. Bind a new synthetic or authorized controlled event to those exact references; never promote fixture output to source observation.
3. Run #5972 handoff verifier and this golden oracle on exact bytes.
4. Run #5923 dependency evaluation separately on the referenced snapshot.
5. Only after independent review, define a physical Track B programme under separate site, legal, privacy and professional safety approvals.
