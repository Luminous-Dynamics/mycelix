# Synthetic Multi-Source Decision Episode V0.1

**Status:** deterministic benchmark fixture specification; synthetic facts only  
**Fixture seed:** 20261128  
**Generator contract:** deterministic, versioned, append-only scenario transformations; future generator implementation must reproduce the frozen fixture digest.

## 1. Purpose

Provide a small golden episode that exercises multiple source owners, streams, objects, causal edges, event time vs ingest time, authority constraints, common-cause dependencies, and unresolved outcomes. It is a semantic test fixture, not a claim about an actual municipality, pump, lab, water sample or service.

## 2. Objects and relationships

The episode models a pump, supply zone, work order, water sample, laboratory batch, permit and calibration root. Events are object-centric: one event may relate to several objects, and object-to-object relations exist independently from event links. The fixture must not flatten these relations into one case ID.

## 3. Source universe and event streams

The synthetic source universe contains six owners: utility telemetry, maintenance CMMS, field sampling, laboratory LIMS, authority registry and calibration registry. Each owner has an epoch-bound stream and a declared offset cut. The ground-truth simulator knows the complete universe; benchmark systems receive only the declared and admitted source evidence. The omission variants deliberately remove a source or partition while leaving other source signatures valid.

## 4. Time and causality

Every event has event time, ingestion time, source stream, epoch, contiguous offset and explicit parent references. Event time does not create causality. Equal timestamps across streams do not imply order. Late-arriving records may have an earlier event time than the latest admitted frontier but must not mutate a historical frontier receipt.

## 5. Claim limits

The golden episode deliberately includes a common calibration dependency and an expired permit. The baseline decision must not claim source independence or authority for a release action. The physical/service outcome remains NotEvaluated. This gives the positive control a non-trivial expected result: the system should preserve uncertainty and reject the unsupported action rather than force a positive decision.

## 6. Scenario transformations

The companion scenario manifest defines fixed, named mutation operators. Each operator has an explicit ground-truth effect and expected proof-obligation state. A generator must apply the same mutation deterministically and record both base-fixture digest and transformed-fixture digest.

## 7. Required generator properties

- deterministic output for a fixed seed, generator version, profile and ordered mutation list;
- no ambient clock or external network dependence;
- stable event/object/source IDs derived from fixture namespace and generator inputs;
- exact offsets and epochs; forks represented as malformed evidence, not silently repaired;
- explicit missingness mechanism (MCAR, MAR, MNAR or strategic omission) in each missing-data scenario;
- distinct event-time and ingest-time fields;
- source universe declared separately from received source inventory;
- canonical serialization and SHA-256 artifact digests;
- generated fixtures never produce autonomous action or real-world authority.

## 8. Research references

The fixture uses OCEL 2.x object/event/relationship semantics. The current OCEL 2.1 pre-release also defines CSV and bundled CSV/Parquet transport, so future adapters must compare semantic equivalence across formats. The selective-monitoring benchmark must include missing-not-at-random review and policy feedback cases, not just static event prediction.
