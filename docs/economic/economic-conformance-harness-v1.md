# Economic Fabric executable conformance reference

This document defines the executable state-machine behavior expected from an implementation of ECO-XONT-001.

## Deterministic transition tests

| Test | Input | Expected |
|---|---|---|
| ECF-001 | Observed + evidence | Evidenced |
| ECF-002 | Evidenced + current valuation basis | Valuated |
| ECF-003 | Valuated + policy eligibility | Entitled |
| ECF-004 | Entitled + explicit authority | Authorized |
| ECF-005 | Authorized + settlement idempotency key | Submitted |
| ECF-006 | Submitted + recipient acceptance + matching authorization | Settled |
| ECF-007 | Submitted + timeout | Indeterminate |
| ECF-008 | Indeterminate + fresh receipt + still-valid authorization | Settled |
| ECF-009 | Disputed + no resolution | Rejected |
| ECF-010 | Disputed + explicit resolution + fresh authorization | Settled |
| ECF-011 | Settled + duplicate logical settlement | DuplicateIdempotent |
| ECF-012 | Settled + correction | Corrected with historical lineage preserved |

## Mutation tests

The following mutations must not be accepted:

- change source event identity between adapters;
- rewrite origin during recognition;
- replace current unit with a numerically equal but semantically different unit;
- remove evidence references;
- convert Valueflows Intent into EconomicEvent;
- convert Integral observation directly into ITC entitlement;
- convert a journal projection into physical evidence;
- mutate a settled event in place during correction;
- reuse a logical settlement identity with changed economic meaning;
- treat timeout/indeterminate as successful settlement;
- bypass authorization;
- treat a foreign instrument as locally issued.

## Oracle-hidden requirement

The conformance implementation must derive results from explicit input state and declared transition rules. A fixture may not contain a hidden expected-result field that the validator merely echoes.

## Cross-ontology identity requirement

Adapters must emit a traceable mapping:

source_id → source_type → adapter_profile → target_id → target_type

and retain the source origin and evidence references.

## Claim ceiling

Passing these tests establishes semantic conformance to this reference contract only. It does not establish financial correctness, legal/regulatory compliance, security, economic fairness, or real-world performance.
