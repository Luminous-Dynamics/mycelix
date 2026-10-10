# Economic Fabric Cross-Domain Conformance Corpus v1

This corpus is the shared semantic test boundary for Economic Fabric interoperability. It composes the existing Integral ITC crosswalk/conformance work with Finance SAP/TEND, external fiat/Web3 recognition, authorization, rail evidence, finality, reconciliation, and outcome projection.

## Purpose

The corpus tests **semantic preservation**, not economic policy correctness.

A conforming adapter must preserve:

- source ownership and schema revision;
- object and event lineage;
- instrument identity;
- unit and scale;
- provenance/origin;
- authorization boundaries;
- settlement versus finality;
- reconciliation evidence;
- correction lineage;
- idempotency.

The machine-readable vectors are in `economic-fabric-cross-domain-conformance-corpus-v1.json`.

## Composition

```text
COS observation
      │
      ▼
ITC assessment
      │
      ▼
ITC ledger / position ───────► FRS/CDS projection
      │
      └──► explicit authorization
                    │
                    ▼
                 execution
                    │
                    ▼
              rail evidence
                    │
                    ▼
                 finality
                    │
                    ▼
             reconciliation
                    │
                    ▼
             outcome projection
```

Finance instruments and external rails enter at their own semantic boundaries:

```text
SAP   ──► SAP lifecycle ──► settlement/finality evidence
TEND  ──► TEND obligation ─► settlement/finality evidence
FIAT  ──► foreign recognition / bridge ─► external rail
Web3  ──► foreign recognition / bridge ─► chain-specific rail
MYCEL ──► reputation/trust observation (non-monetary)
```

## Required fail-closed boundaries

The negative corpus must reject, or quarantine pending explicit source semantics:

1. reputation as money;
2. ITC as SAP, TEND, or MYCEL;
3. credit limit as ownership;
4. position/balance as authorization;
5. accounting acknowledgement as settlement;
6. settlement acknowledgement as finality;
7. foreign recognition as local issuance;
8. valuation as quantity/unit conversion;
9. bridged assets as native assets;
10. simulation as physical execution;
11. corrections that rewrite history;
12. outcome claims without execution/effect evidence.

## Qualification ladder

The corpus does not promote evidence merely because another subsystem recognizes it:

```text
Source observation
  → qualified assessment
  → accounting event
  → position projection
  → explicit entitlement (only where source rules establish it)
  → explicit authorization
  → execution/effect
  → rail evidence
  → qualified finality
  → reconciliation
  → outcome projection
```

Each arrow is a semantic contract. It is not an automatic authority transfer.

## Claim ceiling

This artifact is a reference/conformance corpus. It does **not** establish:

- Integral ratification;
- Finance governance approval;
- legal ownership or legal tender;
- equivalence of ITC, SAP, TEND, MYCEL, fiat, or Web3 instruments;
- real-world economic effectiveness;
- physical authorization;
- external settlement finality beyond qualified evidence.

## Next executable integration

The corpus is intended to become the common fixture set for a cross-domain Rust conformance harness. That harness should consume these vectors and produce machine-readable receipts containing at least:

`case_id`, `decision`, `violation`, `source_schema`, `source_revision`, `instrument_id`, `unit`, `scale`, `origin`, `source_event`, `predecessor`, and `evidence_ids`.

The existing domain-specific conformance suites remain authoritative for their own source semantics; this corpus tests only the interoperability boundary.
