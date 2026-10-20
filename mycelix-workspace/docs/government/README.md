# Whole-of-Government Capability Matrix v0.1

Status: **normative measurement contract**

GOVSYS-001 defines the first repository-owned census of governmental capabilities that Mycelix can represent, partially support, does not yet support, or intentionally leaves outside the platform.

This is a measurement surface, not a claim that Mycelix is a government, a sovereign, or a lawful substitute for public institutions.

## Governing rule

> Mycelix may represent institutional authority, evidence, procedure, review, execution boundaries, and public accountability without making the network itself the source of governmental legitimacy.

The matrix lives in `capability-matrix-v0.1.toml` and is machine checked by `scripts/qualification/govsys_capability_matrix_v0_1.py`.

## Status vocabulary

Only four statuses are permitted:

- `strong_foundation` — substantial reusable semantics/evidence machinery exists, but deployment/legal readiness is not implied.
- `partial` — relevant domain machinery exists, but one or more essential governmental semantics are absent.
- `missing` — no adequate Mycelix semantic boundary currently exists; the row must identify the next planned tranche.
- `external_by_design` — Mycelix may carry evidence/coordination around the function, but the coercive, physical, constitutional, or otherwise reserved act is intentionally not implemented as ordinary platform authority.

There is intentionally no `complete`, `solved`, `production_ready`, or `government_ready` state.

## Classification rules

A positive classification is evidence-bearing. `strong_foundation` and `partial` rows must cite at least one concrete repository artifact or PR lineage.

A `missing` row must name a concrete next tranche. A vague future intention is insufficient.

An `external_by_design` row must state the boundary that remains outside Mycelix authority.

The matrix is capability-oriented, not agency-oriented. One administrative-procedure kernel should support many ministries and services; one registry kernel should support many authoritative registries. The goal is to avoid creating a bespoke digital agency for every governmental noun.

## External reference frames

The matrix uses two non-authoritative classification lenses:

1. **COFOG** — the UN Classification of the Functions of Government, represented here by divisions `01` through `10`.
2. **GovTech capability groups** — coarse labels for core government systems, service delivery, digital participation, and institutional/digital enablers.

These labels classify scope; they do not determine Mycelix authority or semantics.

## Architectural interpretation

The census should drive a small set of reusable public-institution primitives:

```text
institutional authority
        +
administrative procedure
        +
authoritative registries
        +
public records
        +
review / audit / accountability
        +
public financial management
        +
public procurement
        +
regulatory administration
        +
official statistics
        =
composable public institutions
```

Domain-specific systems should then add their own semantic profiles without weakening those common boundaries.

## Immediate implementation sequence

GOVSYS-001 is intended to be followed by:

1. `GOVSYS-002` — public-institution composition constitution;
2. `ADMIN-001` — transport-neutral administrative-procedure core;
3. `ADMIN-002` — notice, hearing/opportunity-to-respond, and reasoned-decision completeness;
4. `ADMIN-003` — reconsideration, appeal, stay, and review lineage;
5. `REGISTRY-001/002/003` — authoritative registry semantics, conflict-preserving current views, and purpose-bound inter-registry disclosure;
6. `RECORDS-001` — public-record lifecycle;
7. `ACCOUNT-001` — independent oversight/audit semantics;
8. `PFM-001` — public expenditure authority chain;
9. `PROC-001/002` — public-law procurement profile and transparency evidence;
10. `STATS-001/002` — official-statistics semantics and release qualification;
11. `REGULATORY-001` — regulatory administration.

## Non-claims

A row classified `strong_foundation` does not establish:

- legal validity in any jurisdiction;
- institutional adoption;
- constitutional legitimacy;
- production readiness;
- complete runtime integration;
- secure external effects;
- coercive authority;
- public-sector certification; or
- successful field deployment.

Likewise, a green GOVSYS-001 qualification means only that the census is internally well-formed and conservatively classified under this schema.

## Update discipline

Every future governmental PR should either:

- close or strengthen one or more matrix rows; or
- introduce a new capability row when the existing taxonomy cannot honestly express the function.

Status upgrades must cite new evidence. Status downgrades are allowed and should be used whenever a stronger review reveals an unqualified dependency or missing authority boundary.
