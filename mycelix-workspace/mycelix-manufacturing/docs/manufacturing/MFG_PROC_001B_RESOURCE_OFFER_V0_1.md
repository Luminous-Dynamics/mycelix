# MFG-PROC-001B — Distributed Manufacturing Resource Offer v0.1

**Parent:** Mycelix MFG-PROC-001 #3085  
**Child:** MFG-PROC-001B #6533  
**Depends on:** MFG-PROC-001A #6531 (typed process/capability references)  
**Status:** Draft contract / synthetic-only

## Purpose

A `ManufacturingResourceOfferV1` is an immutable, revisioned provider statement that an exact resource at an exact site is offered for an exact process definition and capability profile under an exact configuration and prerequisite set.

It connects canonical process semantics to distributed discovery without turning a string machine type, provider web page, location pin or throughput estimate into production authority.

## Core separation

```
resource offer published
!= provider/resource/site relationship verified
!= external process/capability object resolved
!= capability qualified under exact profile
!= calendar slot available
!= capacity bookable
!= reservation accepted
!= process executed
!= output independently accepted
```

The v0.1 validator checks only local structure and cross-reference consistency. It does not fetch owner payloads, authenticate signatures, confirm provider control of a resource, qualify an equipment configuration, inspect a site, interpret calendar contents or schedule a job.

## Contract

The schema is `schemas/mfg_proc_001b/manufacturing_resource_offer_v0_1.schema.json` and the synthetic corpus is `tests/fixtures/mfg_proc_001b_resource_offer_v0_1.json`.

An offer binds:
- a stable provider-issued offer ID plus an immutable revision identifier;
- provider identity and optional provider-authority reference;
- resource/equipment and facility/site references;
- canonical Symthaea process definition and capability profile references;
- exact equipment configuration;
- tooling/workholding, material applicability, environment, utilities, operator competence, safety, inspection and calibration profile references;
- declared-capacity profile reference;
- optional availability-calendar reference;
- optional commercial terms reference;
- capacity-claim provenance class and supporting evidence references;
- provider publication state (Published or Withdrawn only), lifecycle-event references, issue time, validity window, revision and optional supersession pointer.

Engineering quantities, units, dimensional limits, material properties and calendar slot semantics remain in their canonical owner schemas. The resource-offer contract does not define a second unit or measurement ontology.

## Capacity semantics

The field `declared_capacity_profile_ref` points to a source-owned capacity profile. It does not embed a second machine-capacity model.

The optional `availability_calendar_ref` points to time-bounded calendar evidence. A calendar reference does not mean a slot is open, bookable or reserved.

The following remain distinct, separately owned records/states:
- provider-declared capacity;
- manufacturer specification;
- observed capacity under a named profile;
- qualification evidence under an exact process/capability profile;
- calendar availability;
- accepted booking/reservation;
- actual process execution;
- independently inspected output.

A provider's statement that a machine can produce N units per hour is not a calendar, a reservation or evidence of actual accepted throughput.

## Evidence and authority rules

- Owner-native pointers carry owner repository, schema ID/version, source revision, subject ID and SHA-256 payload digest.
- A digest is an identifier/check value, not proof that bytes were fetched, the source is authentic or the issuer controls the resource.
- `provider_authority_ref` is a reference, not verified delegation.
- Non-provider capacity-claim sources require at least one evidence reference; the reference remains unresolved until an owner-specific resolver validates it.
- `publication_state` is limited to Published or Withdrawn; it applies to the provider's immutable offer statement, not the underlying machine's availability or physical state.
- Expiry is derived from `valid_from_unix_s` and `valid_until_unix_s`; it does not rewrite history.
- Disputed is derived from a separately owned lifecycle/dispute event. Superseded is derived when a newer immutable offer references this exact offer ID and revision. Neither is a provider-settable state in v0.1.
- Any unresolved lifecycle-event reference produces `LifecycleEventsUnresolved`, not `ActiveOfferWindow`; a pointer alone cannot establish whether the event is benign, adverse, valid or authoritative.
- A new offer may point to a prior offer revision; self-supersession is rejected locally. Graph-wide cycle/currentness checks require a resolver and are not claimed by this tranche.
- Unknown, unresolved, withdrawn, disputed, superseded or expired offers never become positive production capability through fallback. Only Published/Withdrawn are stored on the provider offer; the other lifecycle dispositions remain projections.

## Explicit non-goals

No scheduler, MRP rewrite, procurement/quote/payment, inventory, custody/lot ledger, reservation protocol, telemetry adapter, machine-control path, process physics, safety assessment or physical qualification is added here.

Mycelix #6533 owns resource-offer and distributed provenance semantics. Symthaea MFG-PROC-000 #5686 owns canonical process/capability/recipe meaning. Symtropy #6786 consumes the offer and process references for route analysis. Sol-Atlas #606 projects owner-native evidence read-only.

## Initial benchmark cases

The 17-case synthetic corpus covers valid typed offers, missing/malformed owner refs, wrong process identity, mixed source commits, malformed digests, duplicate prerequisites, reversed validity, optional calendar evidence, unresolved provider authority, capacity claim source evidence, withdrawal, provider attempts to self-assert Disputed/Superseded, self-supersession rejection and commercial-terms absence.

Every case asserts:
- no promotion to qualified capability;
- no inferred calendar availability;
- no bookable or reserved capacity;
- no executed output.

These cases qualify only the synthetic data contract, not a provider, resource, site, schedule, route or product.

## Next qualification steps

1. Exact-head formatting, focused tests and Clippy on the shared Rust contract.
2. Independent review of the 16-case expected dispositions.
3. Owner-specific provider/resource/site and capacity-profile resolvers, each with source-schema and payload-byte evidence.
4. Only then, separate availability and reservation integrations with explicit unknown/stale/conflict handling.
5. Physical Node 0 / supplier trials with independently inspected conforming parts before any local-capacity claim.
