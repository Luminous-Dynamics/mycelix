# AC-039 — Execution Receipt Lifecycle Binding

## Purpose

AC-035 and AC-036 bind an economic action's identity and scope across lifecycle revisions.

AC-039 extends that binding to actual execution evidence.

A payment, delivery, milestone, or completion record must identify:

- the stable action;
- the exact lifecycle revision;
- the active scope ID;
- the complete scope fingerprint;
- the execution type;
- external transaction/reference identity;
- supporting evidence;
- execution time.

## Execution rule

An execution receipt is accepted only when its lifecycle revision and scope fingerprint exactly match the current lifecycle state.

This blocks:

`authorized under scope A -> scope changes -> execution recorded under stale scope A`

and:

`scope ID unchanged -> scope contents changed -> execution silently accepted`

The second class is specifically closed by the SHA-256 scope fingerprint introduced in AC-036.

## Lifecycle requirements

Payment, delivery, and milestone receipts require the action to be in `Contracted` or `Implementation`.

Completion receipts require `Completed`.

Terminated actions cannot receive ordinary execution receipts because the lifecycle is terminal.

## Atomicity and immutability

The execution ledger is append-only.

Duplicate execution identifiers are rejected.

A receipt is validated completely before it is inserted.

The ledger never rewrites an earlier execution receipt.

## Research basis

OCDS models one contracting process across tendering, awarding, contracting and implementation, joined by a stable contracting-process identifier. It also records changes through immutable releases and provides explicit amendment semantics.

References:

- https://standard.open-contracting.org/latest/en/primer/how/
- https://standard.open-contracting.org/latest/en/primer/releases_and_records/
- https://standard.open-contracting.org/latest/en/schema/identifiers/
- https://standard.open-contracting.org/latest/en/guidance/map/amendments/

AC-039 applies the same general identity/lifecycle engineering principle to Mycelix economic execution evidence without importing OCDS procurement rules.

## Validation

The reference implementation tests:

- execution blocked before contract/implementation;
- execution accepted when bound to current revision and scope;
- stale revision rejection;
- stale scope fingerprint rejection;
- completion evidence requires completed lifecycle;
- duplicate receipt IDs rejected.

## Architectural result

The chain now extends from governance into execution:

`policy -> scope -> fingerprint -> lifecycle -> integrity decision -> execution receipt`

This reduces the authorization-to-execution gap and gives later payment/fulfillment integrations a stable evidence anchor.
