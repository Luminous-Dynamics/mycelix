# Institutional Assurance Exchange v0.1

**Status:** synthetic contract prototype; development only. This is not a production protocol, legal opinion, trust framework, or validated interoperation standard.

Related implementation milestone: [Symthaea #12019](https://github.com/Luminous-Dynamics/symthaea/issues/12019).

## Purpose

This minimal packet carries references between the Institutional Intent Compiler and Mycelix's claim/authority evaluation boundary. It is intentionally an exchange envelope, not a new system of record.

- Symthaea owns interpretation candidates and the accepted intent-baseline reference.
- Mycelix owns institution-native authority, claim/evidence disposition, decision lifecycle, and replay references.
- Domain systems remain authoritative for their own records.
- A human approval reference is required; the compiler cannot approve its own output.

## What the packet binds

The schema binds the accepted baseline and digest, action and purpose, asset and claim dimensions, source references and competence references, authority scope/status, policy/evaluator version, decision reason codes, bitemporal replay references, and operation reconciliation state.

Evidence content is not copied into the packet. Source and authority references are identifiers, not proof that the referenced record is authentic, current, competent, or legally sufficient.

## Unknown-outcome invariant

When execution state is `UNKNOWN`, the packet requires:

- at least one recorded mutation and an operation/idempotency reference;
- `reattempt_permitted = false`;
- `reconciliation_required = true`;
- decision disposition `HOLD_FOR_RECONCILIATION`.

A timeout is not success or failure. Reconciliation must identify the original operation before another mutation is permitted.

## Validation limitations

JSON Schema validates structure and some local conditional constraints only. It does not verify signatures, issuer trust, source digests against remote content, identity, authority, temporal truth, semantic intent, legal validity, or actual side effects. Those require owner-native verification, independent policy evaluation, and observable receipts.

The example and oracle are synthetic engineering known answers. The oracle has not been independently adjudicated. No Rust compilation, production integration, or performance result is claimed.

## Files

- `schema.json`: JSON Schema Draft 2020-12 packet structure.
- `examples/unknown-outcome-handover.json`: complete synthetic packet.
- `oracle.json`: separate expected disposition and prohibited inferences.
