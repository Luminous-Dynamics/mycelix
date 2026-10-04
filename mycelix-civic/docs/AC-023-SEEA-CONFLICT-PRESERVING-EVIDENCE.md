# AC-023 — SEEA Conflict-Preserving Evidence

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-023 addresses a different epistemic failure from staleness.

A published observation can be validly sourced and still disagree with another validly sourced observation for the same semantic accounting slot.

The dangerous responses are:

- silently choosing the higher value;
- silently choosing the lower value;
- averaging without a declared method;
- choosing the "more trusted" source and discarding the other evidence;
- treating disagreement as absence of impact.

AC-023 instead establishes:

> disagreement is evidence.

## Semantic identity

Two observations share the same semantic slot when they have the same:

- account family;
- accounting area;
- ecosystem type;
- native unit;
- period start;
- period end.

Source identity is deliberately not part of the semantic key.

That means independent sources can be compared for the same underlying accounting fact.

## Conflict behavior

When an observation arrives for an occupied semantic slot:

### Same value, independent source

Both observations are retained.

This is corroborating evidence, but AC-023 does not calculate a truth score.

### Same value, same source/content

The incoming observation is rejected as redundant.

The original content remains authoritative for the set.

### Different value

Both observations are retained and an explicit conflict record is returned.

The protocol does not choose a winner.

This keeps evidence history intact while making the ambiguity machine-visible.

## Why retention matters

A validation layer that returns an error but drops the conflicting record can create a second integrity problem:

source A -> retained
source B -> conflict -> discarded

A later audit would incorrectly conclude that source B never existed.

AC-023 therefore follows the same evidence-preservation rule used elsewhere in the Mycelix hardening work:

> failed interpretation does not imply deleted evidence.

## Resolution boundary

AC-023 does not implement reconciliation.

That is intentional.

Possible future reconciliation mechanisms include:

- updated official revision;
- explicit source correction;
- independent scientific reconciliation;
- temporal supersession;
- local governance determination;
- DKG evidence composition;
- uncertainty interval rather than point estimate.

Each of these carries normative or epistemic assumptions.

The evidence layer should record the conflict first and let the separate reconciliation layer choose the method.

## Canonical ordering

Retained observations can be emitted in deterministic semantic-key and observation-ID order.

Submission order therefore cannot change the canonical representation.

This matters for:

- hashing;
- signed evidence packets;
- deterministic simulation;
- cross-node comparison;
- regression fixtures;
- audit snapshots.

## Composition with AC-022

The resulting evidence path becomes:

SEEA observation
-> structural validation
-> freshness qualification
-> conflict-preserving evidence collection
-> explicit reconciliation
-> substrate projection
-> boundary evaluation
-> procurement gating

A conflict-containing set should not be treated as a reconciled substrate state.

## Adversarial cases

The qualification corpus should include:

1. two official-looking sources disagreeing by one unit;
2. large disagreement with identical provenance envelopes;
3. same value from independent sources;
4. repeated identical publication;
5. different source schema versions;
6. period-overlap but non-identical accounting periods;
7. stale source agreeing with current source;
8. current source disagreeing with stale source;
9. malicious source attempting to replace a stronger observation by submission order;
10. conflict persistence through serialization and canonical sorting.

## Relationship to Goodhart pressure

Once a measurement changes procurement eligibility, the measurement becomes consequential.

That creates pressure to manipulate it.

AC-023 reduces one manipulation surface by ensuring that a participant cannot improve a measured state merely by submitting a more favorable value later and causing the prior evidence to disappear.

The correct response is:

new claim -> new evidence -> explicit conflict -> explicit reconciliation.

Not:

new claim -> overwrite -> new truth.

## Research grounding

SEEA states that ecosystem-condition observations are associated with specific points in time and can incorporate more complete time series when required. It also emphasizes documenting measurement units and reference levels. SEEA's guidance on integrating multiple data sources notes that differing reference periods may require explicit adjustments or assumptions.

Sources:

https://seea.un.org/en/methodology/ecosystem-accounting
https://seea.un.org/sites/seea.un.org/files/documents/EA/seea_ea_f124_web_9dec24.pdf
https://seea.un.org/sites/seea.un.org/files/technical_guidance.pdf

## Thesis

The desired epistemic behavior is:

> preserve disagreement until there is enough evidence and authority to resolve it.

That is a better foundation for a consequential economic protocol than pretending every source can be reduced to one number immediately.
