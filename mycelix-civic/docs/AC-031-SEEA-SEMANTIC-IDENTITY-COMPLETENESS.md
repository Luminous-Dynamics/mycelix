# AC-031 — SEEA Semantic Identity Completeness

## Finding

AC-023 introduced semantic conflict detection for SEEA observations.

Its original key distinguished account family, accounting area, ecosystem type, unit, and period.

That was not sufficient for every SEEA account family.

SEEA distinguishes ecosystem condition observations from ecosystem service flows, and service flows are associated with their users/economic units. SEEA also treats different stock/change semantics as meaningful accounting information.

## Hardening

The semantic key now includes:

- account family;
- accounting area;
- ecosystem type;
- economic/institutional unit;
- change semantic;
- native unit;
- accounting period.

This reduces two classes of evidence corruption:

1. distinct facts being incorrectly treated as conflicting;
2. distinct facts being incorrectly treated as redundant agreement.

## Deliberate treatment of missing economic-unit data

`economic_unit_ref` remains optional on the observation, but its presence or absence is part of the semantic key.

This means an observation that identifies a specific economic unit does not silently overwrite or conflict with an otherwise unit-unspecified observation.

A future equivalence/reconciliation layer can deliberately decide whether those observations are compatible. The ingestion layer does not guess.

## Validation

Added tests prove that:

- changing the economic unit changes semantic identity;
- changing stock/increase/decrease semantics changes semantic identity.

Existing conflict preservation and canonical ordering remain unchanged.

## Research basis

The United Nations SEEA EA describes ecosystem condition as observations of ecosystem condition at specific points in time, while ecosystem-services accounts record supply and use by economic units. SEEA therefore treats these as distinct accounting dimensions rather than interchangeable records.

References:

- https://seea.un.org/en/methodology/ecosystem-accounting
- https://seea.un.org/en/Introduction-to-Ecosystem-Accounting
- https://seea.un.org/sites/seea.un.org/files/documents/EA/seea_ea_f124_web_9dec24.pdf

## Security contribution

Evidence identity is itself an integrity boundary.

A conflict detector that conflates distinct facts can create false disputes; one that merges distinct facts can silently destroy information.

AC-031 keeps the semantic identity conservative: observations are only considered the same accounting slot when their material semantic dimensions actually agree.
