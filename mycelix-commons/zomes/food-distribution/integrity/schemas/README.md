# Food Lot Provenance and Household Receipt V1 — Draft Contract

**Status:** draft schema/fixture contract only. Not wired into the Holochain zome; no production storage, receipt processing or household inventory integration is implemented.

## Why this is separate

The current food-distribution zome models markets, listings and orders. A listing currently names a product and has a floating-point `quantity_kg`; an order also has a floating-point `quantity_kg`. The food-preservation zome has a `PreservationBatch`, which is a processing batch and not a received retail/co-op lot. The reusable-container lifecycle already accepts an opaque `product_lot_ref`, but there is not yet a canonical lot schema defining that reference.

Do not conflate:
- product catalogue identity;
- a public market listing/offer;
- an order;
- a producer-side physical lot;
- a household's received portion of that lot;
- a food-preservation processing batch;
- a reusable-container lifecycle event.

## Three contracts

- `FoodLotProvenanceV1.schema.json`: minimum supplier/producer-side lot identity, source-declared quantity, optional origin/handling/date references and evidence class.
- `FoodLotHouseholdReceiptV1.schema.json`: household-local receipt state with received/accepted/rejected quantities. `privacy_scope=local_private` is mandatory.
- `FoodLotHouseholdEventV1.schema.json`: append-only local events for opening, splitting, consumption, recall notice, expiry, discard and reconciliation.

The same canonical producer `lot_ref` can be referenced by Symtropy's recipe ingredient `product_lot_ref` and Mycelix reusable packaging `product_lot_ref`. Household receipt IDs and consumption events remain private/local by default.

## Quantity rules

Quantities use an integer coefficient, decimal scale, dimension and explicit unit. No JSON floating-point quantities are accepted. Mass, volume and count are distinct. A volume quantity cannot satisfy a mass requirement through guessed density, serving size or customary conversion. An explicit independently sourced conversion event would be required before a future converter may compare them.

## Privacy and safety

The producer provenance record contains no household address, local receipt ID, pantry quantity or consumption history. Household receipt and event contracts are marked local-private; schemas alone do not enforce storage privacy. The implementation must store them in a local/private boundary and must not publish them to the shared DHT.

A producer lot declaration or an expiry field is not proof of freshness, food safety, allergen status, cold-chain integrity or suitability. Recall notices must make the lot unavailable pending authoritative disposition; a recall signal is not automatically a safety conclusion.

## Fixture status

The fixture bundle is synthetic and is not a runtime conformance suite. The next implementation slice must add typed Rust/Holochain entry and coordinator validation, sequence/parent reconciliation, private storage enforcement, and property-based negative tests. It must also define migration behavior from current `f64 quantity_kg` listing/order fields without silently treating binary floats as exact quantities.

## Cross-record reconciliation result

- `FoodLotReconciliationReportV1.schema.json` defines a read-only result envelope that binds a recipe ingredient requirement to one exact canonical lot revision, one household-local receipt, the local event-chain tip and evidence references.
- `fixtures/food-lot-reconciliation-v1.json` adds 16 cross-record cases for exact/partial receipt, missing receipt, dimensional mismatch, receipt mass-balance mismatch, overconsumption, split mismatch, event gap/fork/time regression, recall hold, declared use-by elapsed, missing current time, stale lot revision, duplicate events and listing-not-receipt.
- `resolved` is the only disposition that may set `recipe_planning_eligibility=eligible`; `indeterminate` and `conflicted` cannot be used as available household stock.
- The report is read-only. It is not an instruction to buy, consume, transfer, discard or actuate anything.

### Runtime invariants that schemas cannot prove

The future coordinator/local store must perform checked exact arithmetic and verify:
1. received = accepted + rejected within one exact dimension and unit-conversion path;
2. split: parent-before-split = child-receipt quantities + parent remainder;
3. consumed/discarded quantities never exceed the accepted remainder;
4. sequence numbers are contiguous, previous refs match exactly, event IDs are unique and timestamps follow the declared clock policy;
5. any recall hold, ambiguous/quarantined receipt, incompatible dimension or stale revision prevents eligibility;
6. supplier listing/order existence never creates a local receipt;
7. expiry checks require an explicit trustworthy as-of time and do not infer safe/fresh from the date alone.

The report schema encodes shape and disposition compatibility only. Runtime enforcement, property-based tests and local-private storage enforcement remain unimplemented.