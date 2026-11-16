# Food Packaging Schemas V1

Status: proposed data contract; not deployed, not a product passport and not a food-contact approval.

Files:
- `PackDesignV1.schema.json`: design revision, intended food categories, material/contact assessment, capacity, pressure-use flag, cleaning profile and evidence references.
- `PackCycleEvidenceV1.schema.json`: one lifecycle event payload intended to reference canonical Mycelix identity and event receipts.
- `fixtures/adversarial-v1.json`: synthetic positive/negative examples.

## Event ownership

The packaging event payload must be submitted through the existing canonical Mycelix event/receipt boundary. It does not define a second append-only event ledger, new signing protocol, identity namespace or custody database. The `subject_ref`, `design_ref`, `product_lot_ref`, actor and predecessor references must resolve through existing canonical owners.

Relevant architecture: https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-desci/docs/CANONICAL_EVENT_API.md

## Structural vs stateful validation

JSON Schema validates a single document's shape and selected conditional requirements. It cannot validate:
- lifecycle transition order across multiple events;
- uniqueness of an idempotency key across a stream;
- predecessor-chain integrity or replay detection;
- authorization, custody transfer or conflict resolution;
- whether an instrument is calibrated for a measured range;
- whether a laboratory method actually supports the declared material/food/use;
- whether the wash procedure has achieved a microbiological hygiene endpoint;
- whether the return system's lifecycle impact is beneficial.

Those checks belong to the canonical event, authority, evidence and domain-validation owners. The structural schema must not be described as qualification.

## Required adversarial behavior

- `released_for_use` is structurally rejected unless cleaning and inspection states are `pass` and a release-authority reference exists. This still does not prove a safe state; the evidence behind those references must be checked independently.
- A `pressure_use=true` design requires a pressure qualification reference. An identifier alone does not confer qualification; the referenced evidence must be independently reviewed for exact revision, material, temperature, pressure, contact and cleaning scope.
- A syntactically valid out-of-order event must be rejected by the stateful verifier.
- Missing or unknown data remain unknown, never converted to a pass.
- Corrections supersede previous events by reference; do not silently rewrite history.
- Do not expose household addresses or detailed purchase behavior by default.

## Fixture semantics

The fixture bundle contains only synthetic data. The expected outcomes identify whether a case should pass structural validation or be rejected by a stateful verifier. It is not a record of an actual container, food lot, wash or inspection.

## Versioning

- Additive optional fields require an explicit version/migration decision.
- Changed semantics, required fields or evidence classes require a new major schema version.
- Preserve the original canonical evidence payload and provide a deterministic migration test.
- Schema validation never upgrades declared evidence into measured, assayed or independently attested evidence.
