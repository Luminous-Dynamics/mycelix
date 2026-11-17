# Reusable food-container lifecycle V1

This contract extends the existing Mycelix food-preservation zome; it does not create a second inventory or traceability DNA.

- `ReusableContainerV1.schema.json`: immutable container identity, material, capacity, temperature range, closure, food-contact evidence, cleaning protocol, permitted product classes and optional manufacturer reuse-cycle limit.
- `ContainerCycleEventV1.schema.json`: append-only event with parent hash, sequence, lifecycle kind, product/lot or preservation batch reference, condition and measured/declared observations.
- `fixtures/container-lifecycle-v1.json`: synthetic lifecycle sequence and adversarial cases.

## Lifecycle model

A standard cycle is:
`Washed -> Sanitized -> Inspected -> Filled -> Sealed -> Dispatched -> Received -> Emptied -> Returned -> Washed`.

The initial container history must begin with a recorded wash. Each fill references a product class and a lot or existing preservation batch. If the container declares allowed product classes, a fill outside that list is rejected.

## Integrity and authority

- Container registration and all cycle events are immutable.
- The registered steward authors registration, lifecycle events and lifecycle links.
- Each event references the same container and exactly the preceding event; sequence is contiguous and time cannot go backwards.
- A fork or ambiguous history blocks new mutation until reconciled. Reuse-cycle count is derived from Filled events and is not a stored counter.
- Registry/event index links are immutable and checked against the referenced container/event records.

## Claim limits

QR/NFC labels are identifiers, not proof of authenticity, freshness or safety. Cleaning/sanitizing observations are not microbial lethality evidence. Food-contact qualification, cleaning protocols, regulatory compliance, cold-chain integrity and shelf-life claims need their own method, calibration, uncertainty and independent qualification records. No safety claim is inferred from a container material label or a sanitation event.
