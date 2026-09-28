# Integral versioned semantic seam reference model v1

Status: **ReferenceModelOnly**

This is the neutral reference-model layer identified by #3142/#3209. It is not a transport, broker, authorization service, or Integral-specific runtime.

## Identity hierarchy

semantic subject identity != delivery identity != attempt identity != transport receipt != recipient semantic admission != authorization != external effect

## Fail-closed rules

1. Profile/schema mismatch returns `StaleSchema`.
2. Transport acceptance never becomes semantic admission.
3. Admission requires matching delivery and attempt identity.
4. Unknown delivery state remains `Indeterminate`.
5. Retry may change attempt identity but preserves logical delivery and payload identity.
6. Changed payload under the same delivery identity is rejected.
7. Foreign recognition preserves foreign origin.
8. Transport receipts never grant authority.

## Intended Integral profiles

- OAD -> COS Certified Design Package
- COS -> ITC Labor/Materials Record
- FRS -> CDS Signal Packet
- remaining proposed seams only after their source specification is frozen

## Composition

- #3142: semantic seam contract
- #1535: crash-consistent effect/outbox semantics
- #803: durable effect-attempt composition
- #1150: intermediary non-authority
- #3334/#3335: COS evidence and five-system conformance

The model deliberately does not duplicate concrete transport, outbox, or authority owners.

## Qualification path

`ReferenceModelOnly -> source-refined -> executable against real seam adapter -> two internal domains -> external/mock peer -> bounded formal refinement -> pilot observation`.

A reference-model PASS does not prove physical delivery, external exactly-once execution, governance acceptance, production qualification, or Integral validation.