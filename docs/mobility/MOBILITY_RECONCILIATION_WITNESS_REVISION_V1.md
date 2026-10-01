# MOBILITY-COMMONS-017: Reconciliation Witness Immutability and Revision Semantics v1

## Scope

This contract defines semantic/provenance rules for reconciliation-witness identity and revision.

A reconciliation witness is an auditable representation of how two identified claims were temporally reconciled. The witness identity identifies that semantic witness; it does not establish authorship, truth, physical safety, engineering correctness, certification, regulatory approval, or measurement truth.

## Core invariant

A `ReconciliationWitness` identity is immutable by meaning.

The same witness identity may be repeated only when the complete witness payload is unchanged. A materially changed payload is a different witness and therefore requires a new `ReconciliationWitness` identity plus explicit witness-to-witness `Supersedes` lineage.

## Revision rules

1. Same identity + identical payload: accepted as the same witness, not a new revision.
2. Same identity + changed claim identities: rejected.
3. Same identity + changed applicability interval: rejected.
4. Same identity + changed reconciliation inputs/result: rejected.
5. New identity + no explicit supersession lineage: rejected.
6. New identity + explicit `Supersedes` edge from successor to predecessor: accepted.
7. Historical projections remain bound to their original witness identity.
8. A successor cannot silently retarget a predecessor projection.
9. The predecessor remains addressable and valid after supersession.
10. Holochain action/entry identifiers and timestamps remain protocol metadata, not engineering identity or temporal validity.

## Lineage direction

The revision edge is:

`successor --Supersedes--> predecessor`

The edge is semantic lineage only. It does not delete, invalidate, or rewrite the predecessor.

## Content addressing

This contract deliberately does not require a cryptographic content digest. Semantic immutability plus explicit revision lineage is the current boundary. If content-addressed witness identity becomes necessary, canonical serialization and digest semantics must be specified as a separate contract.

## Qualification boundary

Qualification is structural and semantic/provenance-only. A passing vector never implies physical safety, physical equivalence, engineering correctness, certification, regulatory approval, or measurement truth.
