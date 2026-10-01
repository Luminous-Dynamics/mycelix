# MOBILITY-COMMONS-016: Reconciliation Witness Identity V1

## Purpose

A temporal reconciliation witness is a first-class provenance object. Its identity must be distinct from the identities of the claims it compares and from any Holochain protocol identifier.

The identity layer prevents a projection from containing an arbitrary string that merely looks like a witness reference.

## Identity model

A reconciliation witness uses the existing namespaced IdentityRef structure with:

- kind = reconciliation_witness;
- an explicit namespace;
- a stable witness identifier.

The witness identity is carried by TemporalReconciliationWitness.witness_identity.

A ReconciliationEvidenceProjection must carry an IdentityRef of the same kind and it must equal the supplied witness's identity exactly.

## Required distinctions

- witness identity != left claim identity;
- witness identity != right claim identity;
- witness identity != Holochain action/entry hash;
- witness identity != Holochain action timestamp;
- witness identity does not establish authorship;
- witness identity does not establish physical truth, safety, certification, regulatory approval, or engineering correctness.

## Lineage

reconciliation_witness -> supersedes -> reconciliation_witness is permitted. Supersession preserves the predecessor witness rather than erasing it.

## Validation

Validation is fail-closed:

1. validate the witness identity;
2. require reconciliation_witness identity kind;
3. validate the projection reference;
4. require exact equality between projection reference and supplied witness identity;
5. reject claim identities used as witness references;
6. recompute the witness's reconciliation result as before.

This is semantic/provenance qualification only. It is not a physical or regulatory qualification mechanism.

Holochain's validation model reinforces this separation: validation should be deterministic, and unavailable dependencies are unresolved rather than invalid. Protocol identity and engineering identity therefore remain separate layers.
