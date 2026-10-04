# Mycelix AK Creation Provenance v0.1

This theorem adds creation-time provenance without collapsing it into AK/EK lineage or credential activation.

## TPM semantics

`TPM2_Create` returns the created object's public area, creation data, creation hash, and creation ticket. `TPMS_CREATION_DATA` contains PCR selection/digest, locality, parent Name, parent Qualified Name, parent Name algorithm, and outsideInfo. `TPM2_CertifyCreation` consumes the TPM-produced creation hash and ticket and emits a TPM_CREATION attestation signed by a chosen signing key. citeturn252951search0turn252951search1turn729356search0

## Boundary

v0.1 does not pretend that a JSON rendering is the TPM wire format. The creation-data wire bytes remain authoritative; parsed fields are retained for explicit relationship checks, while a future wire parser can close the serialization theorem.

Likewise, `creation_hash_matches` is not treated as proof that the TPM validated the creation ticket. Ticket validation remains its own state, and an offline bundle without an independently authenticated creation certificate is INDETERMINATE.

## Composition

`CertifyCreation` proves a creation-data/object association; `ActivateCredential` proves the live TPM can recover a credential through the credentialed AK. They remain separate propositions.

The ceiling remains `ReferenceModelOnly` and this profile introduces no hardware qualification.