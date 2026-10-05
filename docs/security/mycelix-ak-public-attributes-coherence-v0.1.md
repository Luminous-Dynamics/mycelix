Mycelix AK Public Attributes Coherence v0.1

This theorem closes the remaining splice path between the AK public object and caller-supplied fixedTPM / fixedParent booleans.

The current TCG TPM 2.0 Library Specification v185 defines fixedTPM as TPMA_OBJECT bit 1 and fixedParent as bit 4; it also defines sensitiveDataOrigin as bit 5 and reserves additional bits that must be zero. TCG states that these object attributes are established at creation and are not changed by the TPM. The verifier therefore derives these bits directly from the exact TPMT_PUBLIC bytes that also determine the object Name. Those bits are never accepted as caller-supplied evidence.

The theorem deliberately does not prove object residency, AK/EK parentage, EK authenticity, credential activation, measured boot, or Quote signature validity. Those remain independent claims.

ReferenceModelOnly is the deterministic qualification ceiling. Offline/live-unintegrated ReadPublic provenance remains INDETERMINATE.