# Mycelix EK Certificate SAN → TPM Property Binding v0.1

TCG EK Credential Profile 2.7 requires the EK certificate Subject Alternative Name to carry TPMManufacturer, TPMModel, and TPMVersion as directoryName attributes. The profile defines OIDs 2.23.133.2.1, .2, and .3 for those attributes. citeturn601491search21turn536816search17

The TPM 2.0 fixed-property interface exposes TPM2_PT_MANUFACTURER, TPM2_PT_VENDOR_TPM_TYPE, TPM2_PT_FIRMWARE_VERSION_1, and TPM2_PT_FIRMWARE_VERSION_2 through properties-fixed. The vendor TPM type and firmware encodings are vendor-specific, so this theorem requires an explicitly authorized mapping registry; unknown mappings are INDETERMINATE rather than guessed. citeturn449007search0turn449007search2

The theorem is separate from EK SPKI identity, certificate-chain trust, revocation, and manufacturer authenticity. A certificate can have a valid key and valid issuer while still describing a different TPM implementation.

The reference mapping and certificate are synthetic fixtures. ReferenceModelOnly is the ceiling.
