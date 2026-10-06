# Mycelix EK SAN Identity Binding v0.1

TCG EK Credential Profile 2.7 requires the EK certificate Subject Alternative Name to contain TPM manufacturer, TPM part number, and TPM firmware version using directoryName. The manufacturer value must match TPM2_GetCapability(TPM_PT_MANUFACTURER). The certificate firmware version records the firmware at certificate creation, so it is not silently equated with current firmware after field updates. TCG also assigns OIDs 2.23.133.2.1, 2.23.133.2.2, and 2.23.133.2.3 to TPM manufacturer, model, and version. citeturn601491view0turn449007search2

The architecture separates parsing from identity authorization. An OpenSSL-backed extractor produces the exact SAN OID/value receipt from the certificate DER. A separate binder compares that receipt to independently sourced TPM identity evidence.

The manufacturer name itself is not authorized by this verifier; mapping TPM_PT_MANUFACTURER to an approved vendor name remains a separate registry theorem. Part-number evidence is also explicit because TCG requires a manufacturer-provided retrieval path rather than a universal TPM property.

Missing issuance-era firmware provenance is INDETERMINATE. Current firmware is recorded but is not accepted as evidence for what firmware existed when the EK certificate was created.

The theorem remains independent of EK SPKI key identity, certificate-chain trust, and measured-boot validity.

The SAN receipt is no longer accepted as a standalone authority artifact. The verifier source, exact certificate input, extractor output, and extractor content digest are all rebound by re-executing the DER-native extractor against the exact certificate bytes. The canonical positive fixture itself is produced through that same path.
