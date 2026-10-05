# Mycelix TPM fixed-property capture v0.1

This theorem establishes only the provenance and deterministic parsing of TPM fixed properties.

The capture command is exactly:
tpm2_getcap properties-fixed

The retained source is the exact command output. The parser derives TPM2_PT_MANUFACTURER, TPM2_PT_VENDOR_TPM_TYPE, TPM2_PT_VENDOR_STRING_1 through TPM2_PT_VENDOR_STRING_4, and TPM2_PT_FIRMWARE_VERSION_1/2 from that receipt. The current tpm2-tools interface exposes these properties through the properties-fixed capability query. citeturn449007search0turn449007search2

Manufacturer is a numeric TPM vendor identifier. Vendor TPM type, vendor strings, and firmware fields are not treated as a universal human-readable model/version ontology. Their interpretation belongs to a later, explicitly authorized mapping theorem.

No host inventory database, DMI string, command-line override, network service, or certificate text is accepted as the property source.

ReferenceModelOnly remains the ceiling. Live capture becomes usable evidence only after an actual TPM execution produces the receipt and provenance transcript.
