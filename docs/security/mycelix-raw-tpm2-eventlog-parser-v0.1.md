# Mycelix Raw TPM2 PC-client Event Log Parser v0.1

`tpm2_eventlog` parses a binary TPM2 event log and can render YAML version 1 or 2. citeturn311528search0

This parser intentionally operates on the raw binary structure rather than the YAML presentation. It follows the TCG_EVENT / TCG_EVENT_HEADER2 / TCG_EVENT2 layout used by tpm2-tools, including the SpecID digest-size table. citeturn336196search0turn336196search1

The parser outputs the exact `Event` bytes as `payload_hex`, the exact recorded digest values by algorithm, and the source binary SHA-256. This creates an independent byte-fidelity boundary for the later payload-digest coherence theorem.

Composition:

    binary_bios_measurements
            ↓
    independent raw parser
            ↓
    exact payload bytes + digests
            ↓
    bounded tpm2_eventlog YAML adapter
            ↓
    canonical reconstruction input

`tpm2_checkquote` itself can compare PCR values against a reconstructed event-log state, but that does not eliminate the need to keep parser provenance and event-log trust explicit. citeturn144692search4turn311528search1

Claim ceiling remains `ReferenceModelOnly`.
