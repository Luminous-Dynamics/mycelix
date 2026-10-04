# Mycelix tpm2_eventlog YAML v1 Adapter v0.1

This adapter converts the exact tpm2_eventlog YAML version 1 representation into the canonical reconstruction input consumed by the independent PCR replay.

The upstream utility supports YAML versions 1 and 2; this profile intentionally supports version 1 only. citeturn805127search2

Each event preserves EventNum, PCRIndex, and EventType. For extending events, the adapter selects the SHA-256 measurement explicitly from Digests and rejects ambiguity. Real logs can contain multiple hash-bank digests in one event. citeturn475700search3

EV_NO_ACTION records are emitted as non-extending controls. PCR0 StartupLocality is detected and surfaced explicitly; the replay layer initializes PCR0 with the locality semantics required by the PC-client profile. HCRTM PCR0 likewise initializes locality 4 before its digest extension. citeturn152665search0turn805127search25

The final pcrs.sha256 values emitted by tpm2_eventlog are deliberately ignored as an observation source. The adapter instead requires a separate observed-PCR JSON artifact produced from tpm2_pcrread; that map is the observed side of the reconstruction theorem.

Physical path:

binary_bios_measurements → tpm2_eventlog v1 → bounded adapter → eventlog-reconstruction-input.json → independent PCR replay → platform Evidence verifier

Claim ceiling remains ReferenceModelOnly.