# Mycelix Event Payload → Digest Coherence v0.1

Current tpm2-tools directly recomputes the event payload hash for EV_S_CRTM_VERSION, EV_SEPARATOR, EV_EFI_VARIABLE_DRIVER_CONFIG, and EV_EFI_GPT_EVENT and compares the result against the recorded digest for each bank. citeturn144692search0

This profile intentionally does not universalize that theorem. tpm2-tools treats other event classes differently because some measurements cannot be reconstructed from log-contained data alone. For example, EV_IPL PCR9 measurements cover kernel/initramfs material that is external to the event log. citeturn144692search0turn571524search2

The theorem is therefore event-local:

    canonical payload bytes + recorded SHA-256 digest
                     ↓
             PASS / DENY / INDETERMINATE

`PASS` means only that SHA-256(payload bytes) equals the recorded digest for an event class covered by this profile. It does not prove that firmware measured the correct object.

Canonical payload bytes are deliberately separate from the human-readable `Event` rendering emitted by tpm2_eventlog. The adapter must not synthesize binary payload bytes by hashing the pretty-printed YAML representation.
