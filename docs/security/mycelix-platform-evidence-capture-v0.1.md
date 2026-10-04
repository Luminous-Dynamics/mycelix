# Mycelix Platform Evidence Capture Contract v0.1

## Purpose

This profile is the next execution boundary after the vTPM Evidence profile.

It does **not** attempt to prove that measured boot is globally secure. It proves a narrower proposition:

> A captured platform Evidence bundle is internally coherent when its TPM, boot/session, event log, PCR selection, quote, challenge, toolchain, reference values, trusted time, and independent PCR reconstruction all refer to the same bounded observation.

The critical property is rejection of **mix-and-match evidence**.

```
one boot/session
+ one TPM identity
+ one binary event log
+ one PCR selection
+ one Quote
+ one challenge nonce
+ one tool/profile snapshot
+ one reference-value set
+ one trusted-time object
+ one reconstructed PCR state
    ↓
coherent platform Evidence
```

## Why the event log is a separate theorem

The Linux TPM event-log model treats the preboot event log as richer context accompanying PCR contents; the log is not trusted merely because it exists or parses. citeturn312934search7

The tpm2-tools `tpm2_eventlog` utility parses a binary TPM2 event log according to the TCG PC Client Platform Firmware Profile format. citeturn312934search0

The TCG PC Client profile requires firmware measurements to be extended into PCRs and logged, and `EV_SEPARATOR` is measured across PCRs 0–7 once per boot to delimit the pre-OS/OS-Present transition. citeturn311994view0turn311994view1

Therefore:

```
quote signature valid
    != event-log reconstructed
    != reference values approved
    != authorized workload
```

## PCR artifact hash versus PCR state

This distinction is now explicit.

```
sha256(pcr-post.yaml)
    = artifact-integrity digest

canonical_hash({
  bank: "sha256",
  values: { PCR -> actual PCR value }
})
    = observed PCR-state digest
```

They are intentionally different claims.

`tpm2_pcrread` emits PCR values in YAML and can also write raw PCR values in binary form; the verifier uses the YAML representation for the human-auditable artifact and parses the actual SHA-256 PCR values for the state hash. citeturn562146search0

## Capture boundary

A future physical run captures:

- exact TPM device path and fixed-property evidence;
- observed Endorsement Key public material;
- exact Linux boot identity;
- complete binary PC-client event log;
- exact event-log parser profile/version;
- exact PCR bank and selection;
- fresh external verifier nonce;
- TPM Quote message and signature;
- Attestation Key public material;
- post-quote PCR observation;
- OS/workload identity;
- reference-value set/version;
- trusted-time evidence;
- exact tpm2-tools/TSS provenance.

The default qualified PCR selection is:

```
sha256:0,2,4,7
```

PCR16 remains a separate workload-specific surface and is not silently treated as a PC-client firmware event-log PCR.

## Independent reconstruction

The independent reconstruction layer replays the measurement digests from the event stream:

```
PCR_initial = 32 zero bytes
PCR_next = SHA256(PCR_previous || measurement_digest)
```

For the selected PCR set, the reconstructed value set is canonicalized and hashed.

The capture verifier will only admit a reconstruction PASS when:

```
reconstructed_pcr_values_hash
    ==
observed_live_pcr_values_hash
```

A mismatch is `DENY`.

Unavailable or ambiguous reconstruction is `INDETERMINATE`.

The reconstruction layer intentionally consumes recorded event digests rather than silently substituting locally recomputed payload hashes. Event-type-specific digest validation remains a distinct appraisal theorem.

## Verification flow

```
capture-session.json
        ↓
artifact digest verification
        ↓
session / boot / TPM / EK binding
        ↓
actual PCR-state parsing
        ↓
event-log reconstruction
        ↓
reconstructed-vs-live PCR equality
        ↓
quote selection + nonce binding
        ↓
tpm2_checkquote
        ↓
reference-value + trusted-time evidence
        ↓
bounded platform Evidence
```

The downstream flow remains:

```
platform Evidence
    ↓
RATS Attestation Result
    ↓
Relying Party
    ↓
current local authorization
    ↓
PEP
```

No Quote, reconstruction receipt, or measured component may become a bearer capability.

## Current environment rule

When the TPM device or required utilities are absent, the capture mode exits as:

```
TPM PLATFORM CAPTURE: NOT EXECUTED
```

This is intentional. It avoids converting a software-only environment into a hardware-security claim.

## Qualification ceiling

A green result establishes only:

- capture-session coherence;
- artifact integrity;
- actual observed PCR-state binding;
- exact independent PCR reconstruction binding;
- cryptographic Quote verification using the selected tool;
- recorded tool/profile/reference provenance.

It does **not** establish:

- firmware correctness beyond measured evidence;
- absence of firmware supply-chain compromise;
- kernel correctness;
- resistance to physical TPM or bus attacks;
- manufacturer trust beyond explicitly qualified trust anchors;
- FIPS 140-3 validation;
- CMMC or classified authorization;
- CDS approval;
- system-wide security superiority.

## Execution commands

Semantic corpus, no TPM required:

```text
python3 scripts/security/verify_mycelix_platform_evidence_capture_v0_1.py --self-test
```

Verify an existing bundle:

```text
python3 scripts/security/verify_mycelix_platform_evidence_capture_v0_1.py --verify /path/to/capture
```

Attempt a physical capture:

```text
python3 scripts/security/verify_mycelix_platform_evidence_capture_v0_1.py --capture \
  --reference-values /path/to/reference-values.json \
  --trusted-time /path/to/trusted-time.json \
  --tss-version-evidence-file /path/to/tss-version-evidence.txt \
  --os-image-digest sha256:<digest> \
  --workload-digest sha256:<digest> \
  --nonce-file /path/to/fresh-verifier-nonce.bin
```

The capture path refuses to claim a qualified platform result until the independently produced PCR reconstruction result is present and the reconstructed state equals the observed live PCR state.
