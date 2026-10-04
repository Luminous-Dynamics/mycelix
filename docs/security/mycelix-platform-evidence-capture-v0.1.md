# Mycelix Platform Evidence Capture Contract v0.1

## Purpose

This profile is the next execution boundary after the vTPM Evidence profile.

It does **not** attempt to prove that measured boot is globally secure. It proves a narrower proposition:

> A captured platform Evidence bundle is internally coherent when its TPM, boot/session, event log, PCR selection, quote, challenge, toolchain, reference values, trusted time, and reconstruction result all refer to the same bounded observation.

The critical property is rejection of **mix-and-match evidence**.

```
one boot/session
+ one TPM identity
+ one binary event log
+ one PCR selection
+ one Quote
+ one challenge nonce
+ one tool/profile snapshot
+ one reconstruction result
    ↓
coherent Evidence bundle
```

## Why the event log is a separate theorem

The Linux kernel documentation describes the preboot event log as a richer record accompanying PCR contents, while PCR values provide a way to validate the measurement log. citeturn628539view0

The tpm2-tools `tpm2_eventlog` utility parses a binary TPM2 event log, and `tpm2_checkquote` verifies a TPM Quote plus qualifying data and PCR values. citeturn782502search2turn782502search0

This profile therefore refuses to collapse:

```
quote signature valid
    != event log reconstructed
    != reference values approved
    != trusted platform
```

## Capture boundary

A future physical run should capture:

- exact TPM device path and TPM fixed-property evidence;
- exact Linux boot identity;
- complete binary PC-client event log;
- exact event-log parser profile/version;
- exact PCR bank and selection;
- fresh verifier challenge nonce;
- TPM Quote message and signature;
- Attestation Key public material;
- OS/image identity;
- workload identity;
- reference-value set/version;
- trusted-time evidence;
- exact tpm2-tools/TSS provenance.

The capture tool deliberately uses a small PC-client PCR selection by default:

```
sha256:0,2,4,7
```

These PCRs are the measured-boot/platform side of the theorem. PCR16 remains available for a separate workload-specific measurement and is not silently treated as a firmware event-log PCR.

The TCG PC Client PTP specification defines platform-specific TPM behavior, and the current published PTP 1.07 line is the relevant PC-client profile family. citeturn782502search1

## Coherence rules

The verifier requires:

1. identical TPM device identity across the capture;
2. identical boot/session identity;
3. event-log digest bound into the manifest;
4. quote selection equal to the declared selection;
5. exact nonce/qualifying-data binding;
6. post-quote PCR observation passed into quote verification;
7. explicit event-log reconstruction;
8. reconstruction bound to the exact event-log digest and PCR selection;
9. unavailable or ambiguous upstream evidence -> `INDETERMINATE`;
10. no physical-TPM or authorization claim from presence alone.

## Reference values are intentionally independent

The PC-client Reference Integrity Manifest material exists to provide the reference information needed to validate measurements; a successfully reconstructed log is therefore still not the same proposition as “the observed platform is approved.” citeturn782502search5

The manifest records the reference-value **version and digest**, but the final appraisal remains a separate verifier decision.

## Trusted time

Trusted time is consumed as an evidence object from the existing EVID-TIME substrate.

```
local wall clock
    != trusted time
    != freshness
    != current authorization
```

A capture session that lacks trusted time may still be preserved as evidence, but its qualification state is `INDETERMINATE`, never `PASS`.

## Verification flow

```
capture-session.json
        ↓
artifact digest verification
        ↓
session / boot / TPM identity binding
        ↓
quote PCR selection + nonce binding
        ↓
tpm2_checkquote
        ↓
post-quote PCR consistency
        ↓
event-log reconstruction result
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

No capture receipt, Quote, or measured component may become a bearer capability.

## Current environment rule

When the TPM device or required utilities are absent, the capture mode exits as:

```
TPM PLATFORM CAPTURE: NOT EXECUTED
```

This is intentional. It avoids converting a software-only development environment into a hardware-security claim.

## Qualification ceiling

A green result establishes only:

- exact capture-session coherence;
- exact cryptographic Quote verification using the selected tool;
- exact binding of the event log and reconstruction result;
- exact tool/profile/reference provenance captured in the bundle.

It does **not** establish:

- firmware correctness beyond the measured evidence;
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

The capture path refuses to claim a qualified platform result until the independently produced event-log reconstruction result is present and bound.

## External basis

The PC-client event-log model follows the TCG platform/firmware architecture and Linux's description of the firmware-to-OS event-log handoff. citeturn628539view0turn782502search1

The quote verification boundary follows `tpm2_checkquote`, which verifies the quote signature and can additionally verify qualifying data and PCR values. citeturn782502search0
