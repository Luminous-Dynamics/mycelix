# Mycelix Enclave Qualification Index v0.1

**As of:** 2026-10-04  
**Claim ceiling:** `ReferenceModelOnly`

This index records the current executable/reference-model state of the sovereign enclave qualification program.

## Chain

| PR | Boundary | Head | Execution status |
|---|---|---|---|
| #3976 | sovereign enclave profile | `358e7bd40f1d...` | structural JSON validation |
| #3977 | RATS Evidence/appraisal | `bc28a1aa1304...` | **31/31** independent semantic vectors |
| #3982 | trusted-time freshness | `3a06e6a5410e...` | **17/17** independent interval vectors |
| #3983 | TPM 2.0 Evidence | `e9f1170228ce...` | **NOT EXECUTED** in current environment |
| #3994 | EAT measured component | `865d9cbc3712...` | **16/16** independent semantic vectors |
| #3973 | protected release | `2127b5c3087f...` | **33/33** independent semantic vectors |
| #3974 | comparative benchmark | `d387a233732c...` | **24/24** independent reference-model checks |
| #4001 | policy enforcement decision | `12cff4a51f86...` | **24/24** independent decision-algebra vectors |

## Proven invariants

The current reference layer explicitly exercises:

```
Evidence != Attestation Result != Authorization
trusted-time evidence != wall clock
vTPM != physical TPM security
measured component != authorization
transfer receipt != release authority
DENY > INDETERMINATE > PASS
no composite comparative-security score
```

## Real execution blocker

The available execution environment currently has:

```
/dev/tpm*       absent
swtpm           absent
tpm2_quote      absent
tpm2_checkquote absent
tpm2_pcrread    absent
tpm2_pcrextend  absent
tpm2_eventlog   absent
```

Therefore the TPM layer has intentionally not been marked PASS.

## What remains unproven

The program still has no qualification evidence for:

- physical TPM security;
- measured boot;
- firmware integrity;
- kernel integrity;
- manufacturer/EK trust;
- FIPS 140-3 validation;
- CMMC status;
- classified authorization;
- CDS approval;
- legal export-control authorization;
- system-wide superiority over SIPRNet/NIPRNet.

## Current scientific posture

The program now has executable semantic boundaries before real deployment:

```
security-domain model
    ↓
RATS appraisal
    ↓
trusted-time freshness
    ↓
TPM/platform evidence
    ↓
measured-component representation
    ↓
protected release
    ↓
comparative measurement
    ↓
fail-closed enforcement join
```

The remaining work is increasingly empirical rather than architectural: real platform capture, event-log reconstruction, exact reference values, adversarial runtime campaigns, and independently reproducible measurements.
