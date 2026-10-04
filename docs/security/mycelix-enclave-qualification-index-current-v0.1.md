# Mycelix Enclave Qualification Index v0.1

**Snapshot date:** 2026-10-04  
**Claim ceiling:** `ReferenceModelOnly`

This standalone index freezes the exact Git identities and execution status of the current sovereign-enclave qualification chain.

## Current chain

```
#3976 enclave profile
    ↓
#3977 RATS
    ↓
#3982 trusted time
    ↓
#3983 TPM Evidence
    ↓
#3994 EAT measured component

#3973 protected release
    ↓
#3974 comparative benchmark

#4001 policy enforcement
    ↓
#3975 E1 CUI/CMMC revision boundary
```

All listed PRs are currently draft/open. The SHA recorded for each item is the exact snapshot identity, not a claim that the PR cannot change later.

## Executed reference-model layers

RATS semantics: **31/31**

Trusted-time interval semantics: **17/17**

EAT measured-component semantics: **16/16**

Protected-release semantics: **33/33**

Comparative benchmark: **24/24**

PEP decision algebra: **24/24**

E1 CUI/CMMC revision boundary: **12/12**

These are executable/reference-model results, not deployment accreditation.

## TPM boundary

The TPM Evidence profile is deliberately **NOT EXECUTED** in the current environment.

There is no:

```
/dev/tpm*
swtpm
tpm2_quote
tpm2_checkquote
tpm2_pcrread
tpm2_pcrextend
tpm2_eventlog
```

Consequently, no vTPM or physical-TPM PASS is claimed.

## Current CMMC distinction

The current DoW CIO public CMMC guidance says CMMC Phase II was suspended on July 13, 2026 and that, during the pause, Level 2 uses the 110 requirements of NIST SP 800-171 Rev. 2 through the described assessment paths. DFARS 252.204-7021 remains a contract/solicitation-specific surface.

The engineering architecture nevertheless targets the newer NIST SP 800-171 Rev. 3 family.

Therefore:

```
SP 800-171 Rev.3 engineering target
!=
operative CMMC basis
```

and:

```
semantic mapping
!=
CMMC status
```

## What the program now demonstrates

The architecture has crossed an important threshold: the critical trust transitions are no longer just prose.

There are executable reference models for:

```
Evidence
→ Attestation Result
→ trusted-time freshness
→ release semantics
→ comparative containment
→ final fail-closed policy join
```

The remaining high-value work is empirical: real TPM capture, PC-client event-log reconstruction, runtime attack campaigns, actual enforcement integration, and independent review.

## Unproven

Physical TPM security, measured boot, firmware/kernel integrity, manufacturer trust, FIPS validation, CMMC status, classified authorization, CDS approval, legal export authorization, and system-wide superiority over SIPRNet/NIPRNet remain explicitly unproven.
