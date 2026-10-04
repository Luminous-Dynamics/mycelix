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
    ↓
#4011 physical platform Evidence capture/coherence

#3973 protected release
    ↓
#3974 comparative benchmark
    ↓
#4001 policy enforcement
    ↓
#3975 E1 CUI/CMMC revision boundary
```

The security-evidence execution gate is introduced by **#4016**. Exact-head validation for #4011 is tracked separately by **#4013**.

All listed PRs are currently draft/open unless explicitly stated otherwise. The SHA recorded for each item is the exact snapshot identity, not a claim that the PR cannot change later.

## Executed reference-model layers

RATS semantics: **31/31**

Trusted-time interval semantics: **17/17**

EAT measured-component semantics: **16/16**

Protected-release semantics: **33/33**

Comparative benchmark: **24/24**

PEP decision algebra: **24/24**

E1 CUI/CMMC revision boundary: **12/12**

These are executable/reference-model results, not deployment accreditation.

## New platform Evidence boundary

#4011 defines a new, narrower theorem:

```
one boot/session
+ one TPM identity
+ one binary event log
+ one PCR selection
+ one Quote
+ one challenge
+ one toolchain snapshot
+ one reference-value set
+ one trusted-time object
+ one reconstruction result
    ↓
coherent platform Evidence
```

The verifier explicitly separates:

```
quote verification
!= event-log parsing
!= event-log reconstruction
!= reference-value appraisal
!= trusted time
!= RATS Result
!= local authorization
```

The current semantic corpus contains **22 adversarial vectors**, including session/boot/EK substitution, parser/profile substitution, nonce/PCR/AK substitution, reconstruction tampering, trusted-time ambiguity, key-order invariance, post-quote PCR mismatch, and DENY-over-INDETERMINATE precedence.

## TPM boundary

The physical TPM Evidence layer is deliberately **NOT EXECUTED** in the current environment.

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

Consequently, no physical-TPM or vTPM PASS is claimed.

The capture helper is intentionally conservative: a physical run can preserve raw evidence even when the final qualification remains unqualified because independent event-log reconstruction or trusted time is unavailable.

## CI execution boundary

#4016 installs a dedicated security-evidence workflow that validates:

- contract JSON structure and vector count;
- Python bytecode compilation;
- the complete semantic self-test;
- preservation of the bounded claim ceiling.

#4013 is the exact-head validation PR for #4011. At this snapshot there is **no hosted security-evidence result yet**, because the workflow is not authoritative from `main` until #4016 is merged.

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

The architecture has crossed another useful boundary: the platform evidence layer itself now has an executable **coherence theorem** rather than merely an Evidence schema.

The evidence chain is now:

```
platform observation
    ↓
capture-session coherence
    ↓
TPM Quote verification
    ↓
event-log reconstruction
    ↓
reference-value appraisal
    ↓
trusted-time appraisal
    ↓
RATS Attestation Result
    ↓
Relying Party
    ↓
PEP
    ↓
local authorization / enforcement
```

Each arrow remains a separately bounded claim.

## Remaining empirical work

The highest-value unresolved step is still the real physical platform:

```
physical TPM
    ↓
real PC-client measured boot
    ↓
binary event log
    ↓
independent PCR reconstruction
    ↓
measured-component representation
    ↓
RATS Result
    ↓
PEP
```

The repository is now structurally prepared for that evidence without allowing a vTPM fixture or parser success to impersonate hardware assurance.

## Unproven

Physical TPM security, measured boot, firmware/kernel integrity, manufacturer trust, FIPS validation, CMMC status, classified authorization, CDS approval, legal export authorization, and system-wide superiority over SIPRNet/NIPRNet remain explicitly unproven.
