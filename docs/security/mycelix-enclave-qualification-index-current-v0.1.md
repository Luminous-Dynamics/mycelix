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

The original security-evidence gate is #4016/#4013; the active hardening stack now continues through #4030 → #4034 → #4036 → #4038 → #4040.

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

The original platform corpus contained 22 vectors. The current stack adds a 25-vector PC-client replay corpus, an event payload→digest theorem, an independent raw binary TCG event-log parser, raw/YAML byte-fidelity cross-checks, and an independent reference-value appraisal boundary.

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

#4016 introduced the dedicated security-evidence workflow. The current workflow additionally validates the raw parser, payload-digest theorem, reference appraisal, adapter→replay composition, and end-to-end false-green regressions.

- contract JSON structure and vector count;
- Python bytecode compilation;
- the complete semantic self-test;
- preservation of the bounded claim ceiling.

#4013 is the exact-head validation PR for #4011. At this snapshot the latest active heads are still in draft/open PRs. Current Actions executions are queued on the repaired heads; earlier failed runs exposed and drove parser/receipt fixes. A queued run is not a PASS.

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

## Current hardening stack

```text
#4030 executable reconstruction receipt verification
    ↓
#4034 event payload → digest coherence
    ↓
#4036 independent raw binary event-log parsing
    ↓
#4038 physical capture composition
    ↓
#4040 independent reference-value appraisal
```

All remain `ReferenceModelOnly` / `EvidenceGatePending`. The physical TPM experiment is still not executed.
