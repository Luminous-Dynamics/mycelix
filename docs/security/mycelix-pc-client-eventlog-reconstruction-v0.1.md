# Mycelix PC-Client Event-Log Reconstruction v0.1

## Purpose

This layer performs the missing independent computation between a recorded PC-client event log and the observed TPM PCR state.

The theorem is deliberately narrow:

```
recorded event sequence
    ↓
independent SHA-256 PCR replay
    ↓
reconstructed PCR state
    ↓
compare with observed/quoted PCR
```

It does **not** decide whether any measurement digest is good, approved, authentic, or authorized.

## PC-client basis

The TCG PC Client Platform Firmware Profile requires platform firmware measurements to extend PCRs and to log the corresponding events; it also requires `EV_SEPARATOR` across PCRs 0–7 once per boot to delineate the transition from pre-OS to OS-Present. citeturn311994view0turn311994view1

The tpm2-tools event-log parser consumes the binary PC-client event-log format, but parser success is not itself a PCR reconstruction theorem. citeturn312934search0

## Independent replay rule

For the SHA-256 PCR bank:

```
PCR_0 = 32 zero bytes

PCR_next = SHA256(PCR_previous || measurement_digest)
```

The reconstruction consumes the recorded measurement digest associated with each target-PCR event in source-log order.

It does **not** silently replace the recorded measurement digest with a locally recomputed payload hash. Payload-to-digest validation is a separate event-type-specific check.

## Why this matters

The Linux kernel documentation describes the event log as richer context accompanying PCR contents while warning that the log is not trusted by itself. citeturn312934search7

Therefore:

```
event log parses
    != event log is trusted

PCR replay matches
    != reference values approved

PCR replay + Quote
    != workload authorized
```

## Adversarial corpus

The executable corpus contains **22 vectors** covering digest tampering, event removal/insertion/reordering, PCR-index and sequence substitution, malformed digests, wrong banks, expected-PCR absence/substitution, key-order invariance, metadata-only substitution, reconstruction unavailability, invalid initial state, algorithm substitution, multi-PCR handling, cross-session contamination, separator handling, and final PCR mismatch.

## Output contract

A reconstruction result must include:

```
profile_id
profile_version
event_log_sha256
pcr_bank
target_pcr
event_count
reconstructed_pcr_sha256
observed_pcr_sha256
match
reconstruction_status
```

A missing observed PCR or missing reconstruction is `INDETERMINATE`.

A computed mismatch is `DENY`.

A successful replay does not grant authorization and does not establish manufacturer trust.

## Execution

Semantic reference corpus:

```text
python3 scripts/security/reconstruct_mycelix_pc_client_eventlog_v0_1.py --self-test
```

The next physical-platform step is to add an adapter that converts the exact `tpm2_eventlog` YAML representation of a captured binary log into this canonical event stream, then feed its result into #4011.
