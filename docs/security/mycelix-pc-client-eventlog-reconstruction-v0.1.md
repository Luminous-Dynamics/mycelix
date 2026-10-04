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

The TCG PC Client Platform Firmware Profile requires platform firmware measurements to extend PCRs and to log the corresponding events; it also requires `EV_SEPARATOR` across PCRs 0–7 once per boot to delineate the transition from pre-OS to OS-Present. See: https://trustedcomputinggroup.org/resource/pc-client-specific-platform-firmware-profile-specification/

The tpm2-tools `tpm2_eventlog` utility parses the binary event log; parsing is intentionally not treated as reconstruction: https://tpm2-tools.readthedocs.io/en/latest/man/tpm2_eventlog.1/

The Linux TPM event-log documentation describes the event log as richer context accompanying PCR contents and treats the log as evidence rather than an independently trusted authority: https://www.kernel.org/doc/html/latest/security/tpm/tpm_event_log.html

## Independent replay rule

For the SHA-256 PCR bank:

```
PCR_0 = 32 zero bytes

PCR_next = SHA256(PCR_previous || measurement_digest)
```

The reconstruction consumes the recorded measurement digest associated with each selected-PCR event in source-log order.

It does **not** silently replace the recorded measurement digest with a locally recomputed payload hash. Payload-to-digest validation is a separate event-type-specific check.

## Why this matters

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

The executable corpus contains **25 vectors** covering digest tampering, event removal/insertion/reordering, PCR-index and sequence substitution, malformed digests, wrong banks, expected-PCR absence/substitution, key-order invariance, metadata-only substitution, reconstruction unavailability, invalid initial state, algorithm substitution, multi-PCR handling, cross-session contamination, separator handling, and final PCR mismatch.

## Output contract

A reconstruction result is machine-consumable JSON containing the PCR maps plus explicit input and verifier provenance:

```
profile_id
profile_version
event_log_sha256
session_id
pcr_bank
pcr_selection
event_count
reconstructed_pcr_values
reconstructed_pcrs_sha256
observed_pcr_values
observed_pcrs_sha256
match
reconstruction_status
reason
```

A missing/ambiguous observed PCR state is `INDETERMINATE`.

A computed mismatch is `DENY`.

A successful replay does not grant authorization and does not establish manufacturer trust.

## Execution

Semantic reference corpus:

```text
python3 scripts/security/reconstruct_mycelix_pc_client_eventlog_v0_1.py --self-test
```

Generate a machine-readable reconstruction result:

```text
python3 scripts/security/reconstruct_mycelix_pc_client_eventlog_v0_1.py \
  --reconstruct /path/to/event-stream.json \
  --output /path/to/eventlog-reconstruction.json
```

The replay profile also explicitly refuses to silently mis-model PC-client special cases: EV_NO_ACTION is non-extending; HCRTM PCR0 initialization is outside the zero-state profile; and PCR0 StartupLocality records are treated as initial-state adjustments outside the zero-state profile. Each unsupported case yields INDETERMINATE rather than a fabricated PASS.

The next physical-platform step is to add an adapter that converts the exact `tpm2_eventlog` YAML representation of a captured binary log into this canonical event stream, then feed its result into #4011.

## Verification-closure hardening

The reconstruction receipt is no longer treated as a trusted summary. A qualified receipt must contain the actual reconstructed and observed PCR maps, their independently recomputed canonical state hashes, the exact input-stream digest, and verifier-source provenance.

The platform Evidence verifier executes the reconstruction executable against `eventlog-reconstruction-input.json` and requires the resulting JSON receipt to match the supplied `eventlog-reconstruction.json` byte-for-byte. A self-consistent forged receipt therefore cannot qualify merely by updating its own hashes.

The remaining event-type payload-to-digest theorem is intentionally separate: replay proves that recorded digests produce the observed PCR state; it does not prove that firmware payloads were truthful.

Claim ceiling remains `ReferenceModelOnly`.
