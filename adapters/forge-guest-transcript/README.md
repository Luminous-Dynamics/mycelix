# mycelix-forge-guest-transcript

FORGE-004D3B2C1 defines the **positive guest success transcript** consumed by the M0 evidence chain.

A transcript is not a log and it cannot mint `OfflineEvidence`. It is a canonical statement that one exact guest plan reached every required phase and produced exact phase evidence.

## Exact phase sequence

The transcript carries one digest for each frozen guest phase, in the exact plan order:

1. isolation probe;
2. bundle replay;
3. ambient Git-object-source rejection;
4. strict Git object validation;
5. policy trust inventory;
6. local trust qualification;
7. gittuf verification;
8. transcript emission.

Missing, duplicate, reordered, or extra phases fail closed.

## Subject binding

`GuestTranscriptV1` binds the exact:

- guest-plan digest;
- execution subject;
- repository request digest;
- run challenge;
- ordered phase evidence.

Deserialization revalidates the exact phase sequence.

## Independent bindings

A transcript does not qualify itself. `qualify_guest_transcript` also receives `GuestTranscriptBindings`, derived from the concrete evidence subjects outside the transcript.

For M0, the bindings are expected to come from:

- raw inside-isolation evidence;
- the portable manifest/replay subject;
- strict no-ambient-Git policy;
- strict Git object-validation policy;
- complete policy trust inventory;
- qualified local-key trust evidence;
- exact gittuf local receipt;
- fixed transcript-output policy.

The gittuf phase must additionally equal the plan's `expected_replay_receipt`.

## Derived structural phase subjects

The replay, ambient-source, Git-validation and transcript-output phase identifiers are deterministic protocol subjects derived by this crate. They are not free-form labels supplied by the runner.

The phases backed by concrete evidence (`IsolationProbe`, `PolicyTrustInventory`, `LocalTrustQualification`, and `GittufVerification`) must equal the caller-supplied exact evidence commitments.

## Same-run composition

The later M0 composer should bind the `QualifiedGuestTranscript.evidence_digest()` into the same-run `VerifierExecution` phase, then separately require the transcript's gittuf phase to equal the portable replay receipt. This prevents a valid transcript from one run being paired with another sandbox run.

## Claim boundary

This crate performs no filesystem I/O, process execution, Git/gittuf calls, namespace inspection, or policy inventory. The concrete guest runner must produce the evidence consumed here; the host runner still performs parent observation, pidfd binding, final isolation qualification, and NAR postflight.
