# FORGE-004D3A-v5 — M0 Offline Evidence Composition

This crate is the final M0 composition gate. It does not run Git, gittuf, bubblewrap, or Nix itself; it consumes the already-qualified portable-replay, guest, isolation, runtime-closure, runtime-tool, and same-run evidence produced by the concrete execution stack and refuses to mint repository `OfflineEvidence` unless every subject cross-links exactly.

## Positive theorem

A successful `QualifiedM0OfflineEvidence` requires all of the following to describe one exact verification subject:

- the portable repository bundle manifest and replay receipt;
- `ExecutionSpec` v2 with denied network, `VerificationTimePolicy::NotUsed`, hermetic filesystem, no external trust material, and the exact M0 tool/input role sets;
- a self-contained `GuestVerificationPlanV1` carrying the exact repository request;
- the exact `GuestToolMapV1` bytes mounted into the sandbox;
- the exact Linux isolation policy, Nix closure, guest invocation, and 32-byte run challenge;
- `QualifiedHermeticHostRun`, including sealed input evidence, parent/inside isolation qualification, pre/post NAR equality, pidfd same-process evidence, qualified guest envelope/transcript, and same-run qualification;
- `QualifiedRuntimeToolEvidence` tying every declared M0 `ToolArtifact` digest/size to the executable file bytes actually selected by the host/guest stack;
- an `EvidenceBoundExecution` whose output is the qualified guest-transcript evidence digest and whose execution evidence is the qualified host-run evidence digest;
- repository qualification against `VerificationProfile::m0_protected_source()` with the exact policy-lineage and final offline-evidence commitments.

The resulting repository capability chain is therefore:

```text
portable replay
  + exact execution subject
  + exact capsule bytes
  + sealed immutable inputs
  + exact executed tool bytes
  + NAR-qualified runtime closure
  + two-channel Linux isolation
  + pidfd same-process binding
  + qualified guest workflow
  + exact gittuf replay receipt
  + pre/post runtime equality
  = OfflineEvidence
```

## Exact M0 tool roles

The tool set is exactly:

- `bubblewrap`
- `forge-hermetic-host`
- `forge-hermetic-guest`
- `forge-isolation-probe`
- `git`
- `gittuf`

The NAR auditor is linked into the exact host executable and is separately pinned by `auditor_subject()`; it is not a seventh executable role.

## Exact M0 input roles

The mounted input set is exactly:

- repository bundle
- repository bundle manifest JSON
- guest verification plan JSON
- guest tool map JSON
- Linux isolation policy JSON
- Nix closure manifest JSON
- sandbox invocation JSON
- 32-byte run challenge

Semantic commitments and transport-byte commitments are checked independently.

## Host trust boundary

M0 assumes the Linux kernel and privileged host/root remain trustworthy for the duration of the verification run. This boundary is explicit:

- sealed memfds prevent ordinary host-path writers from changing non-Nix input bytes after qualification;
- bubblewrap makes committed inputs and the committed Nix closure read-only from the verifier sandbox;
- canonical NAR preflight/postflight plus Runtime Tool Evidence detect persistent changes to the committed runtime closure and selected executables;
- parent/inside namespace observations and pidfd binding establish the observed sandbox/process relation.

These mechanisms do **not** prove safety against a malicious kernel, hypervisor, firmware, or privileged host/root capable of lying about kernel observations or transiently modifying runtime state and restoring it before postflight. Such resistance requires a separate measured/attested-host profile and must not inherit `LocalHostM0` evidence implicitly.

## Claim boundary

This type is the authority boundary for the repository-level `OfflineEvidence` capability under the M0 host-trust boundary above, but it only qualifies the exact evidence supplied to it. It does not imply source correctness, absence of vulnerabilities, reproducible builds, release authorization, hostile-host resistance, or Sigstore-hermetic verification. M0 remains the local SSH/GPG verifier profile.
