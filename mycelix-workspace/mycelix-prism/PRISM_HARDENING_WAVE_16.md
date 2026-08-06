# Prism Hardening Wave 16

## Purpose

Wave 16 removes four time- and authority-related ambiguities from Prism's
cross-channel release path:

1. a release-health statement can no longer remain promotion-authoritative
   forever;
2. incident actions are no longer governed by the release signer alone;
3. convergent promotion journals no longer authorize recovery by themselves;
4. a self-contained promotion receipt can now reconstruct its exact verifier
   evidence after partial local loss without downloading or inferring data.

The wave is deliberately fail-closed. It does not weaken the missing
`flake.lock` release blocker and does not claim that Rust, Nix, Bubblewrap, or
production cryptographic agents executed in the patching environment.

## Patch order

### 1. Bounded health epochs

Release-health schema v2 adds a monotonic health epoch, issue time, validity
start, expiry, and incident-policy-history root. Promotion accepts health only
at an explicit evaluation time and bounded clock skew. The old timeless helper
returns `EvaluationTimeRequired` rather than minting a permit.

### 2. Independent incident authority

Each release channel owns an append-only `IncidentPolicyHistory`. A policy
names a distinct incident authority and release cosigner, their exact classical
and post-quantum verification-key digests, admitted incident actions, maximum
health validity, activation sequence, epoch, predecessor, and policy root.

Rotation requires both of the previous epoch's hybrid signatures:

- the previous incident authority; and
- the previous release cosigner.

Activation is monotonic by release sequence and cannot be backdated.

### 3. Measured incident-history verification

`verify_history_with_agent()` replays every transition through the admitted
local verifier agent. `LoadedVerifiedIncidentPolicyHistory` binds the exact
history and its verification receipt. Source and target channels use separate
environment prefixes and separate history roots.

### 4. Health preparation and verification

Health preparation selects the incident policy effective for the exact release
sequence. Active incident actions must be permitted by that policy and signed
by its incident-authority key. Health verification rechecks both release and
incident signatures and binds the verified history root into the resulting
capability.

### 5. Promotion evidence

Promotion receipt schema v3 records:

- explicit evaluation time and clock skew;
- source and target health epochs;
- separate source and target incident-policy-history roots; and
- the exact health-attestation bytes and roots.

`verify_and_create()` consumes private-field `VerifiedReleaseHealth` values,
not unverified JSON status labels.

### 6. Offline incident-policy rotation

Wave 16 adds exact offline tools for incident policy transitions:

- `prism-prepare-incident-policy-transition`;
- `prism-assemble-incident-policy-transition`;
- `prism-assemble-incident-policy-history`; and
- `prism-verify-incident-policy-history`.

Signing requests bind the role, prior key identity, exact transition statement,
and both algorithm-specific messages.

### 7. Threshold recovery authorization

A `PromotionRecoveryReceipt` proves that source, target, and promotion journal
audits converge. It is not actionable authority. Wave 16 adds an independently
anchored `RecoveryAuthorizationPolicy` with a minimum two-guardian threshold,
exact key digests, source/target channels, and maximum validity.

Each guardian hybrid-signs the exact recovery receipt bytes, receipt root,
target sequence, policy digest, issue/expiry window, and fresh 32-byte nonce.
Successful component checks mint private-field verification tokens. A receipt
cannot be constructed from structurally valid but unverified signatures.

### 8. Reverification at finalization

`prism-finalize-promotion-recovery` does not trust a prior process's summary. It
reloads the independently anchored recovery policy, reparses the embedded
bundle, re-verifies every guardian through the measured verifier agent, compares
the resulting tokens to the receipt records, enforces expiry, and only then
publishes `AuthorizedPromotionRecoveryReceipt`.

### 9. Exact evidence reconstruction

`prism-reconstruct-promotion-evidence` recovers the seven exact verifier inputs
embedded in a promotion receipt:

- promotion policy;
- source release attestation;
- source release policy;
- source health attestation;
- target release attestation;
- target release policy; and
- target health attestation.

The destination must be a caller-owned private directory. The tool rejects
symlinks, non-regular entries, extra names, path aliasing, and differing
pre-existing bytes. A deterministic reconstruction receipt binds the complete
closed filename set, sizes, BLAKE3 digests, channels, sequences, and original
promotion receipt.

### 10. Continuity and evidence lanes

`run-wave16-continuity-gates.py` first runs Wave 15 continuity, then re-verifies
both incident histories, recovery quorum, final authorization, and exact
evidence reconstruction. All named outputs must already exist and the complete
file/directory state must remain byte-identical.

The canonical build-evidence command is now `wave16-static`, implemented by
`scripts/verify-wave16-static.py`. Historical Wave 15 receipts are not changed.

## Security invariants

Wave 16 establishes the following invariants:

- health authority expires and is evaluated at an explicit time;
- incident authority is independent of release signing authority;
- source and target incident histories cannot substitute for one another;
- policy rotation requires two prior authorities and cannot activate
  retroactively;
- convergent local journals do not authorize recovery;
- recovery authorization requires a fresh, bounded threshold quorum;
- finalization re-verifies the quorum rather than trusting a prior receipt;
- partial evidence recovery uses only exact embedded bytes; and
- idempotent continuity verification cannot mutate durable state.

## Remaining release blocker

Release admission remains blocked until `flake.lock` is generated on a
Nix-enabled host, committed, semantically reviewed, and hybrid-signed. The
canonical Cargo, Clippy, test, Nix, namespace, verifier-agent, incident,
recovery, and reconstruction lanes must then execute and publish their exact
logs.
