# Prism Release Continuity Ceremony — Wave 16

This document extends the Wave 15 ceremony. All paths must be canonical absolute
paths. Policy, signature, receipt, and state files must be caller-owned,
non-symlink, uniquely linked, and private. Outputs are create-once or
byte-identical idempotent publications.

## 1. Build channel incident-policy histories

Create a genesis `IncidentAuthorityPolicy` for each channel. It must use epoch
1, have no predecessor, and name distinct incident-authority and release-cosigner
keys.

For each rotation, prepare two offline requests over the same previous and next
policies:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-prepare-incident-policy-transition -- \
  incident-authority previous.json next.json authority-request.json

cargo run --frozen -p prism-attestation \
  --bin prism-prepare-incident-policy-transition -- \
  release-cosigner previous.json next.json cosigner-request.json
```

Sign each request's Ed25519 and ML-DSA-65 messages on the corresponding offline
key. Assemble the transition:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-assemble-incident-policy-transition -- \
  previous.json next.json \
  authority.ed25519.sig authority.ml-dsa-65.sig \
  cosigner.ed25519.sig cosigner.ml-dsa-65.sig \
  transition.json
```

Assemble and verify the history:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-assemble-incident-policy-history -- \
  genesis.json transition-1.json transition-2.json incident-history.json

export PRISM_INCIDENT_POLICY_HISTORY_PATH=/secure/prism/incident-history.json
export PRISM_INCIDENT_POLICY_HISTORY_BLAKE3=<exact-file-blake3>
cargo run --frozen -p prism-attestation \
  --bin prism-verify-incident-policy-history -- \
  /secure/prism/incident-history-verification.json
```

Repeat independently for source and target channels. Promotion and health
verification use the `PRISM_SOURCE_...` and `PRISM_TARGET_...` prefixed history
and receipt variables.

## 2. Produce bounded release-health evidence

Set an explicit health epoch and validity interval. The interval may not exceed
the effective incident policy's maximum.

```sh
export PRISM_RELEASE_HEALTH_EPOCH=<monotonic-positive-integer>
export PRISM_RELEASE_HEALTH_ISSUED_AT_UNIX=<unix-seconds>
export PRISM_RELEASE_HEALTH_VALID_FROM_UNIX=<unix-seconds>
export PRISM_RELEASE_HEALTH_EXPIRES_AT_UNIX=<unix-seconds>
```

Prepare, sign, assemble, and independently verify health evidence using the
existing Wave 15 commands. Any active incident must be allowed by the effective
incident policy and signed by its exact incident-authority key epoch.

## 3. Verify promotion at one explicit time

```sh
export PRISM_PROMOTION_EVALUATION_TIME_UNIX=<unix-seconds>
export PRISM_PROMOTION_ALLOWED_CLOCK_SKEW_SECONDS=<bounded-seconds>
```

`prism-verify-release-promotion` loads separately verified source and target
incident-policy histories, verifies both health attestations at that time, and
records both history roots and health epochs in the promotion receipt.

## 4. Prove cross-channel recovery convergence

Produce the Wave 15 `PromotionRecoveryReceipt` from complete source-release,
target-release, and promotion audit receipts:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-verify-promotion-recovery -- \
  source-release-audit.json target-release-audit.json promotion-audit.json \
  promotion-recovery.json
```

This is evidence only; it is not authorization to restore or activate state.

## 5. Anchor the recovery-guardian policy

Provision a private policy containing at least two sorted guardian keys and a
threshold of at least two. Record its exact file digest outside the release
host:

```sh
export PRISM_PROMOTION_RECOVERY_POLICY_PATH=/secure/prism/recovery-policy.json
export PRISM_PROMOTION_RECOVERY_POLICY_BLAKE3=<exact-file-blake3>
export PRISM_PROMOTION_RECOVERY_ISSUED_AT_UNIX=<unix-seconds>
export PRISM_PROMOTION_RECOVERY_EXPIRES_AT_UNIX=<unix-seconds>
export PRISM_PROMOTION_RECOVERY_NONCE_HEX=<64-lowercase-hex>
```

## 6. Obtain guardian signatures

Prepare one request per guardian:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-prepare-promotion-recovery-authorization -- \
  guardian-a promotion-recovery.json guardian-a-request.json
```

Each offline guardian signs the request's exact Ed25519 and ML-DSA-65 messages.
Assemble each signed guardian:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-assemble-promotion-recovery-authorization -- \
  guardian-a-request.json guardian-a.ed25519.sig guardian-a.ml-dsa-65.sig \
  guardian-a.json
```

Assemble the sorted threshold bundle:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-assemble-promotion-recovery-bundle -- \
  guardian-a.json guardian-b.json recovery-authorization-bundle.json
```

## 7. Verify and finalize recovery

Set the explicit evaluation time and verify every guardian through the admitted
local verifier agent:

```sh
export PRISM_PROMOTION_RECOVERY_EVALUATION_TIME_UNIX=<unix-seconds>
cargo run --frozen -p prism-attestation \
  --bin prism-verify-promotion-recovery-authorization -- \
  promotion-recovery.json recovery-authorization-bundle.json \
  recovery-authorization-receipt.json
```

Finalization reloads the independently anchored policy and re-runs every
classical and post-quantum verification. It does not trust the prior process's
summary:

```sh
cargo run --frozen -p prism-attestation \
  --bin prism-finalize-promotion-recovery -- \
  promotion-recovery.json recovery-authorization-receipt.json \
  authorized-promotion-recovery.json
```

Only `authorized-promotion-recovery.json` is actionable recovery evidence.

## 8. Reconstruct lost verifier evidence

Create an empty private directory and reconstruct the exact seven-file set
embedded in the promotion receipt:

```sh
install -d -m 0700 /secure/prism/reconstructed-promotion
cargo run --frozen -p prism-attestation \
  --bin prism-reconstruct-promotion-evidence -- \
  promotion-receipt.json /secure/prism/reconstructed-promotion
```

The command fails if the directory contains any extra name or any expected file
already exists with different bytes.

## 9. Execute the idempotent continuity gate

Populate a private exact-schema configuration using the Wave 16 fields and run:

```sh
python3 scripts/run-wave16-continuity-gates.py \
  --config /secure/prism/wave16-continuity.json
```

The gate reruns Wave 15 continuity, incident-history verification, recovery
quorum verification, finalization, and reconstruction. Every output and the
complete reconstruction directory must remain unchanged.

## 10. Release admission

Execute the canonical static and live lanes only after the generated
`flake.lock` has been committed, semantically reviewed, and hybrid-signed:

```sh
python3 scripts/verify-wave16-static.py \
  --check security/wave16-static-evidence.json
python3 scripts/run-wave16-live-gates.py ...
```

A successful static receipt does not substitute for Cargo, Clippy, Rust tests,
Nix evaluation, measured verifier-agent execution, offline signatures, or the
live continuity ceremony.
