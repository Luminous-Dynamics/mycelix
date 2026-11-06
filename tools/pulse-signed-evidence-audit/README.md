# Pulse signed evidence auditor

Rust prototype for verifying role-separated DSSE/in-toto lifecycle evidence.

## Legacy relative-to-policy mode

```sh
cargo run --release --manifest-path tools/pulse-signed-evidence-audit/Cargo.toml -- \
  --bundle evidence-bundle.json \
  --trace lifecycle-trace.json \
  --trust-policy trust-policy.json
```

This verifies issuer signatures and semantic roles relative to the supplied trust-policy file. Its strongest success result is:

`ROLE_AUTHENTICATED_RELATIVE_TO_SUPPLIED_POLICY`

The trust policy is not authenticated by the bundle digest or `root_keyset_id`; those values establish consistency only.

## Out-of-band pinned-root mode

```sh
cargo run --release --manifest-path tools/pulse-signed-evidence-audit/Cargo.toml -- \
  --bundle evidence-bundle.json \
  --trace lifecycle-trace.json \
  --trust-policy trust-policy.json \
  --root-policy-envelope trust-policy-root.dsse.json \
  --trusted-root-pin relying-party-root-pin.json
```

The root envelope signs the exact bytes of `trust-policy.json` using DSSE PAE with payload type:

`application/vnd.pulse-trust-policy+json;version=1`

The verifier requires exactly one Ed25519 signature. It verifies the signature before parsing the policy. The signature key ID must match the SHA-256 fingerprint of the public key in the relying-party pin. The pin also fixes the exact accepted policy digest, policy ID, semantic version, positive policy epoch and root-keyset ID.

The root pin must be provisioned **out of band** by the relying party and protected like a trust anchor. Never load it from the evidence bundle, a sender-controlled attachment, or an untrusted URL. The policy DSSE envelope and policy file may travel with the bundle; the pin must not.

A successful root-pinned result is:

`ROOT_AUTHENTICATED_POLICY_SNAPSHOT`

This means the exact policy snapshot was signed by the pinned root and matches the relying party's exact snapshot pin. It does not prove the real-world truth of the statements or authorize actions. SCITT COSE and transparency receipts remain unverified and still produce INCOMPLETE findings.

## v1 limitations

- One out-of-band Ed25519 root key, one root signature and one exact current policy snapshot.
- No automatic root rotation, cross-signing, threshold/quorum policy or policy-history service.
- Policy updates require controlled replacement of the relying-party pin, including digest and epoch.
- The pin file itself must be protected from rollback. The verifier cannot protect itself if the operator supplies an old or attacker-controlled pin.
- Historical signature validation, trusted timestamps and compromise-time adjudication are not implemented.
- No SCITT COSE signature, receipt authenticity, inclusion or VDS consistency proof verification.
- A valid signature proves that a key signed bytes; it does not prove the asserted event happened.

## Tests

```sh
cargo fmt --manifest-path tools/pulse-signed-evidence-audit/Cargo.toml -- --check
cargo test --manifest-path tools/pulse-signed-evidence-audit/Cargo.toml
cargo clippy --all-targets --manifest-path tools/pulse-signed-evidence-audit/Cargo.toml -- -D warnings
```

The workflow `.github/workflows/pulse-signed-evidence-audit.yml` runs the exact-source Rust test, formatting and Clippy gates and uploads a source/lock/receipt artifact.
