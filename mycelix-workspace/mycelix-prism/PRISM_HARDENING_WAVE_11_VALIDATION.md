# Prism Hardening Wave 11 Validation

## Result

Wave 11 passed every validation gate available in this environment. The patch
series replayed cleanly onto the untouched Wave 10 bundle and reproduced the
exact hardened Git tree.

Release admission remains deliberately blocked by one unmet requirement:
`flake.lock` has not been generated on a Nix-enabled host, committed,
semantically reviewed, and hybrid-signed.

## Patch replay

The 16 implementation and campaign patches preceding this report were generated
with `git format-patch`, applied with `git am` to the Wave 10 bundle, and compared
by Git tree identity.

- source tree: `7bdedc433173e1d2f21c7f8f6dff34bb5218f2bf`
- replay tree: `7bdedc433173e1d2f21c7f8f6dff34bb5218f2bf`
- replay worktree: clean
- `git fsck --full`: pass

The final sealed 17-patch replay, including this report, reproduced the final
hardened tree exactly. The cumulative 144-patch campaign replayed from the
original imported baseline and produced that same final tree with a clean
worktree.

## Reproducible evidence

```sh
python3 scripts/verify-wave11-static.py \
  --check security/wave11-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

Both checks passed and the committed receipts reproduced exactly.

Release-mode supply-chain verification returned one failure only:

```text
release blocker: flake.lock has not been generated, committed, semantically
reviewed, and hybrid-signed on a Nix-enabled host
```

## Structural checks

- Rust files parsed: **94**
- Rust syntax errors: **0**
- TOML files parsed: **29**
- JSON files parsed: **39**
- Python scripts compiled: **17**
- Workspace crates: **20**
- Cargo lock packages admitted: **838**
- Rust test, Tokio-test, and property-test annotations: **388**
- `git diff --check`: pass
- `git fsck --full`: pass

## Security properties inspected

### Typed build evidence

Release preparation and independent verification use exact typed receipt
schemas. Unknown fields and loose `serde_json::Value` interpretation are not
accepted on the release path.

### Verifier-agent key identity

The protocol binds the expected verification-key digest into the request and
requires the response to echo the same key, algorithm, message digest,
signature digest, and request ID. The verifier executable remains measured
before and after each request.

### Historical key epochs

Release-policy schema v2 binds exact Ed25519 and ML-DSA-65 verification-key
digests. Complete journal audit reconstructs verifier clients from each
checkpoint's embedded policy instead of applying current keys retroactively.

### Exact release evidence

Each checkpoint embeds the exact signed attestation bytes and exact externally
anchored policy bytes. It binds those bytes into a local checkpoint chain in
addition to the signed release-statement chain.

### Crash recovery

The journal admission path is idempotent for the exact already-current release.
A regression test covers the interruption window where checkpoint publication
succeeds but external-head publication has not yet completed.

### Transparency receipts

Complete journal audit publishes a deterministic receipt binding all statement,
checkpoint, policy, key-epoch, attestation, and signature digests to the
externally anchored head.

### Live ceremony

The promoted Wave 11 live gate requires create-once external-head and audit
outputs, performs complete journal re-audit after release verification, and
includes both artifacts in its evidence bundle.

## Unexecuted gates

The environment did not provide `cargo`, `rustc`, Nix, Bubblewrap, Chromium, or
the production cryptographic verifier agent. Consequently, this report does not
claim that the following executed successfully:

- Cargo formatting, compilation, Clippy, or tests
- Nix flake evaluation or closure builds
- live verifier-agent Ed25519 or ML-DSA-65 operations
- Bubblewrap namespace isolation
- Chromium Compatibility sessions
- the complete Wave 11 live release ceremony

Those commands remain mandatory on the admitted release host after the reviewed
`flake.lock` is created.
