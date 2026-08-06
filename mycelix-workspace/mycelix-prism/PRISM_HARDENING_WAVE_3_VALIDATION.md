# Prism Hardening Wave 3 Validation

Date: 2026-07-20

## Validation completed in this environment

The following checks completed successfully on the Wave 3 source tree:

- `git diff --check`
- `git fsck --full --no-progress`
- Tree-sitter Rust syntax parsing for all 68 Rust source files
- TOML parsing for all 23 manifests/configuration files
- JSON parsing for all 26 JSON files
- workspace-member existence audit
- lockfile dependency audit for newly declared `serde_json` dependencies
- production-source checks that Brave/Perplexity client-secret headers no
  longer exist outside regression tests
- UI check that no iframe execution path remains
- UI check that the obsolete localStorage API-key reader is absent
- provider-proxy explicit-enable checks in both server binaries
- clean Git working tree

The repository contains 288 `#[test]`, `#[tokio::test]`, or `proptest!`
annotations after Wave 3.

## Patch replay validation

The Wave 3 mail patch series was replayed with `git am` onto a fresh clone of
the Wave 2 bundle. The replay completed cleanly, the resulting Git tree matched
the hardened source exactly, and the replay repository had a clean working tree
after application.

## Canonical Rust validation still required

This execution environment did not initially contain `cargo`, `rustc`, or Nix,
so compilation and runtime tests were not claimed here. Run these from the
canonical repository environment:

```sh
cargo fmt --all -- --check
cargo check --workspace --all-targets --locked
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
./scripts/verify-core.sh
```

Then run the ecosystem-bound lanes from the parent Mycelix workspace:

```sh
cargo check -p prism-ui --target wasm32-unknown-unknown --locked
cargo test -p prism-ui --locked
cargo check -p prism-tauri --locked
cargo test -p prism-tauri --locked
cargo check -p prism-knowledge-bridge --locked
```

## Integration and adversarial gates

The following must be demonstrated before describing Wave 3 as production
complete:

1. A real renderer and Spore process share a supervisor-provisioned bridge key,
   exchange authenticated frames, and reject replay after reconnect.
2. A review-ledger retraction removes the claim from a rebuilt index and changes
   the manifest digest deterministically.
3. An edited queued DHT claim fails publication reauthorization.
4. Compatibility launch occurs only after exact-origin approval and produces a
   fresh private profile outside the Prism renderer process.
5. A browser-supplied `X-Brave-Key` or `X-Perplexity-Key` cannot influence either
   server binary.
6. Provider routes remain 503 unless `PRISM_ENABLE_PROVIDER_PROXY=true` and a
   server-side provider secret are both present.
7. Public deployments enforce authenticated per-account quotas outside the
   built-in global concurrency ceiling.

## Known limitations

- Keyed BLAKE3 authenticates the bridge but does not negotiate keys; secure key
  provisioning and rotation remain supervisor responsibilities.
- Strict sequence numbers intentionally reject reordered frames; reconnect
  semantics need an explicit fresh session ceremony.
- The review ledger is schema-validated but not yet signed or transparency-
  logged.
- Chromium runs in a separate process and profile but not yet a dedicated OS
  network/user namespace.
- Static syntax validation cannot replace Rust type checking, Clippy, WASM
  compilation, Tauri integration, or runtime fault injection.
