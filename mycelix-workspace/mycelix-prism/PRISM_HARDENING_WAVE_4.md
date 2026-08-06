# Prism Hardening Wave 4

**Date:** 2026-07-20
**Base:** Wave 3 commit `bf575c8`
**Purpose:** turn Wave 3's security primitives into supervised process lifecycles, authenticated review evidence, OS-enforced active-content isolation, and reproducible cross-crate gates.

## Executive summary

Wave 4 closes the gap between having strong library primitives and actually enforcing them across process, corpus, index, rendering, and compatibility boundaries.

The central invariant is now:

> Untrusted bytes do not become trusted markup, encoded memory, published knowledge, or executable active content without an explicit capability whose evidence survives every boundary it crosses.

## Ordered patch sets

### 1. Search claims are data, never markup

`search: render imported claims as data-only HTML`

- Escapes queries, claim text, sources, and tags before constructing local result documents.
- Prevents imported or peer-provided text from changing the trusted search document structure.
- Adds regression coverage for script, image, attribute, and closing-tag injection.

### 2. Supervised authenticated Spore sessions

`bridge: supervise Spore sessions over private authenticated sockets`

- Generates fresh session keys and identifiers per Spore process.
- Delivers bootstrap material through inherited standard input instead of command-line arguments or environment variables.
- Creates a private per-session Unix socket directory.
- Uses the authenticated, ordered bridge codec for every exchange.
- Health-checks the child before declaring the session usable.
- Owns child termination and socket cleanup as one lifecycle.

### 3. Shell-owned Spore lifecycle

`shell: own the authenticated Spore process lifecycle`

- Connects the native shell to `SupervisedSpore` rather than treating the codec as an unused library.
- Supports an operator-selected `PRISM_SPORE_EXECUTABLE`.
- Supports fail-closed startup with `PRISM_REQUIRE_SPORE=true`.
- Exposes authenticated/offline state without exposing key material.

### 4. Authenticated append-only review transparency

`ingest: require authenticated append-only review ledgers`

- Advances the review schema to v2.
- Adds sequence continuity, previous-record hashes, monotonic review times, canonical signing messages, detached signatures, and a stable transparency root.
- Introduces `ReviewSignatureVerifier` so the canonical host selects the trust root and algorithm policy.
- Introduces `VerifiedReviewLedger`; raw JSON cannot alter the corpus.
- Requires reviewer identity, rationale, and evidence for confirmations.
- Rejects modified, removed, reordered, unsigned, or invalidly signed records.

### 5. Atomic evidence-bearing index publication

`search: publish evidence-bound indexes atomically`

- Publishes snapshots through same-directory create-new temporary files, file `fsync`, atomic rename, and parent-directory `fsync`.
- Returns an `IndexPublicationReceipt` containing the manifest, byte count, and exact snapshot digest.
- Prevents readers from observing partial index writes.

### 6. OS-enforced Compatibility isolation

`compatibility: require an explicit OS sandbox boundary`

- Makes Bubblewrap the default Linux compatibility boundary.
- Uses new user, PID, IPC, UTS, and cgroup namespaces; a read-only root; masked user and secret directories; and a fresh private profile.
- Removes sensitive inherited environment variables.
- Immediately destroys the disposable profile after the browser exits.
- Refuses to launch without Bubblewrap unless the operator deliberately enables browser-sandbox-only fallback.
- Never passes Chromium's `--no-sandbox` flag.

### 7. Cross-crate trust-boundary regression gates

`security: add cross-crate trust-boundary regression gates`

- Adds the portable `prism-security-tests` workspace crate.
- Tests privacy capability minting through authenticated IPC, including replay and tamper rejection.
- Tests signed confirmation and retraction through corpus application and index publication.
- Advances index snapshot schema to v4.
- Adds review-specific index build, publish, read, and decode APIs that bind the exact `VerifiedReviewLedger` transparency root.
- Rejects a valid snapshot when opened under a different authenticated review history.
- Tests that imported claims remain data in generated result documents.

### 8. Reproducible static evidence

`security: publish reproducible Wave 4 evidence receipt`

- Adds `scripts/verify-wave4-static.py`, implemented with the Python standard library.
- Validates workspace membership, direct lockfile dependencies, TOML and JSON parsing, forbidden production patterns, and required Wave 4 APIs.
- Emits `security/wave4-static-evidence.json` with counts and SHA-256 digests for security-critical files.
- Corrects stale lockfile dependency entries discovered by the audit.

## Deployment order

1. Merge the data-only search-rendering patch.
2. Merge bridge runtime supervision and shell ownership together.
3. Configure the canonical Spore executable and decide whether startup must be fail-closed.
4. Provide a concrete reviewer signature verifier and trusted key policy.
5. Rebuild the corpus and publish an index with the matching review-specific APIs.
6. Install and pin Bubblewrap; set `PRISM_COMPAT_BWRAP_PATH` to the reviewed executable.
7. Run `cargo test -p prism-security-tests` before enabling publication or Compatibility mode.
8. Regenerate the static evidence receipt and compare it with the committed receipt.

## Required canonical validation

```bash
python3 scripts/verify-wave4-static.py --output security/wave4-static-evidence.json
git diff --exit-code -- security/wave4-static-evidence.json

cargo fmt --all -- --check
cargo check --workspace --all-targets --locked
cargo clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo test --workspace --all-features --locked
cargo test -p prism-security-tests --locked
```

The canonical workspace should additionally execute:

- a real Spore child fixture through the supervised Unix-socket lifecycle;
- signature verification with the production reviewer algorithm and trust root;
- Bubblewrap namespace and filesystem-denial tests on Linux;
- crash/power-loss tests around index publication;
- WASM and Tauri build lanes for ecosystem-bound crates excluded from the portable core workspace.

## Remaining limitations

- `ReviewSignatureVerifier` is intentionally policy-neutral; this snapshot does not select Ed25519, ML-DSA, or a hybrid trust policy.
- Bubblewrap meaningfully reduces host exposure but is not a VM boundary and still shares the host kernel.
- The compatibility browser still needs network egress policy, download mediation, and portal/device restrictions appropriate to the deployment.
- A malicious or compromised Spore executable can misuse content deliberately sent to it; executable provenance and sandboxing remain host responsibilities.
- Static evidence does not substitute for compilation, Clippy, runtime, fuzzing, or OS-level tests.

## Result

Wave 4 changes Prism from a set of hardened boundaries into a more coherent security lifecycle:

```text
imported bytes
  -> data-only document
  -> assessed privacy capability
  -> authenticated supervised IPC
  -> signed review transparency
  -> review-root-bound atomic index
  -> isolated active-content broker
```

The next highest-leverage work is production cryptographic key policy for reviewers, sandboxing the Spore process itself, compatibility-network mediation, and reproducible Cargo/Nix CI evidence from the canonical workspace.
