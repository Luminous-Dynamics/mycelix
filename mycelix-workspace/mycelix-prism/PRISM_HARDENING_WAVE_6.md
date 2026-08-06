# Prism Hardening Wave 6

Wave 6 closes the gap between Prism's source-level policies and a release that
can be independently reconstructed, reviewed, and hybrid-signed. It does not
add browsing features. It strengthens the reviewer cryptographic contract,
loads trust material through admitted filesystem paths, binds reviewed indices
to deterministic receipts, captures canonical build evidence, and defines a
portable release-attestation ceremony.

## Scope

This wave contains fourteen ordered patch sets:

1. **Downgrade-resistant hybrid reviewer signatures**
   - Defines one canonical binary envelope containing mandatory Ed25519 and
     ML-DSA-65 components.
   - Domain-separates both components by key ID, algorithm, and the exact review
     message.
   - Rejects missing, oversized, truncated, downgraded, or cross-key signatures.
   - Leaves primitive cryptography in an independently admitted verifier agent.

2. **Strict review trust-file admission**
   - Opens anchor, policy, and ledger files without following symbolic links.
   - Requires absolute, distinct, regular, bounded files.
   - Rejects group/world-writable files and unexpected hard links.
   - Hashes the exact bytes parsed into the review typestate.

3. **Agent-verified reviewed index construction**
   - Connects Prism to an algorithm-scoped local Unix verifier agent.
   - Binds responses to exact request IDs, key IDs, algorithms, messages, and
     signatures.
   - Verifies Linux peer credentials and applies the authenticated ledger before
     claims enter either core or full search indices.

4. **Verifier-agent executable measurement**
   - Measures `/proc/<pid>/exe` before and after every primitive verification.
   - Requires an exact BLAKE3 pin or a canonical immutable Nix-store path.
   - Rejects deleted, replaced, mismatched, or mid-request-changing agents.

5. **Compatibility executable admission**
   - Applies the same exact-digest or immutable-Nix-store policy to Chromium and
     Bubblewrap.
   - Requires executable permission bits, no-follow reads, canonical paths, and
     pre-launch remeasurement.
   - Records browser, sandbox, and admission identities in launch receipts.

6. **Reviewed-only index release mode and publication receipts**
   - Adds `PRISM_REQUIRE_REVIEWED_INDEX=true` to reject embedded unreviewed
     indices in production.
   - Atomically publishes a deterministic receipt binding core and full snapshot
     manifests, snapshot BLAKE3 digests, sizes, and exact trust-file digests.
   - Labels embedded and authenticated-reviewed builds distinctly.

7. **Canonical Rust and Nix build evidence**
   - Rejects dirty source trees and build-affecting compiler/Nix wrappers.
   - Runs a fixed Cargo command set in frozen mode with Rust 1.85.1.
   - Captures exact commit, tree, lockfile, tool executable, version, command,
     stdout, and stderr identities.
   - Requires a reviewed `flake.lock` for the release Nix lane and forbids lock
     updates.
   - Publishes evidence directories atomically only after every command passes.

8. **Hybrid-signed release statements**
   - Adds the portable `prism-attestation` crate.
   - Canonically binds the source commit/tree, Cargo lock, Nix lock, Rust build
     evidence, Nix build evidence, reviewed index publication, reviewer policy,
     and review-ledger root.
   - Requires both Ed25519 and ML-DSA-65 release signature components over the
     same domain-separated statement.

9. **Live verifier-agent transport tests**
   - Exercises the real Unix framing path on Linux.
   - Tests peer UID measurement, executable pinning, request/response binding,
     and response-substitution rejection.

10. **Release attestation preparation**
    - Reads only absolute, no-follow, bounded, non-writable, single-link evidence
      files.
    - Confirms Rust and Nix receipts describe the same clean Git tree and exact
      dependency locks.
    - Requires an authenticated-reviewed index receipt and derives review roots
      from its core/full manifests.
    - Atomically emits the canonical payload for an external hybrid signer.

11. **Independent release verification**
    - Remeasures all five attested artifacts and requires the repository's exact
      `Cargo.lock` and `flake.lock`.
    - Revalidates build command sets, compiler identity, lock hashes, source
      identity, reviewed index roots, and snapshot receipt structure.
    - Uses an admitted local agent to verify both release signature components.

12. **Honest Compatibility egress policy**
    - Stops treating browser proxy flags as kernel egress enforcement.
    - Requires `PRISM_COMPAT_ALLOW_SHARED_NETWORK_PROXY=true` before admitting a
      loopback proxy while Chromium shares the host network namespace.
    - Keeps fully direct networking behind its own explicit degradation flag.
    - Fails closed when only a proxy URL is configured.

13. **Reproducible Wave 6 static evidence**
    - Extends workspace, dependency, action-pin, policy, and source invariants.
    - Checks the hybrid envelope, trust-file admission, verifier measurement,
      index receipts, executable admission, build evidence, attestation tools,
      and explicit network degradation.
    - Commits separate static and supply-chain receipts without claiming runtime
      execution.

14. **Campaign documentation**
    - Records deployment inputs, release ceremony, validation lanes,
      limitations, merge order, and the remaining security frontier.

## Production inputs

### Reviewed index construction

A production reviewed build requires:

- `PRISM_REVIEW_ANCHOR_PATH`
- `PRISM_REVIEW_POLICY_PATH`
- `PRISM_REVIEW_LEDGER_PATH`
- `PRISM_REVIEW_VERIFIER_SOCKET`
- `PRISM_REVIEW_VERIFIER_UID`
- either `PRISM_REVIEW_VERIFIER_EXPECTED_BLAKE3` or an agent in `/nix/store`
- `PRISM_REQUIRE_REVIEWED_INDEX=true`

The three trust files must be independently provisioned, absolute, bounded,
non-writable by group/others, and not hard-linked. The verifier socket alone is
not trust: Prism also checks the peer UID and executable identity for every
request.

### Compatibility process

Normal executable admission requires:

- `PRISM_COMPAT_CHROMIUM_PATH`
- `PRISM_COMPAT_BWRAP_PATH`
- exact `PRISM_COMPAT_CHROMIUM_EXPECTED_BLAKE3` and
  `PRISM_COMPAT_BWRAP_EXPECTED_BLAKE3`, unless both files are immutable
  Nix-store paths

Wave 6 does **not** implement kernel-mediated Compatibility egress. The current
choices are explicit degradations:

- shared host network plus browser-enforced loopback proxy:
  `PRISM_COMPAT_PROXY_URL=http://127.0.0.1:<port>` and
  `PRISM_COMPAT_ALLOW_SHARED_NETWORK_PROXY=true`;
- fully direct browser networking:
  `PRISM_COMPAT_ALLOW_DIRECT_NETWORK=true`.

A deployment that does not accept either degradation must keep Compatibility
mode disabled until a namespace egress broker exists.

### Release verifier agent

Independent release verification requires:

- `PRISM_RELEASE_VERIFIER_SOCKET`
- `PRISM_RELEASE_VERIFIER_UID`
- either `PRISM_RELEASE_VERIFIER_EXPECTED_BLAKE3` or an immutable Nix-store
  verifier agent

The agent exposes algorithm-specific Ed25519 and ML-DSA-65 verification. Prism
owns the release statement, hybrid envelope, domain separation, and requirement
that both components pass.

## Canonical build evidence

Pinned Rust lane:

```sh
scripts/verify-ci.sh
```

The command writes evidence below `target/prism-evidence/canonical-rust/` only
when metadata, formatting, check, test, Clippy, static policy, and supply-chain
commands all pass.

Pinned Nix lane, after generating and independently reviewing `flake.lock`:

```sh
python3 scripts/capture-build-evidence.py \
  --lane nix \
  --release \
  --output target/prism-evidence/canonical-nix
```

The Nix lane uses `--no-update-lock-file`. CI separately performs a metadata
preflight with the same restriction.

## Reviewed index publication

```sh
PRISM_REQUIRE_REVIEWED_INDEX=true \
PRISM_REVIEW_ANCHOR_PATH=/absolute/review-anchor.json \
PRISM_REVIEW_POLICY_PATH=/absolute/review-policy.json \
PRISM_REVIEW_LEDGER_PATH=/absolute/review-ledger.json \
PRISM_REVIEW_VERIFIER_SOCKET=/absolute/review-verifier.sock \
PRISM_REVIEW_VERIFIER_UID=<uid> \
cargo run --frozen -p prism-search --bin build_index -- /absolute/index-output
```

Successful output includes:

- `prism-index-core.bin`
- `prism-index.bin`
- `prism-index-publication.json`

The publication receipt is the artifact supplied to the release preparer.

## Release ceremony

1. Start from the exact clean commit intended for release.
2. Generate and review `flake.lock` on the canonical Nix host.
3. Capture successful Rust and Nix evidence directories.
4. Build authenticated-reviewed core and full index snapshots.
5. Prepare the unsigned canonical statement:

```sh
cargo run --frozen -p prism-attestation --bin prism-prepare-release -- \
  <version> \
  /absolute/canonical-rust/receipt.json \
  /absolute/canonical-nix/receipt.json \
  /absolute/prism-index-publication.json \
  /absolute/unsigned-release.json
```

6. Submit `canonical_payload_hex` to the approved hybrid signer and construct a
   `SignedReleaseAttestation` with the exact returned envelope.
7. Independently verify the signed release:

```sh
PRISM_RELEASE_VERIFIER_SOCKET=/absolute/release-verifier.sock \
PRISM_RELEASE_VERIFIER_UID=<uid> \
cargo run --frozen -p prism-attestation --bin prism-verify-release -- \
  /absolute/signed-release.json \
  "$PWD/Cargo.lock" \
  "$PWD/flake.lock" \
  /absolute/canonical-rust/receipt.json \
  /absolute/canonical-nix/receipt.json \
  /absolute/prism-index-publication.json
```

## Static verification

```sh
python3 scripts/verify-wave6-static.py \
  --check security/wave6-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

Release-mode supply-chain admission is expected to fail until `flake.lock`
exists:

```sh
python3 scripts/verify-supply-chain.py --release
```

## What Wave 6 proves statically

The committed evidence establishes that the source tree contains:

- an all-or-nothing hybrid reviewer envelope;
- admitted and measured review trust inputs and verifier processes;
- reviewed-only index configuration and atomic publication receipts;
- admitted and remeasured Compatibility executables;
- a fixed frozen Cargo/Nix evidence command set;
- a canonical release statement and independent hybrid verifier;
- explicit rather than implicit acceptance of shared Compatibility networking;
- admitted registry, checksum, workspace path, and immutable workflow-action
  policies.

## What Wave 6 does not prove

This environment did not provide Cargo, rustc, Nix, Bubblewrap, Chromium, or a
real reviewer/release verifier agent. Wave 6 therefore does not claim:

- successful Rust formatting, compilation, Clippy, tests, WASM builds, or
  benchmarks;
- successful Nix evaluation or reproducible derivation output;
- a generated or reviewed `flake.lock`;
- correctness or side-channel resistance of concrete Ed25519 or ML-DSA
  implementations supplied by an external agent;
- live process, namespace, mount, seccomp, Chromium, proxy, or lifecycle
  enforcement;
- kernel-mediated Compatibility egress;
- resistance to a compromised kernel, display server, CI runner, Nix daemon,
  reviewer trust channel, signing agent, or release operator.

## Merge order

Apply every Wave 6 patch in order. The hybrid envelope changes reviewer
verification; the secure store and agent patches feed reviewed index
construction; index receipts feed release preparation; build-evidence and
attestation schemas are hashed by the final static receipt. Reordering creates a
source tree whose evidence no longer describes its effective release contract.

## Next frontier

1. Generate, review, and commit `flake.lock`; then run the canonical Rust and Nix
   lanes and retain their receipts and logs.
2. Integrate independently reviewed Ed25519 and ML-DSA-65 implementations in a
   production verifier/signing agent, preferably hardware- or enclave-backed.
3. Implement a dedicated Compatibility network namespace connected only to an
   authenticated Prism egress broker, eliminating the shared-network
   degradation.
4. Add seccomp and Landlock policies to Spore, Compatibility, reviewer-agent,
   and release-agent processes.
5. Publish signed release statements and reviewer transparency roots through an
   append-only external transparency service.
