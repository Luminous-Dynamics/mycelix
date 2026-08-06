# Prism Hardening Wave 5

Wave 5 turns the remaining production assumptions from Wave 4 into explicit,
reviewable admission policies. It does not add browser features. It narrows who
may classify knowledge, which executable may become Spore, how active-content
networking may leave the host, and what build evidence may authorize release.

## Scope

This wave contains ten ordered patch sets:

1. **Reviewer key policy and quorum**
   - Maps every admitted key to a named reviewer, algorithm, role set, validity
     interval, and optional revocation time.
   - Requires decision-specific distinct-reviewer quorum and required roles.
   - Prevents multiple keys owned by one person from satisfying a multi-person
     quorum.
   - Carries the verified policy digest into the review-bound search manifest.

2. **Default-deny Spore sandbox**
   - Runs Spore in Bubblewrap with private user, PID, IPC, UTS, cgroup, and
     network namespaces.
   - Presents a read-only host root, private writable session directories, and
     masked user, storage, and secret locations.
   - Clears the inherited environment and reconstructs only the minimal runtime
     values required by the child.
   - Requires an explicit emergency flag for an unsandboxed child.

3. **Executable admission and pinning**
   - Admits Spore and Bubblewrap through an exact operator-supplied BLAKE3 digest
     or a canonical immutable `/nix/store` path.
   - Requires separate emergency flags for mutable unpinned executables.
   - Rehashes admitted files immediately before spawning the child.
   - Reports the selected admission mode and digest through shell security
     status without exposing session secrets.

4. **Compatibility network mediation**
   - Requires a literal-loopback HTTP proxy by default.
   - Configures Chromium to avoid implicit loopback bypass, direct hostname
     resolution, QUIC, and non-proxied WebRTC UDP.
   - Records the network mode in each compatibility launch receipt.
   - Requires a deliberate operator downgrade for direct networking.

5. **Pinned Rust and Nix verification lanes**
   - Pins Rust 1.85.1, Clippy, rustfmt, and the WASM target.
   - Defines a Nix portable-core closure that uses the same toolchain through
     rust-overlay and the committed Cargo lockfile.
   - Adds self-hosted CI contracts that refuse to update Nix pins.
   - Pins the checkout action by immutable commit rather than a mutable tag.

6. **Dependency and workflow admission**
   - Rejects Git dependencies, unknown registries, missing registry checksums,
     wildcard versions, workspace paths escaping the repository, and mutable
     GitHub Action references.
   - Provides a cargo-deny policy for advisory, source, license, and wildcard
     review.
   - Treats a missing `flake.lock` as an explicit release blocker.

7. **External reviewer trust anchor**
   - Requires a separately provisioned policy ID and policy digest in addition
     to a valid reviewer policy.
   - Rejects changes to reviewers, algorithms, roles, validity windows,
     revocations, or quorum rules before ledger signatures are considered.
   - Keeps the trust root outside the ledger and policy files it authenticates.

8. **Reproducible Wave 5 evidence**
   - Adds a standard-library-only verifier and committed machine-readable
     receipt.
   - Checks source policy invariants, lockfile source admission, immutable
     workflow actions, exact toolchain declarations, Spore isolation,
     executable admission, reviewer anchoring, and Compatibility mediation.
   - Reports unsupported runtime claims and release blockers instead of
     silently promoting static checks into runtime evidence.

9. **Spore endpoint boundary correction**
   - Removes supervisor-only fields accidentally referenced by the child-side
     endpoint's `Debug` implementation.
   - Regenerates the evidence receipt after the correction.

10. **Campaign documentation**
    - Records merge order, deployment ceremony, validation lanes, limitations,
      and the next security frontier.

## Production configuration

### Reviewer governance

A production host must provision three independent inputs:

1. the append-only review ledger;
2. the reviewer policy defining keys, roles, validity, revocation, and quorum;
3. a read-only `ReviewTrustAnchor` containing the expected policy ID and BLAKE3
   digest.

The anchor should arrive through a channel independent of the ledger and policy,
such as a Nix closure, measured-boot policy, signed operator configuration, or
hardware-backed trust store. Replacing all three files together is not an
independent trust boundary.

### Spore process

Required normal-mode settings:

- `PRISM_SPORE_EXECUTABLE`
- `PRISM_SPORE_BWRAP_PATH`
- either `PRISM_SPORE_EXPECTED_BLAKE3` or an immutable Nix-store executable
- either `PRISM_SPORE_BWRAP_EXPECTED_BLAKE3` or an immutable Nix-store Bubblewrap

Emergency downgrade settings are intentionally separate:

- `PRISM_SPORE_ALLOW_UNSANDBOXED=true`
- `PRISM_SPORE_ALLOW_UNPINNED_EXECUTABLE=true`
- `PRISM_SPORE_ALLOW_UNPINNED_BWRAP=true`

These flags should never appear in normal service definitions.

### Compatibility process

Normal mode requires:

- `PRISM_COMPAT_CHROMIUM_PATH`
- `PRISM_COMPAT_BWRAP_PATH`
- `PRISM_COMPAT_PROXY_URL=http://127.0.0.1:<port>` or an equivalent literal IPv6
  loopback address

`PRISM_COMPAT_ALLOW_DIRECT_NETWORK=true` is an explicit degradation. Browser
flags reduce direct egress opportunities but are not a kernel firewall. High
assurance deployments should also enforce compatibility egress through a
network namespace, transparent proxy, VM, cgroup policy, or external firewall.

## Canonical validation

On the pinned Rust host:

```sh
scripts/verify-ci.sh
```

On the pinned Nix host after `flake.lock` has been generated and reviewed:

```sh
nix --extra-experimental-features 'nix-command flakes' \
  flake check --no-update-lock-file --print-build-logs
```

Static source and supply-chain evidence:

```sh
python3 scripts/verify-supply-chain.py
python3 scripts/verify-wave5-static.py \
  --check security/wave5-static-evidence.json
```

Release admission additionally requires:

```sh
python3 scripts/verify-supply-chain.py --release
```

The release command is expected to fail until a reviewed `flake.lock` exists.
CI must not generate or update that lock implicitly.

## What this wave proves statically

The committed evidence verifies that:

- review records require a scoped reviewer policy, quorum, and external policy
  digest anchor;
- the Spore command construction requests a no-network Bubblewrap boundary;
- admitted Spore and Bubblewrap files are content-bound and rechecked;
- Compatibility mode requires an explicit proxy or explicit direct-network
  downgrade;
- workspace path dependencies remain inside the portable source tree;
- Cargo registry entries use the admitted registry and carry checksums;
- GitHub Actions are pinned to immutable commits;
- Rust and Nix lanes refer to the declared toolchain and lockfile contracts.

## What this wave does not prove

The current environment did not provide Cargo, rustc, Nix, Bubblewrap, a Spore
binary, or a controlled Compatibility proxy. Therefore this campaign does not
claim:

- successful Rust compilation, Clippy, tests, benchmarks, or WASM builds;
- successful Nix evaluation or reproducible derivation output;
- live namespace, mount, seccomp, or process-lifecycle enforcement;
- live proxy authentication, DNS pinning, redirect control, or egress blocking;
- cryptographic correctness of the host-provided reviewer signature verifier;
- resistance to a compromised kernel, display server, Chromium build, Nix
  daemon, CI runner, or operator trust-anchor channel.

Those claims require the canonical workspace and deployment environment.

## Merge order

Apply the patch sets exactly in numeric order. The reviewer-policy digest changes
search snapshot evidence; the trust-anchor patch changes the non-empty ledger
verification API; and the evidence receipt hashes the final security-critical
files. Reordering these patches can produce a source tree whose evidence no
longer describes its policy surface.

## Next frontier

The next highest-leverage work is:

1. generate and review `flake.lock`, then capture successful Nix and Cargo
   evidence from the canonical workspace;
2. integrate a concrete Ed25519/ML-DSA or hybrid reviewer verifier and a
   hardware- or agent-backed trust-anchor loader;
3. give the Compatibility process a dedicated network namespace connected only
   to an authenticated Prism egress proxy;
4. add seccomp/Landlock policy and executable measurement to the Spore launch;
5. publish signed build and review transparency statements suitable for
   independent verification outside the running Prism host.
