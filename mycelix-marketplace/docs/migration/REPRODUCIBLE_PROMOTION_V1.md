# Reproducible Marketplace Promotion v1

## Purpose

A Marketplace build is not promoted merely because source tests pass. Promotion requires an exact artifact build, real signed multi-agent Holochain execution, artifact-bound receipts, and an independently verifiable release signature.

This process is deliberately stricter than the ordinary development loop. It is intended for release evidence, not rapid local iteration.

## Trust boundary

The Nix flake pins the general promotion environment: Node, Python, OpenSSL, Rustup, binary tooling, and validation utilities. The repository pins Rust `1.94.0` in `rust-toolchain.toml` and the browser client to `@holochain/client` `0.20.5`.

Holochain `0.6.1` and `hc 0.6.1` are external trust anchors. Their exact versions, resolved binary paths, and SHA-256 digests are captured in `toolchain.json`. Promotion fails if they are absent or report another version. This repository does not claim that the older Holochain flake references elsewhere reproduce those binaries.

## Profiles

- **base** — lifecycle evidence with active Marketplace role.
- **arbitration** — lifecycle plus three-agent arbitration evidence. The dispute UI remains gated unless conflict injection is proven.
- **settlement** — lifecycle plus Finance settlement and recovery. Requires an exact patched Finance DNA and an executable funding/bootstrap hook.
- **network** — two independent conductors, controlled local bootstrap/signaling, measured propagation, partition-local divergent writes, identical explicit conflict after healing, and matching bilateral authority over one preserved branch.

## Build

From a clean checkout:

```sh
nix develop .#promotion
rustup toolchain install 1.94.0 --profile minimal \
  --component rustfmt,clippy --target wasm32-unknown-unknown
rustup override set 1.94.0

scripts/build-promotion-artifacts.sh base /tmp/marketplace-promotion
```

Settlement additionally requires:

```sh
export FINANCE_DNA_PATH=/absolute/path/to/patched-finance.dna
scripts/build-promotion-artifacts.sh settlement /tmp/marketplace-promotion
```

The builder:

1. Refuses a dirty source tree.
2. Verifies the pinned toolchain.
3. Builds the backend workspace with `--locked --release` for `wasm32-unknown-unknown`.
4. Maps the 14 exact Cargo outputs into globally unique DNA resource paths.
5. Packs the Marketplace DNA and selected hApp profile.
6. Emits zome, toolchain, artifact, source-revision, and SHA-256 receipts.

## Disposable live execution

```sh
scripts/run-disposable-promotion.sh base /tmp/marketplace-promotion
```

The harness starts a temporary conductor using a dangerous test keystore in a disposable data root. It installs a distinct copy of the exact hApp for each actor, with distinct agent keys, installed app IDs, app interfaces, authentication tokens, and signing credentials.

The topology is intentionally `single_conductor_multi_agent`. This proves real signed zome calls and agent separation without depending on an external bootstrap or signaling service. It does not prove wide-area network behavior, partition tolerance, or multi-conductor convergence.

For settlement, `MARKETPLACE_SETTLEMENT_BOOTSTRAP` must name an executable controlled-runner hook. The hook receives the private actor environment-file path and artifact-manifest path. Its digest enters the evidence bundle; its contents and credentials do not.

Tokens are stored in a mode-0600 temporary file, sourced only for the live process, deleted at cleanup, and rejected by the release sealer if found in evidence.

## Multi-conductor network execution

The separate `network` profile uses `scripts/run-network-promotion.sh`. It requires controlled local network-service and partition/heal hooks, launches distinct seller and buyer conductor processes, and records both logs and configuration digests.

The behavioral gate is stronger than process counting. It first proves a safe pre-shipment cancellation projection, then proves that shipped-versus-cancelled remains unresolved after healing. Buyer and seller must subsequently publish separate approvals for the same existing head, both approvals must propagate, and both peers must expose the same `AuthorizedResolved` projection while retaining the original shipped and cancelled heads. Public bootstrap/signaling endpoints, unilateral authority, hidden branches, and arbitrary conflict winners are rejected.

See `MULTI_CONDUCTOR_NETWORK_EVIDENCE_V1.md` for the hook contract, assertions, and limitations.

## Signed release bundle

Generate an Ed25519 key outside the repository or use a protected release key:

```sh
openssl genpkey -algorithm ED25519 -out /secure/marketplace-release-key.pem
export MARKETPLACE_RELEASE_SIGNING_KEY=/secure/marketplace-release-key.pem
scripts/seal-promotion-bundle.sh base /tmp/marketplace-promotion \
  /tmp/mycelix-marketplace-base.tar.gz
```

The signed bundle contains:

- Exact hApp and DNA artifacts
- Finance DNA for settlement profiles
- Artifact, zome, and toolchain receipts
- Live lifecycle/arbitration/settlement receipts
- Promotion decision and limitations
- Sanitized conductor log and configuration digest
- Public release key
- Ed25519 signature over a canonical release manifest

The release manifest records the path, byte size, and SHA-256 digest of every payload file. Symlinks, path traversal, unmanifested files, private keys, token files, and credential-like content are rejected.

Verify without access to the private key:

```sh
scripts/verify-promotion-bundle.sh /tmp/mycelix-marketplace-base.tar.gz
```

Verification checks the signature, every payload digest, release/artifact identity, exact live receipts, required roles, scenario assertions, and promotion result.

## Controlled workflow

`.github/workflows/disposable-promotion.yml` is manual and restricted to a self-hosted runner carrying the labels:

- `self-hosted`
- `mycelix-conductor`
- `nix`

The runner must provide exact Holochain and `hc` binaries. Settlement additionally requires the patched Finance DNA, funded test setup, and bootstrap hook. The workflow uploads only the signed tarball, not raw tokens, private keys, actor environment files, or the mutable working directory.

## Claims this evidence does not establish

A passing bundle does not establish:

- Production readiness
- Legal settlement finality
- Atomic rollback across Holochain and Finance
- Byzantine tolerance at any percentage
- Internet-scale or geographically distributed convergence
- Security of the controlled runner or release private key
- Correctness of dependencies beyond their pinned artifacts and exercised behavior

Those require separate threat models and evidence.
