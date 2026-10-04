# D6U runtime qualification trust model

Status: Experimental; ReferenceModelOnly

## Execution authority

The D6U workflow is introduced by this pull request. The `pull_request` event executes the workflow definition associated with the pull request merge context, while the job deliberately checks out and executes the exact pull-request head commit.

This means the D6U result is strong evidence that the submitted head executed successfully under the declared D6U harness, but it is not an independent base-branch policy authority.

A future promotion from `ReferenceModelOnly` to a stronger qualification claim should use a trusted workflow or equivalent policy root that is established outside the change being qualified.

## Evidence chain

The current PR-controlled chain is:

1. Static manifest verification.
2. Exact PR-head checkout assertion.
3. Pinned Rust/toolchain identity.
4. Direct dependency version/source verification.
5. Generated Cargo.lock substrate-package verification.
6. Real Holochain 0.7 Sweettest execution.
7. Per-case evidence verification against D6S-CANON-2.
8. Runtime evidence-record reconstruction against repository state and workflow identity.
9. Negative regression checks proving the evidence verifiers reject tampering.
10. Unprivileged upload of the captured runtime evidence artifacts.

The PR-controlled workflow deliberately does **not** mint artifact attestations and does not request `id-token` or `attestations: write`. Signed provenance is deferred to a trusted default-branch builder that does not execute PR-controlled code.

The exact PR-head qualification subject is recorded separately from GitHub's `GITHUB_SHA` merge-context digest; the former identifies what the runtime job actually checked out, while the latter is reserved as an input to a future trusted attestation workflow.

## Claim boundary

The harness currently demonstrates native Holochain 0.7 behavior for 14 of the 17 D6S-CANON-2 reference cases.

The three non-native cases remain reference-only:

- authenticated-but-not-yet-authorized as an isolated intermediate state;
- distinct stale/older nonce;
- pre-zome D6S commitment-integrity rejection.

The D6S payload mutation is therefore reported only as a probe-local application check and must not be interpreted as native Holochain enforcement.

No successful runtime execution of this harness upgrades the claim ceiling by itself.
