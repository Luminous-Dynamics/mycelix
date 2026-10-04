# D6U trusted-builder boundary

Status: Experimental; security architecture

## Purpose

The D6U runtime workflow executes pull-request code and therefore must be treated as an untrusted measurement environment. The default branch owns the signing authority separately.

The trusted workflow in `.github/workflows/d6u-trusted-evidence-attestation.yml` is triggered by completion of the D6U runtime workflow through `workflow_run`. It runs from `main`, downloads the completed run's artifact into the runner's temporary directory, and validates the artifact as data only.

It does not check out the pull-request head, execute files from the downloaded artifact, import pull-request Python or Rust modules, or use pull-request code as its policy root. It also rejects fork-origin runs; the signing boundary is same-repository only.

## Trusted policy

`docs/integral/d6u-trusted-builder-policy.json` is the independently reviewed policy root.

It pins:

- the D6U workflow identity;
- the D6U manifest and D6S-CANON-2 fixture identities;
- the D6S-CANON-1 corpus identity;
- the D6U harness executable surface;
- the expected runtime versions;
- all native case outcomes and zome reachability;
- supplemental substrate witnesses;
- lockfile substrate versions and crates.io provenance;
- the `ReferenceModelOnly` claim ceiling.

Before signing, the trusted verifier obtains the actual Git blob SHA for every tracked source file from GitHub at the triggering run's exact `head_sha`. The artifact's self-reported hashes therefore cannot substitute for the repository state.

## Attestation boundary

The D6U pull-request workflow has `contents: read` only and explicitly defers attestation. The trusted workflow owns:

- `id-token: write`;
- `attestations: write`;
- `artifact-metadata: write`.

The trusted workflow signs the exact downloaded evidence, runtime log, and Cargo.lock only after independent verification.

A successful D6U pull-request run therefore means runtime evidence was produced and uploaded. A trusted signed attestation means the default-branch verifier accepted that evidence against its independently reviewed policy and signed the exact resulting bytes.

Neither event upgrades the D6S claim ceiling beyond `ReferenceModelOnly`.
