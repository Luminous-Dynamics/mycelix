# D6U trusted-builder boundary

Status: Experimental; security architecture

## Purpose

The D6U runtime workflow executes pull-request code and therefore must be treated as an untrusted measurement environment. The default branch owns the signing authority separately.

The trusted workflow in `.github/workflows/d6u-trusted-evidence-attestation.yml` is triggered by completion of the D6U runtime workflow through `workflow_run`. The trusted job checks out the exact `github.workflow_sha` for the trusted workflow execution, asserts that the checkout matches that SHA, downloads the completed run's artifact into the runner's temporary directory, and validates the artifact as data only.

It does not check out the pull-request head, execute files from the downloaded artifact, import pull-request Python or Rust modules, or use pull-request code as its policy root. The executor records the upstream D6S run ID/attempt, source branch/repository/SHA, trigger-workflow identity, and its own executor-workflow identity. The source-side verifier redundantly rejects the wrong qualification branch, while the trusted builder independently re-fetches the upstream D6S run and checks the exact source tree. It also rejects fork-origin runs; the signing boundary is same-repository only.

## Trusted policy

`docs/integral/d6u-trusted-builder-policy.json` is the independently reviewed policy root.

It pins:

- the D6U workflow identity and exact main-branch executor blob;
- the D6U manifest and harness executable surface;
- the D6S-CANON-1 manifest, golden corpus, and independent verifier;
- the D6S-CANON-2 manifest, authority-boundary fixture, and independent verifier;
- the expected runtime versions;
- all native case outcomes and zome reachability;
- supplemental substrate witnesses;
- lockfile substrate versions and crates.io provenance;
- the `ReferenceModelOnly` claim ceiling.

Before signing, the trusted verifier obtains the complete Git tree for the triggering run's exact `head_sha`. The D6S success status is not treated as sufficient proof by itself: the canonical verifier programs and canonical reference inputs are independently pinned in the trusted policy. It requires every policy-listed tracked path to resolve to an ordinary Git blob with an allowed file mode and the exact expected SHA. A truncated tree, missing path, blob mismatch, symlink mode, or submodule/non-blob entry fails closed. The artifact's self-reported hashes therefore cannot substitute for repository state.

## Artifact boundary

The downloaded artifact is treated as inert data. The verifier requires exactly three regular files at the artifact root:

- `d6u-runtime-evidence.txt`
- `d6u-runtime-test.log`
- `Cargo.lock`

Nested directories, symlinks, special files, missing files, and extra files are rejected before any signing operation.

## Attestation boundary

The D6U main-owned runtime executor has `contents: read` only and explicitly defers attestation. The trusted workflow owns:

- `id-token: write`;
- `attestations: write`;
- `artifact-metadata: write`.

The trusted workflow signs the exact downloaded evidence, runtime log, and Cargo.lock only after independent verification.

A successful D6U pull-request run therefore means runtime evidence was produced and uploaded. A trusted signed attestation means the default-branch verifier accepted that evidence against its independently reviewed policy and signed the exact resulting bytes.

Neither event upgrades the D6S claim ceiling beyond `ReferenceModelOnly`.

## Fail-closed self-test boundary

The read-only trusted-verifier suite contains thirteen deterministic checks: valid evidence acceptance; case-outcome tampering rejection; duplicate-case rejection; live executor-run identity rejection; D6S prerequisite-policy pin coverage; Cargo.lock checksum tampering rejection; duplicate-record-key rejection; trigger-run identity and workflow-blob tampering rejection; exact Git-blob acceptance; executor workflow-identity tampering rejection; Git symlink-mode rejection; Git submodule/non-blob rejection; truncated-tree rejection; and regular-file artifact-layout symlink rejection.

The self-test has no signing permissions and is not itself an authority root.

## Workflow-run chain

The intended chain is exactly three levels: `D6S Canonical Qualification` → `D6U Exact-Head Runtime Executor` → `D6U Trusted Evidence Attestation`. GitHub documents that `workflow_run` chaining is limited to three levels, so this design deliberately stops at the privileged attestation root.


Current trusted policy revision: v7.
