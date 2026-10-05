# D6U trusted-builder boundary

Status: Experimental; security architecture

## Purpose

The D6U runtime workflow executes pull-request code and therefore must be treated as an untrusted measurement environment. The default branch owns the signing authority separately.

The privileged workflow and policy root are advanced atomically: the reviewed policy revision, the privileged workflow blob, the trusted program blobs, and the workflow's self-cohesion assertion are carried in the same Git commit. The privileged workflow compares every trusted executable component against its tracked Git index blob SHA and requires a regular Git file mode before invocation. The trusted workflow in `.github/workflows/d6u-trusted-evidence-attestation.yml` is triggered by completion of the D6U runtime workflow through `workflow_run`. The trusted job checks out the exact `github.workflow_sha` for the trusted workflow execution, asserts both the commit and workflow-ref identity, and verifies the privileged workflow blob against the policy, preflights the exact artifact identity and archive size, downloads the exact GitHub artifact archive, verifies its SHA-256 digest, and safely extracts only the bounded expected members into the runner's temporary directory.

It does not check out the pull-request head, execute files from the downloaded artifact, import pull-request Python or Rust modules, or use pull-request code as its policy root. The executor records the upstream D6S run ID/attempt, source branch/repository/SHA, trigger-workflow identity, and its own executor-workflow identity. The source-side verifier redundantly rejects the wrong qualification branch, while the trusted builder independently re-fetches the upstream D6S run, binds the executor definition to the executor run's GitHub-supplied main-branch `head_sha`, and checks the exact source tree. It also rejects fork-origin runs; the signing boundary is same-repository only.

## Trusted policy

`docs/integral/d6u-trusted-builder-policy.json` is the independently reviewed policy root.

It pins:

- the D6U workflow identity and exact main-branch executor blob;
- the trusted attestation workflow's own exact blob identity;
- the executor workflow blob as observed at the executor run's GitHub-supplied `head_sha`;
- the D6U manifest and harness executable surface;
- the D6S-CANON-1 manifest, golden corpus, and independent verifier;
- the D6S-CANON-2 manifest, authority-boundary fixture, and independent verifier;
- the expected runtime versions;
- the absence of Cargo configuration files that could be inherited by the D6U harness;
- the exact tracked D6U harness file set, excluding build scripts and other injected executable sources;
- the maximum trusted artifact archive/member/entry sizes;
- the trusted artifact fetcher and bounded ZIP extraction policy;
- the canonical D6U evidence-attestation predicate and its emitter;
- all native case outcomes and zome reachability;
- supplemental substrate witnesses;
- lockfile substrate versions and crates.io provenance;
- the `ReferenceModelOnly` claim ceiling.

Before signing, the trusted verifier obtains the complete Git tree for the triggering run's exact `head_sha`. The D6S success status is not treated as sufficient proof by itself: the canonical verifier programs and canonical reference inputs are independently pinned in the trusted policy. It requires every policy-listed tracked path to resolve to an ordinary Git blob with an allowed file mode and the exact expected SHA. A truncated tree, missing path, blob mismatch, symlink mode, submodule/non-blob entry, or inherited Cargo configuration fails closed. The privileged workflow also fails closed if its own workflow-ref or pinned workflow blob does not match the reviewed policy. The artifact's self-reported hashes therefore cannot substitute for repository state.

## Artifact boundary

The downloaded artifact is treated as inert data. The trusted workflow first requires exactly one non-expired artifact with the expected run-specific name and a total archive size within policy. It independently downloads the immutable artifact archive, compares its SHA-256 digest to GitHub's artifact API digest, rejects encrypted/symlink/directory/duplicate/unexpected ZIP members, enforces compressed-download and uncompressed-member bounds, and only then extracts exactly three root files:

- `d6u-runtime-evidence.txt`
- `d6u-runtime-test.log`
- `Cargo.lock`

Nested directories, symlinks, special files, missing files, extra files, oversized members, excessive filesystem entries, and oversized aggregate input are rejected before any signing operation.

## Attestation boundary

The privileged attestation job now has only `actions: read`, `contents: read`, `id-token: write`, and `attestations: write`. `artifact-metadata: write` is intentionally absent because GitHub documents that permission as necessary for linked-artifact storage records when `push-to-registry` is used, not for ordinary binary artifact attestations.
The D6U main-owned runtime executor has `contents: read` only and explicitly defers attestation. The trusted workflow owns:

- `id-token: write`;
- `attestations: write`;
- `artifact-metadata: write`.

The trusted workflow signs the exact evidence files only after independent verification, using the dedicated `d6u-trusted-runtime-evidence/v1` evidence predicate. This is an evidence attestation, not a claim that the trusted workflow built the evidence. Attestation verification additionally pins the exact GitHub Actions OIDC issuer and certificate SAN for this workflow, alongside the signer workflow path and signer workflow commit digest. It also requires the certificate `runInvocationURI` to identify the current trusted workflow run/attempt and requires the signed predicate to match the independently verified evidence record. The subject binding is an exact set of three `(name, sha256)` identities, so statement ordering is irrelevant while duplicates, extra algorithms, altered names, or altered digests fail closed. The trusted verifier also requires at least one `verifiedTimestamps` entry from GitHub CLI verification.

A successful D6U pull-request run therefore means runtime evidence was produced and uploaded. A trusted signed attestation means the default-branch verifier accepted that evidence against its independently reviewed policy and signed the exact resulting bytes.

Neither event upgrades the D6S claim ceiling beyond `ReferenceModelOnly`.

## Fail-closed self-test boundary

The read-only trusted-verifier suite contains thirty-one deterministic checks: valid evidence acceptance; case-outcome tampering rejection; duplicate-case rejection; live executor-run identity rejection; D6S prerequisite-policy pin coverage; Cargo.lock checksum tampering rejection; duplicate-record-key rejection; trigger-run identity and workflow-blob tampering rejection; exact Git-blob acceptance; executor workflow-identity tampering rejection; Git symlink-mode rejection; Git submodule/non-blob rejection; truncated-tree rejection; regular-file artifact-layout symlink rejection; artifact size-limit enforcement; artifact entry-count enforcement; trusted-workflow policy-shape enforcement; policy binding to the current trusted workflow blob; executor run-head binding to the reviewed executor workflow blob; bounded ZIP extraction adversaries for exact-member, duplicate-member, symlink-member, and traversal-path rejection; and current-run versus historical-attestation identity checks.

The self-test has no signing permissions and is not itself an authority root.

## Workflow-run chain

The intended chain is exactly three levels: `D6S Canonical Qualification` → `D6U Exact-Head Runtime Executor` → `D6U Trusted Evidence Attestation`. GitHub documents that `workflow_run` chaining is limited to three levels, so this design deliberately stops at the privileged attestation root.


Current trusted policy revision: v24.
