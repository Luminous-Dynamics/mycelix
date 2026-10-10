# D6U trusted-builder boundary

Status: Experimental; security architecture

## Purpose

The D6U runtime workflow executes pull-request code and therefore must be treated as an untrusted measurement environment. The default branch owns the signing authority separately.

The reviewed policy revision, privileged workflow blob, and trusted program blobs are co-resident in the trusted-root commit and cross-pinned by exact blob SHA; the workflow fails closed if those identities diverge. The privileged workflow compares every trusted executable component against its tracked Git index blob SHA and requires a regular Git file mode before invocation. The trusted workflow in `.github/workflows/d6u-trusted-evidence-attestation.yml` is triggered by completion of the D6U runtime workflow through `workflow_run`. The trusted job checks out the exact `github.workflow_sha` for the trusted workflow execution, asserts both the commit and workflow-ref identity, and verifies the privileged workflow blob against the policy, preflights the exact artifact identity and archive size, downloads the exact GitHub artifact archive, verifies its SHA-256 digest, and safely extracts only the bounded expected members into the runner's temporary directory.

It does not check out the pull-request head, execute files from the downloaded artifact, import pull-request Python or Rust modules, or use pull-request code as its policy root. The executor records the upstream D6S run ID/attempt, source branch/repository/SHA, trigger-workflow identity, and its own executor-workflow identity. The source-side verifier redundantly rejects the wrong qualification branch, while the trusted builder independently re-fetches the upstream D6S run, binds the executor definition to the executor run's GitHub-supplied main-branch `head_sha`, and checks the exact source tree. It also rejects fork-origin runs; the signing boundary is same-repository only. The trusted root additionally pins the numeric GitHub repository ID and requires the current `github.repository_id`, triggering event, workflow run, and artifact metadata to agree, preventing a repository-name delete/recreate from silently becoming a new trust domain. The central verifier repeats the same repository-ID checks instead of relying only on workflow-level conditions.

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
- the exact immutable Action revisions used by the privileged workflow, including the retention upload action;
- the exact `gh` CLI version and isolated CLI configuration boundary;
- all native case outcomes and zome reachability;
- supplemental substrate witnesses;
- lockfile format/version, the exact trusted Cargo.toml root package, complete dependency-edge closure, all-package reachability, substrate versions, and crates.io provenance; every non-local lockfile package must use the reviewed crates.io registry and carry a SHA-256 checksum;
- the `ReferenceModelOnly` claim ceiling;
- the public-transparency Tlog requirement and retained offline-attestation packet schema/limits.

Before signing, the trusted verifier obtains the complete Git tree for the triggering run's exact `head_sha`. The D6S success status is not treated as sufficient proof by itself: the canonical verifier programs and canonical reference inputs are independently pinned in the trusted policy. It requires every policy-listed tracked path to resolve to an ordinary Git blob with an allowed file mode and the exact expected SHA. A truncated tree, missing path, blob mismatch, symlink mode, submodule/non-blob entry, or inherited Cargo configuration fails closed. The privileged workflow also fails closed if its own workflow-ref or pinned workflow blob does not match the reviewed policy. The artifact's self-reported hashes therefore cannot substitute for repository state. The runtime evidence record is also treated as a closed-world schema: every policy-listed field must be present exactly once, and unknown fields are rejected rather than silently tolerated.

## Artifact boundary

The trusted GitHub API readers bound JSON response bodies to 8 MiB before parsing; both readers use a dedicated non-forwarding redirect handler that keeps redirects on HTTPS, rejects URL userinfo, and never forwards the bearer Authorization header. This accommodates the current Git tree recursive API ceiling while preventing an unexpectedly large API response or credential-bearing redirect from becoming an unbounded verifier input. See the policy's `trusted_network.github_api_response_max_bytes` value.

Artifact provenance is bound to the triggering workflow run, not to the default branch metadata of the `workflow_run` consumer. For both the executor evidence artifact and the auditor handoff artifact, the trusted fetcher requires the artifact's recorded workflow-run attempt, head branch, and head SHA to match the triggering workflow context. GitHub documents that `workflow_run` consumers receive `GITHUB_SHA` and `GITHUB_REF` for the default branch, so those values are not used as substitutes for the triggering run's `head_sha`/branch.

The downloaded artifact is treated as inert data. The trusted workflow first requires exactly one non-expired artifact with the expected run-specific name and an archive size within policy. It independently downloads the immutable artifact archive with a streaming byte cap, compares its SHA-256 digest to GitHub's artifact API digest, preflights the ZIP EOCD entry count before opening the ZIP parser (and rejects Zip64), rejects encrypted/symlink/directory/duplicate/unexpected members, permits only stored/deflate (Zlib) compression, and enforces compressed-download and uncompressed-member bounds, and only then extracts the exact expected root files. The auditor handoff uses the same bounded/digest-checked fetcher rather than `actions/download-artifact`, so the handoff cannot cause an unbounded archive ingest before its post-download checks. The current workflow-run lookup is also bound to both repository name and numeric repository ID before artifact selection.

- `d6u-runtime-evidence.txt`
- `d6u-runtime-test.log`
- `Cargo.lock`

Nested directories, symlinks, special files, missing files, extra files, oversized members, excessive filesystem entries, and oversized aggregate input are rejected before any signing operation.

## Attestation boundary

The signer is the only privileged job and has only `contents: read`, `id-token: write`, and `attestations: write`; `actions: read` and `artifact-metadata: write` are intentionally absent. Attestation registry publication, storage-record creation, and workflow-summary attachment are explicitly disabled with `push-to-registry: false`, `create-storage-record: false`, and `show-summary: false`. The signer receives only four bounded SHA-256 values from the verifier. The D6U main-owned runtime executor has `contents: read` only and explicitly defers attestation.

The verifier independently validates the evidence and derives exact SHA-256 identities for the three attested subjects. Before the custom attestation verifier parses an online `gh attestation verify` report, the trusted workflow requires the report to be non-empty and no larger than the policy-defined 4 MiB bound. The signer receives no original subject files and uses `subject-checksums` to attest those exact digests, with a minimal `d6u-trusted-runtime-evidence-attestation/v2` commitment predicate containing the canonical evidence-predicate SHA-256, claim ceiling, policy version, schema, and attestation kind. This is an evidence attestation, not a claim that the trusted workflow built the evidence. Attestation verification additionally pins the exact GitHub Actions OIDC issuer and certificate SAN for this workflow, alongside the signer workflow path and signer workflow commit digest. The signed commitment predicate is bound to the retained canonical `d6u-trusted-runtime-evidence/v1` predicate by its exact byte-level SHA-256. It also requires the certificate `runInvocationURI` to identify the current trusted workflow run/attempt and requires the signed predicate to match the independently verified evidence record. The subject binding is an exact set of three `(name, sha256)` identities, so statement ordering is irrelevant while duplicates, extra algorithms, altered names, or altered digests fail closed. The trusted verifier now requires a verified `Tlog` witness rather than accepting an RFC3161-only timestamp, and records the verification as relying on the `sigstore-public-good` instance.

After signing, the same trusted root downloads the exact attestation bundles and the current Sigstore trusted-root material, then performs offline verification using the retained bundle and custom trusted root. It also performs a negative control with `--no-public-good`; the control must fail, so a successful retained verification cannot silently substitute the non-public verification path. A bounded retention packet records the exact subject digests, bundle/root/report hashes, current run identity, signer/source identities, policy/predicate schema, and claim ceiling. The packet is uploaded under a run/attempt-derived immutable artifact name.

A successful D6U pull-request run therefore means runtime evidence was produced and uploaded. A trusted signed attestation means the default-branch verifier accepted that evidence against its independently reviewed policy and signed the exact resulting bytes. The retained packet additionally makes the successful public-transparency verification reproducible offline.

Neither event upgrades the D6S claim ceiling beyond `ReferenceModelOnly`.

## Fail-closed self-test boundary

The read-only trusted-verifier suite contains seventy-three deterministic checks: valid evidence acceptance; case-outcome tampering rejection; duplicate-case rejection; live executor-run identity rejection; D6S prerequisite-policy pin coverage; Cargo.lock checksum tampering rejection; duplicate-record-key rejection; trigger-run identity and workflow-blob tampering rejection; exact Git-blob acceptance; executor workflow-identity tampering rejection; Git symlink-mode rejection; Git submodule/non-blob rejection; truncated-tree rejection; regular-file artifact-layout symlink rejection; artifact size-limit enforcement; artifact entry-count enforcement; trusted-workflow policy-shape enforcement; policy binding to the current trusted workflow blob; executor run-head binding to the reviewed executor workflow blob; bounded ZIP extraction adversaries for exact-member, duplicate-member, symlink-member, and traversal-path rejection; and current-run versus historical-attestation identity checks.

The self-test has no signing permissions and is not itself an authority root. It also asserts the exact three job-level permission maps, exact four verifier-to-signer outputs, numeric repository-identity guards, and the executable test registry's closure over all defined checks. Its policy-shape test pins the retention verifier, upload Action revision, public Tlog requirement, and retained packet limits; dedicated regressions cover the offline workflow controls, unexpected retention-packet members, bounded handoff download, streamed archive overflow, pre-parser ZIP entry-count exhaustion, non-Zlib compression rejection, unknown configured compression names, redirect transport/credential boundaries, signer publication behavior, and executable test-registry completeness.

## Branch-validation boundary

The trusted attestation workflow is deliberately not branch-executable. GitHub documents that `workflow_run` only triggers when the workflow file exists on the default branch, and the resulting run uses the default branch for `GITHUB_SHA`/`GITHUB_REF`. Feature-branch pushes that surface a workflow-file run without jobs are therefore not treated as failed attestation evidence; they cannot create signing authority. The deterministic self-test is the branch-side validation surface. Real trusted attestation execution occurs only after the reviewed workflow/policy root is present on the default branch and a qualifying main-owned executor run completes.

## Workflow-run chain

The intended chain is exactly three levels: `D6S Canonical Qualification` → `D6U Exact-Head Runtime Executor` → `D6U Trusted Evidence Attestation`. GitHub documents that `workflow_run` chaining is limited to three levels, so this design deliberately stops at the privileged attestation root.


Current trusted policy revision: v61.
