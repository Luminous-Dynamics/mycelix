# GitHub Public Qualification Attestation Verifier V1

Status: **contract / unqualified**

Tracking issue: #2069

This document freezes the first concrete verifier-execution theorem for Mycelix
qualification receipts. It builds on the canonical/authentication core and strict
untrusted-wire boundary; it does not itself qualify GitHub CLI, Sigstore, or any
production admission path.

## 1. Governing separation

```text
strict JSON accepted
    != verifier executable trusted
    != bundle cryptographically verified
    != signer workflow trusted
    != signed predicate independently true
    != authenticated qualification receipt
    != production proof admission
    != application authority
```

The v1 verifier MUST preserve every boundary above as separately inspectable
evidence.

## 2. Profile identity

The initial deployment profile is:

`GitHubPublicQualificationAttestationVerifierV1`

It is only valid for a public GitHub repository using GitHub Actions and the
Sigstore Public Good trust path.

The profile MUST NOT be silently reused for private/internal repositories,
GitHub Enterprise Server, GitHub's private Sigstore instance, self-hosted
runners, OCI-registry verification, or a pure-Rust verifier backend.

## 3. Canonical subject

The verification subject is never an arbitrary caller-supplied file.

The backend MUST:

1. accept a validated `QualificationReceiptV1`;
2. reconstruct its exact v1 canonical bytes;
3. compute `QualificationReceiptDigestV1` from those bytes;
4. write those exact canonical bytes to a private temporary subject file;
5. independently SHA-256 the temporary file and require equality with the
   canonical receipt digest;
6. use that temporary file as the artifact supplied to GitHub attestation
   download/verification.

The attested in-toto subject name MUST equal the exact name required by
`ReceiptAuthenticationPolicyV1.expected_attestation_subject_name`.

This prevents a path supplied by a caller from becoming receipt identity.

## 4. Verifier executable identity

A `gh` version string alone is not verifier identity.

Every authority-bearing invocation MUST bind an execution profile containing at
least:

```text
VerifierExecutableIdentityV1
  profile_id
  backend_family = github-cli-attestation
  semantic_version
  executable_sha256
  platform_profile
  command_profile_id
  nix_store_path?          # when executed from Nix
  nix_closure_digest?      # when retained by deployment profile
```

For Nix deployments, the preferred profile additionally binds the exact Nix
store/closure identity that produced the executable.

Changing any theorem-bearing identity above creates a new verifier evidence
lineage. A binary with the expected version text but a different digest MUST be
rejected.

## 5. Process isolation

The backend MUST invoke the exact executable path from the verified execution
identity rather than searching `$PATH` after admission.

The process environment MUST be created from an explicit allowlist. In
particular:

- use an isolated temporary `GH_CONFIG_DIR`;
- set the GitHub host explicitly on every command;
- do not inherit repository-selection configuration;
- do not inherit aliases/extensions as authority-bearing behavior;
- treat credentials only as API-access capability, never signer trust;
- bound stdout and stderr;
- impose a process timeout;
- retain exit status and output digests.

Environment additions that can alter trust roots, host selection, TLS behavior,
or verifier semantics require a distinct qualified execution profile.

## 6. Preferred v1 acquisition/verification flow

V1 uses **online acquisition followed by retained local verification**.

### 6.1 Materialize subject

Create `qualification-receipt.bin` from the exact canonical receipt bytes and
verify its SHA-256 against the receipt digest.

### 6.2 Download candidate bundles

Use the exact verifier executable with policy-equivalent arguments including:

```text
gh attestation download qualification-receipt.bin
  --repo Luminous-Dynamics/mycelix
  --predicate-type <exact Mycelix predicate URI>
  --limit <bounded value>
  --hostname github.com
```

Downloaded bundles are **untrusted input** until verified.

The resulting JSON/JSONL bytes and SHA-256 digest MUST be retained in the
execution evidence.

### 6.3 Resolve current trusted-root material

Immediately before verification, use the same exact admitted `gh` executable to
obtain trusted-root material through its authenticated TUF path:

```text
gh attestation trusted-root --hostname github.com
```

Retain the exact trusted-root bytes, their SHA-256 digest, acquisition time, and
the verifier executable identity that resolved them.

Root acquisition success does not itself authenticate the qualification receipt.

### 6.4 Verify retained bytes

Verify the canonical subject file against the retained bundle and retained
trusted-root material, using explicit constraints equivalent to:

```text
gh attestation verify qualification-receipt.bin
  --repo Luminous-Dynamics/mycelix
  --bundle <retained-bundle>
  --custom-trusted-root <retained-trusted-root>
  --predicate-type <exact Mycelix predicate URI>
  --cert-oidc-issuer https://token.actions.githubusercontent.com
  --signer-workflow <exact trusted signer workflow>
  --signer-digest <exact allowed signer revision when required>
  --source-digest <exact source commit>
  --source-ref <exact source ref when required>
  --deny-self-hosted-runners
  --limit <bounded value>
  --hostname github.com
  --format json
```

Every omitted constraint must be justified by a different named verifier
profile. V1 does not fall back from exact repository scope to owner-only scope.

## 7. Trusted builder requirement

Cryptographic verification proves that the certificate-authenticated signer made
the signed in-toto statement. It does not make workflow-controlled predicate
content independently true.

Therefore `GitHubPublicQualificationAttestationVerifierV1` requires a trusted
signer workflow profile.

The signer workflow MUST be identified by exact repository/path and a revision
policy. The initial production-oriented profile SHOULD use a reusable qualifier/
signer workflow whose theorem-bearing inputs cannot be replaced by arbitrary
caller-provided predicate JSON after qualification.

If the signer workflow is freely parameterized by untrusted caller input, its
custom predicate cannot establish qualification truth merely because the
signature verifies.

## 8. Exact GitHub policy fields

The verifier profile records and enforces:

- exact repository: `Luminous-Dynamics/mycelix`;
- exact GitHub host;
- exact OIDC issuer;
- exact predicate type URI;
- exact signer workflow;
- signer workflow revision/digest policy;
- exact source commit digest;
- exact source ref policy;
- hosted-runner-only requirement;
- bounded attestation result count;
- exact Mycelix predicate schema/version;
- exact canonical receipt subject name and SHA-256.

The policy may be generalized only by defining and qualifying a new profile.

## 9. Verification JSON is evidence, not authority

`gh attestation verify --format json` output MUST be treated as untrusted process
output until all of the following hold:

1. the exact admitted verifier executable ran;
2. the process exited successfully;
3. output stayed within configured resource limits;
4. output parsed against a strict versioned DTO/schema;
5. certificate-derived identity matches the exact authentication policy;
6. verified timestamp/transparency facts satisfy the selected policy;
7. in-toto subject name/digest match the canonical receipt;
8. predicate type is exact;
9. predicate bytes pass the strict Mycelix parser from #2066;
10. predicate receipt/profile/subject/coherence/result fields match the canonical
    receipt;
11. #2058's sealed capability minting boundary independently re-checks all
    policy-bindable facts.

A saved JSON report supplied by a caller is never sufficient to mint an
authenticated capability.

## 10. Multiple verified attestations

V1 fails closed when more than one verified result survives all exact identity
constraints unless the profile defines an unambiguous deterministic selection
rule over an authenticated identity tuple.

"First result wins" is forbidden.

The preferred v1 behavior is **exactly one matching attestation**.

## 11. Trust-root modes

### 11.1 OnlineFetchThenRetainedVerifyV1

This is the preferred initial profile.

It:

- obtains trust material immediately before verification through the admitted
  verifier's authenticated TUF path;
- retains the exact resolved material and digest;
- retains the candidate bundle bytes and digest;
- performs verification against those retained bytes;
- records verification time.

The resulting evidence proves the root material used at that verification event;
it does not promise eternal future currentness.

### 11.2 OfflinePinnedRootV1

A separate profile may verify a retained bundle against an exact pinned trusted
root without network access.

It MUST retain:

- exact root bytes/digest;
- root acquisition provenance;
- acquisition timestamp;
- exact bundle bytes/digest;
- verifier executable identity.

It MUST NOT claim knowledge of trust-root revocations or rotations occurring
after that snapshot was acquired.

The two modes are not interchangeable.

## 12. Execution receipt

A verifier run emits structural evidence similar to:

```text
VerifierExecutionReceiptV1
  evidence_version
  verifier_executable_identity
  command_profile_id
  canonical_receipt_digest
  attestation_bundle_digest
  trust_root_mode
  trusted_root_material_digest
  verifier_stdout_digest
  verifier_stderr_digest
  started_at
  completed_at
  exit_status
  parsed_result_count
```

This receipt is evidence about verifier execution. It does not itself establish
receipt authentication unless the concrete backend theorem is qualified and the
sealed capability was successfully minted.

## 13. Authentication capability creation

Only after cryptographic verification and policy parsing succeed may the backend
construct its crate-internal verified-fact objects and call the sealed minting
boundary introduced by #2058.

The output remains:

`AuthenticatedQualificationReceiptV1`

with authority scope:

`ReceiptAuthenticationOnly`.

It MUST still report false for production and application authority.

## 14. Required qualification corpus

A future `MYC-ZKP-GH-ATTEST-001AQ` lane must reject at minimum:

1. expected version text with wrong `gh` executable digest;
2. expected executable digest with unapproved platform/execution profile;
3. inherited configuration that changes repository/host/trust semantics;
4. owner-only scope instead of exact repository scope;
5. wrong OIDC issuer;
6. wrong signer workflow;
7. wrong signer revision/digest;
8. wrong source commit;
9. wrong source ref;
10. wrong predicate type;
11. self-hosted-runner attestation under hosted-only policy;
12. wrong in-toto subject name;
13. wrong in-toto subject digest;
14. malformed bundle;
15. invalid bundle signature;
16. trusted-root substitution;
17. malformed or oversized verification JSON;
18. forged detached verification report;
19. unknown verification-result fields under the pinned parser profile;
20. multiple otherwise matching attestations;
21. valid signature over a predicate whose receipt digest is different;
22. valid signature over a predicate whose qualification/coherence/result differs;
23. offline stale root being labeled current-online trust;
24. verifier success being upgraded to production/application authority.

Qualification SHOULD use retained positive and negative fixtures in addition to
live GitHub integration tests so cryptographic failure paths remain replayable.

## 15. Nonclaims

This contract does not:

- bless any current GitHub CLI version or executable digest;
- qualify the Sigstore trust path;
- authenticate any current Mycelix qualification run;
- establish an authenticated revocation registry;
- establish production proof admission;
- establish witness privacy/zero knowledge;
- authorize Governance, Finance, Health, FL, or any other Mycelix domain.

Those remain separate theorems.
