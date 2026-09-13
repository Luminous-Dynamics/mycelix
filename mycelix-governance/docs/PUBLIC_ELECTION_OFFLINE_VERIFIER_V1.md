# Mycelix Public Election Offline Verifier Contract v0.1

Status: **ELECT-011 foundation; portable package + verifier contract, not a cryptographic verifier implementation**

Parent stack:

- `mycelix-public-election-v1`
- `mycelix-public-election-anonymous-authority-v1`
- `mycelix-public-election-transparency-v1`
- `mycelix-public-election-witness-quorum-v1`

Profiles:

- `mycelix-public-election-evidence-package-v1`
- `mycelix-public-election-offline-verifier-v1`

## Purpose

Election verification must survive loss, compromise, or unavailability of the ordinary Mycelix runtime.

The target theorem is:

> A complete public election evidence package can be copied to an offline machine and independently evaluated without Holochain, Symthaea, a Mycelix server, the election vendor's software, a network connection, wall-clock trust, or nondeterministic runtime state.

This tranche defines that boundary. It does **not** pretend to implement the cryptographic verifiers that future ballot, transparency, anonymous-authority, tally, and audit profiles will require.

## Public evidence package

`ElectionEvidencePackageManifestV1` binds:

- the public-election and evidence-package profile identities;
- exact election constitution digest;
- exact package canonicalization profile digest;
- exact artifact-index digest;
- exact package-root digest;
- exact final transparency checkpoint digest;
- a bounded public artifact inventory; and
- optional interoperability profile references.

The core package requires exactly one logical artifact for each of:

1. election constitution;
2. transparency checkpoint chain;
3. witness attestations;
4. anonymous-authority census;
5. tally evidence;
6. physical-audit evidence;
7. challenge ledger; and
8. certification policy.

This is deliberately broader than a cryptographic tally record. Mycelix certification is supposed to explain the entire evidence chain, including challenges and the physical audit.

## Public package means public

Every packaged artifact is explicitly classified as `PublicVerificationEvidence`.

`NonPublicOrSecret` artifacts are rejected from the public manifest.

This does not imply that every underlying election record is legally publishable. Sensitive originals may remain under controlled custody while the public package carries privacy-preserving derived evidence or authenticated digests. What must never happen is accidentally placing trustee private keys, voter identity mappings, credential secrets, or similar material into the public verification package.

ElectionGuard provides a useful precedent: its election record is intended to contain the public artifacts needed for independent verification while excluding sensitive data such as guardian private keys.

Reference:

- https://electionguard.vote/develop/Election_Record/

## Canonical package paths

Artifact paths are part of the package contract and must be canonical relative paths.

The structural validator rejects:

- empty paths;
- leading or trailing `/`;
- Windows-style `\\` separators;
- `.` / `..` components;
- empty path components;
- control characters; and
- paths exceeding the bounded profile limit.

The purpose is both reproducibility and archive safety: a verification bundle must not be able to smuggle path traversal or host-specific path semantics into extraction/verification.

The current crate validates path structure only. A concrete packer/verifier must still defend against symlinks, archive bombs, duplicate archive entries, decompression limits, filesystem races, and parser resource exhaustion.

## Required verification stages

A complete offline verifier run contains typed receipts for all of:

```text
PackageIntegrity
ElectionConstitution
TransparencyLineage
WitnessQuorum
AnonymousAuthorityCensus
TallyEvidence
PhysicalAudit
ChallengeLedger
CertificationEvidence
```

Missing or duplicated stages fail structurally.

Each stage receipt binds the exact package root, exact stage subject, exact verifier release, typed disposition, and detailed-finding digest.

`classify_verifier_run(...)` has only three aggregate dispositions:

- all required stages report pass;
- at least one stage failed; or
- at least one stage is indeterminate.

Failure dominates indeterminacy.

A passing verifier run is **not certification authority**. It is evidence consumed by the frozen election certification policy.

## Offline determinism boundary

`OfflineVerifierExecutionPolicyV1::default()` forbids verifier dependence on:

- network access;
- wall-clock time;
- nondeterministic randomness;
- external process execution;
- mutable global state;
- Holochain runtime; and
- Symthaea runtime.

The verifier may read the supplied immutable package and use local deterministic libraries. Any currentness/time validity needed by an election theorem must already be represented by authenticated election evidence rather than by asking the verifier's local clock or a web service.

This is stronger than merely offering a command-line tool. It makes the result reproducible from the same subject bytes.

## Independent verifier quorum

The foundation's `independent_verifier_quorum_passed` certification gate should not mean “run the same binary three times.”

`VerifierAgreementPolicyV1` therefore distinguishes:

```text
unique verifier releases
unique implementation lineages
unique builder control domains
```

A renamed/forked packaging of the same implementation lineage does not satisfy implementation diversity. Three independently named implementations built and controlled by one organization do not satisfy builder-control diversity when the frozen policy requires multiple control domains.

This is structural independence evidence, not a guarantee that implementations share no common algorithmic bug. Later qualification should deliberately diversify languages, parser stacks, proof libraries, and development teams where feasible.

## Interoperability instead of proprietary exports

The package supports explicit `InteroperabilityProfileRefV1` bindings without making a U.S.-specific schema mandatory for every Mycelix jurisdiction.

For U.S. election adapters, current NIST Common Data Formats already cover important boundaries including:

- Ballot Definition — NIST SP 1500-20;
- Cast Vote Records — NIST SP 1500-103;
- Election Results Reporting — NIST SP 1500-100r2; and
- Election Event Logging — NIST SP 1500-101 work.

The NIST implementation guidance emphasizes interoperable CDFs for ballot definition, CVR, voter records, results, and event logging; CVR exports are particularly useful for ballot-level comparison audits.

References:

- https://www.nist.gov/publications/ballot-definition-common-data-format-specification
- https://www.nist.gov/publications/cast-vote-records-common-data-format-specification-version-10
- https://www.nist.gov/publications/election-results-common-data-format-specification-revision-20
- https://pages.nist.gov/ElectionEventLogging/
- https://nvlpubs.nist.gov/nistpubs/gcr/2024/24-058/NIST.GCR.24-058.html

Mycelix should adapt to these at the boundary while preserving stricter internal evidence identities. The evidence-package manifest binds both the exact interoperability profile identity and the schema/specification digest so a version change cannot occur silently.

## Fail-closed distinctions

The following are deliberately not equivalent:

```text
artifact exists        != artifact digest verified
stage receipt exists   != stage passed
one verifier passed    != independent verifier quorum
verifier quorum passed != election certified
public package valid   != election outcome correct
```

The verifier contract exists to make those boundaries machine-visible.

## Deliberate non-claims

This tranche does not implement:

- package-root hashing/canonicalization;
- archive extraction;
- Merkle inclusion/consistency verification;
- witness signature or credential verification;
- anonymous-eligibility proof verification;
- nullifier derivation verification;
- ballot/tally proof verification;
- physical risk-limiting-audit mathematics;
- challenge adjudication;
- verifier attestation signatures; or
- election certification.

Those implementations must plug into the frozen stage contract and emit evidence-bound receipts.

## Next tranche

Proceed with **ELECT-012 — physical election evidence**:

- paper ballot stock/accounting conservation;
- batches, containers, seals, and custody transitions;
- scanner/CVR reconciliation evidence;
- audit sample and observation evidence;
- discrepancy/escalation states; and
- exact binding of physical-audit results back into the public transparency checkpoint and offline evidence package.

That tranche should remain independent of any one RLA algorithm, because audit laws and approved methods vary by jurisdiction.
